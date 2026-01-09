#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <functional> // 补充bind所需头文件

// 避免命名空间冲突：不使用using namespace，显式指定命名空间
// 核心修复：所有ROS2相关类显式用rclcpp::，OpenCV用cv::

class CompressedImageSaver : public rclcpp::Node // 显式指定rclcpp::Node
{
public:
    CompressedImageSaver() : rclcpp::Node("compressed_image_saver_cpp") // 修正基类初始化
    {
        // 创建保存目录（不存在则创建）
        save_dir_ = "saved_images_cpp";
        std::filesystem::create_directory(save_dir_);

        // 创建点云保存目录
        save_pc_dir_ = "saved_pointclouds_cpp";
        std::filesystem::create_directory(save_pc_dir_);

        // 订阅CompressedImage话题（修复create_subscription调用）
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera_mid_front_up", // 话题名
            10,                     // QoS深度
            std::bind(&CompressedImageSaver::image_callback, this, std::placeholders::_1));

        // 订阅PointCloud2话题，用于逐帧解析点云
        pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar",
            10,
            std::bind(&CompressedImageSaver::pointcloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "图像保存节点已启动！");
        RCLCPP_INFO(this->get_logger(), "图像将保存至：%s", std::filesystem::absolute(save_dir_).c_str());
    }

private:
    // 回调函数：处理接收到的压缩图像
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // 将ROS压缩图像消息转换为OpenCV Mat
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // 生成带时间戳的文件名（避免重复）
            std::string filename = generate_timestamp_filename();
            std::string file_path = save_dir_ + "/" + filename;

            // 保存图像
            if (cv::imwrite(file_path, cv_ptr->image))
            {
                RCLCPP_INFO(this->get_logger(), "保存成功：%s", file_path.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "保存失败：%s", file_path.c_str());
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV_Bridge转换失败：%s", e.what());
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "图像处理异常：%s", e.what());
        }
    }

    // 回调函数：处理接收到的点云消息，转换并保存为 PCD（二进制）
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try
        {
            // 使用 PCL 将 ROS2 PointCloud2 转为 PCLPointCloud2
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*msg, pcl_pc2);

            // 检查 intensity 字段以决定点类型
            bool has_intensity = false;
            for (const auto &f : msg->fields)
            {
                if (f.name == "intensity")
                {
                    has_intensity = true;
                    break;
                }
            }

            // 构造文件名，使用header.stamp（sec_nanosec）以便与图像对应
            std::stringstream ss_fn;
            ss_fn << "pc_" << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec << ".pcd";
            std::string filename = ss_fn.str();
            std::string file_path = save_pc_dir_ + "/" + filename;

            int ret = -1;
            if (has_intensity)
            {
                pcl::PointCloud<pcl::PointXYZI> cloud;
                pcl::fromPCLPointCloud2(pcl_pc2, cloud);
                ret = pcl::io::savePCDFileBinary(file_path, cloud);
            }
            else
            {
                pcl::PointCloud<pcl::PointXYZ> cloud;
                pcl::fromPCLPointCloud2(pcl_pc2, cloud);
                ret = pcl::io::savePCDFileBinary(file_path, cloud);
            }

            if (ret == 0)
            {
                size_t point_count = msg->width * msg->height;
                RCLCPP_INFO(this->get_logger(), "保存点云帧：%s  点数：%zu", file_path.c_str(), point_count);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "保存 PCD 失败：%s (ret=%d)", file_path.c_str(), ret);
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "点云处理异常：%s", e.what());
        }
    }

    // 生成时间戳文件名（格式：image_YYYYMMDD_HHMMSS_微秒.png）
    std::string generate_timestamp_filename()
    {
        // 获取当前系统时间
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;

        // 格式化时间戳
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "image_%Y%m%d_%H%M%S");
        ss << "_" << std::setfill('0') << std::setw(6) << ms.count() << ".png";

        return ss.str();
    }

    // 成员变量（显式指定命名空间）
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscription_;
    std::string save_dir_;
    std::string save_pc_dir_;
};

int main(int argc, char *argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点并运行（修复spin参数类型）
    auto node = std::make_shared<CompressedImageSaver>();
    rclcpp::spin(node);

    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}