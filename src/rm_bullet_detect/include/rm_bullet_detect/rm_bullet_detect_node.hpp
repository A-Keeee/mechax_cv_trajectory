#ifndef RM_BULLET_DETECT_NODE_HPP   
#define RM_BULLET_DETECT_NODE_HPP

#include <iostream>
#include <memory> // 新增
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <image_transport/image_transport.hpp> // 新增
#include <Eigen/Geometry>
#include "detect_bullet.hpp"
#include "do_reproj.hpp"
#include "auto_aim_interfaces/msg/receive_serial.hpp"
#include "auto_aim_interfaces/msg/send_serial.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/bias.hpp"

namespace qianli_rm_bullet_detect
{

class BulletDetectNode : public rclcpp::Node
{
public:
    BulletDetectNode(const rclcpp::NodeOptions & options);

    void bullet_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void angle_callback(const auto_aim_interfaces::msg::ReceiveSerial msg);
    void result_callback(const auto_aim_interfaces::msg::SendSerial msg);



    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr bullet_pose_pub_;
    std::unique_ptr<image_transport::ImageTransport> it_; // 修改为 unique_ptr
    image_transport::Publisher result_image_pub_; // 修改类型为 image_transport::Publisher

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr bullet_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
    rclcpp::Subscription<auto_aim_interfaces::msg::ReceiveSerial>::SharedPtr angle_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::SendSerial>::SharedPtr result_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;


    // 相机矩阵
    cv::Mat camera_matrix_;
    size_t frame_count_;
    rclcpp::Time last_time_;


    // TF2 缓存和监听器
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;


    // // 定时器用于延迟初始化 image_transport
    rclcpp::TimerBase::SharedPtr init_timer_;

    aimer::aim::DetectBullet bullet_detector;

    Eigen::Quaterniond cur_q;


    /// 从 gimbal_link → camera_optical_frame 的旋转
    Eigen::Matrix3d imu2optical_from_urdf() {

    }
private:

    bool tracking = false;
    bool shooting = false;
    rclcpp::Time tracking_time_;
    rclcpp::Time shooting_time_;
    aimer::aim::IdTLatencyAimCorrection cmd;
    aimer::CoordConverter* converter;
    aimer::aim::AimCorrector aim_corrector;
    int aim_id = 0;
    float bullet_v0 = 0.0f;
    float traget_x = 0.0f;
    float traget_y = 0.0f;
    float traget_z = 0.0f;
    // float yaw = 0.0f;
    // float pitch = 0.0f;
    // float roll = 0.0f;
    // float distance = 0.0f;


};


} // namespace qianli_rm_bullet_detect

#endif 