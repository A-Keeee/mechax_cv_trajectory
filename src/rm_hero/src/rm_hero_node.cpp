#include "rm_hero_node.hpp"


namespace qianli_rm_hero
{
    HeroNode::HeroNode(const rclcpp::NodeOptions & options) : Node("rm_hero_node", options),
    frame_count_(0),
    last_time_(this->now())
    
    {
        // 在控制台输出节点启动信息
        RCLCPP_INFO(get_logger(), "Hello, QianLi RM Hero!");

        // 创建订阅者，接受英雄坐标
        hero_pose_sub_ = this->create_subscription<auto_aim_interfaces::msg::ReceiveSerial>(
            "/angle/init", 10, std::bind(&HeroNode::Hero_pose_callback, this, std::placeholders::_1));


        // 创建发布者，用于发布3D预测位置（/hero/prediction）
        hero_pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/hero/prediction", 10);

        // 初始化tf2缓存和监听器，用于将预测的3D坐标转换到不同的坐标系
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    }



    void HeroNode::Hero_pose_callback(const auto_aim_interfaces::msg::ReceiveSerial msg)
    {   

        // 新增帧率计算逻辑
        auto current_time = this->now();
        frame_count_++;
        double elapsed = (current_time - last_time_).seconds();
        
        if (elapsed >= 1.0) {
            double fps = frame_count_ / elapsed;
            RCLCPP_INFO(get_logger(), "[FPS] Current: %.2f", fps);
            frame_count_ = 0;
            last_time_ = current_time;
        }


        // 创建消息并填充预测的3D点位(相机坐标系下的xyz)
        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header.frame_id = "camera_optical_frame";
        // point_msg.header.stamp = msg->header.stamp;
        // point_msg.point.x = tvec.at<double>(0, 0)/100;
        // point_msg.point.y = tvec.at<double>(1, 0)/100;
        // point_msg.point.z = tvec.at<double>(2, 0)/100;


        geometry_msgs::msg::PointStamped transformed_msg;
        try {
            transformed_msg.point = tf2_buffer_->transform(point_msg, "odom").point;
            // double temp = transformed_msg.point.x;
            // transformed_msg.point.x = transformed_msg.point.y;
            // transformed_msg.point.y = temp;
            transformed_msg.header.frame_id = "odom";
            transformed_msg.header.stamp = point_msg.header.stamp;
            hero_pose_pub_->publish(transformed_msg);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "无法将坐标从 camera_link 转换到 odom：%s", ex.what());
        }
    }
} // namespace qianli_rm_hero

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(qianli_rm_hero::HeroNode)