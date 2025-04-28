#include "rm_bullet_detect/rm_bullet_detect_node.hpp"


namespace qianli_rm_bullet_detect
{
    BulletDetectNode::BulletDetectNode(const rclcpp::NodeOptions & options) : Node("bullet_detect_node", options),
    frame_count_(0),
    last_time_(this->now())

    {
        // 在控制台输出节点启动信息
        RCLCPP_INFO(get_logger(), "Hello, QianLi RM Bullet!");


                // 初始化相机内参矩阵
        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);

        // 订阅相机内参
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
                RCLCPP_INFO(this->get_logger(), "Received camera info!");
                RCLCPP_INFO(this->get_logger(), "K matrix: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                        camera_info->k[0], camera_info->k[1], camera_info->k[2],
                        camera_info->k[3], camera_info->k[4], camera_info->k[5],
                        camera_info->k[6], camera_info->k[7], camera_info->k[8]);

                camera_matrix_.at<double>(0, 0) = camera_info->k[0];  // fx
                camera_matrix_.at<double>(0, 2) = camera_info->k[2];   // cx
                camera_matrix_.at<double>(1, 1) = camera_info->k[4];   // fy
                camera_matrix_.at<double>(1, 2) = camera_info->k[5];    // cy
                camera_matrix_.at<double>(2, 2) = 1.0;
                cam_info_sub_.reset();
            });


        RCLCPP_INFO(get_logger(), "model loaded");


        bullet_pose_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/bullet/prediction", 10);

        // 创建订阅者，订阅图像原始数据（/image_raw）
        bullet_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&BulletDetectNode::bullet_image_callback, this, std::placeholders::_1));
        

        angle_sub_ = this->create_subscription<auto_aim_interfaces::msg::ReceiveSerial>(
            "/angle/init", 10, std::bind(&BulletDetectNode::angle_callback, this, std::placeholders::_1));

        result_sub_ = this->create_subscription<auto_aim_interfaces::msg::SendSerial>(
            "/trajectory/result", 10, std::bind(&BulletDetectNode::result_callback, this, std::placeholders::_1));


        // 初始化tf2缓存和监听器，用于将预测的3D坐标转换到不同的坐标系
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        

        // 1) gimbal_link → camera_link （rpy=”0  π/360  π/120”）
        double roll1  = 0.0;
        double pitch1 = M_PI/360.0;  // around Y
        double yaw1   = M_PI/120.0;  // around Z
        Eigen::AngleAxisd R_roll1( roll1,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd R_pitch1(pitch1, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd R_yaw1(  yaw1,   Eigen::Vector3d::UnitZ());
        // before: Eigen::Matrix3d R_cam_link = R_yaw1 * R_pitch1 * R_roll1;
        Eigen::Matrix3d R_cam_link = (R_yaw1 * R_pitch1 * R_roll1).toRotationMatrix();

        // 2) camera_link → camera_optical_frame （rpy=”-π/2  0  -π/2”）
        double roll2  = -M_PI/2.0;  // around X
        double pitch2 =  0.0;       // around Y
        double yaw2   = -M_PI/2.0;  // around Z
        Eigen::AngleAxisd R_roll2( roll2,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd R_pitch2(pitch2, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd R_yaw2(  yaw2,   Eigen::Vector3d::UnitZ());
        // before: Eigen::Matrix3d R_optical = R_yaw2 * R_pitch2 * R_roll2;
        Eigen::Matrix3d R_optical = (R_yaw2 * R_pitch2 * R_roll2).toRotationMatrix();


        // 3) 合成：gimbal_link → camera_optical_frame
        Eigen::Matrix3d imu_eigen = R_optical * R_cam_link;
        cv::Mat imu(3, 3, CV_64F);
        cv::eigen2cv(imu_eigen, imu);



        bullet_detector = aimer::aim::DetectBullet(
            aimer::aim::DoReproj(camera_matrix_, imu)
        );

        cur_q = Eigen::Quaterniond::Identity();



        //debug
        // 创建一次性定时器，用于延迟初始化 image_transport
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 延迟时间，可以根据需要调整
            [this]() {
                try {
                    // 初始化 image_transport::ImageTransport，传递 shared_ptr<Node>
                    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
                    result_image_pub_ = it_->advertise("bullet/result_image", 10);
                    RCLCPP_INFO(get_logger(), "Initialized image_transport publisher for /bullet/result_image");

                    // 取消定时器，因为只需要初始化一次
                    init_timer_->cancel();
                }
                catch (const std::bad_weak_ptr & e) {
                    RCLCPP_ERROR(get_logger(), "Failed to initialize ImageTransport: %s", e.what());
                }
            }
        );
    }



    void BulletDetectNode::angle_callback(const auto_aim_interfaces::msg::ReceiveSerial msg)
    {
        // 将接收到的角度信息转换为四元数
        cur_q = Eigen::Quaterniond(
            Eigen::AngleAxisd(msg.roll * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(msg.pitch * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(msg.yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ())
        );

        // RCLCPP_INFO(get_logger(), "Received angles: roll = %f, pitch = %f, yaw = %f", msg.roll, msg.pitch, msg.yaw);
    }

    void BulletDetectNode::result_callback(const auto_aim_interfaces::msg::SendSerial msg)
    {
        auto now = this->now();
        tracking = msg.is_tracking;
        shooting = msg.is_can_hit;
        if (tracking)
        {
            tracking_time_ = now;
            aim_corrector.add_aim(aimer::aim::IdTLatencyAimCorrection {
                aim_id,
                tracking_time_,
                0.015, //写死0.015s 实际上是 图像采集时刻 到 开始做弹道预测 时刻
                aimer::AimInfo aim_info {
                    const aimer::math::YpdCoord& ypd = aimer::math::YpdCoord(msg.yaw, msg.pitch, msg.distance),
                    const aimer::math::YpdCoord& ypd_v = ,
                    const aimer::ShootParam& shoot_param,
                    const ::ShootMode& shoot
                }  //
                aim_correction // 上一次的校正的反馈
                }
            );
        }
        if (shooting)
        {
            shooting_time_ = now;
        }
    }




    /*
    图像处理的回调函数，处理接收到的图像信息，进行图像处理、预测并发布3D点位信息。
    参数:
    - msg: sensor_msgs::msg::Image类型，表示接收到的图像消息。
    */
    void BulletDetectNode::bullet_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
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

        cv::Mat bullet_image;
        try
        {
            // 使用cv_bridge将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            bullet_image = cv_ptr->image;
        }
        catch (cv_bridge::Exception & e)
        {
            // 如果图像转换失败，输出错误信息
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat result_image; // 声明用于存储处理后图像的变量

        // 如果没有相机信息，无法计算3D点位，输出错误信息
        if (cam_info_->k.empty()) {
            RCLCPP_ERROR(get_logger(), "没有相机信息，无法计算3D点位信息");
            return;
        }

        // std::vector<aimer::aim::ImageBullet> bullets = bullet_detector.process_new_frame(bullet_image, cur_q); 

        // //可视化检测结果
        // cv::Mat vis = bullet_detector.print_bullets();      
        // cv::imshow("Bullet Detect", vis);


























        // 将处理后的图像转换为 ROS 消息并发布
        if (it_ && result_image_pub_)
        {   
            // debug
            auto result_msg = cv_bridge::CvImage(msg->header, "rgb8", vis).toImageMsg();
            result_image_pub_.publish(result_msg); // 使用 image_transport 发布
        }
        else
        {
            RCLCPP_WARN(get_logger(), "ImageTransport not initialized yet. Skipping image publish.");
        }


        // 创建消息并填充预测的3D点位
        // geometry_msgs::msg::PointStamped point_msg;
        // point_msg.header.frame_id = "camera_optical_frame";
        // point_msg.header.stamp = msg->header.stamp;
        // point_msg.point.x = tvec.at<double>(0, 0)/100;
        // point_msg.point.y = tvec.at<double>(1, 0)/100;
        // point_msg.point.z = tvec.at<double>(2, 0)/100;



        geometry_msgs::msg::PointStamped transformed_msg;
        try {
            // transformed_msg.point = tf2_buffer_->transform(point_msg, "odom").point;
            // transformed_msg.header.frame_id = "odom";
            // transformed_msg.header.stamp = point_msg.header.stamp;
            // bullet_pose_pub_->publish(transformed_msg);
            // RCLCPP_INFO(get_logger(), "Published bullet prediction: x = %f, y = %f, z = %f", point_msg.point.x, point_msg.point.y, point_msg.point.z);
            // RCLCPP_INFO(get_logger(), "Published bullet prediction: x = %f, y = %f, z = %f", transformed_msg.point.x, transformed_msg.point.y, transformed_msg.point.z);
        } catch (tf2::TransformException& ex) {
            // RCLCPP_WARN(get_logger(), "无法将坐标从 camera_link 转换到 odom：%s", ex.what());
        }
    }
} // namespace qianli_rm_bullet_detect

#include "rclcpp_components/register_node_macro.hpp"

// 注册组件，确保该节点在库加载时可以被发现并使用"CPUExecutionProvider"
RCLCPP_COMPONENTS_REGISTER_NODE(qianli_rm_bullet_detect::BulletDetectNode)