# 2025重庆大学千里战队 - 装甲板自瞄闭环项目

## 1. 项目概述

本项目旨在通过视觉手段实现对机器人射击弹丸的检测与跟踪，并结合弹道学模型，对原有的开环自瞄系统进行闭环校准，从而提升对敌方装甲板的打击精度。核心思路是：通过摄像头捕捉实际发射的弹丸，将其位置与系统根据当前瞄准状态和弹道模型预测的弹丸位置进行对比，利用此差异信息（通过卡尔曼滤波器更新）不断优化瞄准参数，实现射击的闭环控制。

## 2. 当前进展

### 2.1 已完成

*   **小弹丸视觉识别**: 能够通过图像处理算法在视频流中检测出发射的弹丸。具体算法见 `detect_bullet.cpp`。
*   **模拟小弹丸生成**: 系统能够根据当前的瞄准信息和弹道模型预测弹丸的理论飞行轨迹和在特定时间点的位置。具体实现见 `aim_corrector.cpp` 中的 `ProjectileSimulator`。
*   **模拟弹丸与识别弹丸的初步匹配**: 实现了将视觉检测到的实际弹丸与同一时刻由模型生成的模拟弹丸进行关联匹配的初步逻辑，用于计算瞄准误差。具体实现见 `aim_corrector.cpp` 中的 `sample_aim_errors` 方法。

### 2.2 未完成与待办事项

*   **精确时间戳对齐**:
    *   视觉检测到的弹丸 (`rm_bullet_detect_node.cpp` 中 `bullet_image_callback` 的 `msg->header.stamp`)。
    *   电控系统状态，如枪口姿态 (`rm_bullet_detect_node.cpp` 中 `angle_callback` 接收的 `msg.roll, msg.pitch, msg.yaw`，需要关联精确时间戳)。
    *   弹道模型的计算 (`aim_corrector.cpp` 中 `ProjectileSimulator` 使用的时间 `t`)。
    *   各环节之间的时间戳需要精确对齐，这是实现有效闭环控制的基础。
*   **获取精确发弹时间**:
    *   目前 `rm_bullet_detect_node.cpp` 中的 `result_callback` 接收 `msg.is_can_hit` (在代码中为 `shooting`) 作为发弹信号，但缺少从电控系统准确获取“弹丸真实从枪口射出时刻”的机制。
    *   需要得到从上位机发送发弹指令到弹丸实际发射之间精确的延迟数据，以便 `AimCorrector::update_bullet_id` 能准确创建 `ProjectileSimulator`。
*   **卡尔曼滤波更新**:
    *   基于 `AimCorrector::sample_aim_errors` 计算出的瞄准误差 (`error_y`, `error_p`)，设计并实现卡尔曼滤波器的测量更新步骤。
    *   用以修正和优化瞄准模型或状态估计，最终目标是调整 `AimCorrector::get_aim_error()` 的输出，用于补偿后续的瞄准指令。

## 3. 主要模块说明

本项目主要围绕 `rm_bullet_detect` 包展开，其核心组件包括：

### 3.1 `BulletDetectNode` (`src/rm_bullet_detect/src/rm_bullet_detect_node.cpp`)

*   **角色**: ROS 节点，作为整个弹丸检测与自瞄闭环系统的总控制和数据流中心。
*   **订阅**:
    *   `/camera_info` (`sensor_msgs::msg::CameraInfo`): 获取相机内参 `camera_matrix_`，用于后续的3D坐标计算和重投影。
    *   `/image_raw` (`sensor_msgs::msg::Image`): 接收原始图像数据，在 `bullet_image_callback` 中进行处理。
    *   `/angle/init` (`auto_aim_interfaces::msg::ReceiveSerial`): 从电控获取当前的云台姿态（roll, pitch, yaw）和弹丸初速 `bullet_v0`，用于更新 `cur_q` (当前姿态四元数) 和弹道模型参数。
    *   `/trajectory/result` (`auto_aim_interfaces::msg::SendSerial`): 获取轨迹规划模块的输出，判断是否处于追踪 `tracking` 和可射击 `shooting` 状态。
    *   `/tracker/target` (`auto_aim_interfaces::msg::Target`): (当前代码中 `target_callback` 为空，可能为后续扩展或调试预留)。
    *   `/detector/armors` (`auto_aim_interfaces::msg::Armors`): 在 `armor_callback` 中获取目标装甲板在相机坐标系下的位置 `cam_traget_x, y, z`，可能用于辅助判断或目标信息。
*   **发布**:
    *   `/bullet/prediction` (`geometry_msgs::msg::PointStamped`): 发布预测的弹丸3D点位信息（当前代码中相关发布逻辑被注释）。
    *   `/bullet/result_image` (`sensor_msgs::msg::Image`): (通过 `image_transport` 发布) 可视化弹丸检测结果的图像，用于调试。
*   **核心逻辑**:
    *   初始化 `camera_matrix_` 和 `imu` (gimbal_link 到 camera_optical_frame 的旋转矩阵)。
    *   初始化 `bullet_detector` (类型为 `aimer::aim::DetectBullet`)，并传入 `aimer::aim::DoReproj` 对象（使用相机内参和云台-相机光心变换）。
    *   `bullet_image_callback`:
        *   将ROS图像消息转换为 `cv::Mat`。
        *   如果处于 `shooting` 状态，调用 `aim_corrector_->sample_aim_errors(cur_q, bullet_image)` 来处理新帧，进行弹丸检测和误差采样。
        *   （被注释部分）调用 `bullet_detector.process_new_frame` 进行弹丸检测。
        *   （被注释部分）发布弹丸的3D预测点。
    *   使用 `tf2_ros::Buffer` 和 `tf2_ros::TransformListener` 进行坐标变换。

### 3.2 弹丸检测模块 (`src/rm_bullet_detect/src/detect_bullet.cpp` - `aimer::aim::DetectBullet`)

*   **角色**: 实现核心的弹丸视觉识别算法。
*   **关键常量**:
    *   `DIFF_STEP`: 帧差法采样步长，用于加速。
    *   `DIFF_THRESHOLD`: 帧差法中判定为显著变化的阈值。
    *   `KERNEL1_SIZE`, `KERNEL2_SIZE`: 形态学操作（膨胀、开运算）的核大小。
    *   `COLOR_LOWB`, `COLOR_UPB`: HSV颜色空间中用于粗略筛选弹丸颜色的上下阈值。
    *   `MIN_VUE`: HSV颜色空间中V通道的最小阈值，用于过滤过暗区域。
*   **核心方法**:
    *   `init(const DoReproj& do_reproj)`: 保存传入的 `DoReproj` 对象，用于后续的图像重投影。
    *   `process_new_frame(const cv::Mat& new_frame, const Eigen::Quaterniond& q)`:
        1.  更新当前帧图像 (`cur_frame`) 和对应的IMU姿态四元数 (`cur_fr_q`)。
        2.  将当前帧从BGR转换为HSV色彩空间 (`cur_hsv`)。
        3.  如果存在上一帧 (`!lst_frame.empty()`)，则调用 `get_possible()` 和 `get_bullets()`。
        4.  返回检测到的弹丸列表 (`std::vector<ImageBullet>`)。
    *   `get_possible()`: 筛选可能的弹丸区域。
        1.  使用 `do_reproj.reproj()` 将上一帧的HSV图像 (`lst_hsv`) 根据IMU姿态变化重投影到当前帧视角，得到 `lst_reproj`。
        2.  对当前HSV图像 (`cur_hsv`) 进行颜色范围过滤 (`cv::inRange` 使用 `COLOR_LOWB`, `COLOR_UPB`) 和亮度过滤 (`MIN_VUE`)，得到初步掩码 `res`。
        3.  调用 `do_diff.get_diff()` 在 `cur_hsv` 和 `lst_reproj` 之间计算帧差，并与 `res` 和上一帧的弹丸掩码 `lst_msk` 结合，进一步精确运动物体。
        4.  对结果进行开运算 (`cv::morphologyEx` 使用 `MORPH_OPEN` 和 `kernel2`) 去除噪声。
        5.  寻找轮廓 (`cv::findContours`)，结果存入 `this->contours`。
    *   `get_bullets()`: 从 `contours` 中提取最终的弹丸信息。
        1.  遍历轮廓，对于每个轮廓：
            *   计算最小外接矩形 (`cv::minAreaRect`)。
            *   过滤面积过小 (`rect_size.area() < 30`) 或填充比过低 (`cv::contourArea(contour) / rect_size.area() < 0.5`) 的轮廓。
            *   调用 `test_is_bullet(contour)` 进一步验证轮廓是否为真实弹丸。
            *   如果验证通过，则将弹丸中心和半径存入 `this->bullets`。
        2.  同时生成下一帧可能用到的弹丸掩码 `lst_msk`。
    *   `test_is_bullet(std::vector<cv::Point> contour)`:
        1.  对轮廓点进行排序 (`sort_points`)。
        2.  在轮廓内部按列扫描，寻找最亮且符合 `test_is_bullet_color` 的像素点。
    *   `test_is_bullet_color(const cv::Vec3b& hsv_col)`: 判断单个HSV像素颜色是否符合弹丸特征（亮度足够，色相在特定范围，且阈值随饱和度和亮度动态调整）。
    *   `DoFrameDifference::get_diff()`:
        1.  以 `DIFF_STEP` 步长遍历图像。
        2.  比较 `s1` (当前帧HSV子区域) 和 `s2` (重投影的上一帧HSV子区域) 的像素，若差异大于 `DIFF_THRESHOLD` 且满足其他条件，则标记为运动。
        3.  对结果进行膨胀操作 (`cv::dilate` 使用 `kernel1`)。

### 3.3 图像重投影模块 (`src/rm_bullet_detect/src/do_reproj.cpp` - `aimer::aim::DoReproj`)

*   **角色**: 处理相机自身的运动，使得基于帧间比较的算法（如帧差法）能够更准确地工作。
*   **核心方法**:
    *   `init(const cv::Mat& cam, const cv::Mat& imu)`:
        *   `cam`: 相机内参矩阵 (3x3)，被转换为3x4再扩展为4x4的齐次相机投影矩阵 `this->cam` (Eigen::Matrix4d)。
        *   `imu`: IMU到相机的旋转矩阵 (3x3) `this->imu` (Eigen::Matrix3d)。
    *   `from_q_get_trans_mat(const Quat& q)`: 根据输入的IMU姿态四元数 `q` (表示世界坐标系到当前机体坐标系的旋转)，计算出从世界坐标系到相机坐标系的4x4变换矩阵。具体为 `this->imu * q.matrix().inverse()` 作为旋转部分。
    *   `get_fr_trans_mat(const Quat& q1, const Quat& q2)`: 计算两帧姿态（`q1`上一帧，`q2`当前帧）之间的3x3透视变换矩阵。公式为 `(this->cam * T_world_to_cam(q2) * (this->cam * T_world_to_cam(q1))^{-1})` 的左上3x3部分，其中 `T_world_to_cam(q)` 是通过 `from_q_get_trans_mat(q)` 得到的。这个矩阵可以将上一帧图像中的点变换到当前帧图像中对应的位置。
    *   `reproj(const cv::Mat& src, const Quat& q1, const Quat& q2)`:
        1.  调用 `get_fr_trans_mat(q1, q2)` 计算变换矩阵。
        2.  使用 `cv::warpPerspective` 将源图像 `src` 根据计算得到的变换矩阵进行重投影，输出到与原图等大的图像。

### 3.4 瞄准校正模块 (`src/rm_bullet_detect/src/aim_corrector.cpp` - `aimer::aim::AimCorrector`)

*   **角色**: 实现自瞄闭环的核心校正逻辑，通过比较模拟弹丸和实际检测到的弹丸来计算瞄准误差。
*   **子类与结构**:
    *   `AimHistory`:
        *   存储历史瞄准记录 (`aim::IdTLatencyAimCorrection`，包含发射ID、图像时间戳、延迟、瞄准参数 `aim::AimCorrection`)。
        *   `add_aim()`: 添加新的瞄准记录，保持队列大小不超过 `AIM_CORRECTOR_AIM_HISTORY_MAX_SZ`。
        *   `find_by_id()`: 根据ID查找瞄准记录。
    *   `ProjectileSimulator`:
        *   `aim`: 存储该模拟器对应的 `IdTLatencyAimCorrection` 记录。
        *   `fire_t`: 记录该弹丸的理论发射时间。
        *   `converter`: 指向 `CoordConverter` 对象的指针，用于坐标转换和参数获取。
        *   `get_pos_by_t(const double& t)`:
            1.  获取发射参数 `shoot_param` (包含初速 `v0`、发射角 `aim_angle`、目标在相机下的位置 `target_xyz_i_camera`)。
            2.  获取空气阻力系数 `k` (通过 `get_param_k()` 读取配置文件中的 `launching-mechanism.bullet.resistance-k`)。
            3.  计算在时间 `t` 时，弹丸相对于枪口在水平方向的位移 `w` 和垂直方向的位移 `h`（考虑重力和空气阻力）。
            4.  将目标点从相机坐标系转换到枪管坐标系 `target_xyz_i_barrel`。
            5.  根据 `w` 和 `h` 以及目标点的方向向量，计算弹丸在枪管坐标系下的3D位置 `bullet_xyz_i_barrel`。
            6.  将弹丸位置转换回相机坐标系 `bullet_xyz_i_camera`。
            7.  判断是否击中目标（通过比较弹丸和目标在枪管坐标系下XY平面的距离）。
            8.  返回 `aim::HitPos` 包含是否击中和相机坐标系下的位置。
        *   `get_circle_by_t(const double& t)`:
            1.  调用 `get_pos_by_t(t)` 获取弹丸在相机坐标系下的3D位置。
            2.  将3D位置投影到图像平面，并考虑弹丸的物理半径 (`base::get_param<double>("launching-mechanism.bullet.radius")`)，计算出其在图像上的2D圆形包围框 `aimer::math::CircleF`。
            3.  返回 `aim::HitCircle` 包含是否击中和图像上的圆形包围框。
*   **`AimCorrector` 主类**:
    *   构造函数: 初始化 `bullet_detector` (传入 `DoReproj` 对象，该对象使用 `converter` 提供的相机内参和IMU-相机旋转矩阵)。
    *   `add_aim()`: 将新的瞄准信息添加到 `aim_history`。
    *   `update_bullet_id(const int& last_shoot_id)`:
        1.  当接收到新的已发射子弹ID `last_shoot_id` 时，如果该ID尚未处理：
        2.  从 `aim_history` 中查找该ID对应的瞄准记录。
        3.  计算理论发射时间 `fire_t` (瞄准记录中的图像时间 + 瞄准记录中的延迟)。
        4.  创建一个新的 `ProjectileSimulator` 对象，并存入 `bullets_` (队列，最大 `AIM_CORRECTOR_BULLETS_MAX_SZ`)。
    *   `sample_aim_errors(Eigen::Quaterniond cur_q, cv::Mat bullet_image)`:
        1.  调用 `bullet_detector.process_new_frame(bullet_image, cur_q)` 获取当前帧检测到的所有弹丸 (`detected_bullets`，类型为 `std::vector<aim::ImageBullet>`)。
        2.  遍历当前所有正在模拟的弹丸 (`this->bullets_`，即 `std::vector<ProjectileSimulator>`)：
            *   对每个模拟器，调用 `get_circle()` 获取其在当前图像上预测的2D圆形包围框 `simulated_circle`。
            *   遍历所有 `detected_bullets`，寻找与 `simulated_circle` 最佳匹配的实际检测弹丸（基于距离和半径比例，使用 `CATCH_CIRCLE_DIS_MAX_RATIO` 等常量）。
            *   如果找到匹配：
                *   计算模拟圆心与检测圆心之间的偏差 (`error_u`, `error_v`)。
                *   将此误差样本 (`error_y`, `error_p`，可能经过了从像素误差到角度误差的转换) 添加到 `error_angles_` 队列 (最大 `AIM_CORRECTOR_ERROR_ANGLES_MAX_SZ`)。
                *   （可选）可以从 `bullets_` 中移除已匹配的模拟器。
    *   `get_aim_error() const`: 计算并返回平均的瞄准误差（例如，从 `error_angles_` 中计算均值），用于反馈给自瞄系统进行补偿。
    *   `undistorted_circle()`: 对输入的圆进行去畸变处理（当前实现为空）。

### 3.5 坐标转换与弹道解算模块 (`src/rm_bullet_detect/src/coord_converter.cpp` - `aimer::CoordConverter`)

*   **角色**: 提供统一的坐标系转换接口、时间管理以及弹道计算所需参数。
*   **关键参数与方法**:
    *   从配置文件 (`param.yml`) 读取相机内参、畸变系数、IMU与相机的相对位姿、枪管与相机的相对位姿等。
    *   `pc_to_pi()`: 相机坐标系 (camera frame) 点云到IMU坐标系 (IMU frame, 以相机光心为原点) 的转换。
    *   `pi_to_pc()`: IMU坐标系到相机坐标系的转换。
    *   `pc_to_pu()`: 相机坐标系3D点到图像2D像素点 (undistorted) 的转换（不执行畸变校正）。
    *   `pu_to_pc_norm()`: 图像2D像素点到相机坐标系下归一化3D向量的转换。
    *   `xyz_i_camera_to_xyz_i_barrel()`: 将以相机光心为原点的IMU坐标系下的点，转换到以枪口中心为原点的IMU坐标系下。
    *   `xyz_i_barrel_to_xyz_i_camera()`: 反向转换。
    *   `target_pos_to_shoot_param(const Eigen::Vector3d& target_pos)`:
        *   输入目标在相机坐标系下的3D位置 `target_pos`。
        *   结合当前的弹丸初速 (`get_bullet_speed()`) 和重力，解算弹道，计算出需要的发射参数 `aimer::ShootParam` (包含实际的 `v0`，抬枪角度 `aim_angle`，以及目标在相机坐标系下的位置 `target_xyz_i_camera`)。这是 `ProjectileSimulator` 所需的核心输入。
    *   `get_f_cv_mat_ref()`: 返回相机内参矩阵 (cv::Mat)。
    *   `get_rot_ic_sup_cv_mat_ref()`: 返回IMU到相机（或其他参考系）的旋转矩阵 (cv::Mat)。这些被 `AimCorrector` 用于初始化 `DoReproj`。
    *   `update(const cv::Mat& img, const Eigen::Quaternionf& q, const double& timestamp)`: 更新内部状态，如当前图像、IMU四元数和时间戳。

### 3.6 数学工具库 (`src/rm_bullet_detect/src/math.cpp` - `aimer::math`)

*   **角色**: 提供项目中通用的数学运算函数。
*   **主要功能**:
    *   `get_ypd_v()`: 根据XYZ坐标及其速度，计算目标在世界坐标系下的YPD（偏航、俯仰、距离）及其变化率。
    *   `camera_get_ypd_v()`: 根据XYZ坐标及其速度，计算目标在相机坐标系下的YPD及其变化率。
    *   `distort_points()`: 对2D点集进行畸变校正（或添加畸变）。
    *   包含其他几何计算，如点距 `get_dis()`，比例 `get_ratio()` 等。

## 4. 参考资料
*   **参考**: 上海交通大学2023 RoboMaster “青工会”技术分享 - [rm.cv.fans on GitHub](https://github.com/julyfun/rm.cv.fans)

## 5. 未来工作展望

在完成上述“未完成事项”后，未来的工作可以包括：
*   **完整的卡尔曼滤波器设计与集成**:
    *   定义状态向量（例如，瞄准偏差的yaw和pitch）。
    *   建立状态转移模型和观测模型。
    *   实现预测步骤和基于 `AimCorrector` 输出误差的更新步骤。
*   **将闭环校准系统与更高级的决策或目标选择策略集成**。
*   **进行大量的实弹测试与参数调优**，验证闭环系统的性能提升。