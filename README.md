# RM2025 重庆大学“千里”战队英雄机器人视觉系统

本项目基于 `rm_vision` 框架，参考华中科技大学狼牙战队2023年英雄辅瞄开源文档，为 RoboMaster 2025 赛季重庆大学“千里”战队英雄机器人量身打造。
核心功能包括：
*   **目标识别**: 采用传统视觉方法进行灯条检测与匹配，并结合卷积神经网络 (CNN) 实现装甲板数字识别。在英雄自瞄代码中，只保留了前哨战和基地的识别，忽略其他装甲板模型。
*   **运动预测**: 当前预测策略中，扩展卡尔曼滤波器 (EKF) 的过程噪声协方差 (Q) 设置为一个极大值 (例如 `1e12`)，这实质上是优先信任测量值，削弱了模型的预测作用。测量噪声协方差 (R) 则根据实际情况赋予一个合理值，主要用于滤除传感器和环境带来的噪声。

## 前哨战延迟击打

本功能旨在实现对 RoboMaster 比赛中前哨战的精确延迟打击。其核心思想是，在稳定跟踪前哨战目标后，通过对前哨战的周期性特征进行估计，并结合弹丸飞行时间、系统延迟等因素，计算出最佳的开火提前量，从而准确命中目标。

### 工作流程

1.  **数据采集与平滑处理**:
    *   当 `is_outpost` 标志为真时，系统会持续收集目标的距离 (`distance`)、解算出的目标偏航角 (`send_yaw`)、俯仰角 (`send_pitch`)、弹丸飞行时间 (`fly_t`) 以及当前帧处理延迟 (`delta_time`，即当前时间与接收到目标消息时间戳的差值)。
    *   这些数据被存储在各自的列表中（如 `distance_list`, `yaw_list`, `pitch_list`, `fly_time_list`, `delta_time_list`）。
    *   当 `distance_list` 中收集到足够数量的数据点（代码中为20个）后，系统会进行一次数据处理：
        *   **异常值剔除**: 移除 `distance_list` 中值最大的N个（代码中N=10）数据点及其在其他列表中对应的参数，以减少噪声干扰。
        *   **均值计算**: 计算剔除异常值后各列表内剩余数据点的平均值，得到临时的 `temp_distance`, `temp_yaw`, `temp_pitch`, `temp_fly_time`, `temp_delta_time`。
        *   **滑动平均更新**: 使用加权滑动平均（例如，新均值权重0.2，历史平均值权重0.8）更新全局的前哨战目标参数：`outpost_distance`, `outpost_yaw`, `outpost_pitch`, 以及平均飞行时间 `fly_time` (此处变量名与成员变量 `fly_t` 有所区别，代码中实际更新的是成员变量 `this->fly_time`) 和平均延迟 `delta_time_average`。
        *   处理完毕后，清空数据收集列表，为下一轮数据采集做准备。
    *   在数据采集和处理阶段，`result.is_can_hit` 通常设为 `false`，表示尚不能开火，但 `result.is_tracking` 设为 `true`。

2.  **稳定锁定与计时启动 (`start_flag`)**:
    *   系统会持续比较当前计算出的目标偏航角 (`send_yaw`)、距离 (`distance`) 与平滑后的前哨战参数 (`outpost_yaw`, `outpost_distance`)。
    *   当目标当前参数与平滑参数的差异足够小（代码中条件为：`abs(send_yaw - outpost_yaw) < 0.2f && abs(distance - outpost_distance) < 0.2f`）且 `start_flag` 为 `false` 时，认为目标装甲板进入阈值范围。
    *   此时，记录当前时间为 `outpost_start_time_`，并将 `start_flag` 置为 `true`，开始为延迟射击计时。

3.  **周期时间估计 (`outpost_time_3`)**:
    *   系统尝试估计前哨战的某种周期性行为（对应于其装甲板旋转周期）。
    *   当目标再次满足一定的接近条件（代码中条件为：`abs(send_yaw - outpost_yaw) < 0.3f && abs(distance - outpost_distance) < 0.1f`）并且 `fit_flag` 为 `false` 时，会将当前时间存入 `time_list`，并将 `fit_flag` 置为 `true`（`fit_flag` 的重置逻辑在代码中未明确显示，可能在条件不满足时重置或依赖其他逻辑）。
    *   当 `fit_flag` 为 `true` 且 `time_list` 中至少有两个时间点时，计算最新两个时间点之间的时间差 `time_diff`。
    *   如果 `time_diff` 在特定范围（代码中为 0.5秒到1.0秒之间），则认为这是一个有效的周期信号，并将其存入 `diff_list`。
    *   当 `diff_list` 中收集到足够数量的有效时间差（代码中为5个）后，计算这些时间差的平均值 (`average`)。此 `average` 值在代码中通过 `std::cout` 输出，注释显示它可能曾用于更新 `outpost_time_3` (如 `// outpost_time_3 = (average + outpost_time_3)/2;`)。`outpost_time_3` 是一个关键的周期参数。

4.  **延迟开火决策**:
    *   当 `start_flag` 为 `true`（即已稳定锁定并开始计时）后，系统会持续检查开火条件。
    *   开火条件为：`(this->now() - outpost_start_time_).seconds() >= 2 * outpost_time_3 - delta_time_average - fly_time - delay_time`。
        *   `outpost_start_time_`: 稳定锁定的起始时间。
        *   `outpost_time_3`: 三分之一前哨战周期。`2 * outpost_time_3` 预测下下块装甲板。
        *   `delta_time_average`: 平滑后的平均消息处理延迟。
        *   `fly_time`: 平滑后的弹丸飞行时间。
        *   `delay_time`: 一个可配置的额外固定延迟参数。
    *   当上述条件满足时，意味着预测的打击窗口已经到达，系统会将 `result.is_can_hit` 置为 `true`，允许开火，并将 `start_flag` 重置为 `false`。

5.  **参数重置**:
    *   如果 `is_outpost` 标志变为 `false`（通过操作手键盘切换模式，用于自身位置改变时刷新参数），所有与前哨战相关的状态变量（如 `outpost_distance`, `outpost_yaw`, `outpost_pitch`, `fly_time`, `delta_time_average`, `count`）、标志位（`start_flag`, `fit_flag`）以及数据列表（`time_list`, `diff_list`, `delta_time_list`, `distance_list`, `yaw_list`, `pitch_list`, `fly_time_list`）都会被清空或重置为初始状态。

### 输出控制
当 `is_outpost` 模式激活时，最终发送给执行机构（如云台串口驱动）的 `result` 消息中的 `pitch`, `yaw`, `distance` 将使用平滑后的前哨战参数 `outpost_pitch`, `outpost_yaw`, `outpost_distance`。如果 `is_outpost` 是off模式且摄像头中看到目标装甲板，yaw和pitch将会返回0，此时
开启跟随将会直接转向陀螺仪yaw和pitch的0位。


### 注意事项
*   `outpost_time_3` 目前是根据官方规则参数写的定值，但赛场上也可能存在5%的误差，目前拟合前哨站转速代码仍有问题，待进一步改进。
*   `delay_time` 是机械发弹延迟（包含电控通信延迟，也可以当作忽略不计）。


## 英雄吊射辅瞄（rm_hero）

本模块旨在根据雷达站（或其他定位系统）提供的本机英雄机器人坐标以及预设的敌方基地坐标，实现对敌方基地的快速吊射定位辅助。其核心功能是计算出目标点（敌方基地）相对于英雄机器人当前位置的3D坐标，并发布给弹道解算模块。

### 工作流程

1.  **订阅英雄位置**:
    *   节点 `rm_hero_node` 订阅话题 `/angle/init` (消息类型 `auto_aim_interfaces::msg::ReceiveSerial`)。此消息中包含了英雄机器人当前的X、Y坐标 (`msg.hero_pose_x`, `msg.hero_pose_y`)，以及消息头中的时间戳。

2.  **目标Z轴高度判断**:
    *   根据英雄机器人当前的X、Y坐标以及目标队伍颜色 (`detect_color`，1为蓝色方，其他为红色方)，结合预设的地图区域（如 `blue_x1, blue_x2, blue_y1, blue_y2`），判断英雄机器人所处地形（例如平地、坡道高点/低点）。
    *   基于此判断，为目标（敌方基地）设定一个基础的Z轴高度（例如，平地区域对应200mm，坡道高点区域对应600mm）。
    *   在此基础上，加上英雄机器人自身的云台高度 `hero_gimbal_height_`，得到一个初步的、相对于地面的目标Z坐标。

3.  **目标相对坐标计算**:
    *   系统预设了双方基地的固定坐标（`blue_base_point_`, `red_base_point_`）以及基地本身的高度 `base_height_`。
    *   计算目标基地相对于英雄机器人当前位置的X和Y方向的偏移：
        *   `target_x_in_odom = enemy_base_x_odom - hero_x_odom`
        *   `target_y_in_odom = enemy_base_y_odom - hero_y_odom`
    *   计算目标基地相对于英雄机器人调整后Z坐标的相对高度：
        *   `target_z_final = base_height_ - (calculated_target_ground_z + hero_gimbal_height_)`
        *   这里的 `calculated_target_ground_z` 是步骤2中根据英雄位置判断的200mm或600mm。这个 `target_z_final` 实质上是基地顶面相对于英雄云台水平面的高度差。

4.  **发布预测点**:
    *   将计算得到的相对X, Y坐标和最终的Z坐标（高度差）填充到 `geometry_msgs::msg::PointStamped` 消息中。
    *   该消息的 `header.frame_id` 设置为 `"odom"`，时间戳使用来自输入消息 `msg.header.stamp`。
    *   通过话题 `/hero/prediction` 发布此目标点。弹道解算节点将订阅此话题，获取吊射目标点信息。

### 注意事项
*   英雄机器人的坐标 (`hero_pose_x`, `hero_pose_y`) 来源及其精度对吊射的准确性至关重要。
*   地图区域划分 (`blue_x1` 等) 和对应的Z轴高度值 (`200.0`, `600.0`) 需要根据实际比赛场地进行精确标定。
*   `hero_gimbal_height_` 和 `base_height_` 参数也需要精确测量。
*   当前实现中，发布的预测点坐标 (`point_msg.point.x`, `point_msg.point.y`) 是基地相对于英雄在odom坐标系下的X, Y差值，而 `point_msg.point.z` 是基地顶面与英雄云台的高度差。弹道解算模块需要正确理解这些坐标的含义。
