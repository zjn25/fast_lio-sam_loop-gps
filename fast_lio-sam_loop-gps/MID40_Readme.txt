Livox Mid-40 使用说明
包名: fast_lio_sam_loop
配置: config/mid40.yaml（无 GPS）、config/mid40_gps.yaml（带 GPS）
RViz: 默认不启动。需要时 roslaunch ... rviz:=true，配置 rviz_cfg/loam_livox.rviz

Mid-40 是单线非重复扫描，没有内置 IMU。点云必须是 livox_ros_driver/CustomMsg
（livox_lidar_msg.launch）。PointCloud2 没有逐点时间戳，不能用于去畸变。
lidar_type 固定为 1，scan_line 固定为 1（驱动里 line 恒为 0）。


1. 编译

    cd /home/nan/catkin_18/fast_lio_sam_gps
    source /opt/ros/noetic/setup.bash
    catkin_make --pkg fast_lio_sam_loop -j2
    source devel/setup.bash

保存地图服务的参数改过之后，必须重新编译并重启建图节点，旧进程不认识新服务。


2. 启动

无 GPS:

    roslaunch fast_lio_sam_loop mapping_mid40.launch

带 GPS:

    roslaunch fast_lio_sam_loop mapping_mid40_gps.launch

打开 RViz（NX 上不要开，会订阅全局地图并每秒重拼）:

    roslaunch fast_lio_sam_loop mapping_mid40.launch rviz:=true

雷达驱动另开一个终端，发布 CustomMsg 到 lid_topic（默认 /livox/lidar）。
外部 IMU 发到 imu_topic。当前两个配置里的话题并不相同，以各自 yaml 为准:

    mid40.yaml      imu_topic: /imu
    mid40_gps.yaml  imu_topic: /imu/data
                    gps_topic: /fix

外参 extrinsic_T / extrinsic_R 是雷达在 IMU 机体坐标系下的位姿，默认是单位阵，
必须改成实际安装关系。extrinsic_est_en 为 false 时不会在线估计外参。


3. 保存地图

建图节点运行中，且已经出现 "Create new keyframe by LIO" 之后:

只改这一次的体素大小（米）。0 表示用 yaml 里的 global_map_server_leaf_size:

    rosservice call /service/save_map "resolution: 0.2
    destination: ''"

同时指定输出文件:

    rosservice call /service/save_map "resolution: 0.2
    destination: '/home/nan/maps/mid40.pcd'"

destination 若是目录而不是以 .pcd 结尾的文件，则在该目录下写
globalmap_lidar_feature.pcd。目录必须已存在，路径建议以 / 结尾。
destination 为空时，使用 yaml 的 service/save_directory 再接上文件名。
当前 mid40.yaml 的结果文件是:

    /home/nan/catkin_18/fast_lio_sam_gps/src/fast_lio-sam_loop-gps/fast_lio-sam_loop-gps/map/globalmap_lidar_feature.pcd

成功时服务返回 success: True，终端打印 leaf 和最终路径。
还没有关键帧时返回失败，并打印 NO ENCOUGH POSE!。

这条服务保存的是关键帧点云（回环关闭时用前端 LIO 位姿），体素由 resolution 决定。
city 分支默认关掉 pcd_save_en，建图过程中不把每帧点云堆在内存里。
需要整图时调用 /service/save_map。


4. GPS

mapping_mid40_gps.launch 会多启动 fast_lio_sam_loop_gpsOdometry。
输入是 common/gps_topic 上的 sensor_msgs/NavSatFix（默认 /fix）。
该节点转成 /gps_odom 后给建图节点，不要自己改这个话题名。

用到的字段:

    header.stamp              必须和雷达帧尾时刻接近
    latitude, longitude       度，WGS84
    altitude                  米
    position_covariance[0]    东向方差，m^2
    position_covariance[4]    北向方差，m^2
    position_covariance[8]    天向方差，m^2

不检查 RTK status。方差为 0 会被当成完美测量。水平方差大于
gps_cov_threshold 的点不进位姿图。

时间对齐窗口是 ±1/gps_frequence 秒。默认 gps_frequence 为 10，窗口只有 ±0.1 秒。
1 Hz 的 GPS 要把 gps_frequence 改成 1。

车先移动超过 0.1 m 后才会发出 /gps_odom。航向是这段位移在 ENU 下的运动方向，
并用来初始化 IMU 姿态，默认认为车体 x 轴沿当时的行驶方向。
若已知航向，设 manual_gps_init: true，manual_init_yaw 为弧度。

GPS 因子只加在关键帧上，且与上一个已采用的 GPS 点至少相距 gps_distance。
攒满 gpc_factor_init_num 个之后才写入图。
use_gps_elevation 目前没有生效，高度总会被使用。

不开 GPS 时用 mapping_mid40.launch。此时不需要 /fix，也不会等待 GPS 航向。


5. 可改参数

5.1 launch 里的参数（覆盖 yaml，改完重新 launch）

    rviz                    是否启动 RViz，默认 false。NX 上保持关闭
    feature_extract_enable  0。Mid-40 用直接法，不要打开特征提取
    point_filter_num        2。每 N 个点取 1 个。Mid-40 点较稀，不要设得太大
    max_iteration           3。单帧迭代次数
    filter_size_surf        0.3 m。前端匹配用的体素
    filter_size_map         0.3 m。地图体素
    cube_side_length        1000 m。局部地图立方体边长
    runtime_pos_log_enable  0。是否写运行日志

5.2 common

    lid_topic                    雷达话题，类型 livox_ros_driver/CustomMsg
    imu_topic                    外部 IMU 话题，sensor_msgs/Imu
    gps_topic                    仅 GPS 配置。NavSatFix，默认 /fix
    time_sync_en                 软件对齐雷达和 IMU 时间。只有没有硬件同步时才开
    time_offset_lidar_to_imu     雷达相对 IMU 的时间偏移，秒。未知时保持 0

5.3 frame

    odometry_frame   里程计坐标系，默认 odom。/cloud_registered、/odom_path 在这个系
    map_frame        地图坐标系，默认 map。/map_path、/visualization/map_global 在这个系

5.4 preprocess

    lidar_type   1。Livox CustomMsg，不要改
    scan_line    1。Mid-40 只有 line 0，不要改成 Avia 的 6
    blind        0.5 m。小于该距离的点丢弃

5.5 mapping

    acc_cov, gyr_cov         IMU 测量噪声
    b_acc_cov, b_gyr_cov     IMU 零偏随机游走
    fov_degree               38.4。Mid-40 圆形视场，度
    det_range                100 m。局部地图维护距离
    extrinsic_est_en         是否在线估计雷达-IMU 外参
    extrinsic_T              雷达在 IMU 系下的平移，米，[x, y, z]
    extrinsic_R              雷达在 IMU 系下的旋转，行优先 3x3

5.6 publish

    path_odom_en              发布 /odom_path
    path_map_en               发布 /map_path（回环校正后的轨迹）
    scan_publish_en           发布点云
    dense_publish_en          true 时 /cloud_registered 用未降采样点，显示更密
    scan_bodyframe_pub_en     发布 IMU 体系点云 /cloud_registered_body
    scan_lidarframe_pub_en    发布雷达体系点云，关键帧需要它
    keyframe_pub_en           必须为 true，否则没有关键帧，服务保存会失败

5.7 pcd_save / time_save

    pcd_save_en          city 分支默认 false。true 且 interval 为 -1 时每帧点云堆在内存里
    interval             -1 表示全程一个文件；大于 0 表示每 N 帧写盘后清空
    timestamp_save_en    是否另存雷达时间戳

5.8 keyframe

    interval_th    关键帧最小时间间隔，秒
    trans_th       平移超过该值（米）则新建关键帧
    rot_rad_th     转动超过该值（弧度）则新建关键帧

5.9 loop

    loop_en                         回环开关。关掉时仍缓存 LIO 关键帧，/service/save_map 可存图
    loop_frequency                  回环检测频率，Hz
    sparse_raw_point_cloud_en       关键帧点云是否再抽稀，用于控制内存
    history_kf_search_radius        回环搜索半径，米
    history_kf_search_time_diff     只和更早这么多秒的关键帧做回环
    history_kf_search_num           拼成局部地图的历史关键帧数量
    history_kf_fitness_score        ICP 得分阈值，越小越严
    surrounding_kf_search_radius    重建局部地图时的搜索半径，米
    surrounding_kf_density          重建时关键帧抽稀间距，米
    mapping_surf_leaf_size          回环 ICP 和局部地图的体素，米
    correct_fe_en                   true 时回环后重建 ikd-tree。当前为 false

5.10 gps（仅 mid40_gps.yaml，gps_en: true）

    gps_en               是否使用 GPS 因子
    gps_frequence        GPS 频率，Hz。同时决定和时间戳对齐的窗口 1/频率
    gpc_factor_init_num  积累多少个 GPS 因子后才写入图
    gps_cov_threshold    水平方差上限，m^2。超过则丢弃
    gps_distance         与上一个 GPS 因子的最小间距，米
    manual_gps_init      true 时用 manual_init_yaw，不用运动方向估计航向
    manual_init_yaw      手动航向，弧度

5.11 visualization

    global_map_visualization_search_radius   显示全局图时的搜索半径，米。0 表示不发布 /visualization/map_global
    global_map_visualization_pose_density    显示用的关键帧间距，米。越小越密
    global_map_visualization_leaf_size       显示用体素，米。只影响 /visualization/map_global，不影响保存文件

全局图大约每秒发布一次，话题 /visualization/map_global。半径为 0 或没有订阅者时不拼图。
city 分支默认半径 0，避免 NX 上有订阅者时按 1000 m 每秒重拼整图。

5.12 camera

    img_en            false 时不订阅相机。true 时读取 img_topic 给点云赋色
    img_topic         默认 /camera/color/image_raw，sensor_msgs/Image
    img_time_offset   加到图像时间戳上再和雷达对齐，秒
    img_time_th       图像和雷达时间差超过该值则这一帧不赋色，秒
    fx fy cx cy       针孔内参，像素。必须和 image_raw 的分辨率一致
    k1 k2 p1 p2       径向/切向畸变。已去畸变的图像保持 0
    Rcl               雷达到相机的旋转，行优先 3x3
    Pcl               雷达到相机的平移。p_camera = Rcl * p_lidar + Pcl

投到图像外的点保留为灰色。赋色后 /cloud_registered、/visualization/map_global
和 save_map 保存的 PCD 带 rgb。RViz 里把对应 PointCloud2 的 Color Transformer
改成 RGB8 才能看到颜色，排版不用改。

5.13 debug（city 分支）

    resource_log_en       是否周期性打印系统资源和建图规模。默认 true
    resource_log_period   打印间隔，秒。默认 5

日志同时写到终端和 <包目录>/Log/resource_monitor.csv。
看 rss、ikdtree、kf/raw_pts 的增长，以及 lidar_q 是否堆积，用来判断大场景该优化内存还是算力。
save_map 前后也会各打一条，tag 为 save_map_begin / save_map_end。

5.14 service

    save_directory                 默认保存目录。代码直接拼接文件名，末尾要有 /
    global_map_server_leaf_size    服务调用时 resolution 为 0 所使用的体素，米


6. 运行时可以看到的话题

    /cloud_registered                  世界系当前帧
    /cloud_registered_body             IMU 系当前帧
    /Odometry                          前端里程计
    /odom_path                         前端轨迹
    /map_path                          后端轨迹
    /visualization/map_global          回环校正后的显示地图
    /loop/loop_closure_constraints     回环约束
    /gps_odom                          仅 GPS 模式，由 gpsOdometry 发布
    /gps_path                          仅 GPS 模式

RViz 使用原来的 loam_livox.rviz。其中 AerialMapDisplay 已关闭，
避免 Mapbox 瓦片请求失败刷屏。卫星底图不影响建图。
