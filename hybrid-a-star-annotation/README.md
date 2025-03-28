# 混合 Astar 算法的 ROS 功能包，用于路径规划。

### 功能包名："hybrid_astar"

#### 节点 1：

- 节点名称："scale_up_map"
- 节点功能：增大栅格地图的分辨率，提高搜索效率；同时发布无人车到局部地图的坐标系变换。
- 主要输入话题：`"/welkin/local_map/occupy_grid [nav_msgs/OccupancyGrid]"`
- 主要输出话题：`"/local_map_occ [nav_msgs/OccupancyGrid]"`, `"/tf [tf2_msgs/TFMessage]"`
- 单独启动命令行：`"rosrun hybrid_astar scale_up_map"`

#### 节点 2：

- 节点名称："pub_optimal_goal"
- 节点功能：检测目标点周围是否有障碍物，选择周围没有障碍物的点作为目标点。
- 主要输入话题：`"/local_map_occ [nav_msgs/Path]"`
- 主要输出话题：`"/move_base_simple/goal [geometry_msgs/PoseStamped]"`
- 单独启动命令行：`"rosrun hybrid_astar pub_optimal_goal"`

#### 节点 3：

- 节点名称："hybrid_astar"
- 节点功能：根据栅格地图信息、当前位置信息和目标点位置信息，使用混合 Astar 算法规划可行路径。
- 主要输入话题以及话题格式：`"/local_map_occ [nav_msgs/OccupancyGrid]"`, `"/move_base_simple/goal [geometry_msgs/PoseStamped]"`, `"/tf [tf2_msgs/TFMessage]"`
- 主要输出话题以及话题格式：`"/recommend_speed [std_msgs/Float32]"`, `"/path_in_globalmap [nav_msgs/Path]"`
- 单独启动命令行：`"rosrun hybrid_astar hybrid_astar"`

#### launch 文件

路径规划功能 launch 文件启动命令行：`"roslaunch hybrid_astar path_planning.launch"`
可视化功能 launch 文件启动命令行：`"roslaunch hybrid_astar rviz.launch"`
