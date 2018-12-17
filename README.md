# smartcar_mini
## 说明
本项目是上海交通大学第一届智慧出行挑战赛的底层驱动文件和实例文件，基于NVIDIA Jetson TX2 构建
## 模块说明
### simple_controller
简单控制器，对底盘进行开环控制
### pid_controller
PID控制器，对底盘进行反馈控制（未调通）
### rplidar_ros
fork自[rplidar_ros](https://github.com/Slamtec/rplidar_ros)，激光雷达的驱动
### smartcar
上层控制程序，包含车道线检测、障碍物识别、红绿灯识别等