# ros2_olei
ros2 olei driver

欧雷 olei 推出的 LR16F 雷达是市面上少见的具有防爆认证雷达，能够应用于易燃易爆场合的巡检机器人中，但是其所提供的资料并不完善，没有包含ros2版本的驱动代码。因此本项目基于激光雷达的tcp/ip协议，构建了一个适用与ros2环境的雷达驱动代码。
- 相比于厂商提供的ros1代码进行了大量的改进，尽可能降低了内存复制、传输过程的消耗，实时性更好。
- 提供独立于ROS2的点云数据接口，可以通过PCL显示点云数据方式而不采用ROS2。


![image](https://user-images.githubusercontent.com/50650063/192136089-739dc81f-6059-4b50-a1bd-fe040a683b46.png)
