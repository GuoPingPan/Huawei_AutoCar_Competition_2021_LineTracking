## 华为无人车车道线检测与倒车定位

>   注意：以下的算法包都应该放在ROS工作空间下使用

#### 1.车道线检测模块

算法包：`linetracking`

-   linetracking
    -   launch	用于存放launch启动脚本文件，学过ROS应该知道
        -   cameraPro.launch
        -   cameraTest.launch
    -   src 源程序存放文件
        -   cameraPro.cpp 实际使用的车道先检测文件
        -   cameraTest.cpp 车道线可视化测试文件
    -   rviz 用于存放可视化参数，不必理会



#### 2.倒车模块

倒车定位模块使用的是Google的开源slam算法cartographer，硬件设备为思岚科技的A2激光雷达。

使用cartographer时需要进行一定修改，可跟从下面链接操作

[Cartographer用于机器人纯定位 - 古月居 (guyuehome.com)](https://www.guyuehome.com/33683)

算法包：`backup`

-   backup
    -   launch
        -   followTrajectory.launch 路径发送到控制模块，并由控制模块完成跟随
        -   huawei_car_2d.launch 二维建图启动脚本
        -   huawei_car_localization_2d.launch 二维定位启动脚本
        -   saveTracking.launch 路径保存脚本
    -   src 源程序存放文件
        -   followTrajectory.cpp
        -   saveTrajectory.cpp