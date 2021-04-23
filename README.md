## Gimbal

---
## Byte13 V1.1

---
###  能量机关模块通讯协议说明
|消息      |msg.x|msg.y|msg.z|msg.status|
|电控给视觉|pitch|yaw|弹速|小/大/反能量机关|
|视觉给电控|x|y|z|是否发弹|


---
### 依赖库

| 依赖               | 作用                                                         |
| ------------------ | ------------------------------------------------------------ |
| OpenCV3+           | ROS自带，有4更好，不过不用折磨特地去编译安装OpenCV4          |
| ROS-melodic-full   | ROS                                                          |
| ROS-melodic-serial | 需要自己安装的串口包，没有则使用`sudo apt-get install ros-melodic-serial`进行安装 |
| Ceres 1.14         | 官网有2.0版本 但是没试过，先下1.14保险                       |
| libTorch           | 1.8.0 C++11 api版，可以找HQY拿包，需要解压在/opt/libtorch/下 |

​		关于libtorch，CMakeLists.txt需要多加一句。否则无法编译。

```cmake
set(Torch_DIR "/opt/libtorch/share/cmake/Torch")
```

​		注意，带libtorch时编译万一报 **<u>Scalar</u>** 相关的错误，应该是 **<u>OpenCV</u>** 的 cv::Scalar 与torch的 c10::Scalar冲突。所以说：**<u>不要用using namespace cv</u>**，找到所有的using namespace cv，删掉。
---
### TO DO
1.大能量机关的预测
2.发弹逻辑的更改，需要看实测的效果
3.掉帧问题的解决
4.反能量机关模块
#### 更新记录
--更新2021.4.20 -lym
 -重新构建了能量机关的识别逻辑，鲁棒性有所提高，但开始出现掉帧的问题
 -重新构建了能量机关模块的所有代码，可读性提高
 -加入了旋转方向、发弹控制逻辑

