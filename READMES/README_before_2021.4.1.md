## Gimbal

---
## Byte13 V1.1

- 本文档由 **<u>HQY</u>** 于2021.3.31 小队动身去创新港的那个周三下午编辑，存在的问题请看 **<u>TODOs</u>**.

- Release 即可使用
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
### TODOs
- [ ] 数字识别不同环境下的预测 需要实测
- [ ] 多车装甲板 / transfomer数据集，大装甲板数据集扩增，最好能有别的学校的视频
- [ ] 预测debug后的实测
- [ ] 大装甲弹道，还没有测试过打击大装甲板
- [ ] 不同车PID的参数调优
- [ ] 敌方红色看看效果，29日之后没有测过了，但是应该效果还行
- [ ] 开机自启动（简单bashrc）
- [ ] 决策，需要进行简单的更改（打击距离视野最中心的目标，这个很好改，AimDistance中修改即可）
- [ ] 哨兵试试41bytes，不行就直接换release v1.1

---
#### 更新记录
- 更新 2021.3.7 -HQY
  - 取流简单修改，flag使用原子操作替换，线程安全
  - 多线程OpenMP重构
  - 修复了AimRos中的bug
  - 修改了的LightMatch中的部分实现，加速
- 更新 2021.3.17 -HQY
  - LightMatch已经完全被Ceres优化替换了，在没有进行fine-tune的情况下效果仍然很不错，稳定性很好
  - **已经可以不使用低曝光了，曝光可以上调至700，进行数字识别 + 高曝光下对于远距离灯条有更好的保真度**
  - Armorlate
    - ANN_MLP 学习的方式得到的装甲板匹配，但是效果不能算太好，测试集上平均正确率99.25%，也就是说发生错误的期望大约是125帧一错
    - 但实际上，每一帧装甲板的数量期望大约是3，也就是说40多帧可能就错一次，但是在视频上看好像还行
    - 简单KM算法（给定灯条车号的情况下的算法），匹配过滤
- 2021.3.29
  - 省赛Plan B
  - MLP in Pytorch / libTorch
- 2021.3.30
  - v1.0 release
- 2021.3.31
  - v1.1 大装甲板 + 预测bug修复