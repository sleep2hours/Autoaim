# Autoaim
## RM 2020 Autoaim framework and code backup
### 考试周以及此后的一些方案

> * 2020.6.13
>> * 现阶段需要处理的问题：
>>> * 对运动状态进行模式识别，在被瞄准车辆行驶状态良好时，进行加入了子弹滞空时间的预测
>>> * 测试弹道模型（步兵/英雄）
>>> * 和步兵电控联调，确定视觉组通信协议(27日考试结束晚上进行讨论)
>>>> * 打符和自瞄使用的一套通信协议，在打符中有一些位置被闲置了
>>> * 暂时放弃进行模式识别预测(只使用ARKF优化的随动)
>> * 想到一个优化的方式（不过可能是增加复杂度）
>>> * 不使用明暗交替，全部使用低曝光以确保用最大帧率获得最准确的装甲板信息
>>> * 多采用一个低帧率的USB摄像头（具体帧率还需要确定），使用USB摄像头粗略提取装甲板，主要是为了提取数字信息
>>> * 关于USB摄像头，假设加入这个辅助性的摄像头进行数字特征提取，需要一套提取后数字和工业相机提取的装甲板进行匹配的算法(可能单点最邻近吧)
>>> * USB帧率不匹配，达不到88fps，虽然加入减小了数字预测的不确定度，但还是做不到精确匹配
>>> * USB摄像头是否存在不稳定问题？外加USB摄像头辅助对处理时间的影响？可以使用ROS再多一个节点，使用sensor_msg发送出去（细节有待讨论）
>>> * USB摄像头加入后无需采样/调节曝光,算法需要重构
> * 2020.6.30
>> * 优化了阈值，找到了一种可以减小阈值同时可以滤除反光的方法，降低了几个拟合函数的阶数并调整了自适应阈值、自适应亚像素窗口的函数
>> * 修改了亚像素检测中存在的一些问题，让远距离的亚像素检测变得比较鲁棒
>> * 目前需要解决的优化问题
> * 2020.7.1
>> * 优化了亚像素检测x轴方向，但亚像素还存在一定的问题（与阈值有关）
>> * 修改了灯条检测`方向矩形的提取`，舍弃了方向矩形包络这个累赘的设定,使用蓝绿两色通道，结合OTSU阈值化算法进行图像分割
>>> * 角度优化的效果非常好，5个视频中出现了2次分割失败，掉帧情况减少了很多
>> * 修改了轮廓提取的方式，改用sort
> * 2020.7.2
>> * 加入了大装甲板角度优化，以在某些灯条重合的时刻发生误判
>> * 加入了红色通道的OTSU阈值,角度优化分割轮廓点失败的情况消失了
>> * 进一步减小阈值，鲁棒性提高
>> * 今天晚上所调出来的版本是最为鲁棒的一个版本，错误率很低,但仍然存在一些问题
> * 2020.7.3
>> * 过滤算法加入了位置过滤，直接消除了视频中的误匹配
>> * 修正了灯条过小时可能出现的灯条长度异常与角度异常debug
>> * 对于过小灯条不使用findContours找到的轮廓点进行角度优化，而直接使用(异步的！！！！)cv::Mat::forEach函数处理
> * 2020.7.5/6/7
>> * 处理有关双摄像头的问题
>> * 重投影已经做了，并且可以完成双目标定
> * 2020.7.8-7.10
>> * 忙活了三天有关重投影的问题，使用原来的垃圾摄像头，发现非常不好用，标定误差很大，并且无法自适应
>> * Projection.hpp 和 .cc被暂时弃用了，接口函数会留下
>> * 使用了新的山狗相机，准备采用在线标定的方式
> * 2020.7.13
>> * 修正了ArmorPlate里对大装甲板进行匹配的函数
>> * 主函数形式变更需要重构
> * 2020.7.20
>> * 修正了部分蓝色反光灯条
>> * 进行了红色装甲板的适配debug, 检测不够稳定，原因是：红色装甲板的亮度更低
> * 2020.7.21
>> * SDK问题修复
>> * 红色装甲板检测加入
>> * 云台通信debug
> * 2020.7.22
>> * 修改了主循环里引起通信不稳定的实现
>> * 测试了弹道模型，发现了弹道模型参数计算的错误, (review未完成)
>> * 加入了两轴速度弹道模型
> * 2020.7.23
>> * 弹道模型已经检查过了，无误，摩擦轮的速度会受到电池电量的影响
>> * 加入了静止目标判定，修改了通信协议中的错误
>> * 可以使用云台的信息，主函数进行了debug，已经修复了收到NaN的问题
>> * GQR添加了更为方便的rosParam调试， merged 并且消除了其中的warnings
>> * 步兵可以实现 4/5/7m的精准打击
> * 2020.7.25
>> * 测定了静止物体的方差，其值在英雄上是比较小的，但是静止的滑动平均滤波还是失败了，可能的原因有：
>>> * 成功帧数过少，太容易进入滑动平均模式
>>> * 方差阈值过大，滑动平均容器过小，容易进入滑动平均
>> * 不成功的红色灯条过滤，可能的解决方式
>>> * minAreaRect 进行长宽比的先行判定
> * 2020.7.27
>> * 未测试GQR加入的UKF
>> * 解决了ROS subscriber callBack导致的数据污染情况（1.不使用全局变量 2.ros::spinOnce的位置）
>> * 验证了静止装甲板滤波的效果，测定了新弹道模型的情况（比第二版弹道模型好了（还需review））
> * 2020.8.4
>> * 解决了灯条优化引起的灯条重合问题
>> * 将弹道模型移交给电控，并且测试了随动的性能，下一步是做一版鲁棒的预测算法出来
- [ ] 红色装甲板的稳定
- [ ] 静止打击滤波（尝试不同方法）
> * 2020.7.28
>> * 加入打符模块
