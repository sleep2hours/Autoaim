## Gimbal

---
## Byte13 V1.1

---
###  能量机关模块通讯协议说明
|消息      |msg.x|msg.y|msg.z|msg.status|
|电控给视觉|pitch|yaw|弹速|小/大/反能量机关|
|视觉给电控|x|y|z|是否发弹|

### TO DO
1.发弹逻辑的更改，需要看实测的效果
2.掉帧问题的解决
3.反能量机关模块
#### 更新记录
--更新2021.4.20 -lym
 -重新构建了能量机关的识别逻辑，鲁棒性有所提高，但开始出现掉帧的问题
 -重新构建了能量机关模块的所有代码，可读性提高
 -加入了旋转方向、发弹控制逻辑
--更新2021.4.24 -lym
  -构建了正弦风车的预测

