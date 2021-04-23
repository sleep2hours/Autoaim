#include "serial_com/SerialCom.hpp"

SerialCom::SerialCom(){
    try{
        char path[128]="/dev/serial/by-id/";
        if(!serialOK(path)==0){
            ROS_ERROR_STREAM("Unable to open port. Not exception. Possible null path detected.");
            exit(-1);
        }
        ser.setPort(path);
        std::cout<< path <<std::endl;     
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setBaudrate(115200);
        //串口设置timeout
        ser.setTimeout(to);
        ser.open();
        printf("Serial port opened.\n");
    }
    catch (serial::IOException &e){
        ROS_ERROR_STREAM("Unable to open port with Exception.");
        exit(-1);
    }
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized.");
    }
    else{
        ROS_ERROR_STREAM("Open serial port failed.");
        exit(-1);
    }
    para_pub = nh.advertise<serial_com::comm>("gimbalData", 4);
    para_sub = nh.subscribe("cameraData", 4, &SerialCom::infoExchange, this);
    ROS_INFO_STREAM("Topic subscribed.\n");
    old_val.x               = 0.0;
    old_val.y               = 0.0;
    old_val.z               = 0.0;
    old_val.status          = 0;
}

SerialCom::~SerialCom(){
    ser.close();
}

//把需要的数据转换成uint_8
// 工控机 -> 云台版数据转换
void SerialCom::getBuffer(uint8_t *buffer, float x, float y, float z, uint8_t stat){
    unsigned int f1 = (*((unsigned int *)&(x)));
    unsigned int f2 = (*((unsigned int *)&(y)));
    unsigned int f3 = (*((unsigned int *)&(z)));
    //buffer是8位unsigned int 数组，每一位代表一字节
    for (int i = 0; i < 4; ++i){                //4个字节的float x转换为buffer中的4位
        uint8_t tmp = (f1 >> (8 * i)) & 0xff;
        buffer[i] = tmp;                    
    }
    for (int i = 0; i < 4; ++i){                //4个字节的float y转换为buffer中的4位
        uint8_t tmp = (f2 >> (8 * i)) & 0xff;
        buffer[i + 4] = tmp;
    }
    for (int i = 0; i < 4; ++i){                //4个字节的float32 z
        uint8_t tmp = (f3 >> (8 * i)) & 0xff;
        buffer[i + 8] = tmp;
    }
    buffer[12] = stat;                        //机器人状态码：0-255，所以只需要一个字节
}

//串口收发数据测试
//此函数是subscriber的回调函数
void SerialCom::infoExchange(const serial_com::comm::ConstPtr &msg){
    uint8_t buffer[13];
    size_t num = 0;
    serial_debug("Msg from Frame: x, y, z: %f, %f, %f\n", msg->x, msg->y, msg->z);
    float _x = 0.0, _y = 0.0, _z = 0.0;
    if(msg->z > 200.0){
        _x = msg->x;
        _y = msg->y;
        _z = msg->z;
    }
    getBuffer(buffer, _x, _y, _z, msg->status);                 // 视觉->电控 数据转换

    num = ser.write(buffer, 13);
    ros_debug("Sent comm to serial port. %lu bits.", num);
    //写入usb后立即从usb取数据，然后发布获取的解码数据
    serial_com::comm to_send;
    if(getDataFromSerial(to_send) == 1){                        //从云台板读取信息成功
        updateOld(to_send);                                     //保存历史位置
        para_pub.publish(to_send);                              //发送云台数据给另一节点   

        serial_debug("Pit: %f, Yaw: %f\n", to_send.y, to_send.x);
        ros_debug("Incoming comm from the chip processed.\n");
    }
    else{
        getFromOld(to_send);                                    //从历史数据中取值
        para_pub.publish(to_send);                              //从云台取得信息失败，发送旧值

        serial_debug("No incoming comm from the chip. Old comm sent.\n");
    }
}

//从串口接收数据
void SerialCom::receiveData(const uint8_t *buffer, float &yaw, float &pitch, float &ser, uint8_t &stat){
    unsigned int hor = 0, ver = 0, ser_num = 0;
    for(int i = 3; i >= 0 ; i--){            //vertical pitch
        hor += buffer[i];
        if(i) hor <<= 8;
    }
    for(int i = 7; i >= 4 ; i--){            //horizontal yaw
        ver += buffer[i];
        if(i != 4) ver <<= 8;
    }
    for(int i = 11; i>= 8; i--){           //ser
        ser_num += buffer[i];
        if(i != 8) ser_num <<= 8;
    }
    pitch   = (*((float*)&ver));
    yaw     = (*((float*)&hor));
    ser     = (*((float*)&ser_num));                            
    stat    = (uint8_t)buffer[12];                 //最后一位是状态码
}

//查找需要的串口
int SerialCom::serialOK(char *output){
    std::cout<<"Directory dev/serial exists."<<std::endl;
    DIR *dir = opendir("/dev/serial/by-id");
    if(!dir){                                               //是否能正常打开目录
        std::cout<<"Open directory error."<<std::endl;
        return -1;
    }
    struct dirent* files;
    while((files = readdir(dir))){                            //查找名称长度大于5的为USB设备
        for(int i = 0; i < 256 && files->d_name[i] > 0; ++i){
            if(i>5){
                strcat(output, files->d_name);                  // 云台板断电，重新给予权限
                std::string command = "echo \"bfjg\" | sudo -S chmod 777 " + std::string(output);
                std::cout << "============" << command <<std::endl;
                system(command.c_str());
                return 0;
            }
        }
    }
    return -1;                                                  //所有设备名称不符合要求
}

int SerialCom::getDataFromSerial(serial_com::comm &d){  //输出是两个float值+一个状态码
    if(ser.waitReadable()){                                    //缓冲区内有信息                                    
        size_t num = ser.available();                       //获取缓冲区内数据量
        if(num){
            std_msgs::String res;
            res.data = ser.read(num);                         //一次性读出缓冲区里所有的数据
            if(!res.data.size()){
                ROS_ERROR_STREAM("There has been a Read Error.\n");
            }

            serial_debug("Data received.\n");
            uint8_t result[13];
            for(int i = 0; i< 13 ; ++i){
                result[i] = res.data[i];                    //云台发来的数据全部使用
            }
            receiveData(result, d.x, d.y, d.z, d.status);        //解包

            return 1;
        }else{
            ROS_ERROR_STREAM("Unknown Error with 0 available but failed to detect.");
            return -1;
        }  
    }else{
        ROS_ERROR_STREAM("No data available.");
        return -1;
    }
}

void SerialCom::updateOld(const serial_com::comm &src){
    old_val.x           = src.x;
    old_val.y           = src.y;
    old_val.z           = src.z;
    old_val.status      = src.status;
}

void SerialCom::getFromOld(serial_com::comm &src){
    src.x           = 0.0;       
    src.y           = 0.0; 
    src.z           = 0.0;               
    src.status      = old_val.status;           
}