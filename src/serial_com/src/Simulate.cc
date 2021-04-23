#include "../include/serial_com/Simulate.hpp"

namespace sim{

Simulate::Simulate(ros::NodeHandle nh,int _mode, int is_blue){
    log_judge = false;
    pub = nh.advertise<serial_com::comm>("gimbalData", 1);
    sub = nh.subscribe("cameraData", 1, &Simulate::display, this);
    seed = (uint32_t)time(NULL);
    _sign_p = 1;
    _sign_y = 1;
    linear_pitch = start_pitch;
    linear_yaw = start_yaw;
    frame_cnt = 0;
    mode_cnt = _mode;                   //初始默认mode_cnt & 1的值为1
    _is_blue = is_blue;

    old_pitch = 0.0;
    old_yaw = 0.0;
}

Simulate::~Simulate(){;}

void Simulate::display(const serial_com::comm::ConstPtr &msg){
    print("Data from auto aim node: pitch, yaw, delay:\n");
    print("(%f, %f)\n\n", msg->y, msg->x);
    generator(RANDOM, msg->y, msg->x);
}

void Simulate::decoder(const uint8_t *buffer, serial_com::comm &msg) const{
    unsigned int hor=0, ver=0, ser_num = 0;
    for(int i = 3; i >= 0 ;--i){            //vertical pitch
        ver += buffer[i];
        if(i) ver <<= 8;
    }
    for(int i = 7; i >= 4 ;--i){            //horizontal yaw
        hor += buffer[i];
        if(i != 4) hor <<= 8;
    }
    for(int i = 11; i>= 8; --i){           //ser
        ser_num += buffer[i];
        if(i != 14) ser_num <<= 8;
    }
    msg.x = (*((float*)&hor));
    msg.y = (*((float*)&ver));
    msg.z = (*((float*)&ser_num));                            
    msg.status = (uint8_t)buffer[12];                 //最后一位是状态码
}

void Simulate::encoder(uint8_t *buffer, float x, float y, float z, uint8_t stat) const{
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

void Simulate::generator(MODE mode, float _pit, float _yaw){
    float pitch, yaw;
    float ser = serSwitch();
    uint8_t buffer[19], stat = (_is_blue > 0) ? (mode_cnt & 1) : 128;//(mode_cnt & 1);      //mode_cnt 与 1
    serial_com::comm msg;
    // printf("Incoming pitch and yaw: %f, %f\n", _pit, _yaw);
    switch(mode){
        case(NOISE_LINEAR): linear(pitch, yaw); break;
        case(RANDOM): onePoint(pitch, yaw); break;
        case(FOLLOW):
            pitch = old_pitch + _pit;
            yaw = old_yaw + _yaw;
            follow(pitch, yaw);
            old_pitch = pitch;
            old_yaw = yaw; break;
        default: staticPoint(pitch, yaw);
    }
    encoder(buffer, pitch, yaw, ser, stat);
    decoder(buffer, msg);
    // printf("PITCH, YAW, DELAY, STATUS :\n");
    // printf("%f, %f, %f, %d,\n\n",msg.x, msg.y, msg.z, msg.status);
    pub.publish(msg);
}

void Simulate::follow(float &pitch, float &yaw){
    pitch -= std::abs(rng.gaussian(0.1));
    yaw -= std::abs(rng.gaussian(0.1));
}


void Simulate::linear(float &pitch, float &yaw){
    linear_pitch += _sign_p * PITCH_VEL;
    pitch = linear_pitch;
    pitch += random();
    if(pitch > abs(start_pitch) || pitch < -abs(start_pitch)) _sign_p = -_sign_p;
    linear_yaw += _sign_y * YAW_VEL;
    yaw = linear_yaw;
    yaw += 1.2*random();
    if(yaw > abs(start_yaw) || yaw < -abs(start_yaw)) _sign_y = -_sign_y;
}

void Simulate::onePoint(float &pitch, float &yaw){
    pitch = random();
    yaw = 0.5 * random();

}

void Simulate::staticPoint(float &pitch, float &yaw){
    pitch = 0;
    yaw = 0;
}

float Simulate::random(int range){
    return rng.gaussian(0.1);
}

float Simulate::serSwitch(){
    ++frame_cnt;
    if(frame_cnt < 400){
        return 1.0;
    }
    else if(frame_cnt < 800){
        return 2.0;
    }
    else if(frame_cnt < 1200){
        return 5.0;
    }
    else{
        frame_cnt = 0;                      //重置
        mode_cnt += 2;                      //mode_cnt改变，导致发送的status改变
        //rmlog::LOG::printc(rmlog::F_DARK_GREEN, "Change of frame mode!");
        return 5.0;
    }
}
}   //namespace sim
