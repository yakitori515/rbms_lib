#include "rbms.h"
#include "mbed.h"
rbms::rbms(CAN &can,int moter_num)
    : _can(can),_moter_num(moter_num){
    if(_moter_num<=8){
        _can.frequency(1000000); // CANのビットレートを指定
        _can.mode(CAN::Normal); // CANのモードをNormalに設定
    }
}

int rbms::rbms_send(int* moter) {
    char _byte[_moter_num];
    
    for(int i=0;i<_moter_num;i++){
        _byte[_a++] = (char)(moter[i] >> 8); // int値の上位8ビットをcharに変換
        _byte[_a++] = (char)(moter[i] & 0xFF); // int値の下位8ビットをcharに変換
    }

    _canMessage.id = 0x200;
    _canMessage.len = 8;
    _canMessage2.id = 0x1ff;
    _canMessage2.len = 8;
    _a = 0;
    for(int i=0;i<_moter_num;i++){
        if(i<4){
            _canMessage.data[_a] = _byte[_a]; // CANメッセージのデータにbyte1をセット
            _a++;
            _canMessage.data[_a] = _byte[_a];
            _a++;
        }else{
            _canMessage2.data[_a] = _byte[_a]; // CANメッセージのデータにbyte1をセット
            _a++;
            _canMessage2.data[_a] = _byte[_a];
            _a++;
        }
    }
    while(_a<15){
        if(_a<7){
            _canMessage.data[_a++] = 0; // CANメッセージのデータにbyte1をセット
            _canMessage.data[_a++] = 0;
        }else{
            _canMessage2.data[_a++] = 0;
            _canMessage2.data[_a++] = 0;
        } 
    }
    // CANメッセージの送信
    if (_can.write(_canMessage)&&_can.write(_canMessage2)) {
        return 1;
    }else{
        return -1;
    }
}

void rbms::rbms_read(CANMessage &msg, short *rotation,short *speed) {
            //printf("id:%x\n",msg.id);
            _r = (msg.data[0] << 8) | (msg.data[1] & 0xff);
            _rotation = (float)_r / 8192 * 360;
            *rotation=_rotation;
 
            _speed = (msg.data[2] << 8) | (msg.data[3] & 0xff);
            if (_speed & 0b1000000000000000){
                _speed--;
                _speed = -~_speed;
            }
            *speed=_speed;

            _torque = (msg.data[4] << 8) | (msg.data[5] & 0xff);
            if (_torque & 0b1000000000000000){
                _torque--;
                _torque = -~_torque;
            }

            _temperature = msg.data[6];
            
}

void rbms::can_read(){
    while(true){
        if(_can.read(_msg)){

        }
    }
}

float rbms::pid(float T,short rpm_now, short set_speed,float *delta_rpm_pre,float *ie,float KP, float KI,float KD )
{
    float de;
    float delta_rpm;
    delta_rpm  = set_speed - rpm_now;
    de = (delta_rpm - *delta_rpm_pre)/T;
    *ie = *ie + (delta_rpm + *delta_rpm_pre)*T/2;
    float out_torque  = KP*delta_rpm + KI*(*ie) + KD*de;
    *delta_rpm_pre = delta_rpm;
    return out_torque;
}

void rbms::spd_control(int id,int* set_speed,int* motor){
    short rotation,speed;
    float delta_rpm_pre,ie;
    Timer tm;
    tm.start();
    while(1){
        if(_msg.id==0x1ff+id){
            rbms_read(_msg,&rotation,&speed);
            *motor = (int)pid(tm.read(),speed/19,*set_speed,&delta_rpm_pre,&ie);
            tm.reset();
            if(*motor>16384){*motor=16384;}else if(*motor<-16384){*motor=-16384;}
        }
    }
}