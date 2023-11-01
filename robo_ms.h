#ifndef INCLUDED_robo_ms_H
#define INCLUDED_robo_ms_H

#include "mbed.h"

class rbms {
    public:
        rbms(CAN &can,int moter_num);
        int rbms_send(int moter[4]);
        void rbms_read(CANMessage &msg, short *rotation,short *speed);
        void can_read();
        float pid(float T,short rpm_now, short set_speed,float *delta_rpm_pre,float *ie,float KP=25,float KI=10, float KD=0);
        void spd_control(int id,int* set_speed,int* motor);
        
    private:

        CANMessage _canMessage,_canMessage2,_msg;
        CAN &_can;
        int _moter_num;
        unsigned short _r;
        int _a = 0;
        int _rotation;
        int _speed;
        int _torque;
        int _temperature;



};


#endif