#ifndef INCLUDED_rbms_H
#define INCLUDED_rbms_H

#include "mbed.h"
#include "CANManager.h"

class rbms : public CANManager{
    public:
        rbms(CAN &can,bool motor_type,int motor_num);
        bool handle_message(const CANMessage &msg) override;
        int rbms_send(int* motor);
        void rbms_read(CANMessage &msg, short *rotation,short *speed);
        float pid(float T,short rpm_now, short set_speed,float *delta_rpm_pre,float *ie,float KP=25,float KI=10, float KD=0);
        void spd_control(int* set_speed,int* motor);
        
    private:

        CANMessage _canMessage,_canMessage2,_msg;
        CAN &_can;
        bool _motor_type;//if 0 m2006,if 1 m3508
        int _motor_num,_motor_max;
        unsigned short _r;
        int _rotation;
        int _speed;
        int _torque;
        int _temperature;



};


#endif