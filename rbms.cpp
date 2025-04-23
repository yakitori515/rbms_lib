#include "rbms.h"
#include "mbed.h"
rbms::rbms(CAN &can,bool motor_type,int motor_num)
    : _can(can),_motor_type(motor_type),_motor_num(motor_num){
    if(_motor_type){
        _motor_max=16384;//トルク
    }else{
        _motor_max=25000;//電圧
    }
    if(_motor_num<=8){
        _can.frequency(1000000); // CANのビットレートを指定
        _can.mode(CAN::Normal); // CANのモードをNormalに設定
    }
}

int rbms::rbms_send(int* motor) {//motorへ制御信号を送信する関数
    char _byte[_motor_num*2];//byteデータ変換用
    int _a=0;
    for(int i=0;i<_motor_num;i++){  //int dataを2byteに分割
        if(motor[i]>_motor_max)return 0;//入力値がmotor上限以上の場合return0
        _byte[_a++] = (char)(motor[i] >> 8); // int値の上位8ビットをcharに変換
        _byte[_a++] = (char)(motor[i] & 0xFF); // int値の下位8ビットをcharに変換
    }

    _canMessage.id = 0x1ff;//esc id1~4のcanの送信id
    _canMessage.len = 8;//can data長(8byte固定)
    _canMessage2.id = 0x2ff;//esc id5~8のcanの送信id
    _canMessage2.len = 8;
    _a = 0;
    int _i=0;
    for(int i=0;i<_motor_num;i++){//canmessageにbyte dataをセット
        if(i<4){
            _canMessage.data[_a] = _byte[_a];
            _a++;
            _canMessage.data[_a] = _byte[_a];
            _a++;
        }else{
            _canMessage2.data[_i++] = _byte[_a++];
            _canMessage2.data[_i++] = _byte[_a++];
        }
    }
    while(_a<15){//あまりのcanmassage.dataに0を代入(NULL値を残さない)
        if(_a<7){
            _canMessage.data[_a++] = 0;
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

void rbms::rbms_read(CANMessage &msg, short *rotation,short *speed) {//motorからの受信データを変換する関数
            _r = (msg.data[0] << 8) | (msg.data[1] & 0xff);//2byteに分割されているdataを結合
            _rotation = (float)_r / 8191 * 360;//8192=360°
            *rotation=_rotation;
 
            _speed = (msg.data[2] << 8) | (msg.data[3] & 0xff);
            if (_speed & 0b1000000000000000){//マイナス値の場合(最上位ビットが1のとき)(2の補数)
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

void rbms::can_read(){//motorからdata受信
    while(true){
        if(_can.read(_msg)){//_msgにcanの受信dataを代入
        }
    }
}

float rbms::pid(float T,short rpm_now, short set_speed,float *delta_rpm_pre,float *ie,float KP, float KI,float KD )//pid制御
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

void rbms::spd_control(int* set_speed,int* motor){//速度制御用関数
    short rotation[_motor_num],speed[_motor_num];
    float delta_rpm_pre[_motor_num],ie[_motor_num];
    Timer tm[_motor_num];//タイマーインスタンス生成(配列ではない)
    for(int i=0;i<_motor_num;i++){//初期化
        delta_rpm_pre[i]=0.0;
        ie[i]=0.0;
        tm[i].start();
    }
    
    while(1){
        for(int id=0;id<_motor_num;id++){
            if(_msg.id==0x205+id){//esc idごとに受信データ割り振り
                CANMessage msg=_msg;
                rbms_read(msg,&rotation[id],&speed[id]);//data変換
                if(_motor_type){
                    motor[id] = (int)pid(tm[id].read(),speed[id]/19,set_speed[id],&delta_rpm_pre[id],&ie[id]);
                }else{
                    motor[id] = (int)pid(tm[id].read(),speed[id]/36,set_speed[id],&delta_rpm_pre[id],&ie[id],15,6);
                }
                tm[id].reset();//timer reset
                if(motor[id]>_motor_max){motor[id]=_motor_max;}else if(motor[id]<-_motor_max){motor[id]=-_motor_max;}//上限確認超えてた場合は上限値にset
            }
        }
        ThisThread::sleep_for(3ms);
    }
}