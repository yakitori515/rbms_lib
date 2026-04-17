#ifndef INCLUDED_rbms_H
#define INCLUDED_rbms_H

#include "mbed.h"
#include "CANManager.h"

class rbms : public CANReceiver{
    public:
        enum ControlMode {
            SPD_MODE,
            TRQ_MODE
        };

        rbms(CAN &can,bool motor_type,int motor_num);

        /**
         * @brief 指定したモーターの制御モードを切り替え
         * @param id モーターインデックス (0 ~ motor_num-1)
         * @param mode SPD_MODE または TRQ_MODE
         */
        void set_control_mode(int id, ControlMode mode);

        /**
         * @brief スピード制御時の目標速度を設定
         * @param id モーターインデックス
         * @param speed 目標RPM
         */
        void set_target_speed(int id, int speed);

        /**
         * @brief トルク制御時の目標トルクを設定
         * @param id モーターインデックス
         * @param torque 出力トルク値 (M2006:±10000, M3508:±16384)
         */
        void set_target_torque(int id, int torque);
        /**
         * @brief PIDゲイン設定
         * @param kp 比例ゲイン
         * @param ki 積分ゲイン
         * @param kd 微分ゲイン
         */
        void set_pid_gains(float kp, float ki, float kd);
        
        bool handle_message(const CANMessage &msg) override;
        void spd_control();
        int rbms_send();
        void rbms_read(CANMessage &msg, short *rotation,short *speed);
        
    private:

        void control_thread_entry();
        float pid_calculate(int id, float target, float current, float dt);
        void parse_can_data(const CANMessage &msg, short *rotation, short *speed);

        CAN &_can;
        bool _motor_type;
        int _motor_num, _motor_max;

        //内部データバッファ
        ControlMode _control_modes[8];
        int _target_speeds[8];
        int _target_torques[8];
        int _output_torques[8];
        float _kp, _ki, _kd;

        Thread _thread;
        Mutex _data_mutex;
        EventFlags _event_flags;

        struct PIDState {
            float prev_err;
            float integral;
            Timer timer;
        } _pid_states[8];

        CANMessage _tx_msg_low, _tx_msg_high;
        CANMessage _msg_buffer[8];
        uint8_t _new_data_mask;



};


#endif