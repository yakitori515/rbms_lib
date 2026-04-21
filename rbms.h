#ifndef INCLUDED_rbms_H
#define INCLUDED_rbms_H

#include "mbed.h"
#include "CANManager.h"

class rbms : public CANReceiver{
    public:
        enum ControlMode {
            SPD_MODE,
            TRQ_MODE,
            POS_MODE
        };

        rbms(CAN &can,bool motor_type,int motor_num);

        /**
         * @brief 指定したモーターの制御モードを切り替え
         * @param id モーターインデックス (0 ~ motor_num-1)
         * @param mode SPD_MODE, TRQ_MODE, または POS_MODE
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

        //角度制御時の目標角度を設定（単位：度）
        void set_target_angle(int id, float angle);

        //現在のエンコーダー角度を 0度 としてリセットする
        void reset_angle(int id);

        /**
         * @brief 速度制御用PIDゲイン設定
         * @param kp 比例ゲイン
         * @param ki 積分ゲイン
         * @param kd 微分ゲイン
         */
        void set_pid_gains(float kp, float ki, float kd);
        // 角度制御用（外側ループ）PIDゲイン設定
        void set_pos_pid_gains(float kp, float ki, float kd);
        // max_speed: 最大RPM (0.0fを指定するとデフォルト動作に戻る)
        void set_speed_limit(int id, float max_speed);
        // max_accel: 1秒間あたりの最大RPM変化量 (0.0fを指定すると制限なし)
        void set_accel_limit(int id, float max_accel);

        bool handle_message(const CANMessage &msg) override;
        void spd_control();
        int rbms_send();
        void rbms_read(CANMessage &msg, short *rotation,short *speed);
        
    private:

        void control_thread_entry();
        float pid_calculate(int id, float target, float current, float dt);
        float pos_pid_calculate(int id, float target, float current, float dt, float limit);
        void parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed);

        CAN &_can;
        bool _motor_type;
        int _motor_num, _motor_max;

        //内部データバッファ
        ControlMode _control_modes[8];
        int _target_speeds[8];
        int _target_torques[8];
        float _target_angles[8];
        int _output_torques[8];
        
        // ゲイン
        float _kp, _ki, _kd;             // 速度ループ用
        float _kp_p, _ki_p, _kd_p;       // 位置ループ用

        Thread _thread;
        Mutex _data_mutex;
        EventFlags _event_flags;

        struct PIDState {
            // 速度ループ用状態
            float prev_err;
            float integral;
            
            // 位置ループ用状態
            float pos_prev_err;
            float pos_integral;
            
            uint16_t last_raw_angle;   // 前回の生エンコーダー値 (0-8191)
            float accumulated_angle;   // 出力軸の累積角度 (度)
            bool is_initialized;       // 初期化フラグ

            float speed_limit_rpm;     // 設定値 (0ならデフォルト動作)
            float accel_limit_rpm_s;   // 設定値 (0なら制限なし)
            float current_target_rpm;  // スルーレート計算用の中間目標速度

            Timer timer;
        } _pid_states[8];

        CANMessage _tx_msg_low, _tx_msg_high;
        CANMessage _msg_buffer[8];
        uint8_t _new_data_mask;

};

#endif