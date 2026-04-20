#include "rbms.h"
#include "mbed.h"

rbms::rbms(CAN &can,bool motor_type,int motor_num)
    : _can(can),_motor_type(motor_type),_motor_num(motor_num){
    if (_motor_type) { // M3508
        _kp = 35.0f; _ki = 50.0f; _kd = 0.0f;
        _kp_p = 5.0f; _ki_p = 0.0f; _kd_p = 0.15f; // 【追加】位置ループの初期ゲイン
        _motor_max = 16384;
    } else { // M2006
        _kp = 15.0f; _ki = 12.0f; _kd = 0.0f;
        _kp_p = 7.0f; _ki_p = 0.0f; _kd_p = 0.3f; // 【追加】位置ループの初期ゲイン
        _motor_max = 10000;
    }
    for(int i = 0; i < 8; i++) {
        _control_modes[i] = SPD_MODE;
        _target_speeds[i] = 0;
        _target_torques[i] = 0;
        _target_angles[i] = 0.0f; // 【追加】
        _output_torques[i] = 0;
        
        // 状態の初期化
        _pid_states[i].prev_err = 0.0f;
        _pid_states[i].integral = 0.0f;
        _pid_states[i].pos_prev_err = 0.0f; // 【追加】
        _pid_states[i].pos_integral = 0.0f; // 【追加】
        _pid_states[i].last_raw_angle = 0;  // 【追加】
        _pid_states[i].accumulated_angle = 0.0f; // 【追加】
        _pid_states[i].is_initialized = false; // 【追加】
        
        _pid_states[i].timer.start();
    }
    if(_motor_num<=8){
        _can.frequency(1000000);
        _can.mode(CAN::Normal);
    }
}

void rbms::set_control_mode(int id, ControlMode mode) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _control_modes[id] = mode;
    // モード切替時に全ての積分項と偏差をリセットし暴走を防ぐ
    _pid_states[id].integral = 0;
    _pid_states[id].prev_err = 0;
    _pid_states[id].pos_integral = 0; // 【追加】
    _pid_states[id].pos_prev_err = 0; // 【追加】
    _data_mutex.unlock();
}

void rbms::set_target_speed(int id, int speed) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _target_speeds[id] = speed;
    _data_mutex.unlock();
}

void rbms::set_target_torque(int id, int torque) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _target_torques[id] = torque;
    _data_mutex.unlock();
}

// 【追加】目標角度のセッター
void rbms::set_target_angle(int id, float angle) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _target_angles[id] = angle;
    _data_mutex.unlock();
}

void rbms::reset_angle(int id) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _pid_states[id].accumulated_angle = 0.0f;
    _target_angles[id] = 0.0f;
    _data_mutex.unlock();
}

void rbms::set_pid_gains(float kp, float ki, float kd) {
    _data_mutex.lock();
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _data_mutex.unlock();
}

void rbms::set_pos_pid_gains(float kp, float ki, float kd) {
    _data_mutex.lock();
    _kp_p = kp;
    _ki_p = ki;
    _kd_p = kd;
    _data_mutex.unlock();
}

float rbms::pid_calculate(int id, float target, float current, float dt) {
    float error = target - current;
    _pid_states[id].integral += (error + _pid_states[id].prev_err) * dt / 2.0f;

    float integral_limit = _motor_max / (_ki > 0.0f ? _ki : 1.0f); 
    if (_pid_states[id].integral > integral_limit) {
        _pid_states[id].integral = integral_limit;
    } else if (_pid_states[id].integral < -integral_limit) {
        _pid_states[id].integral = -integral_limit;
    }

    float derivative = (error - _pid_states[id].prev_err) / dt;
    float out = (_kp * error) + (_ki * _pid_states[id].integral) + (_kd * derivative);
    _pid_states[id].prev_err = error;
    return out;
}

float rbms::pos_pid_calculate(int id, float target, float current, float dt) {
    float error = target - current;
    _pid_states[id].pos_integral += (error + _pid_states[id].pos_prev_err) * dt / 2.0f;

    float integral_limit = 500.0f / (_ki_p > 0.0f ? _ki_p : 1.0f); 
    if (_pid_states[id].pos_integral > integral_limit) {
        _pid_states[id].pos_integral = integral_limit;
    } else if (_pid_states[id].pos_integral < -integral_limit) {
        _pid_states[id].pos_integral = -integral_limit;
    }

    float derivative = (error - _pid_states[id].pos_prev_err) / dt;
    float out_target_speed = (_kp_p * error) + (_ki_p * _pid_states[id].pos_integral) + (_kd_p * derivative);
    _pid_states[id].pos_prev_err = error;
    
    return out_target_speed;
}

void rbms::spd_control() {
    _thread.start(callback(this, &rbms::control_thread_entry));
}

void rbms::control_thread_entry() {
    while (true) {
        _event_flags.wait_any(0x01); 

        for (int id = 0; id < _motor_num; id++) {
            CANMessage local_msg;
            bool has_new = false;
            ControlMode mode;
            int target_s, target_t;
            float target_a;

            _data_mutex.lock();
            if (_new_data_mask & (1 << id)) {
                local_msg = _msg_buffer[id];
                _new_data_mask &= ~(1 << id);
                has_new = true;
            }
            mode = _control_modes[id];
            target_s = _target_speeds[id];
            target_t = _target_torques[id];
            target_a = _target_angles[id];
            _data_mutex.unlock();

            if (has_new) {
                short rot, raw_spd;
                parse_can_data(id, local_msg, &rot, &raw_spd);
                
                int final_out = 0;
                float dt = _pid_states[id].timer.read();
                _pid_states[id].timer.reset();                    
                
                if (dt <= 0.0f || dt > 0.05f) {
                    dt = 0.001f;
                    _pid_states[id].integral = 0.0f;
                    _pid_states[id].prev_err = 0.0f;
                    _pid_states[id].pos_integral = 0.0f;
                    _pid_states[id].pos_prev_err = 0.0f;
                }

                float current_rpm = _motor_type ? (raw_spd / 19.0f) : (raw_spd / 36.0f);

                if (mode == POS_MODE) {
                    float current_angle = _pid_states[id].accumulated_angle;
                    float cascade_target_rpm = pos_pid_calculate(id, target_a, current_angle, dt);
                    
                    if (cascade_target_rpm > 500.0f) cascade_target_rpm = 500.0f;
                    if (cascade_target_rpm < -500.0f) cascade_target_rpm = -500.0f;

                    final_out = (int)pid_calculate(id, cascade_target_rpm, current_rpm, dt);
                    // printf(">speed:%f\n",current_rpm);
                    // printf(">pos:%f\n",current_angle);
                } else if (mode == SPD_MODE) {
                    final_out = (int)pid_calculate(id, (float)target_s, current_rpm, dt);
                } else {
                    final_out = target_t;
                }

                if (final_out > _motor_max) final_out = _motor_max;
                else if (final_out < -_motor_max) final_out = -_motor_max;

                _data_mutex.lock();
                _output_torques[id] = final_out;
                _data_mutex.unlock();
            }
        }
    }
}

int rbms::rbms_send() {
    _tx_msg_low.id = 0x200; _tx_msg_low.len = 8;
    _tx_msg_high.id = 0x1ff; _tx_msg_high.len = 8;
    _data_mutex.lock();
    for(int i = 0; i < _motor_num; i++) {
        int val = _output_torques[i];
        if (i < 4) {
            _tx_msg_low.data[i*2] = (char)(val >> 8);
            _tx_msg_low.data[i*2+1] = (char)(val & 0xFF);
        } else {
            _tx_msg_high.data[(i-4)*2] = (char)(val >> 8);
            _tx_msg_high.data[(i-4)*2+1] = (char)(val & 0xFF);
        }
    }
    _data_mutex.unlock();
    return (_can.write(_tx_msg_low) && (_motor_num > 4 ? _can.write(_tx_msg_high) : true)) ? 1 : -1;
}

void rbms::parse_can_data(int id, const CANMessage &msg, short *rotation, short *speed) {
    uint16_t raw_angle = (msg.data[0] << 8) | (msg.data[1] & 0xff);
    *speed = (int16_t)((msg.data[2] << 8) | (msg.data[3] & 0xff));

    _data_mutex.lock();

    if (!_pid_states[id].is_initialized) {
        _pid_states[id].last_raw_angle = raw_angle;
        _pid_states[id].accumulated_angle = 0.0f;
        _pid_states[id].is_initialized = true;
    }

    int16_t diff = raw_angle - _pid_states[id].last_raw_angle;
    if (diff > 4096) diff -= 8192;
    else if (diff < -4096) diff += 8192;

    _pid_states[id].last_raw_angle = raw_angle;

    float gear_ratio = _motor_type ? 19.2f : 36.0f;
    _pid_states[id].accumulated_angle += ((float)diff / 8192.0f) * 360.0f / gear_ratio;

    _data_mutex.unlock();

    *rotation = (short)(raw_angle / 8192.0f * 360.0f);
}

bool rbms::handle_message(const CANMessage &msg) {
    int id_idx = msg.id - 0x201;
    if (id_idx >= 0 && id_idx < _motor_num) {
        _data_mutex.lock();
        _msg_buffer[id_idx] = msg;
        _new_data_mask |= (1 << id_idx);
        _data_mutex.unlock();
        _event_flags.set(0x01); 
        return true;
    }
    return false;
}