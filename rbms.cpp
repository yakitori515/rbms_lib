#include "rbms.h"
#include "mbed.h"
rbms::rbms(CAN &can,bool motor_type,int motor_num)
    : _can(can),_motor_type(motor_type),_motor_num(motor_num){
    if (_motor_type) { // M3508
        _kp = 25.0f; _ki = 10.0f; _kd = 0.0f;
        _motor_max = 16384;
    } else { // M2006
        _kp = 15.0f; _ki = 6.0f; _kd = 0.0f;
        _motor_max = 10000;
    }
    for(int i = 0; i < 8; i++) {
        _control_modes[i] = SPD_MODE;
        _target_speeds[i] = 0;
        _target_torques[i] = 0;
        _output_torques[i] = 0;
        _pid_states[i].prev_err = 0.0f;
        _pid_states[i].integral = 0.0f;
        _pid_states[i].timer.start();
    }
    if(_motor_num<=8){
        _can.frequency(1000000); // CANのビットレートを指定
        _can.mode(CAN::Normal); // CANのモードをNormalに設定
    }
}

void rbms::set_control_mode(int id, ControlMode mode) {
    if (id < 0 || id >= _motor_num) return;
    _data_mutex.lock();
    _control_modes[id] = mode;
    _pid_states[id].integral = 0;
    _pid_states[id].prev_err = 0;
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

void rbms::set_pid_gains(float kp, float ki, float kd) {
    _data_mutex.lock();
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _data_mutex.unlock();
}

float rbms::pid_calculate(int id, float target, float current, float dt) {
    float error = target - current;
    _pid_states[id].integral += (error + _pid_states[id].prev_err) * dt / 2.0f;
    float derivative = (error - _pid_states[id].prev_err) / dt;
    float out = (_kp * error) + (_ki * _pid_states[id].integral) + (_kd * derivative);
    _pid_states[id].prev_err = error;
    return out;
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

            _data_mutex.lock();
            if (_new_data_mask & (1 << id)) {
                local_msg = _msg_buffer[id];
                _new_data_mask &= ~(1 << id);
                has_new = true;
            }
            mode = _control_modes[id];
            target_s = _target_speeds[id];
            target_t = _target_torques[id];
            _data_mutex.unlock();

            if (has_new) {
                short rot, raw_spd;
                parse_can_data(local_msg, &rot, &raw_spd);
                
                int final_out = 0;

                if (mode == SPD_MODE) {
                    float dt = _pid_states[id].timer.read();
                    _pid_states[id].timer.reset();
                    if (dt <= 0) dt = 0.001f;
                    float current_rpm = _motor_type ? (raw_spd / 19.0f) : (raw_spd / 36.0f);
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


void rbms::parse_can_data(const CANMessage &msg, short *rotation, short *speed) {
    unsigned short r = (msg.data[0] << 8) | (msg.data[1] & 0xff);
    *rotation = (short)((float)r / 8192.0f * 360.0f);
    *speed = (int16_t)((msg.data[2] << 8) | (msg.data[3] & 0xff));
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