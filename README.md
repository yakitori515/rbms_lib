# rbms_lib

RoboMaster用モーター（M2006、M3508）をMbed OS 6で制御するためのライブラリです。  
`CANManager` を利用した非同期受信と、マイコン側でのPID計算スレッドにより、速度・位置・トルクの各モードでモーターを制御できます。

---

## 基本的な関数の説明

### 1. 初期化・設定
* **`rbms(CAN &can, bool motor_type, int motor_num)`**
  * コンストラクタ。
  * `motor_type`: `true` で M3508、`false` で M2006 を指定します。
  * `motor_num`: 使用するモーターの数（最大 8 台）を指定します。
* **`void set_control_mode(int id, ControlMode mode)`**
  * 指定したモーターの制御モードを切り替えます。
  * `mode` には `rbms::TRQ_MODE` (トルク), `rbms::SPD_MODE` (速度), `rbms::POS_MODE` (位置) を指定します。

### 2. PIDゲインの設定
ロボマスター用モーターはマイコン側でPID制御を行うため、ゲイン設定が必要です。（コンストラクタでデフォルト値が設定されています）
* **`void set_pid_gains(float kp, float ki, float kd)`**
  * 速度ループ用のPIDゲインを設定します。
* **`void set_pos_pid_gains(float kp, float ki, float kd)`**
  * 位置（角度）ループ用のPIDゲインを設定します。

### 3. 目標値・制限の設定
* **`void set_target_torque(int id, int torque)`**
  * トルク制御時の目標電流（トルク）値（M2006: `±10000`、M3508: `±16384`）を設定します。
* **`void set_target_speed(int id, int speed)`**
  * 速度制御時の目標回転数（`RPM`）を設定します。
* **`void set_target_angle(int id, float angle)`**
  * 位置制御時の目標角度（累積度数: `float`）を設定します。
* **`void reset_angle(int id)`**
  * 現在のエンコーダー位置を累積角度 `0.0` 度としてリセットします。
* **`void set_speed_limit(int id, float max_speed)`**
  * 位置制御時の最大制限速度（`RPM`）を設定します。
* **`void set_accel_limit(int id, float max_accel)`**
  * 1秒間あたりの最大速度変化量（加速度制限: `RPM/s`）を設定します。`0` を指定すると制限なしになります。

### 4. 制御と送信
* **`void spd_control()`**
  * **【重要】** 内部でPID計算を行うためのバックグラウンド制御スレッド (`Thread`) を起動します。
  * `CANManager` から受信データが届くたびに、このスレッド内で自動的にPID計算が実行され、モータへの出力電流値が更新されます。（※トルク制御時でも、フィードバック受信に同期して出力を反映するため起動する必要があります）
* **`int rbms_send()`**
  * 計算された制御出力を、モーターへCAN一斉送信します。
  * メインループ等で高頻度（例: 1ms周期など）で呼び出してください。

---

## 制御モードごとのサンプルコード

### 1. 速度制御モード (`SPD_MODE`)
モーター（ID: 1）を 1000 RPM で速度制御するサンプルです。

```cpp
#include "mbed.h"
#include "CANManager.h"
#include "rbms.h"
#include <cstdio>

CAN can(PA_11, PA_12, 1000000);
CANManager can_manager(can);
rbms motor(can, true, 1); // M3508、モーター数: 1 (ID: 0x201)

int main() {
    // 1. CANManagerにレシーバとして登録
    can_manager.add_receiver(&motor);

    // 2. バックグラウンドのPID制御スレッドを起動
    motor.spd_control();

    // 3. 制御モードを速度制御に設定し、目標値を設定
    motor.set_control_mode(0, rbms::SPD_MODE);
    motor.set_target_speed(0, 1000); // 1000 RPM

    while (true) {
        // 4. トルク(電流)コマンドを送信
        motor.rbms_send();

        // 1ms周期で送信
        ThisThread::sleep_for(1ms);
    }
}
```

### 2. トルク制御モード (`TRQ_MODE`)
モーター（ID: 1）の出力電流値（トルク）を直接指定して回すサンプルです。

```cpp
#include "mbed.h"
#include "CANManager.h"
#include "rbms.h"
#include <cstdio>

CAN can(PA_11, PA_12, 1000000);
CANManager can_manager(can);
rbms motor(can, true, 1); // M3508、モーター数: 1 (ID: 0x201)

int main() {
    can_manager.add_receiver(&motor);
    motor.spd_control();

    // 制御モードをトルク制御に設定し、目標トルクを設定
    motor.set_control_mode(0, rbms::TRQ_MODE);
    motor.set_target_torque(0, 1500); // 出力電流指定: 1500 (M3508最大: 16384)

    while (true) {
        motor.rbms_send();
        ThisThread::sleep_for(1ms);
    }
}
```

### 3. 位置（角度）制御モード (`POS_MODE`)
最大制限速度や加速度制限を設定し、目標累積角度を +3600度（出力軸換算で10回転）に制御するサンプルです。

```cpp
#include "mbed.h"
#include "CANManager.h"
#include "rbms.h"
#include <cstdio>

CAN can(PA_11, PA_12, 1000000);
CANManager can_manager(can);
rbms motor(can, true, 1); // M3508、モーター数: 1 (ID: 0x201)

int main() {
    can_manager.add_receiver(&motor);
    motor.spd_control();

    // 制御モードを位置制御に設定
    motor.set_control_mode(0, rbms::POS_MODE);
    
    // 各種制限と目標角度の設定
    motor.set_speed_limit(0, 500.0f);     // 最大制限速度: 500 RPM
    motor.set_accel_limit(0, 1500.0f);    // 加速度制限: 1500 RPM/s
    motor.set_target_angle(0, 3600.0f);   // 目標累積角度: +3600度 (10回転)

    while (true) {
        motor.rbms_send();
        ThisThread::sleep_for(1ms);
    }
}
```
