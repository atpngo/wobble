#include <Arduino.h>
#include "Wire.h"
#include "imu.h"
#include "encoder.h"
#include "communication.h"
#include "motor.h"
#include "pid.h"
#include <algorithm>
// #include <ArduinoEigen.h>

//
// Pin Declarations
//
// Left motor
const int LEFT_ENC_DT = 18;
const int LEFT_ENC_CLK = 19;
const int LEFT_MOTOR_A = 4;
const int LEFT_MOTOR_B = 0;
const int LEFT_MOTOR_ENABLE = 16;
// Right motor
const int RIGHT_ENC_DT = 33;
const int RIGHT_ENC_CLK = 32;
const int RIGHT_MOTOR_A = 25;
const int RIGHT_MOTOR_B = 26;
const int RIGHT_MOTOR_ENABLE = 27;

//
// Robot objects
//
Encoder left_wheel_encoder(LEFT_ENC_CLK, LEFT_ENC_DT);
Encoder right_wheel_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT);
IMU imu;
Motor left_motor(LEFT_MOTOR_A, LEFT_MOTOR_B, LEFT_MOTOR_ENABLE);
Motor right_motor(RIGHT_MOTOR_A, RIGHT_MOTOR_B, RIGHT_MOTOR_ENABLE);

//
// Communication between robot and transceiver
//
uint8_t peer[6] = {0xEC, 0x64, 0xC9, 0x85, 0x6F, 0x84};
Communication socket;
Packet in_;
Command data_;

//
// Controller
//
PID controller(10.0, 0.0, 0.0);
portMUX_TYPE trimMux = portMUX_INITIALIZER_UNLOCKED;
volatile float trim_ = 0.0f;

inline float getTrim()
{
    portENTER_CRITICAL(&trimMux);
    float t = trim_;
    portEXIT_CRITICAL(&trimMux);
    return t;
}

void handleIncoming(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int len)
{
    memcpy(&in_, data, len);
    if (in_.type == Message::Command)
    {
        memcpy(&data_, in_.payload, in_.len);
        portENTER_CRITICAL(&trimMux);
        trim_ = data_.trim;
        portEXIT_CRITICAL(&trimMux);
        // left_motor.spin(data_.left_wheel_power);
        // right_motor.spin(data_.right_wheel_power);
    }
}

// Tasks
void BalanceTask(void *pvParameters)
{
    (void)pvParameters;
    unsigned long prev_time = millis();
    unsigned long dt, now;
    for (;;)
    {
        // Calculate DT
        now = millis();
        dt = now - prev_time;
        prev_time = now;

        // Update IMU
        imu.calculateAngles();
        float pitch = imu.get_pitch() + getTrim();

        double signal = controller.get_signal(pitch, 0, dt);
        int sign = (signal >= 0) ? 1 : -1;
        signal = sign * (abs(signal) + 40); // off offset
        int power = std::clamp(int(signal), -255, 255);
        left_motor.spin(power);
        right_motor.spin(power);
        vTaskDelay(pdMS_TO_TICKS(1)); // 200 Hz
    }
}

// Telemetry at 10 Hz
void TelemetryTask(void *pvParameters)
{
    (void)pvParameters;
    Packet out_;
    for (;;)
    {
        Telemetry t{
            .pitch = imu.get_pitch(),
            .yaw = imu.get_yaw(),
            .left_enc = left_wheel_encoder.read(),
            .right_enc = right_wheel_encoder.read(),
        };
        out_.type = Message::Telemetry;
        memcpy(out_.payload, &t, sizeof(t));
        out_.len = sizeof(t);
        socket.send(peer, out_);
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    right_motor.set_type(MotorType::MAIN);
    left_motor.set_type(MotorType::SECONDARY);
    imu.init();
    left_wheel_encoder.init();
    right_wheel_encoder.init();
    socket.init();
    socket.addPeer(peer);
    socket.onReceive(handleIncoming);

    // create tasks, pin to core 1 (ESP32)
    xTaskCreatePinnedToCore(
        BalanceTask, "Balance", 4096, nullptr,
        1, nullptr, 1);
    xTaskCreatePinnedToCore(
        TelemetryTask, "Telemetry", 4096, nullptr,
        1, nullptr, 0);
}

void loop() { vTaskDelete(NULL); }