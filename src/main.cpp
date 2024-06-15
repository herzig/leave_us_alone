#include <Arduino.h>

#include <WiFi.h>
#include <WebServer.h>

#include "ArduinoJson.h"

#include "FastAccelStepper.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"

// #include "MsgPacketizer.h"
#include "MsgPack.h"

#include "LowpassFilter.hpp"
#include "PositionSensor.hpp"


#define B
// #define A

const char *SSID = "Bitwäscherei-Bau";
const char *PWD = "clubmate42";

#ifdef B 
const char *HOSTNAME = "pantilt-robot_b";
#endif

#ifdef  A
A const char *HOSTNAME = "pantilt-robot_a";
#endif

const uint8_t PAN_EN_PIN = 23;
const uint8_t PAN_DIR_PIN = 21;
const uint8_t PAN_PULSE_PIN = 19;

const uint8_t TILT_EN_PIN = 18;
const uint8_t TILT_DIR_PIN = 5;
const uint8_t TILT_PULSE_PIN = 17;

const uint8_t PAN_POT_PIN = 36;
const uint8_t TILT_POT_PIN = 39;

const uint8_t PAN_SENS_IN = 34;
const uint8_t TILT_SENS_IN = 35;

const uint16_t SENS_THRESH = 3000;
const uint8_t MICROSTEPS = 16;
const float GEAR_RATIO = 4*4;

uint32_t acceleration = 60.0 / 360 * 200 * MICROSTEPS*GEAR_RATIO;
uint32_t speed = 100.0 / 360 * 200 * MICROSTEPS*GEAR_RATIO;

int32_t MAX_ACCEL = 90.0 / 360 * 200 * MICROSTEPS*GEAR_RATIO;
int32_t MAX_SPEED = 100.0 / 360 * 200 * MICROSTEPS*GEAR_RATIO;

const int UDP_PORT = 9870;
const IPAddress udp_server_ip(172, 31, 33, 194);
WiFiUDP udp;

StaticJsonDocument<1500> telemetry;
MsgPack::Packer packer;

enum MSG_COMMAND { HOME = 0, PT = 1, PT_ACCEL = 2, P_ACCEL = 3, T_ACCEL = 4};
StaticJsonDocument<500> rcv_json;
uint8_t udp_rcv_buffer[1500];
MsgPack::Unpacker unpacker;

const uint64_t SENSOR_PERIOD_US = 5000;

enum ControlMode { CONTROL_MODE_UDP, CONTROL_MODE_POSITION, CONTROL_MODE_SPEED, };
ControlMode control_mode = CONTROL_MODE_UDP;

FastAccelStepperEngine engine = FastAccelStepperEngine();

FastAccelStepper* pan_stepper = nullptr;
FastAccelStepper* tilt_stepper = nullptr;

LowpassFilter pan_pot_filter(SENSOR_PERIOD_US/1e6f, 2);
LowpassFilter tilt_pot_filter(SENSOR_PERIOD_US/1e6f, 2);
float pan_in = 0;
float tilt_in = 0;

#ifdef B
PositionSensor pan_sensor(2000, 3500);
PositionSensor tilt_sensor(1300, 3500);
float base_pan = 90;
float base_tilt = 90;
#endif
#ifdef A
PositionSensor pan_sensor();
PositionSensor tilt_sensor();
float base_pan = 90;
float base_tilt = 270;
#endif


FastAccelStepper* init_stepper(uint8_t pulse_pin, uint8_t dir_pin, uint8_t en_pin)
{
    auto stepper = engine.stepperConnectToPin(pulse_pin);

    if (stepper) 
    {
       stepper->setDirectionPin(dir_pin);
       stepper->setEnablePin(en_pin);
       stepper->setAutoEnable(false);

       stepper->enableOutputs();

       stepper->setAcceleration(INT32_MAX);
       stepper->move(0);
    }
    return stepper;
}

void home_axis(FastAccelStepper* stepper, PositionSensor* sensor)
{
    printf("start home\n");

    stepper->setSpeedInHz(100*MICROSTEPS);
    delay(5);
    stepper->runBackward();
    while (sensor->has_changed_state == false) { delay(2); }
    stepper->forceStop();
    sensor->reset();

    printf("one step done %f\n", sensor->last_fixed_position);
    if (sensor->last_fixed_position != 0)
    {
        stepper->setSpeedInHz(100*MICROSTEPS);
        delay(5);
        stepper->runBackward();
        while (sensor->has_changed_state == false) { delay(2); }
    }
    stepper->forceStopAndNewPosition(0);

    printf("done home %f\n");
}

void home_and_move_to_start()
{
    home_axis(tilt_stepper, &tilt_sensor);
    home_axis(pan_stepper, &pan_sensor);

    pan_stepper->setAcceleration(acceleration);
    pan_stepper->setSpeedInHz(speed);
    tilt_stepper->setAcceleration(acceleration);
    tilt_stepper->setSpeedInHz(speed);

    int32_t pan_position = base_pan / 360 * 200 * MICROSTEPS * GEAR_RATIO;
    pan_stepper->moveTo(pan_position, false);

    int32_t tilt_position = base_tilt / 360 * 200 * MICROSTEPS * GEAR_RATIO;
    tilt_stepper->moveTo(tilt_position, true);

    pan_stepper->forceStopAndNewPosition(0);
    tilt_stepper->forceStopAndNewPosition(0);
}

void timer_tick(void* arg)
{
    uint16_t pan_sens = analogRead(PAN_SENS_IN);
    int32_t pan_speed = pan_stepper->getCurrentSpeedInMilliHz();
    pan_sensor.update(pan_sens, pan_speed);

    uint16_t tilt_sens = analogRead(TILT_SENS_IN);
    int32_t tilt_speed = tilt_stepper->getCurrentSpeedInMilliHz();
    tilt_sensor.update(tilt_sens, tilt_speed);

    pan_in = pan_pot_filter.filter(analogRead(PAN_POT_PIN) / 4096.0f * 2 - 1);
    tilt_in = tilt_pot_filter.filter(analogRead(TILT_POT_PIN) / 4096.0f * 2 - 1);
}

void setup() 
{
    pinMode(PAN_POT_PIN, INPUT);
    pinMode(TILT_POT_PIN, INPUT);

    pinMode(PAN_DIR_PIN, OUTPUT);
    pinMode(PAN_EN_PIN, OUTPUT);
    pinMode(PAN_PULSE_PIN, OUTPUT);

    pinMode(PAN_SENS_IN, INPUT);

    engine.init();
    pan_stepper = init_stepper(PAN_PULSE_PIN, PAN_DIR_PIN, PAN_EN_PIN);
    tilt_stepper = init_stepper(TILT_PULSE_PIN, TILT_DIR_PIN, TILT_EN_PIN);

    Serial.begin(115200);

    // auto sens_publisher = MsgPacketizer::publish(Serial, PAN_SENSOR_CHANNEL, filtered_pan_sens, raw_pan_sens);
    // sens_publisher->setFrameRate(60);

    // home_pan_axis();

    // sens timer
    esp_timer_create_args_t timer_args;
    timer_args.callback =  timer_tick;
    timer_args.name = "sensor timer";
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, SENSOR_PERIOD_US));
    // ESP_LOGI(TAG, "started sensor timer");

    // WIFI
    Serial.print("Connecting to ");
    Serial.println(SSID);
    WiFi.setHostname(HOSTNAME);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PWD);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
    }
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());

    udp.setTimeout(200);
    udp.begin(UDP_PORT);

    delay(50);
    pan_sensor.has_changed_state = false;
    //home_pan_axis();

}

void loop() 
{
    //MsgPacketizer::update();

    if (abs(pan_in) < 0.08f) pan_in = 0;
    if (abs(tilt_in) < 0.08f) tilt_in = 0;

    if (control_mode == CONTROL_MODE_SPEED) 
    {
    }
    else if (control_mode == CONTROL_MODE_POSITION)
    {
    }
    else if (control_mode == CONTROL_MODE_UDP)
    {

        pan_stepper->setAcceleration(acceleration);
        pan_stepper->setSpeedInHz(speed);

        tilt_stepper->setAcceleration(acceleration);
        tilt_stepper->setSpeedInHz(speed);

        unpacker.clear();
        if (udp.parsePacket()) 
        {
            Serial.println("new packet");
            int bytes = udp.read(udp_rcv_buffer, 1500);
            Serial.println(bytes);
            Serial.println((char*)udp_rcv_buffer);
            unpacker.feed(udp_rcv_buffer, 1500);

            uint8_t cmd;
            MsgPack::arr_t<float> prm;

            if (unpacker.from_array(cmd, prm))
            {
                Serial.println(cmd);
                if (cmd == MSG_COMMAND::HOME)
                {
                    Serial.println("home");
                    home_and_move_to_start();
                }
                else if (cmd == MSG_COMMAND::PT)
                {
                    Serial.print("move");
                    float pan = prm[0];
                    float tilt = prm[1];
                    Serial.print(pan); Serial.print(" ");
                    Serial.println(tilt);

                    int32_t pan_position = pan / 360 * 200 * MICROSTEPS * GEAR_RATIO;
                    pan_stepper->moveTo(pan_position);

                    int32_t tilt_position = tilt / 360 * 200 * MICROSTEPS * GEAR_RATIO;
                    tilt_stepper->moveTo(tilt_position);
                }
                else if (cmd == MSG_COMMAND::PT_ACCEL)
                {
                    Serial.print("move accel");
                    float pan = prm[0];
                    float tilt = prm[1];
                    float pan_accel = prm[2];
                    float tilt_accel = prm[3];

                    if (pan > 360 || pan < -360)
                        pan = 0;
                    if (tilt > 360 || tilt < -360)
                        tilt = 0;

                    int32_t pan_position = pan / 360 * 200 * MICROSTEPS * GEAR_RATIO;
                    pan_stepper->setAcceleration(pan_accel / 360 * 200 * MICROSTEPS * GEAR_RATIO);
                    pan_stepper->setSpeedInHz(UINT32_MAX);
                    pan_stepper->moveTo(pan_position);

                    int32_t tilt_position = tilt / 360 * 200 * MICROSTEPS * GEAR_RATIO;
                    tilt_stepper->setAcceleration(tilt_accel / 360 * 200 * MICROSTEPS * GEAR_RATIO);
                    tilt_stepper->setSpeedInHz(UINT32_MAX);
                    tilt_stepper->moveTo(tilt_position);

                }
                else if (cmd == MSG_COMMAND::P_ACCEL)
                {
                    float pos = prm[0];
                    float accel = prm[1];

                    if (pos > 360 || pos < -360)
                        pos = 0;

                    int32_t position = pos / 360 * 200 * MICROSTEPS * GEAR_RATIO;
                    pan_stepper->setAcceleration(min(MAX_ACCEL, (int32_t)(accel  / 360 * 200 * MICROSTEPS * GEAR_RATIO)));
                    pan_stepper->setSpeedInHz(MAX_SPEED);
                    pan_stepper->moveTo(position);
                }
                else if (cmd == MSG_COMMAND::T_ACCEL)
                {
                    float pos = prm[0];
                    float accel = prm[1];

                    if (pos > 360 || pos < -360)
                        pos = 0;

                    int32_t position = pos / 360 * 200 * MICROSTEPS * GEAR_RATIO;
                    tilt_stepper->setAcceleration(min(MAX_ACCEL, (int32_t)(accel  / 360 * 200 * MICROSTEPS * GEAR_RATIO)));
                    tilt_stepper->setSpeedInHz(MAX_SPEED);
                    tilt_stepper->moveTo(position);
                }
            }
        }

        udp.flush();

    }

    telemetry["timestamp"] = millis()/1000.0f;
    telemetry["pan_pos"] = pan_stepper->getCurrentPosition() / (200*MICROSTEPS*GEAR_RATIO) * 360;
    telemetry["tilt_pos"] = tilt_stepper->getCurrentPosition() / (200*MICROSTEPS*GEAR_RATIO) * 360;

    telemetry["pan_sens"] = pan_sensor.raw_val;
    telemetry["pan_sens_filtered"] = pan_sensor.filtered_val;
    telemetry["pan_sens_state"] = pan_sensor.state;
    telemetry["pan_sens_position"] = tilt_sensor.last_fixed_position;

    telemetry["tilt_sens"] = tilt_sensor.raw_val;
    telemetry["tilt_sens_filtered"] = tilt_sensor.filtered_val;
    telemetry["tilt_sens_state"] = tilt_sensor.state;
    telemetry["tilt_sens_position"] = tilt_sensor.last_fixed_position;

    packer.clear();
    packer.serialize_arduinojson(telemetry);

    udp.beginPacket(udp_server_ip, UDP_PORT);
    udp.write(packer.data(), packer.size());
    udp.endPacket();

    // printf("%d %s %d \n", udp_client.remotePort(), udp_client.remoteIP().toString(), packer.size());

    // printf("%f %fHz %d \n", pan_in, pan_speed, res);
    // printf("%d %d \n", pan_stepper->getCurrentPosition(), tilt_stepper->getCurrentPosition());
    delay(1);
}
