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

uint32_t acceleration = 60*MICROSTEPS*GEAR_RATIO;
uint32_t speed = 300*MICROSTEPS*GEAR_RATIO;

const char *SSID = "Bitwäscherei-Bau";
const char *PWD = "clubmate42";
const char *HOSTNAME = "pantilt-robot";

const int UDP_PORT = 9870;
const IPAddress udp_server_ip(172, 31, 33, 194);
WiFiUDP udp;

StaticJsonDocument<1500> telemetry;
MsgPack::Packer packer;

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

PositionSensor pan_sensor;
PositionSensor tilt_sensor;

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

    stepper->setSpeedInHz(400*MICROSTEPS);
    delay(5);
    stepper->runBackward();
    while (sensor->has_changed_state == false) { delay(2); }
    stepper->forceStop();
    sensor->reset();

    printf("one step done %f\n", sensor->last_fixed_position);
    if (sensor->last_fixed_position != 0)
    {
        stepper->setSpeedInHz(400*MICROSTEPS);
        delay(5);
        stepper->runBackward();
        while (sensor->has_changed_state == false) { delay(2); }
    }
    stepper->forceStopAndNewPosition(0);

    printf("done home %f\n", sensor->last_fixed_position);
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

    if (control_mode == CONTROL_MODE_SPEED) {
        float pan_speed = pan_in * 200 * 10 * MICROSTEPS;
        float tilt_speed = tilt_in * 200 * 10 * MICROSTEPS;

        pan_stepper->setSpeedInHz(abs(pan_speed));
        // tilt_stepper->moveByAcceleration(99999, true);
    
        int8_t res;
        if (pan_speed > 0)
        {
            res = pan_stepper->runForward();
        }
        else if (pan_speed < 0)
        {
            res = pan_stepper->runBackward();
        }
        else
        {
            pan_stepper->forceStop();
        }
    }
    else if (control_mode == CONTROL_MODE_POSITION)
    {
        pan_stepper->setAcceleration(100*MICROSTEPS*16);
        pan_stepper->setSpeedInHz(200*MICROSTEPS*8);

        int32_t pan_position = pan_in * 100 * MICROSTEPS * GEAR_RATIO;

        if (abs(pan_position - pan_stepper->getPositionAfterCommandsCompleted()) > 30*MICROSTEPS)
        {
            pan_stepper->moveTo((int32_t)pan_position);
        }

        tilt_stepper->setAcceleration(100*MICROSTEPS*16);
        tilt_stepper->setSpeedInHz(200*MICROSTEPS*8);

        int32_t tilt_position = tilt_in * 100 * MICROSTEPS * GEAR_RATIO;

        if (abs(tilt_position - tilt_stepper->getPositionAfterCommandsCompleted()) > 30*MICROSTEPS)
        {
            tilt_stepper->moveTo((int32_t)tilt_position);
        }
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
                if (cmd == 0)
                {
                    Serial.println("home");
                    home_axis(tilt_stepper, &tilt_sensor);
                    home_axis(pan_stepper, &pan_sensor);
                }
                else if (cmd == 1)
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
