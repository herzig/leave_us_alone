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
#define ESP32_POE_ISO
#define USE_ETH

const int UDP_PORT = 9870;
const int TCP_PORT = 9870;

#ifdef USE_ETH
#include <ETH.h>
#ifdef A
const IPAddress ip(10, 1, 1, 11);
#else 
const IPAddress ip(10, 1, 1, 12);
#endif
const IPAddress subnet(255,255,255,0);
const IPAddress udp_server_ip(10, 1, 1, 2);
#else
#include <WiFi.h>
const IPAddress udp_server_ip(172, 31, 33, 194);
#endif

WiFiUDP udp;

const char *SSID = "Bitwäscherei-Bau";
const char *PWD = "clubmate42";

#ifdef B 
const char *HOSTNAME = "pantilt-robot_b";
#endif

#ifdef  A
const char *HOSTNAME = "pantilt-robot_a";
#endif

#ifdef ESP32_POE_ISO
const uint8_t PAN_EN_PIN = 33;
const uint8_t PAN_DIR_PIN = 32;
const uint8_t PAN_PULSE_PIN = 16;

const uint8_t TILT_EN_PIN = 15;
const uint8_t TILT_DIR_PIN = 14;
const uint8_t TILT_PULSE_PIN = 13;

const uint8_t PAN_SENS_IN = 36;
const uint8_t TILT_SENS_IN = 35;
#else
const uint8_t PAN_EN_PIN = 23;
const uint8_t PAN_DIR_PIN = 21;
const uint8_t PAN_PULSE_PIN = 19;

const uint8_t TILT_EN_PIN = 18;
const uint8_t TILT_DIR_PIN = 5;
const uint8_t TILT_PULSE_PIN = 17;

const uint8_t PAN_SENS_IN = 34;
const uint8_t TILT_SENS_IN = 35;
#endif

const uint16_t SENS_THRESH = 3000;
const uint8_t MICROSTEPS = 16;
const float GEAR_RATIO = 4*4;

const float DEG2STEPS = 1.0f / 360.0f * 200.0f * MICROSTEPS * GEAR_RATIO;
const float STEPS2DEG = 1.0f / DEG2STEPS;

const int32_t DEFAULT_ACCEL = 50.0f;
const int32_t DEFAULT_SPEED = 60.0f;

const float MAX_PAN = 540.0f;
const float MAX_TILT = 540.0f;

const float MAX_ACCEL = 90.0f; 
const float MAX_SPEED = 100.0f;

// WiFiServer tcp(TCP_PORT);
//NetworkServer server;

StaticJsonDocument<2000> telemetry;
MsgPack::Packer packer;

enum MSG_COMMAND { HOME = 0, PT = 1, PT_ACCEL = 2, PAN_ACCEL = 3, TILT_ACCEL = 4};
StaticJsonDocument<500> rcv_json;
uint8_t udp_rcv_buffer[1500];
MsgPack::Unpacker unpacker;

const uint64_t SENSOR_PERIOD_US = 5000;

enum ControlMode { CONTROL_MODE_UDP };
ControlMode control_mode = CONTROL_MODE_UDP;

FastAccelStepperEngine engine = FastAccelStepperEngine();

FastAccelStepper* pan_stepper = nullptr;
FastAccelStepper* tilt_stepper = nullptr;

unsigned long last_telemetry_time = 0;

#ifdef B
PositionSensor pan_sensor(3100, 3500);
PositionSensor tilt_sensor(2500, 3500);
float base_pan = 90;
float base_tilt = 90;
#endif

#ifdef A
PositionSensor pan_sensor(3800, 4000);
PositionSensor tilt_sensor(2800, 3500);
float base_pan = 90;
float base_tilt = 90;
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
    stepper->setSpeedInHz(9 * DEG2STEPS);
    delay(5);
    stepper->runBackward();
    while (sensor->has_changed_state == false) { delay(2); }
    stepper->forceStopAndNewPosition(sensor->last_fixed_position * DEG2STEPS);
    sensor->reset();
    
    // printf("one step done %f\n", sensor->last_fixed_position);
    // if (sensor->last_fixed_position != 0)
    // {
    //     stepper->setSpeedInHz(9 * DEG2STEPS);
    //     delay(5);
    //     stepper->runBackward();
    //     while (sensor->has_changed_state == false) { delay(2); }
    // }
    // stepper->forceStopAndNewPosition(0);

    // printf("done home %f\n");
}

void set_speed_accel(FastAccelStepper* stepper, float speed, float accel)
{
    stepper->setSpeedInHz(speed * DEG2STEPS);
    stepper->setAcceleration(accel * DEG2STEPS);
}

void home_and_move_to_start()
{
    home_axis(tilt_stepper, &tilt_sensor);
    home_axis(pan_stepper, &pan_sensor);

    set_speed_accel(pan_stepper, DEFAULT_SPEED, DEFAULT_ACCEL);
    set_speed_accel(tilt_stepper, DEFAULT_SPEED, DEFAULT_ACCEL);

    pan_stepper->moveTo(base_pan * DEG2STEPS, false);
    tilt_stepper->moveTo(base_tilt * DEG2STEPS, true);

    delay(1);
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
}

// WARNING: WiFiEvent is called from a separate FreeRTOS task (thread)!
void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      // The hostname must be set after the interface is started, but needs
      // to be set before DHCP, so set it from the event handler thread.
      ETH.setHostname("esp32-ethernet");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("ETH Got IP");
      //ETH.printInfo(Serial);
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      break;
    default:
      break;
  }
}

void setup() 
{
    pinMode(PAN_DIR_PIN, OUTPUT);
    pinMode(PAN_EN_PIN, OUTPUT);
    pinMode(PAN_PULSE_PIN, OUTPUT);
    pinMode(TILT_DIR_PIN, OUTPUT);
    pinMode(TILT_EN_PIN, OUTPUT);
    pinMode(TILT_PULSE_PIN, OUTPUT);

    pinMode(PAN_SENS_IN, INPUT);
    pinMode(TILT_SENS_IN, INPUT);

    engine.init();
    pan_stepper = init_stepper(PAN_PULSE_PIN, PAN_DIR_PIN, PAN_EN_PIN);
    tilt_stepper = init_stepper(TILT_PULSE_PIN, TILT_DIR_PIN, TILT_EN_PIN);

    Serial.begin(115200);

    // sens timer
    esp_timer_create_args_t timer_args;
    timer_args.callback =  timer_tick;
    timer_args.name = "sensor timer";
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, SENSOR_PERIOD_US));
    // ESP_LOGI(TAG, "started sensor timer");

    // WIFI
    #ifdef USE_ETH
    delay(500);
    WiFi.onEvent(WiFiEvent);  // Will call WiFiEvent() from another thread.
    delay(500);
    ETH.begin();
    delay(500);
    ETH.config(ip, (uint32_t)0, subnet);
    Serial.print("ETH Ip addr:"); Serial.println(ETH.localIP());
    #else
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
    #endif

    udp.setTimeout(200);
    udp.begin(UDP_PORT);

    delay(50);
    pan_sensor.has_changed_state = false;
    //home_pan_axis();
}

void loop() 
{
    if (control_mode == CONTROL_MODE_UDP)
    {
        if (udp.parsePacket()) 
        {
            Serial.println("new packet");
            int bytes = udp.read(udp_rcv_buffer, 1500);
            Serial.println(bytes);
            Serial.println((char*)udp_rcv_buffer);
            unpacker.feed(udp_rcv_buffer, bytes);

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
                    Serial.print(pan); Serial.print(" "); Serial.println(tilt);

                    set_speed_accel(pan_stepper, DEFAULT_SPEED, DEFAULT_ACCEL);
                    set_speed_accel(tilt_stepper, DEFAULT_SPEED, DEFAULT_ACCEL);

                    if (abs(pan) < MAX_PAN)
                    {
                        pan_stepper->moveTo(pan * DEG2STEPS);
                    }
                    if (abs(tilt) < MAX_TILT)
                    {
                        tilt_stepper->moveTo(tilt * DEG2STEPS);
                    }
                }
                else if (cmd == MSG_COMMAND::PT_ACCEL)
                {
                    Serial.print("move accel");
                    float pan = prm[0];
                    float tilt = prm[1];
                    float pan_accel = prm[2];
                    float tilt_accel = prm[3];

                    if (abs(pan) < MAX_PAN)
                    {
                        set_speed_accel(pan_stepper, MAX_SPEED, pan_accel);
                        pan_stepper->moveTo(pan * DEG2STEPS);
                    }

                    if (abs(tilt) < MAX_TILT)
                    {
                        set_speed_accel(tilt_stepper, MAX_SPEED, tilt_accel);
                        tilt_stepper->moveTo(tilt * DEG2STEPS);
                    }

                }
                else if (cmd == MSG_COMMAND::PAN_ACCEL)
                {
                    float pos = prm[0];
                    float accel = prm[1];

                    if (abs(pos) < MAX_PAN)
                    {
                        set_speed_accel(pan_stepper, MAX_SPEED, accel);
                        pan_stepper->moveTo(pos * DEG2STEPS);
                    }
                }
                else if (cmd == MSG_COMMAND::TILT_ACCEL)
                {
                    float pos = prm[0];
                    float accel = prm[1];

                    if (abs(pos) < MAX_PAN)
                    {
                        set_speed_accel(tilt_stepper, MAX_SPEED, accel);
                        tilt_stepper->moveTo(pos * DEG2STEPS);
                    }
                }
            }
        }
        udp.flush();
        unpacker.clear();

    }

    if (millis() - last_telemetry_time > 50)
    {
        telemetry["timestamp"] = millis()/1000.0f;
        telemetry["pan"] = pan_stepper->getCurrentPosition() * STEPS2DEG;
        // telemetry["pan_t"] = pan_stepper->targetPos() * STEPS2DEG;
        // telemetry["pan_accel"] = pan_stepper->getAcceleration();
        telemetry["tilt"] = tilt_stepper->getCurrentPosition() * STEPS2DEG;
        // telemetry["tilt_t"] = tilt_stepper->targetPos() * STEPS2DEG;
        // telemetry["tilt_accel"] = tilt_stepper->getAcceleration();

        //telemetry["pan_sens"] = pan_sensor.raw_val;
        telemetry["pan_sens_f"] = pan_sensor.filtered_val;
        telemetry["pan_sens_state"] = pan_sensor.state;
        telemetry["pan_sens_position"] = pan_sensor.last_fixed_position;

        // telemetry["tilt_sens"] = tilt_sensor.raw_val;
        telemetry["tilt_sens_f"] = tilt_sensor.filtered_val;
        telemetry["tilt_sens_state"] = tilt_sensor.state;
        telemetry["tilt_sens_position"] = tilt_sensor.last_fixed_position;
        //telemetry["mem"] = esp_get_free_heap_size();
        //telemetry["contig"] = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);

        packer.clear();
        packer.serialize_arduinojson(telemetry);

        udp.beginPacket(udp_server_ip, UDP_PORT);
        udp.write(packer.data(), packer.size());
        udp.endPacket();
        last_telemetry_time = millis();
    }
    // printf("%d %s %d \n", udp_client.remotePort(), udp_client.remoteIP().toString(), packer.size());

    // printf("%f %fHz %d \n", pan_in, pan_speed, res);
    // printf("%d %d \n", pan_stepper->getCurrentPosition(), tilt_stepper->getCurrentPosition());
    delay(1);
}
