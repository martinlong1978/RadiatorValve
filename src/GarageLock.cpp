#define USE_ESP_IDF_GPIO 1

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include "SECRETS.h"
#include <Update.h>
#include <TMC2209.h>

#define NO_LOCKS 1
#define SUBJECT "garagelock/feeds/onoff"
#define STATUSSUBJECT "garagelock/feeds/onoff/state"
#define UPDATESUBJECT "garagelock/feeds/updates"

//#define CONFIGMODE

HardwareSerial &serial_stream = Serial2;

const long SERIAL_BAUD_RATE = 115200;
const int32_t RUN_VELOCITY = 20000;
const int32_t STOP_VELOCITY = 0;
uint8_t current = 100;
const uint8_t HOLD_CURRENT_PERCENT = 100;
const uint8_t HOLD_CURRENT_PERCENT_REDUCED = 100;

const uint8_t DIR = GPIO_NUM_27;
const uint8_t DIAG = GPIO_NUM_26;
const uint8_t BTN = GPIO_NUM_35;

// Instantiate TMC2209
TMC2209 stepper_driver;
bool invert_direction = false;
char c;

const int count = 10;

int freq = 100;

void read_loop(void *parameters)
{
    int vals[count];
    int i = 0;
    for (;;)
    {
        vals[i] = stepper_driver.getStallGuardResult();
        TMC2209::Status s = stepper_driver.getStatus();
        stepper_driver.getSettings().
        if (++i >= count)
        {
            i = 0;
            int t = 0;
            for (int j = 0; j < count; j++)
            {
                t += vals[j];
            }
            Serial.printf("s = %d %d %d %d %d\n", t / count, s.stealth_chop_mode, s.current_scaling, s.standstill, s.standstill);
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void main_loop(void *parameters)
{
    // stepper_driver.moveAtVelocity(RUN_VELOCITY);

    disableCore0WDT();
    ledcWrite(0, 128);
    for (;;)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        stepper_driver.disable();
        int c = digitalRead(DIR);
        int n = (c == HIGH ? LOW : HIGH);
        digitalWrite(DIR, n);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        stepper_driver.enable();
    }
}

void IRAM_ATTR Stall()
{
    // ledcWrite(0, 0);
    Serial.println("Stall detected");
}

bool pressed = false;

void IRAM_ATTR ButtonPress()
{
    // ledcWrite(0, 0);
    int b = digitalRead(BTN);
    if (b == LOW && pressed == false)
    {
        pressed = true;
        current += 10;
        if (current > 100)
            current = 1;
    }
    else if (b == HIGH && pressed == true)
    {
        pressed = false;
    }
}

void setSpeedDir(int speed, int dir)
{
    digitalWrite(DIR, dir);
    ledcSetup(0, speed, 8);
    if (speed > 0)
    {
        ledcWrite(0, 128);
    }
    else
    {
        ledcWrite(0, 0);
    }
}

void AutoConfig()
{
    stepper_driver.setMicrostepsPerStep(2);
    stepper_driver.disableStealthChop();
    stepper_driver.setRunCurrent(100);
    stepper_driver.moveUsingStepDirInterface();

    vTaskDelay(200 / portTICK_PERIOD_MS);
    stepper_driver.enable();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    setSpeedDir(1000, HIGH);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    setSpeedDir(1000, LOW);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    setSpeedDir(0, HIGH);
}

void enableStealthChop()
{
    stepper_driver.disable();
    stepper_driver.setMicrostepsPerStep(16);
    stepper_driver.enableStealthChop();
    stepper_driver.enableAutomaticCurrentScaling();
    stepper_driver.enableAutomaticGradientAdaptation();
    stepper_driver.setRunCurrent(100);
    stepper_driver.enable();
}

void configureHoming()
{
    stepper_driver.disable();
    stepper_driver.setMicrostepsPerStep(8);
    stepper_driver.enableStealthChop();
    stepper_driver.disableCoolStep();
    stepper_driver.enableAutomaticCurrentScaling();
    stepper_driver.enableAutomaticGradientAdaptation();
    stepper_driver.setRunCurrent(100);
    stepper_driver.setStallGuardThreshold(100);
    stepper_driver.enable();
}

void setup()
{

    Serial.begin(300000);
    serial_stream.begin(115200, SERIAL_8N1, 21, 22);

    Serial.println("\n\nInitialising\n\n");

    pinMode(17, OUTPUT);
    pinMode(DIAG, INPUT_PULLDOWN);
    pinMode(DIR, OUTPUT);
    pinMode(BTN, INPUT_PULLUP);
    digitalWrite(17, LOW);

    ledcSetup(0, 100, 8);
    ledcAttachPin(32, 0);

    stepper_driver.setup(serial_stream);

    if (stepper_driver.isSetupAndCommunicating())
    {
        Serial.println("Stepper driver setup and communicating!");
        Serial.println("");
    }
    else
    {
        Serial.println("Stepper driver not setup and communicating!");
        return;
    }

    AutoConfig();
    enableStealthChop();
    configureHoming();

    setSpeedDir(1000, HIGH);


    attachInterrupt(DIAG, Stall, CHANGE);
    attachInterrupt(BTN, ButtonPress, CHANGE);

    xTaskCreate(
        main_loop,
        "Main",
        2000,
        NULL,
        1,
        NULL);

    xTaskCreate(
        read_loop,
        "Read",
        2000,
        NULL,
        1,
        NULL);
}

void loop()
{
}
