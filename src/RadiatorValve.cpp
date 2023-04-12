#include <TMCStepper.h>
#include <Wire.h>
#include <RotaryEncoder.h>

#define STALL_VALUE 15    // [0..255]
#define STALL_VALUE_LOW 0 // [0..255]

#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver

#define TEMPADDR 0x4D

// Pins
const uint8_t EN_PIN = GPIO_NUM_17;   // 21
const uint8_t STEP_PIN = GPIO_NUM_32; // 22
const uint8_t DIR_PIN = GPIO_NUM_27;  // 25  
const uint8_t BTN_PIN = GPIO_NUM_35;  // 2 // STRAPPING 
const uint8_t DIAG_PIN = GPIO_NUM_26; // 34
const uint8_t IDX_PIN = GPIO_NUM_25;  // 35

const uint8_t RE_A = GPIO_NUM_36;  // 36  
const uint8_t RE_B = GPIO_NUM_37;  // 39

const uint8_t RE_LR = GPIO_NUM_39;  // 27
const uint8_t RE_LG = GPIO_NUM_39;  // 32
const uint8_t RE_LB = GPIO_NUM_39;  // 33


const uint8_t TEMP_SDA = GPIO_NUM_12; // 26  
const uint8_t TEMP_CLK = GPIO_NUM_13; // 4

const uint8_t UART_RX = GPIO_NUM_21; // 16
const uint8_t UART_TX = GPIO_NUM_22; // 17

const uint8_t SPI_CS = GPIO_NUM_5;       // STRAPPING  
const uint8_t SPI_CLK = GPIO_NUM_18; 
const uint8_t SPI_MISO = GPIO_NUM_19;
const uint8_t SPI_MOSI = GPIO_NUM_23; 





// Directions
#define OPEN LOW
#define CLOSE HIGH
#define NULLPOS -99999

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

RotaryEncoder re(RE_A, RE_B);

using namespace TMC2208_n;

// Globals
bool _stalled = false;
int _stallPos = 0;

int volatile _pos = 0;
int _limit = 0;
int _dir = HIGH;
bool _stopOnStall = true;
int _targetPos = NULLPOS;
int _targetSpeed = 0;

int _avg[] = {0, 0, 0, 0, 0};
int _totalSg;
int _avgPtr = 0;

bool _btnPressed = false;

void setSpeedDir(int speed, int d)
{
    // Serial.printf("Setting speed %d direction %d\n", speed, d);
    _dir = d;
    digitalWrite(DIR_PIN, _dir);
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

void reverse()
{
    _dir = _dir == HIGH ? LOW : HIGH;
}

// ISRs
void IRAM_ATTR onButtonPress()
{
    // ledcWrite(0, 0);
    int b = digitalRead(BTN_PIN);
    if (b == LOW && _btnPressed == false)
    {
        _btnPressed = true;
        reverse();
        setSpeedDir(1000, _dir);
    }
    else if (b == HIGH && _btnPressed == true)
    {
        _btnPressed = false;
    }
}

void IRAM_ATTR onStall()
{
    // ledcWrite(0, 0);
    bool st = digitalRead(DIAG_PIN) == HIGH;
    if (st)
    {
        Serial.println("Stall detected");
        if (!_stopOnStall)
            return;
        setSpeedDir(0, _dir);
        _stalled = true;
        _stallPos = _pos;
        _pos = 0;
    }
    else
    {
        Serial.println("Stall cleared");
    }
}

void IRAM_ATTR onEncoder()
{
    re.tick();
}

void adjustSpeed()
{
    if (_targetPos != NULLPOS && (_dir == CLOSE ? _pos <= _targetPos : _pos >= _targetPos))
    {
        setSpeedDir(_targetSpeed, _dir);
        Serial.printf("HIT TARGET %d Pos %d Dir %s\n", _targetPos, _pos, _dir == OPEN ? "OPEN" : "CLOSE");
        _targetPos = NULLPOS;
    }
    else if (_targetPos != NULLPOS && abs(_pos - _targetPos) < 10)
    {
        setSpeedDir(500, _dir);
    }
    else if (_targetPos != NULLPOS && abs(_pos - _targetPos) < 20)
    {
        setSpeedDir(1000, _dir);
    }
    else if (_targetPos != NULLPOS && abs(_pos - _targetPos) < 40)
    {
        setSpeedDir(1500, _dir);
    }
    else if (_targetPos != NULLPOS && abs(_pos - _targetPos) < 50)
    {
        setSpeedDir(2000, _dir);
    }
    else if (_targetPos != NULLPOS && abs(_pos - _targetPos) < 60)
    {
        setSpeedDir(2500, _dir);
    }
    else
    {
        setSpeedDir(3000, _dir);
    }
}

void IRAM_ATTR onIndex()
{
    if (_dir == CLOSE)
        _pos--;
    else
        _pos++;

    _totalSg -= _avg[_avgPtr];
    _avg[_avgPtr] = driver.SG_RESULT();
    _totalSg += _avg[_avgPtr++];
    _avgPtr = _avgPtr % 5;

    adjustSpeed();
}

void enable()
{
    digitalWrite(EN_PIN, LOW);
}

void disable()
{
    digitalWrite(EN_PIN, HIGH);
}

void initCommon()
{
    driver.toff(3);
    driver.blank_time(24);
    driver.microsteps(32);
    driver.hold_multiplier(0.1);
}

void initTune()
{
    Serial.println("Setting motor paramters for motor tuning");
    initCommon();
    driver.rms_current(1800);  // mA
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.SGTHRS(STALL_VALUE_LOW);
    _stopOnStall = false;
}

void initHoming()
{
    Serial.println("Setting motor paramters for homing");
    initCommon();
    driver.rms_current(1300);  // mA
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.SGTHRS(STALL_VALUE);
    _stopOnStall = true;
}

void initFullPower()
{
    Serial.println("Setting motor paramters for full power");
    initCommon();
    driver.rms_current(1800);  // mA
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.pwm_autoscale(false);
    driver.pwm_autograd(false);
    driver.SGTHRS(STALL_VALUE_LOW);
    _stopOnStall = false;
}

void tuneCurrent()
{
    Serial.println("Tuning current");
    initTune();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setSpeedDir(20000, OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setSpeedDir(20000, CLOSE);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    setSpeedDir(0, OPEN);
}

void setupDriverComms()
{
    SERIAL_PORT.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
    driver.begin();

    pinMode(EN_PIN, OUTPUT);
    pinMode(DIAG_PIN, INPUT_PULLDOWN);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(IDX_PIN, INPUT);

    enable();

    ledcSetup(0, 100, 8);
    ledcAttachPin(STEP_PIN, 0);
    attachInterrupt(DIAG_PIN, onStall, CHANGE);
    attachInterrupt(BTN_PIN, onButtonPress, CHANGE);
    attachInterrupt(IDX_PIN, onIndex, RISING);
}

void resetStall()
{
    Serial.println("Resetting");
    disable();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    enable();
    initFullPower();
    reverse();
    setSpeedDir(1000, _dir);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    setSpeedDir(0, _dir);
    initHoming();
    reverse();
    _stalled = false;
}

int waitForStall()
{
    while (!_stalled)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    resetStall();
    return _stallPos;
}

void moveTo(int tgt, int speedafter = 0)
{
    Serial.printf("Moving from %d to %d\n", _pos, tgt);
    if (tgt > _limit && _limit > 0)
        tgt = _limit;
    if (tgt < 0)
        tgt = 0;
    _targetPos = tgt;
    _targetSpeed = speedafter;
    _dir = _pos < tgt ? OPEN : CLOSE;
    adjustSpeed();
    while (_dir == OPEN ? _pos <= _targetPos : _pos >= _targetPos)
    {
        if (_targetPos == NULLPOS)
            return;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void moveSteps(int steps, int dir, int speedafter = 0)
{
    int t = _pos + (dir == OPEN ? steps : (0 - steps));
    moveTo(t, speedafter);
}

void homeClosed()
{
    Serial.println("Homing closed, fast");
    initHoming();
    // moveSteps(400, CLOSE, 1000);
    // Serial.println("Homing closed");
    setSpeedDir(1000, CLOSE);
    int closedPos = waitForStall();
    Serial.printf("Pos %d\n", _pos);
    Serial.printf("Closed pos: %d\n", closedPos);
    _limit = 120;
}

void startupSequence()
{
    tuneCurrent();
    homeClosed();
}

// Arduino lifecycle
void setup()
{
    Serial.begin(300000); // Init serial port and set baudrate
    while (!Serial)
        ; // Wait for serial port to connect
    Serial.println("\nStart...");

    setupDriverComms();
    startupSequence();

    initFullPower();

    pinMode(RE_A, INPUT);
    pinMode(RE_B, INPUT);
    attachInterrupt(RE_A, onEncoder, CHANGE);
    attachInterrupt(RE_B, onEncoder, CHANGE);


    // Test Openclose
    moveTo(_limit);
    moveTo(5);
    moveTo(_limit);
    moveTo(5);

    Wire.begin(TEMP_SDA, TEMP_CLK);
}

void loop()
{
    static uint32_t last_time = 0;
    uint32_t ms = millis();

    while (Serial.available() > 0)
    {
        int8_t read_byte = Serial.read();
        if (read_byte == 'o')
        {
            moveSteps(5, OPEN);
        }
        if (read_byte == 'c')
        {
            moveSteps(5, CLOSE);
        }
        Serial.println(_pos);
    }

    Wire.beginTransmission(TEMPADDR);
    Wire.write(0x00);

    int temperature;

    Wire.requestFrom(TEMPADDR, 1);
    if (Wire.available())
    {
        temperature = Wire.read();
        //Serial.println(temperature);
        Serial.println(re.getPosition());
        //Serial.printf("%d %d\n", digitalRead(RE_A), digitalRead(RE_B));
    }
    else
    {
        Serial.println("---");
    }
    Wire.endTransmission();

    if ((ms - last_time) > 300)
    { // run every 0.1s
        last_time = ms;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
