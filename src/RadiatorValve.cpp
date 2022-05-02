#include <TMCStepper.h>

#define STALL_VALUE 15    // [0..255]
#define STALL_VALUE_LOW 0 // [0..255]

#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver

// Pins
const uint8_t EN_PIN = GPIO_NUM_17;
const uint8_t STEP_PIN = GPIO_NUM_32;
const uint8_t DIR_PIN = GPIO_NUM_27;
const uint8_t DIAG_PIN = GPIO_NUM_26;
const uint8_t BTN_PIN = GPIO_NUM_35;
const uint8_t IDX_PIN = GPIO_NUM_25;

// Directions
#define OPEN LOW
#define CLOSE HIGH

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

using namespace TMC2208_n;

// Globals
bool _stalled = false;
int _stallPos = 0;

int _pos = 0;
int _dir = HIGH;
bool _stopOnStall = true;
int _targetPos = 0;
int _targetSpeed = 0;

int _avg[] = {0, 0, 0, 0, 0};
int _avgPtr = 0;

bool _btnPressed = false;

void setSpeedDir(int speed, int d)
{
    Serial.printf("Setting speed %d direction %d\n", speed, d);
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

void IRAM_ATTR ButtonPress()
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

void enable()
{
    digitalWrite(EN_PIN, LOW);
}

void disable()
{
    digitalWrite(EN_PIN, HIGH);
}

void init_common()
{
    driver.toff(3);
    driver.blank_time(24);
    driver.microsteps(16);
}

void init_tune()
{
    Serial.println("Setting motor paramters for motor tuning");
    init_common();
    driver.rms_current(1800);  // mA
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.SGTHRS(STALL_VALUE_LOW);
    _stopOnStall = false;
}

void init_homing()
{
    Serial.println("Setting motor paramters for homing");
    init_common();
    driver.rms_current(1300);  // mA
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.SGTHRS(STALL_VALUE);
    _stopOnStall = true;
}

void init_full()
{
    Serial.println("Setting motor paramters for full power");
    init_common();
    driver.rms_current(1800);  // mA
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.SGTHRS(STALL_VALUE_LOW);
    _stopOnStall = false;
}

void tune()
{
    Serial.println("Tuning current");
    init_tune();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setSpeedDir(10000, OPEN);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    setSpeedDir(10000, CLOSE);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    setSpeedDir(0, OPEN);
}

void IRAM_ATTR Stall()
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

void IRAM_ATTR Index()
{
    if (_dir == HIGH)
        _pos--;
    else
        _pos++;
    if (_targetPos != 0 && (_dir == CLOSE ? _pos <= _targetPos : _pos >= _targetPos))
    {
        setSpeedDir(_targetSpeed, _dir);
        Serial.printf("HIT TARGET %d Pos %d\n", _targetPos, _pos);
        _targetPos = 0;
    }
    else if (_targetPos != 0 && abs(_pos - _targetPos) < 20)
    {
        setSpeedDir(1000, _dir);
    }
    else if (_targetPos != 0 && abs(_pos - _targetPos) < 50)
    {
        setSpeedDir(3000, _dir);
    }
}

void setup_driver()
{
    SERIAL_PORT.begin(115200, SERIAL_8N1, 21, 22);
    driver.begin();

    pinMode(EN_PIN, OUTPUT);
    pinMode(DIAG_PIN, INPUT_PULLDOWN);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(IDX_PIN, INPUT);

    enable();

    ledcSetup(0, 100, 8);
    ledcAttachPin(STEP_PIN, 0);
    attachInterrupt(DIAG_PIN, Stall, CHANGE);
    attachInterrupt(BTN_PIN, ButtonPress, CHANGE);
    attachInterrupt(IDX_PIN, Index, RISING);
}

void resetStall()
{
    Serial.println("Resetting");
    disable();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    enable();
    init_full();
    reverse();
    setSpeedDir(1000, _dir);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    setSpeedDir(0, _dir);
    init_homing();
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

void move_to(int tgt, int speedafter = 0)
{
    _targetPos = tgt;
    _targetSpeed = speedafter;
    int dir = _pos < tgt ? OPEN : CLOSE;
    setSpeedDir(5000, dir);
    while (dir == OPEN ? _pos <= _targetPos : _pos >= _targetPos)
    {
        if (_targetPos == 0)
            return;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void move_steps(int steps, int dir, int speedafter = 0)
{
    int t = _pos + (dir == OPEN ? steps : (0 - steps));
    move_to(t, speedafter);
}

void home_open()
{
    Serial.println("Homing open");
    init_homing();
    setSpeedDir(1000, OPEN);
    int openPos = waitForStall();
    Serial.printf("Open pos: %d\n", openPos);
}

void home_closed()
{
    Serial.println("Homing closed, fast");
    init_homing();
    move_steps(300, CLOSE, 1000);
    Serial.println("Homing closed");
    setSpeedDir(1000, CLOSE);
    int closedPos = waitForStall();
    Serial.printf("Closed pos: %d\n", closedPos);
}

void startupSequence()
{
    tune();
    home_open();
    home_closed();
}

void setup()
{
    Serial.begin(300000); // Init serial port and set baudrate
    while (!Serial)
        ; // Wait for serial port to connect
    Serial.println("\nStart...");

    setup_driver();
    startupSequence();
}

void loop()
{
    static uint32_t last_time = 0;
    uint32_t ms = millis();

    while (Serial.available() > 0)
    {
        int8_t read_byte = Serial.read();
    }

    if (_stalled)
    {
    }

    if ((ms - last_time) > 300)
    { // run every 0.1s
        last_time = ms;
        vTaskDelay(300 / portTICK_PERIOD_MS);

        // Serial.print("0 ");
        // Serial.print(driver.SG_RESULT(), DEC);
        // Serial.print(" ");
        // Serial.print(driver.cs2rms(driver.cs_actual()), DEC);
        // Serial.print(" ");
        // Serial.print(driver.cs_actual(), DEC);
        // Serial.print(" ");
        // Serial.println(pos, DEC);
    }
}
