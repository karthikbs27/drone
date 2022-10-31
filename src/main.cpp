#include<Wire.h>
#include <Arduino.h>
#include <MPU6050_light.h>

#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "RemoteXY"


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 57 bytes
  { 255,5,0,5,0,50,0,16,182,0,5,0,235,13,42,42,2,26,31,5,
  0,79,13,42,42,2,26,31,2,0,44,43,12,6,2,26,31,31,79,78,
  0,79,70,70,0,65,112,47,15,6,6,69,1,88,2,10,10 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t joystick_1_x; // from -100 to 100  
  int8_t joystick_1_y; // from -100 to 100  
  int8_t joystick_2_x; // from -100 to 100  
  int8_t joystick_2_y; // from -100 to 100  
  uint8_t switch_1; // =1 if switch ON and =0 if OFF 

    // output variables
  uint8_t led_1_r; // =0..255 LED Red brightness 
  uint8_t led_1_g; // =0..255 LED Green brightness 
  uint8_t led_1_b; // =0..255 LED Blue brightness 
  int16_t sound_1; // =0 no sound, else ID of sound, =1001 for example, look sound list in app 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)


#define TIMER_MICRO 5000 //Trigger the alarm with the timer period of TIMER_MICRO

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis

#define STOPPED  0
#define STARTING 1
#define STARTED  2

#define MOTOR_A  36
#define MOTOR_B  39
#define MOTOR_C  34
#define MOTOR_D  35

MPU6050 mpu(Wire);

int output_from = 0;
int output_to = 255;

// Calculated angular motion on each axis: Yaw, Pitch, Roll
float angular_motions[3] = {0, 0, 0};

float measures[3] = {0, 0, 0};

// MPU's temperature
int temperature;

int temp = millis();
int d;
int a;

// Init flag set to TRUE after first loop
// boolean initialized;

/**
 * Hardware timer is set using TIMER_MICRO 
 *  - timer handels timer
 *  - Semaphore indicates the alarm is triggered
 *  - timerMux is to enter cretical program
 */
hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel
int mode_mapping[4];

unsigned long pulse_length_esc1 = 1000,
        pulse_length_esc2 = 1000,
        pulse_length_esc3 = 1000,
        pulse_length_esc4 = 1000;

// ------------- Global variables used for PID controller --------------------
float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll
// Errors
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float delta_err[3]      = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
// PID coefficients
float Kp[3] = {4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3] = {0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3] = {0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll

// ---------------------------------------------------------------------------
/**
 * Status of the quadcopter:
 *   - 0 : stopped
 *   - 1 : starting
 *   - 2 : started
 *
 * @var int
 */
int status = STOPPED;
// ---------------------------------------------------------------------------
int battery_voltage;
// ---------------------------------------------------------------------------


/**
 * Make sure that given value is not over min_value/max_value range.
 *
 * @param float value     : The value to convert
 * @param float min_value : The min value
 * @param float max_value : The max value
 *
 * @return float
 */
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}

/**
 * Calculate the PID set point in °/s
 *
 * @param float angle         Measured angle (in °) on an axis
 * @param int   channel_pulse Pulse length of the corresponding receiver channel
 * @return float
 */
float calculateSetPoint(float angle, int channel_pulse) {
    float level_adjust = angle * 15; // Value 15 limits maximum angle value to ±32.8°
    float set_point    = 0;

    // Need a dead band of 16µs for better result
    if (channel_pulse > 1508) {
        set_point = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        set_point = channel_pulse - 1492;
    }

    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

/**
 * Calculate the PID set point of YAW axis in °/s
 *
 * @param int yaw_pulse      Receiver pulse length of yaw's channel
 * @param int throttle_pulse Receiver pulse length of throttle's channel
 * @return float
 */
float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
    float set_point = 0;

    // Do not yaw when turning off the motors
    if (throttle_pulse > 1050) {
        // There is no notion of angle on this axis as the quadcopter can turn on itself
        set_point = calculateSetPoint(0, yaw_pulse);
    }

    return set_point;
}

void calculateSetPoints() {
    pid_set_points[YAW]   = calculateYawSetPoint(pulse_length[mode_mapping[YAW]], pulse_length[mode_mapping[THROTTLE]]);
    pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse_length[mode_mapping[PITCH]]);
    pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL], pulse_length[mode_mapping[ROLL]]);
}

/**
 * Calculate errors used by PID controller
 */
void calculateErrors() {
    // Calculate current errors
    errors[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    errors[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];

    // Calculate sum of errors : Integral coefficients
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    // Keep values in acceptable range
    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);

    // Calculate error delta : Derivative coefficients
    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
 */
void pidController() {
    float yaw_pid      = 0;
    float pitch_pid    = 0;
    float roll_pid     = 0;
    int   throttle     = pulse_length[mode_mapping[THROTTLE]];

    // Initialize motor commands with throttle
    pulse_length_esc1 = throttle;
    pulse_length_esc2 = throttle;
    pulse_length_esc3 = throttle;
    pulse_length_esc4 = throttle;

    // Do not calculate anything if throttle is 0
    if (throttle >= 1012) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        // Keep values within acceptable range. TODO export hard-coded values in variables/const
        yaw_pid   = minMax(yaw_pid, -400, 400);
        pitch_pid = minMax(pitch_pid, -400, 400);
        roll_pid  = minMax(roll_pid, -400, 400);

        // Calculate pulse duration for each ESC
        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid; //A
        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid; //B
        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid; //C
        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid; //D
    }

    // Prevent out-of-range-values
    pulse_length_esc1 = minMax(pulse_length_esc1, 1100, 2000);
    pulse_length_esc2 = minMax(pulse_length_esc2, 1100, 2000);
    pulse_length_esc3 = minMax(pulse_length_esc3, 1100, 2000);
    pulse_length_esc4 = minMax(pulse_length_esc4, 1100, 2000);
}

/**
 * Read battery voltage & return whether the battery seems connected
 *
 * @return boolean
 */
bool isBatteryConnected() {
    // Reduce noise with a low-pass filter (10Hz cutoff frequency)
    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    return battery_voltage < 1240 && battery_voltage > 800;
}

/**
 * Compensate battery drop applying a coefficient on output values
 */
void compensateBatteryDrop() {
    if (isBatteryConnected()) {
        pulse_length_esc1 += pulse_length_esc1 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc2 += pulse_length_esc2 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc3 += pulse_length_esc3 * ((1240 - battery_voltage) / (float) 3500);
        pulse_length_esc4 += pulse_length_esc4 * ((1240 - battery_voltage) / (float) 3500);
    }
}


/**
 * Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
 * Direct port manipulation is used for performances.
 *
 * This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.
 *
 * @see https:// www.arduino.cc/en/Reference/PortManipulation
 */
void applyMotorSpeed() {
    analogWrite(MOTOR_A, map(pulse_length_esc1, 1000, 2000, output_from, output_to));
    analogWrite(MOTOR_B, map(pulse_length_esc2, 1000, 2000, output_from, output_to));
    analogWrite(MOTOR_C, map(pulse_length_esc3, 1000, 2000, output_from, output_to));
    analogWrite(MOTOR_D, map(pulse_length_esc4, 1000, 2000, output_from, output_to));
    // Refresh rate is 250Hz: send ESC pulses every 4000µs
    // while ((now = micros()) - loop_timer < period);

    // // Update loop timer
    // loop_timer = now;

    // // Set pins #4 #5 #6 #7 HIGH
    // PORTD |= B11110000;

    // // Wait until all pins #4 #5 #6 #7 are LOW
    // while (PORTD >= 16) {
    //     now        = micros();
    //     difference = now - loop_timer;

    //     if (difference >= pulse_length_esc1) PORTD &= B11101111; // Set pin #4 LOW
    //     if (difference >= pulse_length_esc2) PORTD &= B11011111; // Set pin #5 LOW
    //     if (difference >= pulse_length_esc3) PORTD &= B10111111; // Set pin #6 LOW
    //     if (difference >= pulse_length_esc4) PORTD &= B01111111; // Set pin #7 LOW
    // }
}

/**
 * Reset all PID controller's variables.
 */
void resetPidController() {
    errors[YAW]   = 0;
    errors[PITCH] = 0;
    errors[ROLL]  = 0;

    error_sum[YAW]   = 0;
    error_sum[PITCH] = 0;
    error_sum[ROLL]  = 0;

    previous_error[YAW]   = 0;
    previous_error[PITCH] = 0;
    previous_error[ROLL]  = 0;
}

/**
 * Reset gyro's angles with accelerometer's angles.
 */
// void resetGyroAngles() {
//     gyro_angle[X] = acc_angle[X];
//     gyro_angle[Y] = acc_angle[Y];
// }

/**
 * Reset motors' pulse length to 1000µs to totally stop them.
 */
void stopAll() {
    pulse_length_esc1 = 1000;
    pulse_length_esc2 = 1000;
    pulse_length_esc3 = 1000;
    pulse_length_esc4 = 1000;
}

/**
 * Customize mapping of controls: set here which command is on which channel and call
 * this function in setup() routine.
 */
void configureChannelMapping() {
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}

/**
 * Return whether the quadcopter is started.
 * To start the quadcopter, move the left stick in bottom left corner then, move it back in center position.
 * To stop the quadcopter move the left stick in bottom right corner.
 *
 * @return bool
 */
bool isStarted() {
    // When left stick is moved in the bottom left corner
    if (status == STOPPED && pulse_length[mode_mapping[YAW]] <= 1012 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTING;

        pinMode(2, OUTPUT);
        for(int ran = 0; ran < 10; ran++ ){
            delay(50);
            digitalWrite(2,HIGH);
            delay(50);
            digitalWrite(2,LOW);
        }
    }

    // When left stick is moved back in the center position
    if (status == STARTING && pulse_length[mode_mapping[YAW]] == 1500 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STARTED;

        // Reset PID controller's variables to prevent bump start
        resetPidController();

        // resetGyroAngles();
    }

    // When left stick is moved in the bottom right corner
    if (status == STARTED && pulse_length[mode_mapping[YAW]] >= 1988 && pulse_length[mode_mapping[THROTTLE]] <= 1012) {
        status = STOPPED;
        // Make sure to always stop motors when status is STOPPED
        pinMode(2, OUTPUT);
        for(int ran = 0; ran < 10; ran++ ){
            delay(50);
            digitalWrite(2,HIGH);
            delay(50);
            digitalWrite(2,LOW);
        }

        stopAll();
    }

    return status == STARTED;
}

void convertBleToPulse(){
    pulse_length[mode_mapping[THROTTLE]] = map(RemoteXY.joystick_1_y,-100, 100, 1000, 2000);
    pulse_length[mode_mapping[YAW]] = map(RemoteXY.joystick_1_x,-100, 100, 1000, 2000);
    pulse_length[mode_mapping[PITCH]] = map(RemoteXY.joystick_2_y,-100, 100, 1000, 2000);
    pulse_length[mode_mapping[ROLL]] = map(RemoteXY.joystick_2_x,-100, 100, 1000, 2000);
}


void ARDUINO_ISR_ATTR onTimer(){
  // Increment the counter and set the time of ISR
//   mpu.fetchData();
//   portENTER_CRITICAL_ISR(&timerMux);
  
//   int q = millis();
//   a = temp-q;
//   temp = q;
  // isrCounter++;
  // lastIsrAt = millis();
//   portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void init_timer(){
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, TIMER_MICRO, true);

  // Start an alarm
  timerAlarmEnable(timer);
}



void alarm(){
  // If Timer has fired
  // readSensor();
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    
    // Read the interrupt count and time
    // portENTER_CRITICAL(&timerMux);
    // a = d;
    // // isrCount = isrCounter;
    // // isrTime = lastIsrAt;
    // portEXIT_CRITICAL(&timerMux);

    // int q = millis();
    // a = temp-q;
    // temp = q;
    
    // Serial.println(a);
    
    mpu.fetchData();
    RemoteXY_Handler ();

    mpu.update();
    measures[YAW] = mpu.getAngleZ();
    measures[ROLL] = mpu.getAngleX();
    measures[PITCH] = mpu.getAngleY();

    // Apply low-pass filter (10Hz cutoff frequency)
    angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * mpu.getGyroX();
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * mpu.getGyroY();
    angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * mpu.getGyroZ();

    convertBleToPulse();

    Serial.print("pid_set_points[YAW]:\t");Serial.print(pid_set_points[YAW]);Serial.print("\tpid_set_points[PITCH]:\t");Serial.print(pid_set_points[PITCH]);
    Serial.print("\tpid_set_points[ROLL]\t");Serial.println(pid_set_points[ROLL]);//Serial.print("\tRoll:\t");Serial.println(pulse_length[mode_mapping[ROLL]]);

    // Serial.print("X : ");
    // Serial.print(mpu.getAccX());
    // Serial.print("\tY : ");
    // Serial.print(mpu.getAccY());
    // Serial.print("\tZ : ");
    // Serial.println(mpu.getAccZ());

    

    calculateSetPoints();
    calculateErrors();
    if (isStarted()) {
        // 5. Calculate motors speed with PID controller
        pidController();

        // compensateBatteryDrop();
    }
    // 6. Apply motors speed
    applyMotorSpeed();
    // Serial.println(RemoteXY.joystick_1_x);
    

  }
}


void setup(){
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  
  byte status = mpu.begin(1, 2);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);

  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

  configureChannelMapping();

  pinMode(2, OUTPUT);
  for(int ran = 0; ran < 10; ran++ ){
    delay(250);
    digitalWrite(2,HIGH);
    delay(250);
    digitalWrite(2,LOW);
  }


  init_timer();
  RemoteXY_Init ();

}

void loop(){
  alarm();
  
}


