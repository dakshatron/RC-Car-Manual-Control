// We'll use the ESP32 microcontroller to:
    // specify steering and throttle
    // establish ROS connection via rosserial
    // adjust steering and throttle in real time using discrete PID (numerical integration and subtraction, very simple)

#include <SCServo.h> // SCServo library for steering motor control, https://github.com/workloads/scservo/blob/main/src/SCServo.h
#include <Timer.h>
#include <ros.h> // ROS library for serial communication
#include <geometry_msgs/Twist.h> // for transporting velocity info vectors
#include <math.h>

// Define rear motor output pins and PWM properties, and attach PWM channels
const int leftMotorPinA = 12; // --> AIN1
const int leftMotorPinB = 13; // --> AIN2
const int rightMotorPinA = 18; // --> BIN2
const int rightMotorPinB = 19; // --> BIN1
const int freq = 1000;
const int leftChannel1 = 0;
const int leftChannel2 = 1;
const int rightChannel1 = 2;
const int rightChannel2 = 3;
ledcSetup( leftChannel1, freq, resolution );
ledcAttachPin( leftMotorPinA, leftChannel1 );
ledcSetup( leftChannel2, freq, resolution );
ledcAttachPin( leftMotorPinB, leftChannel2 );
ledcSetup( rightChannel1, freq, resolution );
ledcAttachPin( rightMotorPinA, rightChannel1 );
ledcSetup( rightMotorChannel2, freq, resolution );
ledcAttachPin( rightMotorPinB, rightChannel2 );
// Define rear motor encoder pins and initialize for sensing interrupt
const int leftEncoderPinA = 25;
const int leftEncoderPinB = 26;
const int rightEncoderPinA = 32;
const int rightEncoderPinB = 33;
pinMode(leftEncoderPinA, INPUT);
pinMode(leftEncoderPinB, INPUT);
pinMode(rightEncoderPinA, INPUT);
pinMode(rightEncoderPinB, INPUT);
attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoderISR, CHANGE);
attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, CHANGE);


//// function that will write the throttle output to the motor of your choice
#define leftMotorID 1
#define rightMotorID 2
int leftPwmOutput;
int rightPwmOutput;

// pwmOutput: joystick desired speed converted to PWM resolution (0 to 4095)
// motorID: specifies which motor to power 'pwmOutput' amount
void pwmControlInput(int pwmOutput, int motorID)
{

}



// hardware interrupt function that keeps track of the encoder counts for each motor

// initializes interrupt
void setup()
{
// if pin senses rising OR falling edge, ie. any change, triggers rightEncoderISR
    attachInterrupt(digitalPinToInterrupt(encoderPinRightA), rightEncoderISR, CHANGE);
}
// increments/decrements position counter

void IRAM_ATTR rightEncoderISR()
{
    // what direction is it going? hint: use digitalRead

    // incrememt if ticker went forward, decrement if it went backward
}

void IRAM_ATTR leftEncoderISR()
{
    // same as rightEncoderISR
}




#define wheelRadius 0.0375 // in meters
#define pulsesPerRotation 30000 // AB phase pulses
#define pidPeriod 0.02 // in seconds, timer for pidCorrection()
#define leftMotorID 1
#define rightMotorID 2

// conversion factor to translate angular pulse distance to linear distance travelled
float encoderToLinearDistance = 2.0 * PI * wheelRadius * 1000000 / (pulsesPerRotation * pidPeriod)
float propGain = 6000, intGain = 50, derivGain = 0; // tuned PID parameters

int32_t leftVelocity, rightVelocity;
int32_t targetLeftVelocity, targetRightVelocity;
int16_t steerPos, targetSteerPos; // 1700-2500 range (not in radians!)
int32_t leftError, prevLeftError;
int32_t rightError, prevRightError;
int16_t leftPwmOutput, rightPwmOutput;

// encoder position counts
uint32_t leftCount, rightCount, oldLeftCount, oldRightCount;
void setup() {timer.every(20, pidCorrection);}


void pidCorrection()
{
    noInterrupts();

    // simple velocity calculation in last interval for both wheels

    interrupts();
    int steeringError = 9999; // local var, must be initialized with placeholder
    sms_sts steeringServo; // steering servo object
    steeringServo.WritePosEx(1, targetSteerPos, 0, 0); // immediately executes
    steerAngle = steeringServo.ReadPos(1); // reads current steering angle

    // find steering error

    // wait until steering error within threshold, do PID correction
        // calculate left and right wheel velocity error
        // calculate new desired pwm outputs for left and right motors
        // write pwm outputs to motors
        // update previous velocity errors for next iteration
    // coast until within threshold
    // update previous encoder errors for next iteration
}



// establish ROS node handle and publishers/subscribers