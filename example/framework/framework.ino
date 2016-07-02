#include <WheelControl.h>
//-- wheel control --
#define wheelPinInterrupt 3    // used by sotfware interrupt when rotation reach threshold
#define leftAnalogEncoderInput A8   // analog input left encoder
#define rightAnalogEncoderInput A10  // analog input right encoder
#define leftWheelId 0           // to identify left wheel Id 
#define rightWheelId 1         // to identify right wheel Id
#define leftWheelEncoderHoles 8  // number of holes of the encoder wheel
#define rightWheelEncoderHoles 8 // number of holes of the encoder wheel
#define encoderPower 48
unsigned int iLeftRevSpeed;              // instant left wheel speed
unsigned int iRightRevSpeed;             // instant right wheel speed
unsigned int leftIncoderHighValue = 650;  // define value above that signal is high
unsigned int leftIncoderLowValue = 450;  // define value below that signal is low
// to adjust low and high value set rightWheelControlOn true, rotate right wheel manualy and read on serial the value with and wihtout hole
unsigned int rightIncoderHighValue = 650; // define value above that signal is high
unsigned int rightIncoderLowValue = 450;  // define value below that signal is low
//#define delayBetweenEncoderAnalogRead  750 //  micro second between analog read of wheel encoder level
#define delayMiniBetweenHoles  40  //  delay millis second between 2 encoder holes at the maximum speed
// create wheel control object
WheelControl Wheels(leftWheelEncoderHoles, leftIncoderHighValue, leftIncoderLowValue, leftAnalogEncoderInput,
                    rightWheelEncoderHoles, rightIncoderHighValue , rightIncoderLowValue, rightAnalogEncoderInput,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    wheelPinInterrupt, delayMiniBetweenHoles);
int iLeftHoles = 8;
int iRightHoles = 8;

void setup() {
  digitalWrite(encoderPower, HIGH);  // to power on the encoder
  digitalWrite(wheelPinInterrupt, LOW);
  attachInterrupt(digitalPinToInterrupt(wheelPinInterrupt), WheelInterrupt, RISING);
  Wheels.StartWheelControl(true, true, iLeftHoles , true, true, iRightHoles , false, false, 0, false, false, 0);
}

void loop() {


}

void WheelInterrupt()   // wheel controler set a software interruption due to threshold reaching
{
  uint8_t wheelId = Wheels.GetLastWheelInterruptId();  // get which wheel Id reached the threshold
  if (wheelId != 5)                                    //  5 means pulse mode (no encoder usage)
  {
    Wheels.ClearThreshold(wheelId);                      // clear the threshold flag to avoid any more interruption
  }
  WheelThresholdReached(wheelId);                      // call the threshold analyse
}

void WheelThresholdReached( uint8_t wheelId)
{
  if (wheelId != 5)                                  //  5 means pulse mode (no encoder usage)
  {
    if (wheelId == leftWheelId)
    {
      //      leftMotor.StopMotor();                             // stop  the motor that reached the threshold
      Wheels.StopWheelControl(true, false, false, false);  // stop wheel control
    }
    if (wheelId == rightWheelId)
    {
      //      rightMotor.StopMotor();                             // stop  the motor that reached the threshold
      Wheels.StopWheelControl(false, true, false, false);  // stop wheel control
    }

    if (Wheels.GetWheelThreshold(leftWheelId) == 0 && Wheels.GetWheelThreshold(rightWheelId) == 0)
    {
      // 2 thresholds reached power off encoder
      detachInterrupt(digitalPinToInterrupt(wheelPinInterrupt));
      pinMode(wheelPinInterrupt, INPUT);
      digitalWrite(encoderPower, LOW);
    }
  }
  else                      // wheel mode pulse
  {

  }
}

