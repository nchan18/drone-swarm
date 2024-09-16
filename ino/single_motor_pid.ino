#include <ESP32Servo.h>
#include <PID_v2.h>


#define X_AXIS_PIN 33
#define ESC_PIN 16
// #define PIN_INPUT 0
#define PIN_OUTPUT 16

Servo esc;

double Kp = 2, Ki = 0, Kd = 1;
PID_v2 myPID(Kp, Ki, Kd, PID::Direct);

void setup() 
{
  esc.attach(ESC_PIN,  1000, 2000);
  esc.write(0);
  myPID.Start(analogRead(X_AXIS_PIN),0,100);  
  delay(2000);  
  Serial.begin(9600);

}
 
void loop() 
{
  // const double input = analogRead(PIN_INPUT);
  // const double output = myPID.Run(input);
  // analogWrite(PIN_OUTPUT, output);
  myPID.Compute();
  int joystickValue = analogRead(X_AXIS_PIN);
  joystickValue = constrain(joystickValue, 0, 4095);  //Read upper half of joystick value from center.
  const double output = myPID.Run(joystickValue);
  // int mmotorSpeed = map(joystickValue, 0, 4095, 0, 255);
  esc.write(output); 
  // Serial.println(mmotorSpeed); 
  Serial.println(output);
}