#include <Servo.h>
#include <HCSR04.h>
#include <PID_v1.h>

const int trigPin = 7;
const int echoPin = 6;
double reading;

const double KP = 0.6;
const double KI = 1;
const double KD = 1;
double P;
double D;
double I;
const double setpoint = 23;

double input, output;
double prev_input = 0.0;
double prev_i = 0.0;

UltraSonicDistanceSensor distanceSensor(trigPin, echoPin);
PID pid(&input, &output, &setpoint, KP, KI, KD, P_ON_E, REVERSE);
Servo servo;

void setup() {
  Serial.begin(9600);
  servo.attach(9);
  double servoMax = 105;
  double servoMin = 75;
  // pid = PID(&input, &output, &setpoint, KP, KI, KD, P_ON_E, DIRECT);
  pid.SetMode(AUTOMATIC);
  //pid.SetOutputLimits(servoMin, servoMax);

  servo.write(90);
  delay(5000);
}

void loop() {
  double reading = distanceSensor.measureDistanceCm();
  if (reading > 0) {
    input = reading - 20.00;
    // // pid.Compute();
    // // Serial.print("The Input is: ");
    // Serial.println(reading);
    // // Serial.print("The Output is: ");
    // // Serial.println(output);
    // Serial.println();
    P = input* KP + 90;
    D = KD * (input - prev_input);
    I = prev_i + KI*input;
    servo.write(P + D + I);

    // for(input = 0; input <= 255; input++){

    //   Serial.println(input);
    //   Serial.println(output);
    // }
    // delay(1000);
    prev_input = input;
  }
}
