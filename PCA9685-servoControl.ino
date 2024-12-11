#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int pos0 = 102;
int pos180 = 512;

void setup() {
  Serial.begin(9600);
  Serial.println("GPIO test!");
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(3000);
  setServo(0, 0);
}

void loop() {
  setServo(0, 180);
  delay(3000);
  setServo(0, 0);
  delay(3000);

}
void setServo(int servo, int angle) {
  int duty;
  duty = map(angle, 0, 180, pos0, pos180);
  pwm.setPWM(servo, 0, duty);
}
