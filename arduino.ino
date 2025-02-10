#include <Arduino.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
int aa = 9;
int bb = 10;
int Speed1 = 255, Speed2 = 240, Speed3 = -200, Speed4 = 100;
const int outPin = 5, s0 = 2, s1 = 3, s2 = 4, s3 = 37;
const int in1 = 7;
const int in2 = 8;
const int in3 = 12;
const int in4 = 11;
const int leftPin = 35, centerPin = 33, rightPin = 31;
int left, center, right;

enum Pos {
  LEFT,
  CENTER,
  RIGHT,
  OFF_LINE
};

Pos getPos() {
  left = digitalRead(leftPin);
  center = digitalRead(centerPin);
  right = digitalRead(rightPin);

  if (!left && !center && !right) {
    return OFF_LINE;
  } else if (left && !center && right) {
    return CENTER;
  } else if (left && !center && !right) {
    return RIGHT;
  } else if (!left && !center && right) {
    return LEFT;
  } else if (!left && center && !right) {
    return RIGHT;
  } else if (left && center && !right) {
    return RIGHT;
  } else if (!left && center && right) {
    return LEFT;
  } else {
    return CENTER;
  }
}

unsigned long readFreq(int s2Val, int s3Val) {
  digitalWrite(s2, s2Val);
  digitalWrite(s3, s3Val);
  unsigned long start = micros();
  while (digitalRead(outPin) == LOW);
  while (digitalRead(outPin) == HIGH);
  return micros() - start;
}



void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in2, LOW);
  digitalWrite(in4, LOW);
  analogWrite(in1, 0);
  analogWrite(in3, 0);
  Serial.begin(9600);
  pinMode(leftPin, INPUT);
  pinMode(centerPin, INPUT);
  pinMode(rightPin, INPUT);
  Serial.begin(9600);
  pinMode(outPin, INPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  Serial.println("TCS230 Test");
}

void motorAV(int speed) {

  digitalWrite(in2, HIGH);
  analogWrite(in1, (speed));
}

void motorBV(int speed) {
  digitalWrite(in4, HIGH);
  analogWrite(in3, (speed));

}

void motorAN(int speed) {
  digitalWrite(in2, LOW);
  analogWrite(in1, (speed));
}

void motorBN(int speed) {
  digitalWrite(in4, LOW);
  analogWrite(in3, (speed));
}
void verh(){
  Serial.println("verh");
  for (int i = 0; i <= 90; i++) {
  servo1.write(i);
  delay(10);
  }
}


void niz(){
  Serial.println("niz");
  servo1.write(0);
  delay(2000);

}


void rights(){
  Serial.println("right");
  servo2.write(90);
  delay(2000);

}
void lefts(){
  Serial.println("left");
  servo2.write(0);
  delay(2000);

}


void left_prim() {
  Serial.println("Right");
  motorAV(Speed4);
  motorBN(Speed1);
  delay(2000);
  Serial.println("Forward");
  motorAV(Speed2);
  motorBV(Speed2);
  delay(2000);
  Serial.println("Left");
  motorBV(Speed4);
  motorAN(Speed1);
  delay(2000);
}



void loop() {
  Pos pos = getPos();
  Serial.print("Датчики:");
  Serial.print(left);
  Serial.print(" ");
  Serial.print(center);
  Serial.print(" ");
  Serial.println(right);
  Serial.print("Позиция: ");
  unsigned long red = readFreq(LOW, LOW);
  unsigned long blue = readFreq(LOW, HIGH);
  unsigned long green = readFreq(HIGH, LOW);
  unsigned long clear = readFreq(HIGH, HIGH);
  Serial.print("Red: "); Serial.println(red);
  Serial.print("Green: "); Serial.println(green);
  Serial.print("Blue: "); Serial.println(blue);
  Serial.print("Clear: "); Serial.println(clear);

  // Define thresholds for color detection.  Adjust these values based on your sensor and environment.
  const unsigned long RED_THRESHOLD = 1000; // Example value, adjust as needed
  const unsigned long GREEN_THRESHOLD = 1000; // Example value, adjust as needed
  const unsigned long BLUE_THRESHOLD = 1000;  // Example value, adjust as needed


  if (red < RED_THRESHOLD && red != 0) { //check the sensor is not return 0
  left_prim();
  }
  else if(green < GREEN_THRESHOLD && green != 0) { //check the sensor is not return 0
  niz();
  lefts();
  verh();
  }
  else if (blue < BLUE_THRESHOLD && blue != 0) { //check the sensor is not return 0
  niz();
  lefts();
  verh();
  }
  else {


    switch (pos) {
      case CENTER:
        Serial.println("CENTER");
        motorAV(Speed2);
        motorBV(Speed2);
        break;
      case LEFT:
        Serial.println("LEFT");
        motorBV(Speed4);
        motorAN(Speed1);
        break;
      case RIGHT:
        Serial.println("RIGHT");
        motorAV(Speed4);
        motorBN(Speed1);
        break;
      case OFF_LINE:
        Serial.println("OFF_LINE");
        motorAV(Speed1);
        motorBV(Speed1);

        break;
    }
  }
  delay(1);
}
