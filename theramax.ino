/*
Based on http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/step3/Upload-the-sketch/
*/

#include <Time.h>

#define trigPin 13
#define echoPin 12
#define speakerPin 9
#define led 11
#define led2 10

long elapsed;
long currentDistance = 0;

int state;
const int STATE_WAITING_FOR_ECHO = 0;
const int STATE_RECEIVED_DISTANCE = 1;
long ms_since_echo = 0;

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode (speakerPin, OUTPUT);
  elapsed = 0;
}

void loop() {
  doDistanceStuff(elapsed);
  buzz(currentDistance * 4000.0 / 24.0);
  
  elapsed++;
  delayMicroseconds(1);
}

void doDistanceStuff(long elapsed) {
  if(elapsed % 12 == 0) {
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    currentDistance = (duration/2) / 29.1;
    Serial.print(currentDistance);
    Serial.println(" cm");
  }
  else if(elapsed % 12 == 2) {
    digitalWrite(trigPin, HIGH);
  }
}

long getDistance() {
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  return distance;
}

void buzz(int tone) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(tone);
}
