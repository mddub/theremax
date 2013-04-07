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
  currentDistance = 10;
  buzz(currentDistance * 4000.0 / 24.0);
  
  if(state == STATE_WAITING_FOR_ECHO) {
    if(ms_since_echo == 8) {
      digitalWrite(trigPin, LOW);
    }
  }
  
  elapsed++;
  delayMicroseconds(1);
}

long getDistance() {
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(1); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(5); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  Serial.print(distance);
  Serial.println(" cm");  
  return distance;
}

void buzz(int tone) {
    digitalWrite(speakerPin, HIGH);
    delayMicroseconds(tone);
    digitalWrite(speakerPin, LOW);
    delayMicroseconds(tone);
}
