/*
Based on http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/step3/Upload-the-sketch/
*/

#define trigPin 13
#define echoPin 12
#define speakerPin 9

#include <NewPing.h>
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(trigPin, echoPin, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

long currentDistance = 0;

unsigned long currentTime;
double currentTone;

double distanceCheckTimer[1] = {0};
double toneUpdateTimer[1] = {0};
double buzztimer[1] = {0};

double cMajorScale[8] = {130.81, 146.83, 164.81, 174.61, 196.0, 220.0, 246.94, 261.63};

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode (speakerPin, OUTPUT);
}

boolean tick(int delay, double timekeeper[1]){
  currentTime = millis();
  if(currentTime >= (timekeeper[0] + delay)){
    timekeeper[0] = currentTime;
    return true;
  }
  else {
    return 0;
  }
}

void loop() {
  if(tick(50, distanceCheckTimer)) {
    updateDistance();
  }
  currentTone = distanceToFreq(currentDistance);
  if(currentTone == -1) {
    noTone(speakerPin);
  }
  else {
    tone(speakerPin, currentTone); 
  }
}

double distanceToFreq(unsigned int distance) {
  if(distance < 300 || distance > 2300) {
    return -1;
  }
  int index = (distance - 300.0) / 250.0;
  index = index % 8;
  return cMajorScale[index];
}

void updateDistance() {
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  if(uS) {
    currentDistance = uS;
    //currentDistance = sonar.convert_cm(uS);
    Serial.print("Ping: ");
    Serial.println(currentDistance); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  }
}
