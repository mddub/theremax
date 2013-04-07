/*
Based on http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/step3/Upload-the-sketch/
*/

#define trigPin 13
#define echoPin 12
#define speakerPin 9

#include <NewPing.h>
#define MAX_SENSOR_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(trigPin, echoPin, MAX_SENSOR_DISTANCE); // NewPing setup of pins and maximum distance.

long currentDistance = 0;

unsigned long currentTime;
double currentTone;

double distanceCheckTimer[1] = {0};

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

int consecutiveOutOfRange = 0;
#define OFF_CYCLE_THRESHOLD 5

void loop() {
  if(tick(50, distanceCheckTimer)) {
    updateDistance();
  }
  else {
    return;
  }
  double newTone = distanceToFreq(currentDistance, currentTone);
  if(newTone == -1) {
    if(++consecutiveOutOfRange >= OFF_CYCLE_THRESHOLD) {
      noTone(speakerPin);
    }
  }
  else {
    consecutiveOutOfRange = 0;
    tone(speakerPin, newTone);
    currentTone = newTone;
  }
}

const int MIN_DISTANCE = 300;
const int NUM_NOTES = 8; // c major for now
const double NOTE_WIDTH = 300.0;
const double NOTE_BOUNDARY_WIDTH = 50;
double MAX_DISTANCE = MIN_DISTANCE + NUM_NOTES * NOTE_WIDTH;

double distanceToFreq(unsigned int distance, double currentTone) {
  if(distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
    return -1;
  }

  int index = (distance - MIN_DISTANCE) / NOTE_WIDTH;
  index = index % NUM_NOTES;

  // there is a valley between notes where no sound is produced
  if(distance < MIN_DISTANCE + index * NOTE_WIDTH + NOTE_BOUNDARY_WIDTH || distance > MIN_DISTANCE + (index + 1) * NOTE_WIDTH - NOTE_BOUNDARY_WIDTH) {
    return -1;
  }

  return cMajorScale[index];
}

void updateDistance() {
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  if(uS) {
    currentDistance = uS;
    Serial.print("Ping: ");
    Serial.println(currentDistance); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  }
}
