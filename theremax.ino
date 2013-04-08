/*
Based on http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/step3/Upload-the-sketch/
*/

#define trigPin 3
#define echoPin 2
#define speakerPin 9
#define buttonPin 4

#include <NewPing.h>
#define MAX_SENSOR_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(trigPin, echoPin, MAX_SENSOR_DISTANCE); // NewPing setup of pins and maximum distance.

long currentDistance = 0;

unsigned long currentTime;

double distanceCheckTimer[1] = {0};
double buttonCheckTimer[1] = {0};

double cMajorScale[8] = {130.81, 146.83, 164.81, 174.61, 196.0, 220.0, 246.94, 261.63};

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(buttonPin, INPUT);
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
#define OFF_CYCLE_THRESHOLD 8

int buttonPressed = 0;
boolean playing = false;
double playingTone;

void loop() {
  if(tick(50, distanceCheckTimer)) {
    updateDistance();
  }
  if(tick(10, buttonCheckTimer)) {
    buttonPressed = digitalRead(buttonPin);
  }

  double newTone = distanceToFreq(currentDistance, playingTone);

  Serial.print(" distance ");
  Serial.print(currentDistance);
  Serial.print(" button pressed ");
  Serial.print(buttonPressed);
  Serial.print(" playingTone ");
  Serial.print(playingTone);
  Serial.print(" newTone ");
  Serial.print(newTone);
  Serial.println();

  if(newTone == -1) {
    consecutiveOutOfRange++;
    if(consecutiveOutOfRange >= OFF_CYCLE_THRESHOLD) {
      if(playing) {
        Serial.println("notone");
        noTone(speakerPin);
        playing = false;
      }
    }
  }
  else if(!buttonPressed || newTone == -2) {
    consecutiveOutOfRange = 0;
    if(playing) {
      Serial.println("notone");
      noTone(speakerPin);
      playing = false;
    }
  }
  else if(newTone > 0) {
    consecutiveOutOfRange = 0;
    if(!playing || newTone != playingTone) {
      Serial.println("tone");
      tone(speakerPin, newTone);
      playing = true;
      playingTone = newTone;
    }
  }
}

const int MIN_DISTANCE = 300;
const int NUM_NOTES = 8; // c major for now
const double NOTE_WIDTH = 300.0;
const double NOTE_BOUNDARY_WIDTH = 40;
double MAX_DISTANCE = MIN_DISTANCE + NUM_NOTES * NOTE_WIDTH;

double distanceToFreq(unsigned int distance, double currentTone) {
  if(distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
    return -1;
  }

  int index = (distance - MIN_DISTANCE) / NOTE_WIDTH;
  index = index % NUM_NOTES;

  // there is a valley between notes where no sound is produced
  if(distance < MIN_DISTANCE + index * NOTE_WIDTH + NOTE_BOUNDARY_WIDTH || distance > MIN_DISTANCE + (index + 1) * NOTE_WIDTH - NOTE_BOUNDARY_WIDTH) {
    return -2;
  }

  return cMajorScale[index];
}

void updateDistance() {
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  if(uS) {
    currentDistance = uS;
  }
}
