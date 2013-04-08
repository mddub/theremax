/*
Based on http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/step3/Upload-the-sketch/
*/

#define trigPin 3
#define echoPin 2
#define speakerPin 9
#define buttonPin 4
#define sharpPin 7
#define metronomeLightPin 5

//Pin connected to ST_CP of 74HC595
int latchPin = 8;
//Pin connected to SH_CP of 74HC595
int clockPin = 12;
////Pin connected to DS of 74HC595
int dataPin = 11;

#include <NewPing.h>
#define MAX_SENSOR_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(trigPin, echoPin, MAX_SENSOR_DISTANCE); // NewPing setup of pins and maximum distance.

long currentDistance = 0;

unsigned long currentTime;

double distanceCheckTimer[1] = {0};
double buttonCheckTimer[1] = {0};
double metronomeTimer[1] = {0};

double cMajorScale[8] = {130.81, 146.83, 164.81, 174.61, 196.0, 220.0, 246.94, 261.63};

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(sharpPin, INPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(metronomeLightPin, OUTPUT);
}

boolean tick(int delay, double timekeeper[1]){
  currentTime = micros() / 1000;
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

int lastToneIndex;
int sharpPressed = 0;

boolean metronomeOn = false;

void loop() {
  int toneIndex = lastToneIndex;
  if(tick(50, distanceCheckTimer)) {
    updateDistance();
    toneIndex = distanceToIndex(currentDistance, lastToneIndex);
    if(toneIndex >= 0 && toneIndex != lastToneIndex) {
      noteLightOn(toneIndex);
    }
    else if(toneIndex < 0 && toneIndex != lastToneIndex) {
      noteLightOff();
    }
    lastToneIndex = toneIndex;
  }

  if(tick(10, buttonCheckTimer)) {
    buttonPressed = digitalRead(buttonPin);
    sharpPressed = digitalRead(sharpPin);
  }

  double newTone = toneIndex >= 0 ? cMajorScale[toneIndex] : toneIndex;

  Serial.print(" distance ");
  Serial.print(currentDistance);
  Serial.print(" button pressed ");
  Serial.print(buttonPressed);
  Serial.print(" sharp pressed ");
  Serial.print(sharpPressed);
  Serial.print(" playingTone ");
  Serial.print(playingTone);
  Serial.print(" newTone ");
  Serial.print(newTone);
  Serial.println();

  if(newTone == -1) {
    consecutiveOutOfRange++;
    if(consecutiveOutOfRange >= OFF_CYCLE_THRESHOLD) {
      if(playing) {
        noTone(speakerPin);
        playing = false;
      }
    }
  }
  else if(!buttonPressed || newTone == -2) {
    consecutiveOutOfRange = 0;
    if(playing) {
      noTone(speakerPin);
      playing = false;
    }
  }
  else if(newTone > 0) {
    consecutiveOutOfRange = 0;
    if(sharpPressed) {
      newTone = newTone * 1.059463094359;
    }
    if(!playing || newTone != playingTone) {
      tone(speakerPin, newTone);
      playing = true;
      playingTone = newTone;
    }
  }

  if(tick(1000, metronomeTimer)) {
    metronomeOn = !metronomeOn;
    analogWrite(metronomeLightPin, metronomeOn ? 255 : 0);
  }
}

void noteLightOn(int index) {
  index = 7 - index;
  shiftValue(1 << index);
}

void noteLightOff() {
  shiftValue(0);
}

void shiftValue(int value) {
  // take the latchPin low so
  // the LEDs don't change while you're sending in bits:
  digitalWrite(latchPin, LOW);
  // shift out the bits:
  shiftOut(dataPin, clockPin, MSBFIRST, value);
  //take the latch pin high so the LEDs will light up:
  digitalWrite(latchPin, HIGH);
}

const int MIN_DISTANCE = 300;
const int NUM_NOTES = 8; // c major for now
const double NOTE_WIDTH = 300.0;
const double NOTE_BOUNDARY_WIDTH = 40;
double MAX_DISTANCE = MIN_DISTANCE + NUM_NOTES * NOTE_WIDTH;

int consecutiveChangedNote = 0;
const int NOTE_CHANGE_THRESHOLD = 4;

int distanceToIndex(unsigned int distance, double currentIndex) {
  if(distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
    return -1;
  }

  int index = (distance - MIN_DISTANCE) / NOTE_WIDTH;
  index = index % NUM_NOTES;

  // there is a segment at either edge of each note which must be debounced before it is accepted as a note change
  if(distance < MIN_DISTANCE + index * NOTE_WIDTH + NOTE_BOUNDARY_WIDTH || distance > MIN_DISTANCE + (index + 1) * NOTE_WIDTH - NOTE_BOUNDARY_WIDTH) {
    if(index != currentIndex) {
      if(++consecutiveChangedNote > NOTE_CHANGE_THRESHOLD) {
        return index;
      }
      else {
        return currentIndex;
      }
    }
  } else {
    consecutiveChangedNote = 0;
  }
  return index;
}

void updateDistance() {
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  if(uS) {
    currentDistance = uS;
  }
}
