/*
Theremax, a theremin-like instrument for Arduino.

Some resources which were helpful:
http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/step3/Upload-the-sketch/
http://playground.arduino.cc/Code/NewPing
*/

#include <NewPing.h>

///////////////////////////////////
// constants

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

double cMajorScale[8] = {130.81, 146.83, 164.81, 174.61, 196.0, 220.0, 246.94, 261.63};
double HALF_STEP_RATIO = 1.059463094359;

#define MAX_SENSOR_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

///////////////////////////////////
// globals

NewPing sonar(trigPin, echoPin, MAX_SENSOR_DISTANCE); // NewPing setup of pins and maximum distance.

long currentDistance = 0;

double distanceCheckTimer[1] = {0};
double buttonCheckTimer[1] = {0};
double metronomeTimer[1] = {0};

int buttonPressed = 0;
boolean playing = false;
double playingTone;

int currentToneIndex;
int sharpPressed = 0;

boolean metronomeOn = false;

///////////////////////////////////

void setup() {
  Serial.begin(9600);
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

void loop() {
  boolean buttonChanged = updateButtonState();
  boolean toneChanged = updateCurrentToneIndex();

  if(toneChanged) {
    updateHighlightedTone(currentToneIndex);
  }
  if(toneChanged || buttonChanged) {
    updatePlayingTone(currentToneIndex);
  }

  tickMetronome();
}

///////////////////////////////////

boolean updateButtonState() {
  if(tick(11, buttonCheckTimer)) {
    int oldButton = buttonPressed;
    int oldSharp = sharpPressed;
    buttonPressed = digitalRead(buttonPin);
    sharpPressed = digitalRead(sharpPin);
    return (oldButton != buttonPressed || oldSharp != sharpPressed);
  }
  return false;
}

boolean updateCurrentToneIndex() {
  if(tick(50, distanceCheckTimer)) {
    updateDistance();
    int oldToneIndex = currentToneIndex;
    updateIndexFromDistance(currentDistance);
    return oldToneIndex != currentToneIndex;
  }
  return false;
}

void updateHighlightedTone(int toneIndex) {
  if(toneIndex >= 0) {
    noteLightOn(toneIndex);
  }
  else if(toneIndex < 0) {
    noteLightOff();
  }
}

void updatePlayingTone(int toneIndex) {
  boolean shouldPlay;
  if(toneIndex == -1 || !buttonPressed) {
    if(playing) {
      Serial.println("notone");
      noTone(speakerPin);
      playing = false;
    }
  }
  else if(toneIndex >= 0) {
    double newTone = toneIndex >= 0 ? cMajorScale[toneIndex] : toneIndex;
    if(sharpPressed) {
      newTone = newTone * HALF_STEP_RATIO;
    }
    if(!playing || newTone != playingTone) {
      Serial.println("tone");
      tone(speakerPin, newTone);
      playing = true;
      playingTone = newTone;
    }
  }
}

void tickMetronome() {
  if(tick(1000, metronomeTimer)) {
    metronomeOn = !metronomeOn;
    analogWrite(metronomeLightPin, metronomeOn ? 255 : 0);
  }
}

///////////////////////////////////
// distance to index, with debouncing of out-of-range notes and note changes

const int MIN_DISTANCE = 300;
const int NUM_NOTES = 8; // c major for now
const double NOTE_WIDTH = 300.0;
const double NOTE_BOUNDARY_WIDTH = 50;
double MAX_DISTANCE = MIN_DISTANCE + NUM_NOTES * NOTE_WIDTH;

int consecutiveOutOfRange = 0;
const int OUT_OF_RANGE_THRESHOLD = 8;

int consecutiveChangedNote = 0;
const int NOTE_CHANGE_THRESHOLD = 3;

void updateIndexFromDistance(unsigned int distance) {
  if(distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
    if(++consecutiveOutOfRange > OUT_OF_RANGE_THRESHOLD) {
      currentToneIndex = -1;
    }
    return;
  }
  else {
    consecutiveOutOfRange = 0;
  }

  int index = (distance - MIN_DISTANCE) / NOTE_WIDTH;
  index = index % NUM_NOTES;

  // there is a segment at either edge of each note which must be debounced before it is accepted as a note change
  if(distance < MIN_DISTANCE + index * NOTE_WIDTH + NOTE_BOUNDARY_WIDTH || distance > MIN_DISTANCE + (index + 1) * NOTE_WIDTH - NOTE_BOUNDARY_WIDTH) {
    if(index != currentToneIndex) {
      if(++consecutiveChangedNote >= NOTE_CHANGE_THRESHOLD) {
        currentToneIndex = index;
      }
    }
  }
  else {
    consecutiveChangedNote = 0;
    currentToneIndex = index;
  }
}

///////////////////////////////////

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

void updateDistance() {
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  if(uS) {
    currentDistance = uS;
  }
}

boolean tick(int delay, double timekeeper[1]){
  unsigned long currentTime = millis();
  if(currentTime >= (timekeeper[0] + delay)){
    timekeeper[0] = currentTime;
    return true;
  }
  else {
    return 0;
  }
}
