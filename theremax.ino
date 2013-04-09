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
double playButtonCheckTimer[1] = {0};
double sharpCheckTimer[1] = {0};
double loopTimer[1] = {0};

int playButtonPressed = 0;
boolean playing = false;
double playingTone;

int currentToneIndex;
int sharpPressed = 0;

boolean metronomeOn = false;

const boolean MODE_FREESTYLE = true;
const boolean MODE_LOOP = false;
boolean mode = MODE_LOOP;

#define NUM_SAMPLES_PER_NOTE 16
#define MAX_NOTES 128
int currentTempo = 50; // length of sixteenth note, in ms

double samples[NUM_SAMPLES_PER_NOTE];
double currentSong[MAX_NOTES];
//double currentSong[64] = {164.81, -1, 164.81, -1, 174.61, -1, 196, -1, 196, -1, 174.61, -1, 164.81, -1, 146.83, -1, 130.81, -1, 130.81, -1, 146.83, -1, 164.81, -1, 164.81, 164.81, 164.81, -1, 146.83, -1, 146.83, 146.83, 146.83, 146.83, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

int currentSampleIndex = 0;
int currentLoopPosition = 0;

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
  for(int i = 0; i < MAX_NOTES; i++) {
    currentSong[i] = -1;
  }
}

void loop() {
  boolean playButtonChanged = updatePlayButtonState();
  boolean sharpButtonChanged = updateSharpButtonState();
  boolean toneIndexChanged = updateCurrentToneIndex();

  boolean loopChanged = updateLoopPosition();

  tickMetronomeLight();

  if(toneIndexChanged) {
    updateHighlightedTone(currentToneIndex);
  }

  if(mode == MODE_FREESTYLE) {
    if(toneIndexChanged || playButtonChanged || sharpButtonChanged) {
      updatePlayingTone();
    }
  }
  else if(mode == MODE_LOOP) {
    if(toneIndexChanged || playButtonChanged || sharpButtonChanged || loopChanged) {
      updateRecordingTone(playButtonChanged, loopChanged);
    }
  }
}

///////////////////////////////////

boolean updatePlayButtonState() {
  if(tick(11, playButtonCheckTimer)) {
    int oldButton = playButtonPressed;
    playButtonPressed = digitalRead(buttonPin);
    return oldButton != playButtonPressed;
  }
  return false;
}

boolean updateSharpButtonState() {
  if(tick(11, sharpCheckTimer)) {
    int oldSharp = sharpPressed;
    sharpPressed = digitalRead(sharpPin);
    return oldSharp != sharpPressed;
  }
  return false;
}

boolean updateCurrentToneIndex() {
  if(tick(40, distanceCheckTimer)) {
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

void updatePlayingTone() {
  boolean shouldPlay;
  if(currentToneIndex == -1 || !playButtonPressed) {
    if(playing) {
      noTone(speakerPin);
      playing = false;
      playingTone = -1;
    }
  }
  else if(currentToneIndex >= 0) {
    double newTone = currentToneIndex >= 0 ? cMajorScale[currentToneIndex] : currentToneIndex;
    if(sharpPressed) {
      newTone = newTone * HALF_STEP_RATIO;
    }
    if(!playing || newTone != playingTone) {
      tone(speakerPin, newTone);
      playing = true;
      playingTone = newTone;
    }
  }
}

boolean updateLoopPosition() {
  int sampleFreq = currentTempo / NUM_SAMPLES_PER_NOTE;
  if(tick(sampleFreq, loopTimer)) {
    currentSampleIndex++;
    if(currentSampleIndex == NUM_SAMPLES_PER_NOTE) {
      currentSampleIndex = 0;
      currentLoopPosition = (currentLoopPosition + 1) % MAX_NOTES;
    }
    return true;
  }
  return false;
}

void updateRecordingTone(boolean playButtonChanged, boolean loopChanged) {
  if(loopChanged) {
    if(!playButtonPressed) {
      double newTone = currentSong[currentLoopPosition];
      if(newTone >= 0 && (!playing || newTone != playingTone)) {
        tone(speakerPin, newTone);
        playing = true;
        playingTone = newTone;
      }
      else if(newTone < 0 && playing) {
        noTone(speakerPin);
        playing = false;
      }
    }
    else {
      updatePlayingTone();
	  updateRecordedNote();
    }
  }
}

void updateRecordedNote() {
  samples[currentSampleIndex] = playingTone;
  if(currentSampleIndex == 0) {
    // need to decide what to save for previous note
    int previousNoteIndex = (currentLoopPosition - 1) % MAX_NOTES;
    currentSong[previousNoteIndex] = determineSampledNoteValue();
  }
}

double determineSampledNoteValue() {
  // do a really dumb O(n^2) search through the array to
  // find the first value with >= 50% representation.
  int minSamples = NUM_SAMPLES_PER_NOTE / 2;

  for(int i = 0; i < NUM_SAMPLES_PER_NOTE; i++) {
    double candidateValue = samples[i];
	int count = 0;
	for(int j = 0; j < NUM_SAMPLES_PER_NOTE; j++) {
	  if(abs(candidateValue - samples[j]) < 0.001) {
	    count++;
	  }
	}
	if(count >= minSamples) {
	  return candidateValue;
	}
  }

  // if no candidate, record silence.
  return -1;
}

void tickMetronomeLight() {
  int currentNote = currentLoopPosition / 4;
  boolean metronomeState = currentNote % 2;
  if(metronomeState != metronomeOn) {
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
