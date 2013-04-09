/*
Theremax, a theremin-like instrument for Arduino.

Some resources which were helpful:
http://www.instructables.com/id/Simple-Arduino-and-HC-SR04-Example/step3/Upload-the-sketch/
http://playground.arduino.cc/Code/NewPing
https://code.google.com/p/rogue-code/wiki/ToneLibraryDocumentation
*/

#include <NewPing.h>
#include <Tone.h>

///////////////////////////////////
// constants

#define trigPin 3
#define echoPin 2
#define speakerPin1 9
#define speakerPin2 10
#define buttonPin 4
#define sharpPin 7
#define modePin 13
#define metronomeLightPin 5

//Pin connected to ST_CP of 74HC595
int latchPin = 8;
//Pin connected to SH_CP of 74HC595
int clockPin = 12;
////Pin connected to DS of 74HC595
int dataPin = 11;

double scales[5][8] = {
  {NOTE_C3, NOTE_D3, NOTE_E3, NOTE_F3, NOTE_G3, NOTE_A3, NOTE_B3, NOTE_C4},
  {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5},
  {NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6},
  {NOTE_C6, NOTE_D6, NOTE_E6, NOTE_F6, NOTE_G6, NOTE_A6, NOTE_B6, NOTE_C7},
  {NOTE_C7, NOTE_D7, NOTE_E7, NOTE_F7, NOTE_G7, NOTE_A7, NOTE_B7, NOTE_C8}
};

int currentScale = 0;
double HALF_STEP_RATIO = 1.059463094359;

#define MAX_SENSOR_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

///////////////////////////////////
// globals

NewPing sonar(trigPin, echoPin, MAX_SENSOR_DISTANCE); // NewPing setup of pins and maximum distance.

Tone notePlayer[2];

long currentDistance = 0;

double distanceCheckTimer[1] = {0};
double playButtonCheckTimer[1] = {0};
double sharpCheckTimer[1] = {0};
double loopTimer[1] = {0};

int playButtonPressed = 0;
boolean playing = false;
int playingTone;

int currentToneIndex;
int sharpPressed = 0;

boolean metronomeOn = false;

const boolean MODE_FREESTYLE = true;
const boolean MODE_LOOP = false;
boolean mode = MODE_FREESTYLE;

const int BUTTON_NO_CLICK = 0;
const int BUTTON_CLICKED = 1;
const int BUTTON_DOUBLE_CLICKED = 2;

const int DOUBLE_CLICK_WINDOW = 500;

#define NUM_SAMPLES_PER_NOTE 16
#define MAX_NOTES 128
int currentTempo = 50; // length of sixteenth note, in ms

double samples[NUM_SAMPLES_PER_NOTE];
double currentSong[MAX_NOTES];

int currentSampleIndex = 0;
int currentLoopPosition = 0;

///////////////////////////////////

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(sharpPin, INPUT);
  pinMode(modePin, INPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(metronomeLightPin, OUTPUT);

  notePlayer[0].begin(speakerPin1);
  notePlayer[1].begin(speakerPin2);

  for(int i = 0; i < MAX_NOTES; i++) {
    currentSong[i] = -1;
  }
}

void loop() {
  boolean playButtonChanged = updatePlayButtonState();
  boolean sharpButtonChanged = updateSharpButtonState();
  boolean toneIndexChanged = updateCurrentToneIndex();

  boolean loopChanged = updateLoopPosition();

  int modeButtonState = checkModeButtonState();

  if(modeButtonState == BUTTON_CLICKED) {
    // switch knob mode (change tempo vs change octave)
  }
  else if(modeButtonState == BUTTON_DOUBLE_CLICKED) {
    // switch instrument mode (freestyle vs loop)
    mode = (mode == MODE_FREESTYLE ? MODE_LOOP : MODE_FREESTYLE);
  }

  if(mode == MODE_LOOP) {
    tickMetronomeLight();
  }
  else {
    disableMetronomeLight();
  }


  if(toneIndexChanged) {
    updateHighlightedTone(currentToneIndex);
  }

  if(mode == MODE_FREESTYLE) {
    if(toneIndexChanged || playButtonChanged || sharpButtonChanged) {
      updatePlayingToneFromCurrentToneIndex();
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

int checkModeButtonState() {
  int buttonDown = digitalRead(modePin);
  int wasButtonPress = button_press(buttonDown);
  int buttonState = interpretButtonPress(wasButtonPress);
  return buttonState;
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

void updatePlayingToneFromCurrentToneIndex() {
  int newTone;
  if(currentToneIndex == -1 || !playButtonPressed) {
    newTone = -1;
  }
  else if(currentToneIndex >= 0) {
    newTone = currentToneIndex >= 0 ? scales[currentScale][currentToneIndex] : currentToneIndex;
    if(sharpPressed) {
      newTone = (int)((double)newTone * HALF_STEP_RATIO);
    }
  }
  playTone(newTone);
}

void playTone(int newTone) {
  if(newTone >= 0 && (!playing || newTone != playingTone)) {
    notePlayer[0].play(newTone);
    playing = true;
    playingTone = newTone;
  }
  else if(newTone < 0 && playing) {
    notePlayer[0].stop();
    playing = false;
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
      int newTone = currentSong[currentLoopPosition];
      playTone(newTone);
    }
    else {
      updatePlayingToneFromCurrentToneIndex();
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

int determineSampledNoteValue() {
  // do a really dumb O(n^2) search through the array to
  // find the first value with >= 50% representation.
  int minSamples = NUM_SAMPLES_PER_NOTE / 2;

  for(int i = 0; i < NUM_SAMPLES_PER_NOTE; i++) {
    int candidateValue = samples[i];
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

void disableMetronomeLight() {
  metronomeOn = false;
  analogWrite(metronomeLightPin, 0);
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
const int NOTE_CHANGE_THRESHOLD = 9999;

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

///////////////////////////////////
// below adapted from https://github.com/jerwil/RTC_demo_rotary_alarm

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

int button_press_initiated[1];
int button_press_completed[1];

int button_press(int button_indicator) {
  if (button_indicator == 0 && button_press_initiated[0] == 1) {
    button_press_completed[0] = 1;
    button_press_initiated[0] = 0;
  }
  else if (button_indicator == 1) {
    button_press_initiated[0] = 1;
    button_press_completed[0] = 0;
  }
  else {
    button_press_completed[0] = 0;
  }
  return button_press_completed[0];
}

unsigned long double_click_timeout;
int click_once;

int interpretButtonPress(int button_pushed) {
  int click_complete = 0;
  int double_click_complete = 0;
  if (button_pushed == 1) {
    if (click_once == 1 && millis() <= double_click_timeout + DOUBLE_CLICK_WINDOW) {
      double_click_complete = 1;
      click_once = 0;
    }
    else if (click_once == 0) {
      click_once = 1;
      double_click_complete = 0;
      double_click_timeout = millis();
    }
  }
  else if (button_pushed == 0 && millis() >= double_click_timeout + DOUBLE_CLICK_WINDOW) {
    if(click_once) {
      click_complete = 1;
    }
    click_once = 0;
    double_click_complete = 0;
  }
  if(click_complete) {
    return BUTTON_CLICKED;
  }
  else if(double_click_complete) {
    return BUTTON_DOUBLE_CLICKED;
  }
  else {
    return BUTTON_NO_CLICK;
  }
}
