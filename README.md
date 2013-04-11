# theremax

A theremin-like instrument for Arduino, using a HC-SR04 ultrasonic ranging module.

![](https://github.com/mddub/theremax/blob/master/photos/IMG_5648-450x300.jpg?raw=true "Theremax")

## Demo

* [Demo video](http://youtu.be/Jf77zi0XjPk)

## Features

* Play notes of a C major scale based on distance from instrument
* 6 octaves, adjustable by rotary encoder
* Sharp button (modulate up half a step)
* Loop recording mode with metronome light and tempo adjustable by rotary encoder

## Requirements

### Hardware

* Arduino Uno
* HC-SR04 ultrasonic ranging module
* Speaker/buzzer
* Rotary encoder with built-in push button
* 74HC595 shift register
* 9 LEDs
* 2 push buttons

See the included Fritzing diagram for details (+ wires, resistors, etc.).

### Arduino Libraries

* [NewPing][1] (for ultrasonic sensor)
* [Tone][2] (for speaker)

## TODO/Wishlist
* Fix some intermittent tempo issues during loop playback
* Indicator of current recording bar on LEDs (using [ShiftPWM][3])
* Record/playback two tones at once ([Tone][2] supports this)

## Etc.

* [A time-lapse video of us prototyping and building this](http://youtu.be/0IkWHqgWssk)

[1]: http://playground.arduino.cc/Code/NewPing
[2]: https://code.google.com/p/rogue-code/wiki/ToneLibraryDocumentation
[3]: http://www.elcojacobs.com/shiftpwm/
