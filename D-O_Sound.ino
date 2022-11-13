/************************************************************************
  D-O Sound for HotRC D600
  by Bithead942

  Board:  Arduino Pro Mini

  Controls Sound for D-O

  Pins:
   0 - Serial TX
   1 - Serial RX
   2 -
   3 - H Bridge - Motor A PWM
   4 - Adafruit Sound FX Board - Reset Pin
   5 - Adafruit Sound FX Board - TX Pin
   6 - Adafruit Sound FX Board - RX Pin
   7 - RC Channel - Trigger audio
   8 - RC Channel - Set Mood
   9 - DIO - Reverse

*************************************************************************/

#include <SoftwareSerial.h>
#include "Adafruit_Soundboard.h"

const int _chSoundTrigger = 7;
const int _chMoodSet = 8;
const int _Reverse = 9;

// Choose any two pins that can be used with SoftwareSerial to RX & TX
#define SFX_TX 5
#define SFX_RX 6

// Connect to the RST pin on the Sound Board
#define SFX_RST 4

int ReverseState = 1;  //Normally HIGH until pulled LOW
int ReverseVal = 0;
int MoodMode = 0;
int LastSound = 0;

// Software Serial Communication to SFX Board
SoftwareSerial ss = SoftwareSerial(SFX_TX, SFX_RX);

// pass the software serial to Adafruit_soundboard, the second
// argument is the debug port (not used really) and the third
// arg is the reset pin
Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, SFX_RST);

void setup() {
  pinMode(_chSoundTrigger, INPUT);   //RC channel for Sound Trigger
  pinMode(_chMoodSet, INPUT);   //RC channel for Mood
  pinMode(_Reverse, INPUT_PULLUP);  //DIO channel for Reverse

  // initialize Serial ports
  //Serial.begin(9600);
  // softwareserial at 9600 baud
  ss.begin(9600);

  sfx.volUp();
  sfx.volUp();
  sfx.volUp();
  sfx.volUp();
  sfx.volUp();
  sfx.volUp();
  //Serial.println("Ready.");
  sfx.playTrack((uint8_t)0);
}

void loop() {

  int SoundTrigger = pulseIn(_chSoundTrigger, HIGH, 25000); // RC channel for Sound Trigger
  delay(30);   // required or will read next as 0
  int MoodSet = pulseIn(_chMoodSet, HIGH, 25000); // RC channel for Mood

  if (SoundTrigger > 1500) {
    PlayAudio();
  }
  if (MoodSet > 0) {
    ChangeMood(MoodSet);
  }

  ReverseVal = digitalRead(_Reverse);
  if (ReverseVal == LOW && ReverseState == 1) {   // Normally HIGH until pulled low externally
    sfx.playTrack(8);
    ReverseState = 0;
  }
  else if (ReverseVal == HIGH && ReverseState == 0) {
    ReverseState = 1;
  }

  /*Serial.print(SoundTrigger);
    Serial.print(", ");
    Serial.print(MoodSet);
    Serial.print(", ");
    Serial.print(MoodMode);
    Serial.print(", ");
    Serial.print(ReverseState); 
    Serial.print(", ");
    Serial.println(LastSound); */

  delay(10);
}

/*********************************
   Play Sound
 *********************************/
void PlayAudio()
{
  uint8_t x = LastSound;
  
  if (MoodMode == 0)  //Positive
  {
    //Pick random number between 1 and 7
    //x = random(1, 8); // start (inclusive), end (exclusive)
    while (x == LastSound) {
      x = random(1, 8); // start (inclusive), end (exclusive)
    }
  }
  else                //Negative
  {
    //Pick random number between 9 and 18
    //x = random(9, 19); // start (inclusive), end (exclusive)
    while (x == LastSound) {
      x = random(9, 19); // start (inclusive), end (exclusive)
    }
  }
  LastSound = x;
  sfx.playTrack(x);
  delay(2000);  //Let audio play before checking button press again
}

/*********************************
   Change Mood
 *********************************/
void ChangeMood(int MoodSet)
{
  if (MoodSet < 1500) {
    MoodMode = 1;
    //sfx.playTrack(13);  //Sad
  }
  else {
    MoodMode = 0;
    //sfx.playTrack(6);  //Happy
  }
}
