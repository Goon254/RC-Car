#include <SoftwareSerial.h>
#include "protothreads.h" // Include Protothreads library

// Pins for joystick and Bluetooth
//*******************************************************
#define JOYSTICK_X A1
#define JOYSTICK_Y A0
#define JOYSTICK_BUTTON 2

#define SPEAKER_PIN 9  // Speaker pin for sound feedback
#define DEADZONE 100   // Threshold for joystick sensitivity
#define MAX_SPEED 150  // Maximum speed

SoftwareSerial BTserial(10, 11); // TX, RX pins for Bluetooth

// Protothreads setup
static struct pt ptJoystick, ptSound;

// Function declarations
//**********************************************************
static int handleJoystick(struct pt *pt);
static int playSound(struct pt *pt);
void playMelody();

// Global variables
//**********************************************************
static String commandBuffer = "";  // Buffer to store commands
static bool buttonState = false;    // State of button
static bool previousButtonState = true; // Previous state of button
static bool musicPlaying = false;   // Toggle for music
//Music
//**********************************************************
const float songSpeed = 1.0;
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_B5 988

// Music notes of "He's a Pirate"
//**********************************************************
int notes[] = {
    NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0
};

int durations[] = {
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 250, 125, 125,
    125, 125, 375, 125
};

void setup() {
  // Initialize serial communication
  //**********************************************************
  Serial.begin(9600);
  BTserial.begin(38400);

  // Pin configurations
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
  pinMode(SPEAKER_PIN, OUTPUT);

  Serial.println("Joystick ready with Protothreads!");

  // Initialize protothreads
  PT_INIT(&ptJoystick);
  PT_INIT(&ptSound);
}

void loop() {
  // Schedule and run protothreads
  //**********************************************************
  PT_SCHEDULE(handleJoystick(&ptJoystick) &
              playSound(&ptSound));
}

//Protothread for handling joystick input
//********************************************************** 
static int handleJoystick(struct pt *pt) {
  PT_BEGIN(pt);

  static int xVal, yVal, speed;
  static String direction;

  while (1) {
    xVal = analogRead(JOYSTICK_X) - 512; // Centered at 0
    yVal = analogRead(JOYSTICK_Y) - 512;

    speed = map(max(abs(xVal), abs(yVal)), 0, 512, 0, MAX_SPEED); // Map speed
    direction = "";

    // Calculate direction
    if (abs(xVal) > DEADZONE || abs(yVal) > DEADZONE) {
      if (abs(yVal) > abs(xVal)) {
        direction = (yVal > 0) ? "F" : "B"; // Forward or Backward
      } else {
        direction = (xVal > 0) ? "R" : "L"; // Right or Left
      }
    } else {
      direction = "S"; // Stop
      speed = 0;
    }

    // Combine direction and speed, send to Bluetooth
    //**********************************************************
    commandBuffer = direction + ":" + String(speed); // No extra \n
    BTserial.println(commandBuffer); // Send command
    Serial.println("Sent: " + commandBuffer);

    PT_SLEEP(pt, 200); // Adjusted sleep to balance responsiveness and reliability
  }

  PT_END(pt);
}

// Protothread for toggling music on button press 
//**********************************************************
static int playSound(struct pt *pt) {
  PT_BEGIN(pt);

  while (1) {
    buttonState = (digitalRead(JOYSTICK_BUTTON) == LOW); // Read button state

    // Detect button press and release
    if (buttonState == LOW && previousButtonState == HIGH) {
      musicPlaying = !musicPlaying; // Toggle music state
      if (musicPlaying) {
        Serial.println("Button Pressed: Playing Music");
        BTserial.println("MUSIC_ON");
        playMelody();
      } else {
        Serial.println("Button Pressed: Stopping Music");
        BTserial.println("MUSIC_OFF");
        digitalWrite(SPEAKER_PIN, LOW); // Stop the sound
      }
    }

    previousButtonState = buttonState; // Update previous state
    PT_SLEEP(pt, 50); // Check button state every 50ms
  }

  PT_END(pt);
}

// Function to play the "He's a Pirate" melody
//**********************************************************
void playMelody() {
  for (int i = 0; i < sizeof(notes) / sizeof(notes[0]); i++) {
    int noteDuration = durations[i] * songSpeed;
    if (notes[i] == 0) {
      delay(noteDuration); // Rest
    } else {
      tone(SPEAKER_PIN, notes[i], noteDuration);
      delay(noteDuration * 1.3); // Add pause between notes
    }
  }
  noTone(SPEAKER_PIN); // Stop the tone after the melody
}
