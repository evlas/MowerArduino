#ifndef BUZZER_H
#define BUZZER_H

#include "Arduino.h"
#include "config.h"
#include "pin_config.h"

// Definizione delle note musicali
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

// Durata delle note
#define WHOLE 1000
#define HALF 500
#define QUARTER 250
#define EIGHTH 125
#define SIXTEENTH 62

// Tipi di suono predefiniti
typedef enum {
    SOUND_NONE = 0,
    SOUND_STARTUP,
    SOUND_SHUTDOWN,
    SOUND_SUCCESS,
    SOUND_ERROR,
    SOUND_WARNING,
    SOUND_ALARM,
    SOUND_BUTTON_PRESS,
    SOUND_MODE_CHANGE,
    SOUND_LOW_BATTERY,
    SOUND_OBSTACLE_DETECTED
} SoundType;

/**
 * @class Buzzer
 * @brief Controls the buzzer for audio feedback
 * 
 * This class provides methods to control a piezo buzzer for various
 * audio feedback purposes like tones, alarms, and status indicators.
 */
class Buzzer {
  public:
    /**
     * @brief Construct a new Buzzer object
     */
    Buzzer();
    
    /**
     * @brief Initialize the buzzer
     * 
     * Sets up the buzzer pin as an output.
     */
    void begin();
    
    /**
     * @brief Play a button press sound
     * 
     * Plays a short, high-pitched beep to indicate a button press event.
     */
    void buttonPressSound();
    
    /**
     * @brief Stop any ongoing sound
     */
    void stop();
    
    /**
     * @brief Play an alarm sound
     */
    void update();
    
    // Predefined sounds for common events
    void startupSound();
    void shutdownSound();
    void successSound();
    void errorSound();
    void warningSound();
    void alarmSound();
    void modeChangeSound();
    void lowBatterySound();
    void obstacleDetectedSound();
    
  private:
    // Current playing state
    bool _isPlaying;
    bool _isRepeating;
    uint8_t _repeatCount;
    
    // Current melody data
    const int* _currentMelodyNotes;
    const int* _currentMelodyDurations;
    uint8_t _melodyLength;
    uint8_t _currentNote;
    unsigned long _noteStartTime;
    bool _notePlaying;
    
    // Play the next note in the current melody
    void _playNextNote();
    
    // Stop the current note
    void _stopNote();
    
    // Play a single note
    void _playNote(int frequency, int duration);
    
    /**
     * @brief Play a predefined sound
     * @param soundType The type of sound to play
     * @param repeat Number of times to repeat the sound (0 = play once)
     */
    void playSound(SoundType soundType, uint8_t repeat = 0);
    
    /**
     * @brief Play a custom melody
     * @param notes Array of note frequencies (in Hz)
     * @param durations Array of note durations (in ms)
     * @param length Number of notes in the melody
     * @param repeat Number of times to repeat the melody (0 = play once)
     */
    void playMelody(const int *notes, const int *durations, uint8_t length, uint8_t repeat = 0);
  private:
    bool isBeeping;
};

extern Buzzer buzzer;

#endif // BUZZER_H
