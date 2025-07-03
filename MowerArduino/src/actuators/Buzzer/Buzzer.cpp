/**
 * @file Buzzer.cpp
 * @brief Implementation of the Buzzer class with advanced sound capabilities
 */

#include "Buzzer.h"

Buzzer buzzer;

// Melodie predefinite
const int startupMelody[] = {
  NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5, 0
};
const int startupNoteDurations[] = {
  EIGHTH, EIGHTH, EIGHTH, QUARTER, QUARTER
};

const int shutdownMelody[] = {
  NOTE_C5, NOTE_G4, NOTE_E4, NOTE_C4, 0
};
const int shutdownNoteDurations[] = {
  EIGHTH, EIGHTH, EIGHTH, QUARTER, QUARTER
};

const int successMelody[] = {
  NOTE_E6, NOTE_G6, 0
};
const int successNoteDurations[] = {
  EIGHTH, QUARTER, EIGHTH
};

const int errorMelody[] = {
  NOTE_A2, 0, NOTE_A2, 0, NOTE_A2, 0
};
const int errorNoteDurations[] = {
  EIGHTH, EIGHTH, EIGHTH, EIGHTH, QUARTER, QUARTER
};

const int warningMelody[] = {
  NOTE_E5, NOTE_D5, 0
};
const int warningNoteDurations[] = {
  EIGHTH, QUARTER, EIGHTH
};

const int alarmMelody[] = {
  NOTE_A5, 0, NOTE_A5, 0, NOTE_A5, 0, NOTE_F5, 0, NOTE_C6, 0, NOTE_A5, 0, 
  NOTE_F5, 0, NOTE_C6, 0, NOTE_A5, 0
};
const int alarmNoteDurations[] = {
  EIGHTH, EIGHTH, EIGHTH, EIGHTH, EIGHTH, EIGHTH, 
  EIGHTH, EIGHTH, QUARTER, EIGHTH, QUARTER, EIGHTH, 
  EIGHTH, EIGHTH, QUARTER, EIGHTH, QUARTER, QUARTER
};

const int buttonPressMelody[] = {
  NOTE_C6, 0
};
const int buttonPressNoteDurations[] = {
  SIXTEENTH, SIXTEENTH
};

const int modeChangeMelody[] = {
  NOTE_C5, NOTE_G5, 0
};
const int modeChangeNoteDurations[] = {
  EIGHTH, QUARTER, EIGHTH
};

const int lowBatteryMelody[] = {
  NOTE_C4, 0, NOTE_C4, 0, NOTE_C4, 0
};
const int lowBatteryNoteDurations[] = {
  EIGHTH, EIGHTH, EIGHTH, EIGHTH, QUARTER, QUARTER
};

const int obstacleDetectedMelody[] = {
  NOTE_C5, 0, 0, 0, 0, 0, 0, 0
};
const int obstacleDetectedNoteDurations[] = {
  SIXTEENTH, SIXTEENTH, SIXTEENTH, SIXTEENTH, SIXTEENTH, SIXTEENTH, SIXTEENTH, SIXTEENTH
};

Buzzer::Buzzer() {
    _isPlaying = false;
    _isRepeating = false;
    _repeatCount = 0;
    _currentNote = 0;
    _notePlaying = false;
}

void Buzzer::begin() {
    pinMode(BUZZER_PIN, OUTPUT);
    _currentMelodyNotes = nullptr;
    _currentMelodyDurations = nullptr;
    _melodyLength = 0;
}

void Buzzer::_playNote(int frequency, int duration) {
    if (frequency > 0) {
        tone(BUZZER_PIN, frequency, duration);
    } else {
        noTone(BUZZER_PIN);
    }
    _noteStartTime = millis();
    _notePlaying = (frequency > 0);
}

void Buzzer::_stopNote() {
    noTone(BUZZER_PIN);
    _notePlaying = false;
}

void Buzzer::_playNextNote() {
    if (!_currentMelodyNotes || _currentNote >= _melodyLength) {
        // Fine della melodia corrente
        if (_isRepeating && _repeatCount > 0) {
            _repeatCount--;
            _currentNote = 0;
            if (_repeatCount == 0) {
                _isRepeating = false;
                _isPlaying = false;
                return;
            }
        } else {
            _isPlaying = false;
            return;
        }
    }

    int noteDuration = _currentMelodyDurations[_currentNote];
    _playNote(_currentMelodyNotes[_currentNote], noteDuration);
    _currentNote++;
}

void Buzzer::stop() {
    _stopNote();
    _isPlaying = false;
    _isRepeating = false;
    _repeatCount = 0;
}

void Buzzer::update() {
    if (!_isPlaying) return;
    
    if (_notePlaying) {
        unsigned long currentTime = millis();
        int currentNoteIndex = (_currentNote > 0) ? _currentNote - 1 : _melodyLength - 1;
        unsigned long noteDuration = _currentMelodyDurations[currentNoteIndex];
        
        if (currentTime - _noteStartTime >= noteDuration) {
            _stopNote();
            _playNextNote();
        }
    } else {
        _playNextNote();
    }
}

void Buzzer::playSound(SoundType soundType, uint8_t repeat) {
    switch(soundType) {
        case SOUND_STARTUP:       startupSound(); break;
        case SOUND_SHUTDOWN:      shutdownSound(); break;
        case SOUND_SUCCESS:       successSound(); break;
        case SOUND_ERROR:         errorSound(); break;
        case SOUND_WARNING:       warningSound(); break;
        case SOUND_ALARM:         alarmSound(); break;
        case SOUND_BUTTON_PRESS:  buttonPressSound(); break;
        case SOUND_MODE_CHANGE:   modeChangeSound(); break;
        case SOUND_LOW_BATTERY:   lowBatterySound(); break;
        case SOUND_OBSTACLE_DETECTED: obstacleDetectedSound(); break;
        default: stop();
    }
    
    if (repeat > 0) {
        _isRepeating = true;
        _repeatCount = repeat;
    }
}

void Buzzer::playMelody(const int *notes, const int *durations, uint8_t length, uint8_t repeat) {
    stop();
    _currentMelodyNotes = notes;
    _currentMelodyDurations = durations;
    _melodyLength = length;
    _currentNote = 0;
    _isPlaying = true;
    _isRepeating = (repeat > 0);
    _repeatCount = repeat;
    _playNextNote();
}

// Suoni predefiniti
void Buzzer::startupSound() {
    playMelody(startupMelody, startupNoteDurations, 
              sizeof(startupMelody) / sizeof(startupMelody[0]));
}

void Buzzer::shutdownSound() {
    playMelody(shutdownMelody, shutdownNoteDurations, 
              sizeof(shutdownMelody) / sizeof(shutdownMelody[0]));
}

void Buzzer::successSound() {
    playMelody(successMelody, successNoteDurations, 
              sizeof(successMelody) / sizeof(successMelody[0]));
}

void Buzzer::errorSound() {
    playMelody(errorMelody, errorNoteDurations, 
              sizeof(errorMelody) / sizeof(errorMelody[0]));
}

void Buzzer::warningSound() {
    playMelody(warningMelody, warningNoteDurations, 
              sizeof(warningMelody) / sizeof(warningMelody[0]));
}

void Buzzer::alarmSound() {
    playMelody(alarmMelody, alarmNoteDurations, 
              sizeof(alarmMelody) / sizeof(alarmMelody[0]));
}

void Buzzer::buttonPressSound() {
    playMelody(buttonPressMelody, buttonPressNoteDurations, 
              sizeof(buttonPressMelody) / sizeof(buttonPressMelody[0]));
}

void Buzzer::modeChangeSound() {
    playMelody(modeChangeMelody, modeChangeNoteDurations, 
              sizeof(modeChangeMelody) / sizeof(modeChangeMelody[0]));
}

void Buzzer::lowBatterySound() {
    playMelody(lowBatteryMelody, lowBatteryNoteDurations, 
              sizeof(lowBatteryMelody) / sizeof(lowBatteryMelody[0]));
}

void Buzzer::obstacleDetectedSound() {
    playMelody(obstacleDetectedMelody, obstacleDetectedNoteDurations, 
              sizeof(obstacleDetectedMelody) / sizeof(obstacleDetectedMelody[0]));
}
