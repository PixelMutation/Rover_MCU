#include "Arduino.h"

#define MAX_DURATION 10

class Mast {
    int step,dir,en;
    bool reverse;
    uint freq;

    void run(uint duration_seconds) {
        // limit max duration to protect motor
        duration_seconds=constrain(duration_seconds,0,MAX_DURATION);
        // set this up each time to ensure the frequency is sane...
        analogWriteFreq(freq);
        analogWriteRange(255);

        digitalWrite(en,LOW); // enable motor
        analogWrite(step,127); // start stepping
        sleep_ms(1000*duration_seconds); // wait until complete
        analogWrite(step,0);
        digitalWrite(en,HIGH); // disable motor
    }
public:
    Mast(int step, int dir, int en, bool reverse, uint freq) :
    step(step),dir(dir),en(en),reverse(reverse),freq(freq)
    {
        pinMode(step,OUTPUT);
        pinMode(dir,OUTPUT);
        pinMode(en,OUTPUT);
    }

    void extend(uint duration_seconds=5) {
        digitalWrite(dir,reverse);
        run(duration_seconds);
    }

    void extend(uint duration_seconds=5) {
        digitalWrite(dir,!reverse);
        run(duration_seconds);
    }
};