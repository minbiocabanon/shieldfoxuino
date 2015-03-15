// shieldfox library - inspired by http://akeru.cc
//
// copyleft 2014
// this code is public domain, enjoy!

#ifndef SHIELDFOX_H
#define SHIELDFOX_H

#include <Arduino.h>
#include <SoftwareSerial.h>

class Shieldfox_ {
    public:
        Shieldfox_();
        ~Shieldfox_();
        void begin();
        bool isReady();
        bool send(const void* data, uint8_t len);
        uint8_t getRev();
        unsigned long getID();
        bool setPower(uint8_t power);

        enum RETURN_CODE {
            OK = 'O',
            KO = 'K',
            SENT = 'S'
        };

    private:
        SoftwareSerial _serial;
        unsigned long _lastSend;

        uint8_t _nextReturn();
};

extern Shieldfox_ Shieldfox;

#endif