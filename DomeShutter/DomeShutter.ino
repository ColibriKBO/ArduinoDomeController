/*******************************************************************************
ArduinoDomeShutter
Shutter controller for an astronomical dome using Arduino
Adapted for Exploradome EDII by Michael Mazur


The MIT License

Copyright (C) 2017 Juan Menendez <juanmb@gmail.com>
Copyright (C) 2019 Michael Mazur <mjmazur@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*******************************************************************************/

#include <avr/wdt.h>
#include "MonsterMotorShield.h"
#include "SerialCommand.h"
#include "shutter.h"

// Pin definitions
#define LED_ERR  13     // error LED
#define SW_A1    12     // shutter closed switch (NC)
#define SW_A2    11     // shutter open switch (NO)
#define BUTTONS  A4     // analog input for reading the buttons
#define VBAT_PIN A5     // battery voltage reading

#define BUTTON_REPS 80  // Number of ADC readings required to detect a pressed button

// Timeouts in ms
#define COMMAND_TIMEOUT 60000   // Max. time from last command
#define SHUTTER_TIMEOUT 75000   // Max. time the shutter takes to open/close

enum {
    BTN_NONE,
    BTN_A_OPEN,
    BTN_A_CLOSE
};

Motor motorA(0);
Shutter shutter(&motorA, SW_A1, SW_A2, SHUTTER_TIMEOUT);
SerialCommand sCmd;

unsigned long lastCmdTime = 0;

// Detect a pressed button by reading an analog input.
// Every button puts a different voltage at the input.
int readButton()
{
    int buttonLimits[] = {92, 303, 518, 820};
    int val = analogRead(BUTTONS);

    for (int i = 0; i < 4; i++) {
        if (val < buttonLimits[i]) {
            return i + 1;
        }
    }
    return 0;
}

// Return dome status by combining shutter and flap statuses
State domeStatus()
{
    State sst = shutter.getState();

    if (sst == ST_ERROR)
        return ST_ERROR;
    else if (sst == ST_OPENING)
        return ST_OPENING;
    else if (sst == ST_CLOSING)
        return ST_CLOSING;
    else if (sst == ST_OPEN)
        return ST_OPEN;
    else if (sst == ST_CLOSED)
        return ST_CLOSED;

    return ST_ABORTED;
}

void cmdOpenShutter() {
    lastCmdTime = millis();
    shutter.open();
}

void cmdClose()
{
    lastCmdTime = millis();
    shutter.close();
}

void cmdAbort()
{
    lastCmdTime = millis();
    shutter.abort();
}

void cmdExit()
{
    lastCmdTime = 0;
    shutter.close();
}

void cmdStatus()
{
    lastCmdTime = millis();
    State st = domeStatus();
    Serial.write('0' + st);
}

void cmdGetVBat()
{
    lastCmdTime = millis();
    int val = analogRead(VBAT_PIN);
    char buffer[8];
    sprintf(buffer, "v%04d", val);
    Serial.write(buffer);
}

void setup()
{
    wdt_disable(); // Disable the watchdog timer
    wdt_enable(WDTO_1S); // Enable the watchdog timer to fire after a 1S freeze/hang/stall/crash

    pinMode(LED_ERR, OUTPUT);

    // Map serial commands to functions
    sCmd.addCommand("open", cmdOpenShutter);
    sCmd.addCommand("close", cmdClose);
    sCmd.addCommand("abort", cmdAbort);
    sCmd.addCommand("exit", cmdExit);
    sCmd.addCommand("stat", cmdStatus);
    sCmd.addCommand("vbat", cmdGetVBat);

    Serial.begin(9600);

    digitalWrite(LED_ERR, HIGH);
    delay(200);
    digitalWrite(LED_ERR, LOW);
}


void loop()
{
    int btn = readButton();
    static int btn_prev = 0;
    static int btn_count = 0;

    if (btn && (btn == btn_prev))
        btn_count++;
    else
        btn_count = 0;
    btn_prev = btn;

    if (btn_count == BUTTON_REPS) {
        switch(btn) {
        case BTN_A_OPEN:
            shutter.open();
            break;
        case BTN_A_CLOSE:
            shutter.close();
            break;
        }
    }

    // Close the dome if time from last command > COMMAND_TIMEOUT
    if ((lastCmdTime > 0) && ((millis() - lastCmdTime) > COMMAND_TIMEOUT)) {
        if (domeStatus() != ST_CLOSED) {
            lastCmdTime = 0;
            shutter.close();
        }
    }

    int err = (shutter.getState() == ST_ERROR);
    digitalWrite(LED_ERR, err);

    shutter.update();
    sCmd.readSerial();

    wdt_reset();
}
