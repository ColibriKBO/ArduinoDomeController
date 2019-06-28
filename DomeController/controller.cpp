/******************************************************************
    Author:     Juan Menendez Blanco    <juanmb@gmail.com>

    This code is part of the ArduinoDomeController project:
        https://github.com/juanmb/ArduinoDomeController

*******************************************************************/

#include <Arduino.h>
#include "controller.h"
#include <util/atomic.h>

#define MOTOR_CW 0
#define MOTOR_CCW 1
#define SPEED 1023
#define DEFAULT_TIMEOUT 30000 // controller timeout (in ms)

// Controller constructor.
// motor: pointer to an instance of Motor
// sw1: Limit switch (closed)
// sw2: Limit switch (fully open)
// swInt: Interference switch
Controller::Controller(Motor *motorPtr, int homedSwitch, unsigned long timeout)
{
    motor = motorPtr;
    swHomed = homedSwitch;    // normally closed (1 if shutter is closed)
    runTimeout = timeout;
    nextAction = DO_NONE;
    initState();
}


void Controller::initState()
{
    if (digitalRead(swHomed))
        state = ST_HOMED;
    else
        state = ST_ABORTED;
}


void Controller::cw() { nextAction = DO_CW; }
void Controller::ccw() { nextAction = DO_CCW; }
void Controller::abort() { nextAction = DO_ABORT; }

State Controller::getState() { return state; }


// Controller state machine
void Controller::update()
{
    Action action;
    static unsigned long t0;

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        action = nextAction;
        nextAction = DO_NONE;
    }

    switch (state) {
    case ST_CCW:
        if (action == DO_CW) {
            t0 = millis();
            state = ST_CW;
        }
        break;
    case ST_CW:
        if (action == DO_CCW) {
            t0 = millis();
            state = ST_CCW;
        }
        break;
    case ST_ABORTED:
    case ST_ERROR:
        if (action == DO_CW) {
            t0 = millis();
            state = ST_CW;
        } else if (action == DO_CCW) {
            t0 = millis();
            state = ST_CCW;
        }
        break;
    case ST_CWING:
        if (interference(state))
            motor->brake();
        else
            motor->run(MOTOR_CW, SPEED);

        if (action == DO_ABORT || action == DO_CCW) {
            state = ST_ABORTED;
            motor->brake();
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->brake();
        }
        break;
    case ST_CCWING:
        if (interference(state))
            motor->brake();
        else
            motor->run(MOTOR_CCW, SPEED);

        if (digitalRead(swHomed)) {
            state = ST_CCW;
            motor->brake();
        } else if (action == DO_ABORT || action == DO_CW) {
            state = ST_ABORTED;
            motor->brake();
        } else if (millis() - t0 > runTimeout) {
            state = ST_ERROR;
            motor->brake();
        }
        break;
    }
}
