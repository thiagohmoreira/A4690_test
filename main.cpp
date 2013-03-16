/* 
 * File:   main.cpp
 * Author: firegun
 * 
 * Created on 12 de Março de 2013, 23:09
 */

#include "main.h"

#include <HardwareSerial.h>
#include <SPI.h>

void setup() {
    //Set pin modes
    //PWM pin don´t need pinMode call ("You do not need to call pinMode() to set the pin as an output before calling analogWrite().")
    pinMode(TACHO_PIN, INPUT);
    pinMode(CS_PIN, OUTPUT);
    
    pinMode(SW2_PIN, INPUT_PULLUP);
    pinMode(SW3_PIN, INPUT_PULLUP);

    Serial.begin(9600);

    SPI.begin();
    //SPI Config (Datasheet, pg. 10,23,24)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3); //Clock pol = 1 | Clock phase = 1
    //SPI.setClockDivider(SPI_CLOCK_DIV4); //Default 4 MHz speed should be OK
    
    getConfigs();
    initConfigs();
    getConfigs();
}

void loop() {
    uint32_t now = millis();

    //Check for buttons changes
    checkButtons();
    
    //Check for errors
    uint16_t diag = spiTransfer(DIAG_R);
    //If there is any error
    if ((diag & ERROR_MASK) > 0) {
        printError(diag);
    }

    //Read tacho
    readTacho(now);

    //Adjust PWM
    if (bitRead(state, 0) == 1 && pwmValue < 255 && now - lastPWMUpdate > 1000) {
        analogWrite(PWM_PIN, ++pwmValue);
        lastPWMUpdate = now;
    }

    //Send serial info, every 500ms
    if (now - lastSerialUpdate > 500) {
        Serial.print("Time: ");
        Serial.print(now);
        Serial.print(" - State: ");
        Serial.print(bitRead(state, 0));
        Serial.print(" - RPM: ");
        Serial.print(rpm);
        Serial.print(" - PWM: ");
        Serial.println(pwmValue);

        lastSerialUpdate = now;
    }

    //...
}

void initConfigs() {
    //Set A4960 initial parameters
    //CFG 0: Blank
    Serial.print("Set CFG 0: ");
    Serial.println(CFG_0_W | 0b101010010100, BIN);
    spiTransfer(CFG_0_W | 0b101010010100);
    delay(50);

    //CFG 1: Current and Voltage Limits
    Serial.print("Set CFG 1: ");
    Serial.println(CFG_1_W | 0b001111101000, BIN);
    spiTransfer(CFG_1_W | 0b001111101000);
    delay(50);

    //CFG 2: Current and Voltage Limits
    Serial.print("Set CFG 2: ");
    Serial.println(CFG_2_W | 0b000000010000, BIN);
    spiTransfer(CFG_2_W | 0b000000010000);
    delay(50);

    //CFG 3: Start-up Parameters 1
    Serial.print("Set CFG 3: ");
    Serial.println(CFG_3_W | 0b000000000100, BIN);
    spiTransfer(CFG_3_W | 0b000000000100);
    delay(50);

    //CFG 4: Start-up Parameters 2
    Serial.print("Set CFG 4: ");
    Serial.println(CFG_4_W | 0b000001110100, BIN);
    spiTransfer(CFG_4_W | 0b000001110100);
    delay(50);

    //CFG 5: Ramp
    Serial.print("Set CFG 5: ");
    Serial.println(CFG_5_W | 0b000111110010, BIN);
    spiTransfer(CFG_5_W | 0b000111110010);
    delay(50);
}

void getConfigs() {
    uint16_t response;

    response = spiTransfer(CFG_0_R);
    Serial.print("Got CFG 0: ");
    Serial.println(response, BIN);
    
    response = spiTransfer(CFG_1_R);
    Serial.print("Got CFG 1: ");
    Serial.println(response, BIN);

    response = spiTransfer(CFG_2_R);
    Serial.print("Got CFG 2: ");
    Serial.println(response, BIN);

    response = spiTransfer(CFG_3_R);
    Serial.print("Got CFG 3: ");
    Serial.println(response, BIN);

    response = spiTransfer(CFG_4_R);
    Serial.print("Got CFG 4: ");
    Serial.println(response, BIN);

    response = spiTransfer(CFG_5_R);
    Serial.print("Got CFG 5: ");
    Serial.println(response, BIN);
}

void checkButtons() {
    //If button pressed
    if (digitalRead(SW2_PIN) == LOW) {
        if (bitRead(state, 0)) {
            motorCoast();
            bitClear(state, 0);
        } else {
            motorRun();
            bitSet(state, 0);
        }
    }
}

uint16_t motorRun() {
    Serial.println("Running...");
    //spiTransfer(RUN_W | 0b001000001011); //Reverse
    return spiTransfer(RUN_W | 0b001000001001); //Run
}

uint16_t motorCoast() {
    Serial.println("Coast...");
    return spiTransfer(RUN_W | 0b001000001000); //Coast
    analogWrite(PWM_PIN, 0);
}

uint16_t motorBrake() {
    Serial.println("Brake...");
    return spiTransfer(RUN_W | 0b001000001101); //Break
}

/**
 * Read current RPM
 * Note: Tacho must change 6 times for one revolution
 * @TODO: Improve this (with filter, moving avg...)
 * @TODO: this would be more reliable using some sort of interruption
 * 
 * @param now
 */
void readTacho(uint64_t now) {
    uint8_t currentState = digitalRead(TACHO_PIN);
    if (currentState != tachoState) {
        tachoState = currentState; //Update tacho state
        revolutionPhase++;

        //One revolution completed
        if (revolutionPhase > 6) {
            //rpm = (revolution_time_in_ms) / one_minute_in_ms ;
            rpm = (now - revolutionStart) / 60000;

            revolutionPhase = 1; //Reset the phase
            revolutionStart = now; //Set the start time of this revolution
        }
    }
}

/**
 * This function will transfer 16 bits to the device and return the read bits.
 * 
 * @param value Data to be transfered
 * @return The bits read from the slave device
 */
uint16_t spiTransfer(uint16_t value) {
    uint16_t ret = 0U;

    digitalWrite(CS_PIN, LOW);

    ret |= (SPI.transfer(highByte(value)) << 8);
    ret |= SPI.transfer(lowByte(value));

    digitalWrite(CS_PIN, HIGH);

    //If there is any error
    if ((ret & ERROR_MASK) > 0) {
        Serial.print("Sent: ");
        Serial.print(value, BIN);
        Serial.print(" - ");
        printError(ret);
    }

    return ret;
}

void printError(uint16_t diag) {
    Serial.print("Error: ");
    Serial.println(diag, BIN);
}