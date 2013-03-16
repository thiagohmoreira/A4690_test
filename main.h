/* 
 * File:   main.h
 * Author: firegun
 *
 * Created on 12 de Março de 2013, 23:09
 */

#ifndef MAIN_H
#define	MAIN_H

#include <Arduino.h>

//A4690 registers
#define WRITE_MASK (1U << 12)
#define ERROR_MASK (1U << 15)

#define CFG_0_R (0b000 << 13)
#define CFG_1_R (0b001 << 13)
#define CFG_2_R (0b010 << 13)
#define CFG_3_R (0b011 << 13)
#define CFG_4_R (0b100 << 13)
#define CFG_5_R (0b101 << 13)
#define RUN_R   (0b111 << 13)
#define DIAG_R  (0b110 << 13)

#define CFG_0_W (CFG_0_R | WRITE_MASK)
#define CFG_1_W (CFG_1_R | WRITE_MASK)
#define CFG_2_W (CFG_2_R | WRITE_MASK)
#define CFG_3_W (CFG_3_R | WRITE_MASK)
#define CFG_4_W (CFG_4_R | WRITE_MASK)
#define CFG_5_W (CFG_5_R | WRITE_MASK)
#define RUN_W   (RUN_R | WRITE_MASK)

//A4690 pin definitions
#define CS_PIN 5
#define PWM_PIN 6
#define TACHO_PIN 7
//Buttons
#define SW2_PIN 3
#define SW3_PIN 4

//App vars
boolean tachoState = LOW;
uint8_t revolutionPhase = 1U;
uint32_t revolutionStart = 0L;
double rpm = 0.0;

uint8_t pwmValue = 0U;

uint32_t lastSerialUpdate = 0L;
uint32_t lastPWMUpdate = 0L;

uint8_t state = 0U;


void initConfigs();
void getConfigs();
void checkButtons();
uint16_t motorRun();
uint16_t motorCoast();
uint16_t motorBrake();
void readTacho(uint64_t now);

uint16_t spiTransfer(uint16_t value);
void printError(uint16_t diag);

#endif	/* MAIN_H */
