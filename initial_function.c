/*
 * File:   initial_function.c
 * Author: qoo09
 *
 * Created on 2024?12?22?, ?? 9:12
 */
#include <xc.h>
#include <pic18f4520.h>
#include <stdint.h>  
#include <stdbool.h>
#include "initial_function.h"

void init(void) {
    interrupt_init();
    timer_init();
    I2C_Init();
    PCA9685_Init();
    PCA9685_SetPWMFreq(50);
}

void OSCCON_init(void) {
    OSCCON = 0x40;
}

void interrupt_init(void) {
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB4 = 1;
    PORTBbits.RB0=0;
    PORTBbits.RB4=0;
    INTCON2bits.RBPU = 1;     // Disable port B pull-ups
    INTCONbits.RBIF = 0;      // Clear RB interrupt flag
    INTCONbits.RBIE = 1;      // Enable RB interrupt
    INTCONbits.GIE = 1; //General INT enable
    INTCONbits.PEIE = 1;
    
    TRISBbits.RB0 = 1;
    INTCON2bits.INTEDG1 = 0;
    INTCONbits.INT0IE = 1;
    INTCONbits.INT0IF = 0;
}

void timer_init(void) {
    T1CON = 0b00010000;    // Timer1 with 1:4 prescaler
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 0;  // Timer1 initially off
    PIR1bits.TMR1IF = 0;   // Clear Timer1 interrupt flag
}

void I2C_Init(void) {
    TRISC3 = 1; // SCL ????
    TRISC4 = 1; // SDA ????
    SSPCON1 = 0x28; // ?? I²C ????
    SSPADD = ((_XTAL_FREQ / 4) / I2C_BAUD) - 1; // ?????
    SSPSTAT = 0;
}

void I2C_Start(void) {
    SEN = 1; // ????
    while (SEN); // ????
}

void I2C_Stop(void) {
    PEN = 1; // ????
    while (PEN); // ????
}

void I2C_Write(unsigned char data) {
    SSPBUF = data; // ????
    while (BF);     // ????
    while (SSPCON2 & 0x1F); // ??????
}

unsigned char I2C_Read(unsigned char ack) {
    RCEN = 1; // ??????
    while (!BF); // ????
    unsigned char data = SSPBUF; // ????
    ACKDT = (ack) ? 0 : 1; // ????
    ACKEN = 1; // ??????
    while (ACKEN); // ????
    return data;
}

void PCA9685_Init(void) {
    I2C_Start();
    I2C_Write(0x40 << 1); // ?? PCA9685 ??
    I2C_Write(0x00);      // MODE1 ???
    I2C_Write(0x20);      // ?? AI = 1?????
    I2C_Stop();
    __delay_ms(10);
}

void PCA9685_SetPWMFreq(unsigned char freq) {
    unsigned char prescale = (25000000.0 / (4096.0 * freq)) - 1.0;
    I2C_Start();
    I2C_Write(0x40 << 1);
    I2C_Write(0x00); // MODE1 
    I2C_Write(0x10); // SLEEP
    I2C_Stop();

    I2C_Start();
    I2C_Write(0x40 << 1);
    I2C_Write(0xFE); // PRE_SCALE ???
    I2C_Write(prescale);
    I2C_Stop();

    I2C_Start();
    I2C_Write(0x40 << 1);
    I2C_Write(0x00);
    I2C_Write(0x20); // WAKE UP
    I2C_Stop();
}

void PCA9685_SetPWM(unsigned char channel, int angle) {
    float pulse_width = ((float)(angle + 90) / 180.0) * (2400.0 - 500.0) + 500.0;
    unsigned int off = (unsigned int)(pulse_width / 4.88);
    I2C_Start();
    I2C_Write(0x40 << 1);                 // ????????
    I2C_Write(0x06 + 4 * channel);        // ?? PWM ?????
    I2C_Write(0x00);                      // ON_L??? 0?
    I2C_Write(0x00);                      // ON_H??? 0?
    I2C_Write(off & 0xFF);               // OFF_L?? 8 ??
    I2C_Write(off >> 8);                 // OFF_H?? 8 ??
    I2C_Stop();
}