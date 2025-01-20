/*
 * File:   main.c
 * Author: qoo09
 *
 * Created on 2024?12?22?, ?? 7:39
 */


#include <xc.h>
#include <pic18f4520.h>
#include <stdint.h>  
#include <stdbool.h>
#include "initial_function.h"

#pragma config OSC = INTIO67    // Oscillator Selection bits
#pragma config WDT = OFF        // Watchdog Timer Enable bit 
#pragma config PWRT = OFF       // Power-up Enable bit
#pragma config BOREN = ON       // Brown-out Reset Enable bit
#pragma config PBADEN = OFF     // Watchdog Timer Enable bit 
#pragma config LVP = OFF        // Low Voltage (single -supply) In-Circuit Serial Programming Enable bit
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config MCLRE = OFF  

#define High 1
#define Low 0
#define True 1
#define False 0

// using namespace std;

int has_ultrasound = False;
enum leg {right_back_big, right_back_small, left_back_big, left_back_small, right_front_big, right_front_small, left_front_big, left_front_small};
//right front big ++
//right back big --
//left front big ++
//left back big --

unsigned int duration = 0;
float distance = 0;
int mode = 0;

void setup_timer1(void) {
    T1CON = 0b00100000;    // Timer1 with 1:4 prescaler
    TMR1H = 0;
    TMR1L = 0;
    PIR1bits.TMR1IF = 0;   // Clear Timer1 interrupt flag
}

void setup_adc(void) {
    // Configure ADC
    ADCON1 = 0x0D;            // Configure all pins as analog
    ADCON0 = 0x01;            // Enable ADC, Select AN0 channel
    ADCON2 = 0xA4;            // Right justified, 4TAD, FOSC/4
    
    TRISAbits.TRISA0 = 1;     // RA0 as input (touch sensor)
    TRISAbits.TRISA1 = 1;     // RA1 as input (touch sensor)
//    TRISAbits.TRISA1 = 0;     // RA1 as output (LED)
//    LATAbits.LATA1 = 0;       // Initialize LED as off
    PIE1bits.ADIE = 1;
    PIR1bits.ADIF = 0;
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    INTCON2bits.RBPU = 1;     // Disable port B pull-ups
    INTCONbits.RBIF = 0;      // Clear RB interrupt flag
    INTCONbits.RBIE = 1;      // Enable RB interrupt
}

void reset_timer1(void) {
    TMR1H = 0;
    TMR1L = 0;
}
unsigned int get_timer1(void) {
    return (TMR1H << 8) | TMR1L;
}

int lift = True;
int i = 0;
unsigned int touchThreshold = 800;
//Adjust this value based on your sensor sensitivity

void __interrupt(high_priority)H_ISR(void){
    if(PIR1bits.ADIF) {
        int touchValue = ADRES;
        if(touchValue > touchThreshold){
                if(ADCON0bits.CHS0 == 1){
                    PCA9685_SetPWM(left_back_big, -30);
                    __delay_ms(5);
                }
                else{
                    PCA9685_SetPWM(right_back_big, 45);
                    __delay_ms(5);
                }
    //            PCA9685_SetPWM(right_back_big, 20);
    //            __delay_ms(80);

    //        }
        }
        else if(touchValue <= touchThreshold){
            if(ADCON0bits.CHS0 == 1){
                PCA9685_SetPWM(left_back_big, 0);
                __delay_ms(5);
            }
            else{
                PCA9685_SetPWM(right_back_big, 0);
                __delay_ms(5);
            }
        }
        if(ADCON0bits.CHS0 == 1){
            ADCON0 = 0x01;
        }
        else{
            ADCON0 = 0x05;
        }
        PIR1bits.ADIF = 0;
        __delay_us(4);
        ADCON0bits.GO = 1;
    }
    
    if(INTCONbits.INT0IF) {
        if(mode == 1) {
            LATDbits.LATD1 = 0;
            LATDbits.LATD2 = 1;
            mode = 2;
            ADCON0bits.ADON = 0;
            ADCON0bits.GO = 0;
        }
        else {
            ADCON0bits.ADON = 1;
            ADCON0bits.GO = 1;
            LATDbits.LATD1 = 1;
            LATDbits.LATD2 = 0;
            mode = 1;
        }
        INTCONbits.INT0IF = 0;
        __delay_ms(500);
    }
    
    return;
}

void main(void) {
    init();
    TRISBbits.TRISB4 = 1;  // ECHO pin as input (RC1)
    TRISBbits.TRISB0 = 0;  // TRIGGER pin as output (RC0)  
    
    setup_adc();
    //setup_timer1();
    PCA9685_SetPWM(right_back_big, 0);
    PCA9685_SetPWM(left_back_big, 0);
    PCA9685_SetPWM(right_front_big, 0);
    PCA9685_SetPWM(left_front_big, 0);
    ADCON0bits.GO = 0;
    
    TRISDbits.RD2 = 0;
    TRISDbits.RD1 = 0;
    LATDbits.LATD2 = 0;
    LATDbits.LATD1 = 0;
    TRISBbits.TRISB0 = 1;
    while(1) {
        // Generate trigger pulse
//        if(!has_ultrasound || TMR1 >= 30000) {
//            LATBbits.LATB0 = 1;
//            __delay_us(10);
//            LATBbits.LATB0 = 0;
//        }
        if(mode == 2){
            PCA9685_SetPWM(left_back_big, -30);
            __delay_ms(200);
            PCA9685_SetPWM(left_back_big, 0);
            __delay_ms(300);
            PCA9685_SetPWM(left_back_big, -30);
            __delay_ms(200);
            PCA9685_SetPWM(left_back_big, 0);
            __delay_ms(300);
            PCA9685_SetPWM(right_back_big, 45);
            __delay_ms(200);
            PCA9685_SetPWM(right_back_big, 0);
            __delay_ms(300);
            PCA9685_SetPWM(right_back_big, 45);
            __delay_ms(200);
            PCA9685_SetPWM(right_back_big, 0);
            __delay_ms(300);
            PCA9685_SetPWM(left_back_big, -30);
            __delay_ms(200);
            PCA9685_SetPWM(left_back_big, 0);
            __delay_ms(300);
            PCA9685_SetPWM(left_back_big, -30);
            __delay_ms(200);
            PCA9685_SetPWM(left_back_big, 0);
            __delay_ms(300);
            PCA9685_SetPWM(right_back_big, 45);
            __delay_ms(200);
            PCA9685_SetPWM(right_back_big, 0);
            __delay_ms(500);  
        }

    }
    return;
}
