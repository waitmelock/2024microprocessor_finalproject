#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H
#define _XTAL_FREQ 1000000 //Need to same as osccon freq
#define I2C_BAUD 100000

void interrupt_init(void);
void timer_init(void);
void init(void);

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(unsigned char data);
unsigned char I2C_Read(unsigned char ack);

void PCA9685_Init(void);
void PCA9685_SetPWMFreq(unsigned char freq);
//void PCA9685_SetPWM(unsigned char channel, unsigned int on, unsigned int off);
void PCA9685_SetPWM(unsigned char channel, int angle);
void pwm_init(void);
#endif

