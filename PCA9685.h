
#ifndef PCA9685_H
#define	PCA9685_H

//I2C addresses
#define PCA9685_ADDRESS         0x80    //8 bit address
#define PCA9685_ALL_CALL        0xe0    //below are default values
#define PCA9685_SUBADR1         0xe2    //they can be changed in firmware
#define PCA9685_SUBADR2         0xe4
#define PCA9685_SUBADR3         0xe8
#define PCA9685_SW_RESET        0x06

//PCA9685 registers
#define PCA9685_MODE1_REG       0x00
#define PCA9685_MODE2_REG       0x01
#define PCA9685_SUBADR1_REG     0x02
#define PCA9685_SUBADR2_REG     0x03
#define PCA9685_SUBADR3_REG     0x04
#define PCA9685_ALLCALLADR_REG  0x05

//LED base registers and offsets
#define PCA9685_LED_BASE_REG    0x06
#define ON_L_OFFSET             0
#define ON_H_OFFSET             1
#define OFF_L_OFFSET            2
#define OFF_H_OFFSET            3

#define PCA9685_LED0_BASE_REG   0x06
#define PCA9685_LED1_BASE_REG   0x0a
#define PCA9685_LED2_BASE_REG   0x0e
#define PCA9685_LED3_BASE_REG   0x12
#define PCA9685_LED4_BASE_REG   0x16
#define PCA9685_LED5_BASE_REG   0x1a
#define PCA9685_LED6_BASE_REG   0x1e
#define PCA9685_LED7_BASE_REG   0x22
#define PCA9685_LED8_BASE_REG   0x26
#define PCA9685_LED9_BASE_REG   0x2a
#define PCA9685_LED10_BASE_REG   0x2e
#define PCA9685_LED11_BASE_REG   0x32
#define PCA9685_LED12_BASE_REG   0x36
#define PCA9685_LED13_BASE_REG   0x3a
#define PCA9685_LED14_BASE_REG   0x3e
#define PCA9685_LED15_BASE_REG   0x42

#define PCA9685_ALL_LED_BASE_REG 0xfa

#define PCA9685_PRESCALE_REG    0x0fe

//ouput enable pin
#define PCA9685_OE_PIN          (LATDbits.LATD7)

//MODE1 bits
#define PCA9685_RESET           0x80
#define PCA9685_EXTCLK          0x40
#define PCA9685_AI              0x20
#define PCA9685_SLEEP           0x10
#define PCA9685_SUB1            0x08
#define PCA9685_SUB2            0x04
#define PCA9685_SUB3            0x02
#define PCA9685_ALLCALL         0x01

//MODE2 bits
#define PCA9685_INVRT           0x10
#define PCA9685_OCH             0x08
#define PCA9685_OUTDRV          0x04
#define PCA9685_OUTNE_ZERO      0x00
#define PCA9685_OUTNE_ONE       0x01
#define PCA9685_OUTNE_HI_Z      0x10

//PWM channel bits
#define PCA9685_FULL_ON         0x10
#define PCA9685_FULL_OFF        0x10
#define PCA9685_LED_MASK        0x0fff

//api functions
void PCA9685Init(void);
void PCA9685Reset(void);
void PCA9685OuputEnable(void);
void PCA9685OuputDisable(void);
void PCA9685WriteRegister(unsigned char reg, unsigned char value);
void PCA9685WriteData(unsigned char reg, char *data, char count);
unsigned char PCA9685ReadRegister(unsigned char reg);
void PCA9685ReadData(unsigned char reg, char *data, char count);
void PCA9685SetPWMFrequency(unsigned int freq);
int PCA9685GetPWMFrequency(void);
void PCA9685SetPWMOutput(char output, int onValue, int offValue);
void PCA9685SetPWMOn(char output);
void PCA9685SetPWMOff(char output);
void PCA9685SetAllPWM(int onValue, int offValue);
void PCA9685SetAllPWMOn(void);
void PCA9685SetAllPWMOff(void);
void PCA9685Sleep(void);
void PCA9685Wake(void);
void PCA9685Restart(void);

#endif	/* PCA9685_H */

