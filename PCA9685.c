
#include "PCA9685.h"
#include <xc.h>
#include <math.h>

#define _XTAL_FREQ      32000000

void PCA9685Init(void) {
    TRISDbits.TRISD5 = 1;
    TRISDbits.TRISD6 = 1;
    PCA9685OuputDisable();
    TRISDbits.TRISD7 = 0;
    SSP2ADD = 19; //400kHz
    SSP2STATbits.SMP = 0; //Slew for 400kHz
    SSP2CON1bits.SSPM = 0b1000; //I2C Master mode
    SSP2CON1bits.SSPEN = 1; //Enable MSSP
    PCA9685Reset();
    unsigned char mode;
    mode = PCA9685ReadRegister(PCA9685_MODE1_REG);
    mode |= PCA9685_AI; //Set auto increment
    PCA9685WriteRegister(PCA9685_MODE1_REG, mode);
    //Set mode2 for desired output options 
    //This is good for servos
    //PCA9685WriteRegister(PCA9685_MODE2_REG, PCA9685_OUTDRV | PCA9685_OUTNE_ZERO);
    //This is good for directly attached leds connected to V+ (see datasheet for direct connection diagram)  
    PCA9685WriteRegister(PCA9685_MODE2_REG, PCA9685_INVRT | PCA9685_OUTNE_ONE);
}

void PCA9685Reset(void) {
    SSP2CON2bits.SEN = 1; //Start condition
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = 0x00; //General call address
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2BUF = PCA9685_SW_RESET;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.PEN = 1; //Stop
    while (SSP2CON2bits.PEN == 1);
    __delay_us(5);
}

void PCA9685OuputEnable(void) {
    PCA9685_OE_PIN = 0;
}

void PCA9685OuputDisable(void) {
    PCA9685_OE_PIN = 1;
}

void PCA9685WriteRegister(unsigned char reg, unsigned char value) {
    SSP2CON2bits.SEN = 1; //Start condition
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = PCA9685_ADDRESS; //address with R/W clear for write
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2BUF = reg; //Send register address
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2BUF = value; //send value
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.PEN = 1; //Stop
    while (SSP2CON2bits.PEN == 1);
    __delay_us(5);
}

void PCA9685WriteData(unsigned char reg, char *data, char count) {
    SSP2CON2bits.SEN = 1; //Start condition
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = PCA9685_ADDRESS; //address with R/W clear for write
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2BUF = reg; //Send register address
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    while (count > 0) {
        SSP2BUF = *data; //send value
        while (SSP2STATbits.BF || SSP2STATbits.R_W);
        ++data;
        --count;
    }
    SSP2CON2bits.PEN = 1; //Stop
    while (SSP2CON2bits.PEN == 1);
    __delay_us(5);
}

unsigned char PCA9685ReadRegister(unsigned char reg) {
    unsigned char data;
    SSP2CON2bits.SEN = 1; //Start condition
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = PCA9685_ADDRESS; //address with R/W clear for write
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2BUF = reg; //Send register address
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.RSEN = 1; //Restart
    while (SSP2CON2bits.RSEN == 1);
    SSP2BUF = PCA9685_ADDRESS | 1; //address with R/W set for read
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.RCEN = 1; // enable master for 1 byte reception
    while (!SSP2STATbits.BF); // wait until byte received
    data = SSP2BUF;
    SSP2CON2bits.ACKDT = 1; //NACK last byte
    SSP2CON2bits.ACKEN = 1; //Send ACK
    while (SSP2CON2bits.ACKEN != 0);
    SSP2CON2bits.PEN = 1; //Stop condition
    while (SSP2CON2bits.PEN == 1);
    return data;
    __delay_us(5);
}

void PCA9685ReadData(unsigned char reg, char *data, char count) {
    SSP2CON2bits.SEN = 1; //Start condition
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = PCA9685_ADDRESS; //address with R/W clear for write
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2BUF = reg; //Send register address
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.RSEN = 1; //Restart
    while (SSP2CON2bits.RSEN == 1);
    SSP2BUF = PCA9685_ADDRESS | 1; //address with R/W set for read
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    while (count > 0) {
        SSP2CON2bits.RCEN = 1; // enable master for 1 byte reception
        while (!SSP2STATbits.BF); // wait until byte received
        *data = SSP2BUF;
        SSP2CON2bits.ACKDT = (count > 1) ? 0 : 1;
        SSP2CON2bits.ACKEN = 1; //Send ACK or NACK
        while (SSP2CON2bits.ACKEN != 0);
        --count;
        ++data;
    }
    SSP2CON2bits.PEN = 1; //Stop condition
    while (SSP2CON2bits.PEN == 1);
    __delay_us(5);
}

void PCA9685SetPWMFrequency(unsigned int freq) {
    double prescale = round(25000000.0 / (4096.0 * freq)) - 1;
    if (prescale < 3) {
        prescale = 3;
    } else if (prescale > 255) {
        prescale = 255;
    }
    unsigned char mode = PCA9685ReadRegister(PCA9685_MODE1_REG);
    PCA9685WriteRegister(PCA9685_MODE1_REG, mode | PCA9685_SLEEP); //Set sleep mode
    PCA9685WriteRegister(PCA9685_PRESCALE_REG, (unsigned char) prescale);
    PCA9685WriteRegister(PCA9685_MODE1_REG, mode); //restore mode
}

int PCA9685GetPWMFrequency(void) {
    float prescale = PCA9685ReadRegister(PCA9685_PRESCALE_REG);
    prescale += 1;
    int freq = (int) round(25000000.0 / (prescale * 4096));
    return freq;
}

void PCA9685SetPWMOutput(char output, int onValue, int offValue) {
    int buffer[2];
    unsigned char reg = PCA9685_LED_BASE_REG + 4 * output;
    buffer[0] = onValue & PCA9685_LED_MASK;
    buffer[1] = offValue & PCA9685_LED_MASK;
    PCA9685WriteData(reg, (char *) buffer, 4);
}

void PCA9685SetPWMOn(char output) {
    unsigned char reg = PCA9685_LED_BASE_REG + 4 * output;
    unsigned char currentValue = PCA9685ReadRegister(reg + ON_H_OFFSET);
    currentValue |= PCA9685_FULL_ON;
    PCA9685WriteRegister(reg + ON_H_OFFSET, currentValue);
    //Need to clear all off bit as it takes precedence
    currentValue = PCA9685ReadRegister(reg + OFF_H_OFFSET);
    currentValue &= ~PCA9685_FULL_OFF;
    PCA9685WriteRegister(reg + OFF_H_OFFSET, currentValue);
}

void PCA9685SetPWMOff(char output) {
    unsigned char reg = PCA9685_LED_BASE_REG + 4 * output;
    unsigned char currentValue = PCA9685ReadRegister(reg + OFF_H_OFFSET);
    currentValue |= PCA9685_FULL_OFF;
    PCA9685WriteRegister(reg + OFF_H_OFFSET, currentValue);
    //Should clear all on bit
    currentValue = PCA9685ReadRegister(reg + ON_H_OFFSET);
    currentValue &= ~PCA9685_FULL_ON;
    PCA9685WriteRegister(reg + ON_H_OFFSET, currentValue);
}

void PCA9685SetAllPWM(int onValue, int offValue) {
    int buffer[2];
    buffer[0] = onValue & PCA9685_LED_MASK;
    buffer[1] = offValue & PCA9685_LED_MASK;
    PCA9685WriteData(PCA9685_ALL_LED_BASE_REG, (char *) buffer, 4);
}

void PCA9685SetAllPWMOn(void) {
    unsigned char currentValue = PCA9685ReadRegister(PCA9685_ALL_LED_BASE_REG + ON_H_OFFSET);
    currentValue |= PCA9685_FULL_ON;
    PCA9685WriteRegister(PCA9685_ALL_LED_BASE_REG + ON_H_OFFSET, currentValue);
    //Need to clear all off bit as it takes precedence
    currentValue = PCA9685ReadRegister(PCA9685_ALL_LED_BASE_REG + OFF_H_OFFSET);
    currentValue &= ~PCA9685_FULL_OFF;
    PCA9685WriteRegister(PCA9685_ALL_LED_BASE_REG + OFF_H_OFFSET, currentValue);
}

void PCA9685SetAllPWMOff(void) {
    unsigned char currentValue = PCA9685ReadRegister(PCA9685_ALL_LED_BASE_REG + OFF_H_OFFSET);
    currentValue |= PCA9685_FULL_OFF;
    PCA9685WriteRegister(PCA9685_ALL_LED_BASE_REG + OFF_H_OFFSET, currentValue);
    //Should clear all on bit
    currentValue = PCA9685ReadRegister(PCA9685_ALL_LED_BASE_REG + ON_H_OFFSET);
    currentValue &= ~PCA9685_FULL_ON;
    PCA9685WriteRegister(PCA9685_ALL_LED_BASE_REG + ON_H_OFFSET, currentValue);
}

void PCA9685Sleep(void) {
    unsigned char mode;
    mode = PCA9685ReadRegister(PCA9685_MODE1_REG);
    mode |= PCA9685_SLEEP;
    PCA9685WriteRegister(PCA9685_MODE1_REG, mode);
}

void PCA9685Wake(void) {
    unsigned char mode;
    mode = PCA9685ReadRegister(PCA9685_MODE1_REG);
    mode &= ~PCA9685_SLEEP;
    PCA9685WriteRegister(PCA9685_MODE1_REG, mode);
    __delay_us(500);
}

void PCA9685Restart(void) {
    unsigned char mode;
    mode = PCA9685ReadRegister(PCA9685_MODE1_REG);
    if (mode & PCA9685_RESET) {
        mode &= ~PCA9685_SLEEP;
        mode &= ~PCA9685_RESET;
        PCA9685WriteRegister(PCA9685_MODE1_REG, mode);
        __delay_us(500);
        mode |= PCA9685_RESET;
        PCA9685WriteRegister(PCA9685_MODE1_REG, mode);
    }
}
