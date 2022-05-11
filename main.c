//CONFIG bits for PIC18F87J11
#pragma config FOSC=HSPLL
#pragma config WDTEN=OFF
#pragma config XINST=OFF

#include <xc.h>
#include "LCD.h"
#include "PCA9685.h"

//Pins
//SCL = RD6
//SDA = RD5
//OE = RD7

void main(void) {
    
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    lprintf(0, "PCA9685 Demo");
    PCA9685Init();
    PCA9685SetPWMFrequency(1000);
    PCA9685Wake();
    PCA9685OuputEnable();
    PCA9685SetPWMOutput(1, 0, 2000);
    int end = 0;
    while (1) {
        PCA9685SetPWMOutput(0, 0, end);
        end += 100;
        if (end > 4095) {
            end = 0;
        }
        __delay_ms(50);
    }
}
