//CONFIG bits for PIC18F87J11
#pragma config FOSC=HSPLL
#pragma config WDTEN=OFF
#pragma config XINST=OFF

#include <xc.h>
#include "LCD.h"

void main(void) {
    
    OSCTUNEbits.PLLEN = 1;
    LCDInit();
    lprintf(0, "PCA9685 Demo");
    
    while (1) {
        
    }
}
