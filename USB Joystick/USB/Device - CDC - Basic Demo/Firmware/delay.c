/* delay.c 
 *
 * Updated 12-6-2016 to allow delays > 255 milliseconds
 * For PIC18F2520 running at 16 Mhz
 * Compiled with XC8 V1.37
 * 
 */

#include	"delay.h"

void
DelayMs(unsigned short count){
	unsigned char i;
	while (count--) {
		i=100;
		while(i--) {
			DelayUs(4);	/* Adjust for error */
		} 
	} 
}


