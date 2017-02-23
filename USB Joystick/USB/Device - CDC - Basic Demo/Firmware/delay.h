/* delay.h
 *
 * Updated 12-6-2016 to allow delays > 255 milliseconds
 * For PIC18F2520 running at 16 Mhz
 * Compiled with XC8 V1.37
 * 
 */

#define SHORT_PULSE_HI 0
#define SHORT_PULSE_LOW 1
#define LONG_PULSE_HI 2
#define LONG_PULSE_LOW 3

#define	DelayUs(x)	{ unsigned char downCount; downCount = x; while(downCount--);}

extern void DelayMs(unsigned short);


