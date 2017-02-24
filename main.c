/********************************************************************
 * 	FileName:      main.c 
 *	Processor:     PIC 18F2550 
 *  Projects: USB Joystick
 *	Compiled using C18 toolsuite
 * 
 *	1-14-17 JBS: Loaded onto XBEE DUAL JOYSTICK V1.0 BOARD
 *  NOTE: USES USB_POLLING
 *  LED L1 flashes FAST when powered, SLOW when USB is working
 *  Code emulates serial port, receives ASCII characters from Hyper Terminal
 *  increments value of character, sends that back.
 *  2-21-17: Adapted from USB Device - CDC - Basic Demo - C18 - PICDEM FS USB
 *  2-23-17: Got UART working with XBEE at 57600 baud, but SPBRG must be initialized twice!
 *  2-24-17: Read both joysticks. Right joystick readings 
 *          converted to Roomba Drive Direct command.
 * 
 ********************************************************************/
#include "USB/usb.h"
#include "USB/usb_function_cdc.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "delay.h"
#include <stdio.h>
#include <stdlib.h>

#define STX 36
#define ETX 13
#define DLE 16

#define leftJoystickY   ADresult[0]
#define leftJoystickX   ADresult[1]
#define rightJoystickY  ADresult[2]
#define rightJoystickX  ADresult[3]

// These configs are for 18F4550 / 18F2550
#pragma config PLLDIV   = 4
#pragma config USBDIV   = 2         // Clock source from 96MHz PLL/2
#pragma config CPUDIV   = OSC1_PLL2
#pragma config FOSC     = HSPLL_HS
#pragma config FCMEN    = OFF
#pragma config IESO     = OFF
#pragma config PWRT     = OFF
#pragma config BOR      = ON
#pragma config BORV     = 3
#pragma config VREGEN   = ON      //USB Voltage Regulator
#pragma config WDT      = OFF
#pragma config WDTPS    = 32768
#pragma config MCLRE    = ON
#pragma config LPT1OSC  = OFF
#pragma config PBADEN   = OFF
//      #pragma config CCP2MX   = ON
#pragma config STVREN   = ON
#pragma config LVP      = OFF
//      #pragma config ICPRT    = OFF       // Dedicated In-Circuit Debug/Programming
#pragma config XINST    = OFF       // Extended Instruction Set
#pragma config CP0      = OFF
#pragma config CP1      = OFF
//      #pragma config CP2      = OFF
//      #pragma config CP3      = OFF
#pragma config CPB      = OFF
//      #pragma config CPD      = OFF
#pragma config WRT0     = OFF
#pragma config WRT1     = OFF
//      #pragma config WRT2     = OFF
//      #pragma config WRT3     = OFF
#pragma config WRTB     = OFF       // Boot Block Write Protection
#pragma config WRTC     = OFF
//      #pragma config WRTD     = OFF
#pragma config EBTR0    = OFF
#pragma config EBTR1    = OFF
//      #pragma config EBTR2    = OFF
//      #pragma config EBTR3    = OFF
#pragma config EBTRB    = OFF



/** V A R I A B L E S ********************************************************/
#pragma udata

char USB_In_Buffer[64];
char USB_Out_Buffer[64];

BOOL stringPrinted;
volatile BOOL buttonPressed;
volatile BYTE buttonCount;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(unsigned char flag);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);
void init(void);
void ADsetChannel(unsigned char channel);
short ADconvertAndRead(void);
short ADread(void);
void putch(unsigned char TxByte);
void readJoySticks(void);
unsigned char insertByte(unsigned char dataByte, unsigned char *ptrBuffer, unsigned char *index);
unsigned char BuildPacket(unsigned char command, unsigned char *ptrData, unsigned char dataLength, unsigned char *ptrPacket);
#define MAXPACKET 32
unsigned char packet[MAXPACKET];


/** VECTOR REMAPPING ***********************************************/
#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#define APP_VERSION_ADDRESS                     0x1016 //Fixed location, so the App FW image version can be read by the bootloader.
#define APP_SIGNATURE_ADDRESS                   0x1006 //Signature location that must be kept at blaknk value (0xFFFF) in this project (has special purpose for bootloader).

#define APP_FIRMWARE_VERSION_MAJOR  1   //valid values 0-255
#define APP_FIRMWARE_VERSION_MINOR  0   //valid values 0-99

ROM unsigned char AppVersion[2] = {APP_FIRMWARE_VERSION_MINOR, APP_FIRMWARE_VERSION_MAJOR};
ROM unsigned short int SignaturePlaceholder = 0xFFFF;

#pragma code

#define NUM_AD_CHANNELS 4
unsigned short ADresult[NUM_AD_CHANNELS] = {0, 0, 0, 0};

union convertType {
    unsigned char byte[2];
    int integer;
} convert;

#define LSBbyte byte[0]
#define MSBbyte byte[1]

#define SLEEP_ENABLE PORTAbits.RA5
#define LED1 LATBbits.LATB2
#define LED2 LATBbits.LATB1
#define LED3 LATBbits.LATB0

#define REDPUSHED 0x60
#define BLUEPUSHED 0xA0
#define YELLOWPUSHED 0xC0
#define RELEASED 0xE0

unsigned short ConvertToTwosComplement(int decimalValue) {
    unsigned short result;

    if (decimalValue >= 0)
        result = (unsigned int) decimalValue;
    else {
        result = (unsigned int) (0 - decimalValue);
        result = result - 1;
        result = ~result;
    }
    return (result);
}

void startAD(void) {
    ADsetChannel(0);
    ADCON0bits.GO_DONE = 1;
}

#define DEBOUNCE_COUNTS 2

unsigned char debounce(void) {
    unsigned char previousPORTBreg = 0;
    unsigned char PORTBreg = 0;
    short debounceCounter = 0;

    if (INTCONbits.RBIF) {
        do {
            PORTBreg = PORTB & 0b11100000;
            INTCONbits.RBIF = 0;
            if (PORTBreg == previousPORTBreg)
                debounceCounter++;
            else debounceCounter--;
            previousPORTBreg = PORTBreg;
            DelayMs(40);
        } while (debounceCounter++ < DEBOUNCE_COUNTS);
        return (PORTBreg);
    } else return (0);
}

void putch(unsigned char TxByte) {
    while (!PIR1bits.TXIF); // set when register is empty 
    TXREG = TxByte;
    return;
}

#define RUN 2
#define HALT 1
#define STANDBY 0 

const unsigned char arrStart[] = {STX, 0, 1, 128, ETX};
const unsigned char arrFull[] = {STX, 0, 1, 132, ETX};
const unsigned char arrHalt[] = {STX, 0, 5, 145, 0, 0, 0, 0, ETX};

void main(void) {
    unsigned int TMR2counter = 0;
    unsigned char TimerFlag = FALSE;
    unsigned char pushState = 0, previousPushState = 0;
    unsigned char LEDcounter = 0;
    unsigned char mode = STANDBY;
    unsigned char i = 0;
    DelayMs(100);

    InitializeSystem();
    init();
    LED1 = LED2 = LED3 = 0;
    SLEEP_ENABLE = 0;
    startAD();

    while (1) {
        TimerFlag = FALSE;
        if (PIR1bits.TMR2IF) {
            PIR1bits.TMR2IF = 0;
            TMR2counter++;
            if (TMR2counter >= 100) {
                TMR2counter = 0;
                TimerFlag = TRUE;
            }
        }

#define LEDONTIME 100
        pushState = debounce();
        if (pushState != previousPushState) {
            previousPushState = pushState;
            if (pushState == REDPUSHED) {
                LED1 = 1;
                LEDcounter = LEDONTIME;
                if (mode) {
                    mode = HALT;
                    for (i = 0; i < 9; i++) putch(arrHalt[i]);
                }
            } else if (pushState == BLUEPUSHED) {
                LED2 = 1;
                LEDcounter = LEDONTIME;
                if (mode == STANDBY)
                    for (i = 0; i < 5; i++) putch(arrStart[i]);
                else
                    for (i = 0; i < 5; i++) putch(arrFull[i]);
                mode = HALT;
            } else if (pushState == YELLOWPUSHED) {
                LED3 = 1;
                LEDcounter = LEDONTIME;
                if (mode == HALT) mode = RUN;
            } else if (pushState == RELEASED) {
                ;
            }
        }
        if (LEDcounter) {
            LEDcounter--;
            if (!LEDcounter) LED1 = LED2 = LED3 = 0;
        }


#if defined(USB_INTERRUPT)
        if (USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)) {
            USBDeviceAttach();
        }
#endif

#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks();
#endif
        if (mode == RUN && TimerFlag) ProcessIO(TRUE);
        else ProcessIO(FALSE);
    }//end while
}//end main

static void InitializeSystem(void) {
    // UserInit();
    // init();
    USBDeviceInit();

}//end InitializeSystem

void UserInit(void) {
    //Initialize all of the debouncing variables
    buttonCount = 0;
    buttonPressed = FALSE;
    stringPrinted = TRUE;

    //Initialize all of the LED pins
    mInitAllLEDs();

    //Initialize the pushbuttons
    mInitAllSwitches();
}//end UserInit

void init(void) {
    INTCONbits.GIE = 0; // Global interrupt disabled

    // Initialize ports
    ADCON0 = 0b00000000; // Turn off A/D for now.

    // Set up 18F2550 for four analog inputs, use VCC and VSS for references.        
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON1bits.PCFG3 = 1;
    ADCON1bits.PCFG2 = 0;
    ADCON1bits.PCFG1 = 1;
    ADCON1bits.PCFG0 = 1;

    ADCON2 = 0; // Clear A/D control register 2
    ADCON2bits.ADFM = 1; // Right justified A/D result

    ADCON2bits.ACQT2 = 1; // Acquisition time max
    ADCON2bits.ACQT1 = 1;
    ADCON2bits.ACQT0 = 1;

    ADCON2bits.ADCS2 = 1; // Conversion time = Fosc/16
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;
    TRISA = 0b11011111; // RA5 is an output for XBEE SLEEP enable pin, the rest of PORT A are inputs
    TRISB = 0b11111000; // RB0, RB1, RB2 are LED outputs, 
    INTCON2bits.RBPU = 0; // Enable pullups
    TRISC = 0b10111111; // RC6 is TX output

    TXSTAbits.TXEN = 0; // Disable UART Tx        
    TRISC = 0b11111111; // Set UART Tx pin to input       

    // Set up the UART         
    BAUDCONbits.BRG16 = 0; // 8 bit baud rate generator
    TXSTAbits.BRGH = 1; // divide by 16

    TXSTAbits.SYNC = 0; // asynchronous
    RCSTAbits.SPEN = 1; // enable serial port pins
    RCSTAbits.CREN = 1; // enable reception
    RCSTAbits.SREN = 0; // no effect
    PIE1bits.TXIE = 0; // disable tx interrupts 
    PIE1bits.RCIE = 0; // disable rx interrupts 
    TXSTAbits.TX9 = 0; // 8-bit transmission
    RCSTAbits.RX9 = 0; // 8-bit reception
    TXSTAbits.TXEN = 0; // Enable transmitter
    BAUDCONbits.TXCKP = 0; // Do not invert transmit and receive data
    BAUDCONbits.RXDTP = 0;
    SPBRG = 51; // Baudrate = 57600 @ 48 Mhz clock 51
    printf(" "); // NOTE: THIS AND THE NEXT LINE MUST BE HERE
    SPBRG = 51; // REPEAT INITIALIZATION


    // Set up Timer 2 to roll over every millisecond (1000 Hz))
    // 48000000 system clock / 4 / 16 prescaler / PR2 = 150 / 5 postscaler = 1000 Hz
    T2CON = 0x00;
    T2CONbits.T2CKPS1 = 1; // 1:16 prescaler 
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.T2OUTPS3 = 0; // 1:5 postscaler
    T2CONbits.T2OUTPS2 = 1;
    T2CONbits.T2OUTPS1 = 0;
    T2CONbits.T2OUTPS0 = 0;
    PR2 = 150;
    PIR1bits.TMR2IF = 0; // Clear rollover flag
    T2CONbits.TMR2ON = 1; // Timer 2 ON 

    // Set up interrupts. 
    INTCON = 0x00; // Clear all interrupts
    INTCONbits.INT0IE = 0; // Disable pushbutton interrupts
    INTCONbits.RBIE = 0;
    INTCONbits.PEIE = 0;
    PIE1bits.TMR2IE = 0;


    INTCON2 = 0x00;
    INTCON2bits.RBPU = 0; // Enable Port B pullups 
    INTCON2bits.INTEDG0 = 0; // Interrupt on falling edge of RB0 pushbutton wakes up PIC from sleep mode.
    INTCONbits.GIE = 0; // Disable all interrupts               
}

void BlinkUSBStatus(void) {
    static WORD led_count = 0;

    if (led_count == 0)led_count = 10000U;
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if (USBSuspendControl == 1) {
        if (led_count == 0) {
            mLED_1_Toggle();
            if (mGetLED_1()) {
                mLED_2_On();
            } else {
                mLED_2_Off();
            }
        }//end if
    } else {
        if (USBDeviceState == DETACHED_STATE) {
            mLED_Both_Off();
        } else if (USBDeviceState == ATTACHED_STATE) {
            mLED_Both_On();
        } else if (USBDeviceState == POWERED_STATE) {
            mLED_Only_1_On();
        } else if (USBDeviceState == DEFAULT_STATE) {
            mLED_Only_2_On();
        } else if (USBDeviceState == ADDRESS_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        } else if (USBDeviceState == CONFIGURED_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                if (mGetLED_1()) {
                    mLED_2_Off();
                } else {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

void USBCBSuspend(void) {
    ;
}

void USBCBWakeFromSuspend(void) {
    ;
}

void USBCB_SOF_Handler(void) {
    if (buttonPressed == sw2) {
        if (buttonCount != 0) {
            buttonCount--;
        } else {
            //This is reverse logic since the pushbutton is active low
            buttonPressed = !sw2;

            //Wait 100ms before the next press can be generated
            buttonCount = 100;
        }
    } else {
        if (buttonCount != 0) {
            buttonCount--;
        }
    }
}

void USBCBErrorHandler(void) {
    ;
}

void USBCBCheckOtherReq(void) {
    USBCheckCDCRequest();
}//end

void USBCBStdSetDscHandler(void) {
    // Must claim session ownership if supporting this request
}//end

void USBCBInitEP(void) {
    //Enable the CDC data endpoints
    CDCInitEP();
}

void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == TRUE) {
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE;

            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);

            //Now drive the resume K-state signaling onto the USB bus.
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)

void USBCBEP0DataReceived(void) {
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}

void ProcessIO(unsigned char sendFlag) {
#define USBBUFFERSIZE 64    
    char USBdataBuffer[USBBUFFERSIZE];
    int stringLength = 0;
    unsigned char numBytesRead;
    short intLeftJoystickY, intLeftJoystickX, intRightJoystickY, intRightJoystickX;
    short rightMotor, leftMotor;
    unsigned char rightMotorLSB, rightMotorMSB, leftMotorMSB, leftMotorLSB;
    unsigned char motorData[5];
    unsigned char packetLength, i;
    unsigned char testChar = 'A';

    readJoySticks();

    if (sendFlag) {
        intLeftJoystickY = ((short) leftJoystickY) - 127;
        intLeftJoystickX = ((short) leftJoystickX) - 127;
        intRightJoystickY = ((short) rightJoystickY) - 127;
        intRightJoystickX = ((short) rightJoystickX) - 127;

        //rightMotor = (intRightJoystickY * 2) - (intRightJoystickX / 2);
        //leftMotor = (intRightJoystickY * 2) + (intRightJoystickX / 2);

        rightMotor = (((intRightJoystickY * 2) - (intRightJoystickX / 2)) * 3) / 2;
        if (rightMotor > 500) rightMotor = 500;
        if (rightMotor < -500) rightMotor = -500;

        leftMotor = (((intRightJoystickY * 2) + (intRightJoystickX / 2)) * 3) / 2;
        if (leftMotor > 500) leftMotor = 500;
        if (leftMotor < -500) leftMotor = -500;

        convert.integer = leftMotor;
        leftMotorLSB = convert.byte[0];
        leftMotorMSB = convert.byte[1];

        convert.integer = rightMotor;
        rightMotorLSB = convert.byte[0];
        rightMotorMSB = convert.byte[1];

        motorData[0] = 145;
        motorData[1] = rightMotorMSB;
        motorData[2] = rightMotorLSB;
        motorData[3] = leftMotorMSB;
        motorData[4] = leftMotorLSB;

        packetLength = BuildPacket(0, motorData, 5, packet);

        packetLength = 9;
        if (packetLength < MAXPACKET) for (i = 0; i < packetLength; i++) putch(packet[i]);
    }

    // Blink the LEDs according to the USB device status
    // BlinkUSBStatus();
    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;
    
    
    stringLength = sprintf(USBdataBuffer, "\r>LEFT: %d, RIGHT: %d", leftMotor, rightMotor);
    if (stringLength > USBBUFFERSIZE) stringLength = USBBUFFERSIZE;
    putUSBUSART(USBdataBuffer, stringLength);


    if (USBUSARTIsTxTrfReady()) {
        numBytesRead = getsUSBUSART(USB_Out_Buffer, 64);
        if (numBytesRead != 0) {
            BYTE i;

            for (i = 0; i < numBytesRead; i++) {
                switch (USB_Out_Buffer[i]) {
                    case 0x0A:
                    case 0x0D:
                        USB_In_Buffer[i] = USB_Out_Buffer[i];
                        break;
                    default:
                        USB_In_Buffer[i] = USB_Out_Buffer[i] + 1;
                        break;
                }
            }
            stringLength = sprintf(USBdataBuffer, "\r<%d, %d, %d, %d", ADresult[0], ADresult[1], ADresult[2], ADresult[3]);
            if (stringLength > USBBUFFERSIZE) stringLength = USBBUFFERSIZE;
            putUSBUSART(USBdataBuffer, stringLength);
        }
    }

    CDCTxService();
} //end ProcessIO

void ADsetChannel(unsigned char channel) {
    ADCON0 = (channel << 2) | 0x01; // enable ADC, RC osc.    
}

short ADconvertAndRead(void) {
    unsigned short highByte;
    short ADresult;
    ADCON0bits.GO_DONE = 1;
    while (ADCON0bits.GO_DONE);
    highByte = (unsigned short) ADRESH;
    highByte = (highByte << 8) & 0x0300;
    ADresult = (short) (highByte | ADRESL);
    return (ADresult);
}

short ADread(void) {
    unsigned short highByte;
    short ADresult;
    highByte = (unsigned short) ADRESH;
    highByte = (highByte << 8) & 0x0300;
    ADresult = (short) (highByte | ADRESL);
    return (ADresult);
}

void readJoySticks(void) {
    static unsigned char ADchannel = 0;
    const unsigned short ADoffset[] = {450, 400, 430, 400};
    const unsigned short ADspan[] = {150, 200, 200, 200};
    unsigned short joyStickReading, offset, span;

    if (!ADCON0bits.GO_DONE) {
        if (ADchannel == 0 || ADchannel == 2) joyStickReading = 1023 - ADread();
        else joyStickReading = ADread();

        offset = ADoffset[ADchannel];
        span = ADspan[ADchannel];
        if (joyStickReading < offset) joyStickReading = 0;
        else joyStickReading = joyStickReading - offset;
        if (joyStickReading > span) joyStickReading = span;

        ADresult[ADchannel] = (joyStickReading * 255) / span;
        if (ADresult[ADchannel] > 255) ADresult[ADchannel] = 255;

        ADchannel++;
        if (ADchannel >= NUM_AD_CHANNELS) ADchannel = 0;
        ADsetChannel(ADchannel);
        ADCON0bits.GO_DONE = 1;
    }
}

unsigned char insertByte(unsigned char dataByte, unsigned char *ptrBuffer, unsigned char *index) {
    if (*index >= MAXPACKET) return (FALSE);
    if (dataByte == STX || dataByte == DLE || dataByte == ETX) {
        ptrBuffer[*index] = DLE;
        *index = *index + 1;
    }
    if (*index >= MAXPACKET) return (FALSE);
    ptrBuffer[*index] = dataByte;
    *index = *index + 1;
    return (TRUE);
}

unsigned char BuildPacket(unsigned char command, unsigned char *ptrData, unsigned char dataLength, unsigned char *ptrPacket) {
    unsigned char packetIndex = 0, i;

    if (dataLength <= MAXPACKET) {
        ptrPacket[packetIndex++] = STX;
        ptrPacket[packetIndex++] = command;
        ptrPacket[packetIndex++] = dataLength;

        for (i = 0; i < dataLength; i++){
            if (!insertByte(ptrData[i], ptrPacket, &packetIndex))
                return(0);
        }
        ptrPacket[packetIndex++] = ETX;

        return (packetIndex);
    } else return (0);
}