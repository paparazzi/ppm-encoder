
/*********************************************************************************************************
 Title  :   C  file for the bootloader (x_modem_bootloader_v3_2.c)
 Author:    Chris Efstathiou 
 E-mail:    hendrix  at vivodinet dot gr
 Homepage:  ........................
 Date:      21/Apr/2009
 Compiler:  AVR-GCC with AVR-AS
 MCU type:  Any AVR MCU device with bootloader support
 Comments:  This software is FREE. Use it at your own risk.
*********************************************************************************************************/
/*                                           COMMENTS                                                   */
/********************************************************************************************************/
/* 
The whole project is based around the ATMEL AVR processor and specifically the ATMEGA168.
Please use the above mentioned processor otherwise you may need to change/modify the source files of the project.
I have tested the bootloader with other processors and it worked well right out of the box but i can't guarrantee anything.

I prefer the grounded pin activation because with the "C" character method the UART must be initialized
every time during boot up within the bootloader code and that means that the Txd UART pin will be set as an
output and if the pin is used as an input during normal operation and something is connected to that pin 
it will be driving it thinking that it is an input!!!
Obviously this is not good for the health of the AVR device so i use the grounded pin activation
which allows me to first disconnect any plugs attached to the cirquit, flash it and reconnect the plugs.
When using the grounded pin activation the UART is not initialized in every boot up so it doesn't
affect any pin.

About the use of the bootloader now.
First use the supplied makefile because the code must be loaded in the target's specific memory area
or the bootloader will not work. The are are comments in the makefile that explain the settings.
Second edit the settings in this makefile to suit your needs.
This makefile can also be used for your applications because it directly outputs the needed binary files.
In order to use the supplied bootloader makefile with your applications, set the bootloader start address to 0,
edit the  file names to match your project files and then type "make bin" in order to produce the *.bin file
needed by this bootloader. There are two files generated this way the
<application name>.bin file (the hex file converted to binary) 
and the <application name>.eep_bin eeprom binary file (the *.eep file converted to binary).
Using X modem the uploaded files must be in binary form for both the flash and the Eeprom.
Always the file that ends up in the avr's flash and eeprom memory are in binary format, the flashing program
usually converts the *.hex file to the required binary format automatically.

If you use the bootloader make sure that you check the fuses that are needed for proper operation.
1) BOOTRST, 2)BOOTSZ0, 3)BOOTSZ1.
Those 3 fuses tell the cpu to move the reset vector to the bootloader address and to reserve
all 2048, 1024 or 512 bytes of bootloader memory space. 

You will  need a 3.3v or 5v FTDI cable (USB to SERIAL TTL) in order to use the Xmodem bootloader.
Use the FTDI cable for connecting the board to be programmed with the PC. 
BEFORE ENTERING THE BOOTLOADER DISCONNECT EVERYTHING FROM THE BOARD TO BE PROGRAMMED !!!
After making sure that everything is disconnected from the board to be programmed
including power place a jumper over the pins that activate the bootloader.
The +5 volts that will power the board can come from the FTDI cable's +5 volts (Vcc)
but make sure that the board does not draw more than 50 mA of current. 
Make a cable that will connect the FTDI's Txd, Rxd, 5V and Ground to the board's Rxd, Txd, 5v input and Ground repectively. 

In order to upload the file needed, reset the board by power cycling it making sure that the 
connection with the terminal program like "HyperTerminal is already opened and connected
at  9600 bps 8 bits, no parity, 1 stop bit, no flow control and carriage return (CR) as terminator of the line.
When you hit the "enter" key is the same as adding "CR" as the line terminator.
In Hypertermina you don't need to add "CR" because every key you press, immediately is transmitted to the bootloader
unlike some other terminal programs like "cutecom" that wait until you hit "ENTER" 
in order to send everything you have typed so far and add the "CR" terminator at the end of the string.
After power cycling (reset) the board you should see the prompt "ATWF=write flash >" prompt on your screen.
Type "ATWF" using capital or small or mixed letters and hit enter.
Now the "C" character will start to fill the screen which means that you must start
the X modem crc transfer from your terminal program.
Select the Xmodem type of file transfer, then select the file to be uploaded ("xxxxxxxxx.bin")
and click the send button or whatever your terminal program uses.
Be carefull "Xmodem 1K" IS NOT THE SAME!!!
Usually “Xmodem CRC” is referred as plain "Xmodem".  

Then the transfer window of your program will show up telling you the transfer progress and after the transfer
is hopefully complete and succesfull the window will automatically close and you will
see the message “update successful” at the terminal 's screen. 
The bootloader commands are:
ATWF = write flash.
ATWE = write Eeprom.
ATRB = Read boot lock fuses.
ATR  = Read boot lock fuses (same as as the 'ATRB").
ATRL = Read the low fuses.
ATRH = Read the high fuses.
ATRE = Read the extended fuses.
ATWB = Write/modify the boot lock fuses

Pay particular attention to the ATWB command it can render the board useless and then
only a flash erase and a fresh programming of the board will make the board work again.
The fuse bytes are presented with binary form for example 00000010 = 2
If you attempt to modify the boot lock fuses you need to issue a command like this:
ATWB11111011 + ENTER
Then a prompt will appear asking for confirmation "Y/N"
If you hit "Y" the BLB02 fuse will be programmed, any other key will abort programming.
I didn't found setting the lockbits usefull as the bootloader code protect the bootloader's space from 
overwritting it during the firmware upload and protecting the program area from writting it again 
cancells the usefuleness of the bootloader i think.
Read carefully the below part from the mega168 datasheet:

SETTING THE BOOT LOCK BITS WITH THE ATWLxxxxxxxx BOOTLOADER COMMAND.

BOOT LOCK BIT programming (BLB12, BLB11, BLB02 and BLB01)
To set the Boot Loader Lock bits and general lock bits using the bootloader
first examine the table below.

ATWL X        X         X        X       X        X        X        X
Bit  7        6         5        4       3        2        1        0
R0   1        1      BLB12     BLB11   BLB02    BLB01     LB2      LB1

Now give the command ATWLxxxxxxxx and press enter. The xxxxxxxx are the binary presentation of lockbits
7,6,5,4,3,2,1 and 0 
If you give the command ATWB11110111 the boot lock bit 3 will be programmed (BLB02)
so 
If bits 5..0 in R0 are cleared (zero), the corresponding Boot Lock bit and general lock bit will be
programmed.
Be careful with the boot lock bits. 

Below are two tables with the functionality of the boot lock bits of the ATMEGA168.

               Boot Lock Bit0 Protection Modes (Application Section)(1)
Table 26-2.
BLB0 Mode      BLB02     BLB01     Protection
                                     
     1            1         1      No restrictions for SPM or LPM accessing the Application 
                                   section.

     2            1         0      SPM is not allowed to write to the Application section.
                                   
                                   
     3            0         0      SPM is not allowed to write to the Application section, and LPM
                                   executing from the Boot Loader section is not allowed to read
                                   from the Application section. If Interrupt Vectors are placed in
                                   the Boot Loader section, interrupts are disabled while executing
                                   from the Application section.

     4            0         1      LPM executing from the Boot Loader section is not allowed to
                                   read from the Application section. If Interrupt Vectors are placed
                                   in the Boot Loader section, interrupts are disabled while
                                   executing from the Application section.

Note:   1. “1” means unprogrammed, “0” means programmed

               Boot Lock Bit1 Protection Modes (Boot Loader Section)(1)
Table 26-3.
BLB1 Mode      BLB12     BLB11     Protection
                                   
     1            1         1      No restrictions for SPM or LPM accessing the Boot Loader
                                   section.

     2            1         0      SPM is not allowed to write to the Boot Loader section.
                                
     3            0         0      SPM is not allowed to write to the Boot Loader section, and LPM
                                   executing from the Application section is not allowed to read
                                   from the Boot Loader section. If Interrupt Vectors are placed in
                                   the Application section, interrupts are disabled while executing
                                   from the Boot Loader section.

     4            0         1      LPM executing from the Application section is not allowed to
                                   read from the Boot Loader section. If Interrupt Vectors are
                                   placed in the Application section, interrupts are disabled while
                                   executing from the Boot Loader section.

Note:   1. “1” means unprogrammed, “0” means programmed

Have fun.

Chris

*/


/********************************************************************************************************/
/*                                        INCLUDE FILES                                                 */
/********************************************************************************************************/
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdlib.h>


/********************************************************************************************************/
/*                                        USER CONFIGURATION                                            */
/********************************************************************************************************/
#ifndef F_CPU
#define F_CPU                             8000000UL //system clock(Hz)
#endif
#define BAUDRATE                          9600UL    //UART baudrate, i have tested it up to 115200 bps
#define SERIAL_PORT_NUMBER                0         //UART that will be used, 0,1 or 2 etc.
#define UART_PORT                         D
#define UART_RX_PIN                       0
#define UART_TX_PIN                       1
#define LEDPORT                           B         //LED control port
#define LEDPORT_PIN                       0         //LED control port pin
#define CHECK_LEVEL_PORT                  B
#define CHECK_LEVEL_PIN                   3
#define USE_TIMER1                        1         //1=use 16 bit timer1 (consumes less memory space), 0= use a delay.
#define USE_LED_INDICATION                1         //1= Use a led for data transfer indication, 0=don't use a led.
#define INTIALIZATION_DELAY               0         // UART initialization delay if needed, 0=no delay >0 delay needed

//FUNCTION ENABLE / DISABLE
#define BOOTLOCK_BIT_SET_SUPPORT          1         //1= use bootlock bit write capability, 0= don't use (smaller code).
#define BOOTLOCK_BIT_READ_SUPPORT         1         //1= use bootlock bit read capability, 0= don't use (smaller code).
#define VERSION_INFO_NEEDED               1         //1= use application version read capability, 0=don't use (smaller code). 
#define BOOTLOADER_SAVE_CODE_SIZE         0         //1= omit some non critical code, 0=don't save (larger code size). 
/* 
ABSOLUTELY minimum code configuration. All above definitions will be auto adjusted for minimum code space
but of course the bare minimum functionality will be available (only uploading of flash and eeprom files).
*/   
#define BOOTLOADER_ABSOLUTELY_MIN_CODE    0         

// The X modem standard needs a 3 second timeout but those days with the power of the modern PC 1 second is enough.
//time to wait for a character to be received in milliseconds.
#define UART_RX_TIMEOUT                   500       

// How many attempts to make for X modem transfer (time= XMODEM_START_ATTEMPTS * UART_RX_TIMEOUT_MAX)
#define XMODEM_START_ATTEMPTS             15  

// Put any extra commands to be executed. Those commands will be executed first.
#define EXTRA_STARTUP_COMMANDS()          { }

// Put any extra commands to be executed. Those commands will be executed just before exit.
#define EXTRA_EXIT_COMMANDS()             { }      

//Bootloader messages
const char success_message[]   = "OK";
const char error_message[]     = "ERROR";
const char update_success_message[] = "update successful";
const char prompt[] = "(ATWF=write flash)>";

#ifndef SPM_PAGESIZE
#error "SPM_PAGESIZE" not defined, add #define SPM_PAGESIZE XXX to the "bootloader.c" file 
#endif
//#define SPM_PAGESIZE   XXX
 
//certain versions of the gcc avr compiler are missing the "FLASHEND" definition
#ifndef FLASHEND
#error "FLASHEND" not defined please add #define FLASHEND XXX to the "bootloader.c" file
#endif
//#define FLASHEND       XXX

/********************************************************************************************************/
/*                                      USER CONFIGURATION ENDS HERE                                    */
/********************************************************************************************************/

/********************************************************************************************************/
/*                                   PREPROCESSOR DIRECTIVES                                            */
/********************************************************************************************************/

#if BOOTLOADER_ABSOLUTELY_MIN_CODE == 1
#undef USE_LED_INDICATION  
#define USE_LED_INDICATION           0 
#undef VERSION_INFO_NEEDED
#define VERSION_INFO_NEEDED          0
#undef BOOTLOCK_BIT_SET_SUPPORT
#define BOOTLOCK_BIT_SET_SUPPORT     0
#undef BOOTLOCK_BIT_READ_SUPPORT
#define BOOTLOCK_BIT_READ_SUPPORT    0
#undef BOOTLOADER_SAVE_CODE_SIZE 
#define BOOTLOADER_SAVE_CODE_SIZE    1
#endif 

//certain versions of the gcc avr compiler are missing the "SPM_PAGESIZE" definition
#ifndef SPM_PAGESIZE
#warning "SPM_PAGESIZE" not defined, "SPM_PAGESIZE" set to 128 
#define SPM_PAGESIZE             128
#endif

#define X_MODEM_PACKET_LENGTH    128  

#if SPM_PAGESIZE >= X_MODEM_PACKET_LENGTH

#if ((SPM_PAGESIZE/X_MODEM_PACKET_LENGTH)*X_MODEM_PACKET_LENGTH) != SPM_PAGESIZE
#error SPM_PAGESIZE is not an exact multiple of X_MODEM_PACKET_LENGTH
#endif

#elif X_MODEM_PACKET_LENGTH > SPM_PAGESIZE 

#if ((X_MODEM_PACKET_LENGTH / SPM_PAGESIZE)* SPM_PAGESIZE) != X_MODEM_PACKET_LENGTH
#error X_MODEM_PACKET_LENGTH is not an exact multiple of SPM_PAGESIZE
#endif

#endif 

//calculate baudrate register
#define BAUDREG            ((unsigned int)((F_CPU * 10) / (16UL * BAUDRATE) - 5) / 10)

//check baudrate error
#define BAUD_ERROR           (16UL * BAUDRATE * (((F_CPU * 10) / (16 * BAUDRATE) + 5)/ 10))
#if ((BAUD_ERROR * 100) > (102 * F_CPU)) || ((BAUD_ERROR * 100) < (98 * F_CPU))
#error "UART_BAUDRATE_ERROR_TOO_HIGH! Change baud rate"
#endif

// Alternative baudrate error calculation.
/*
#define UART_BAUDRATE_ACCURATE   (F_CPU/(BAUDRATE/100))  
#define UART_BAUDRATE_REAL       (((F_CPU/BAUDRATE)/16)*16)
#if  ((UART_BAUDRATE_ACCURATE/UART_BAUDRATE_REAL)-100) > 2
#error "UART_BAUDRATE_ERROR_TOO_HIGH! Change baud rate"
#endif 
*/

//internal use macro
#define _CONCAT2_(a, b)       a ## b
#define CONCAT2(a, b)         _CONCAT2_(a, b)
#define _CONCAT3_(a, b, c)    a ## b ## c
#define CONCAT3(a, b, c)      _CONCAT3_(a, b, c)

//register of PORT and bit define
#define PORT_REG(ADD_THIS)        CONCAT2(PORT, ADD_THIS)
#define DDR_REG(ADD_THIS)         CONCAT2(DDR, ADD_THIS)
#define PIN_REG(ADD_THIS)         CONCAT2(PIN, ADD_THIS)

#if !defined(UDR0)

#define _UDR_               UDR
#define _UBRRH_             UBRRH
#define _UBRRL_             UBRRL
#define _UCSRA_             UCSRA
#define _UCSRB_             UCSRB
#define _UCSRC_             UCSRC
#define _TXC_               TXC
#define _RXC_               RXC
#define _UDRE_              UDRE
#define _RXEN_              RXEN
#define _TXEN_              TXEN
#define _UCSZ1_             UCSZ1
#define _UCSZ0_             UCSZ0
#define _URSEL_             URSEL

#else

#define _UDR_               CONCAT2(UDR, SERIAL_PORT_NUMBER)
#define _UBRRH_             CONCAT3(UBRR, SERIAL_PORT_NUMBER, H)
#define _UBRRL_             CONCAT3(UBRR, SERIAL_PORT_NUMBER, L)
#define _UCSRA_             CONCAT3(UCSR, SERIAL_PORT_NUMBER, A)
#define _UCSRB_             CONCAT3(UCSR, SERIAL_PORT_NUMBER, B)
#define _UCSRC_             CONCAT3(UCSR, SERIAL_PORT_NUMBER, C)
#define _TXC_               CONCAT2(TXC, SERIAL_PORT_NUMBER)
#define _RXC_               CONCAT2(RXC, SERIAL_PORT_NUMBER)
#define _UDRE_              CONCAT2(UDRE, SERIAL_PORT_NUMBER)
#define _RXEN_              CONCAT2(RXEN, SERIAL_PORT_NUMBER)
#define _TXEN_              CONCAT2(TXEN, SERIAL_PORT_NUMBER)
#define _UCSZ1_             CONCAT3(UCSZ, SERIAL_PORT_NUMBER, 1)
#define _UCSZ0_             CONCAT3(UCSZ, SERIAL_PORT_NUMBER, 0)
#define _URSEL_             CONCAT2(URSEL, SERIAL_PORT_NUMBER)

#endif

//some kind of AVR need URSEL to be set during UART initialization.
#if defined(URSEL) || defined(URSEL0)
#define USEURSEL           (1 << _URSEL_)
#else
#define USEURSEL           0
#endif

//timer1 overflow flag register
#ifdef TIFR
#define TIFR_REG            TIFR
#else
#define TIFR_REG            TIFR1
#endif

//Xmoden control command
#define XMODEM_NUL         0x00
#define XMODEM_SOH         0x01
#define XMODEM_STX         0x02
#define XMODEM_EOT         0x04
#define XMODEM_ACK         0x06
#define XMODEM_NAK         0x15
#define XMODEM_CAN         0x18
#define XMODEM_EOF         0x1A
#define XMODEM_NCG         'C'

#if USE_TIMER1 == 1

#define UART_TIMEOUT_VALUE          ( F_CPU  / ((1024UL * 1000) / UART_RX_TIMEOUT) )
#define UART_TIMEOUT_200MS_VALUE    ( F_CPU  / ((1024UL * 1000) / 200) )
#define UART_TIMEOUT_1S_VALUE       ( F_CPU  / ((1024UL * 1000) / 1000) )
#define UART_TIMEOUT_ENABLE()       { OCR1A  = UART_TIMEOUT_200MS_VALUE; TCCR1B = (1 << WGM12)|(1 << CS12)|(1 << CS10); }
#define UART_TIMEOUT_DISABLE()      { TCCR1B = 0; }
#define UART_TIMEOUT_DEFAULT()      { OCR1A  = UART_TIMEOUT_VALUE; } 
#define UART_TIMEOUT_1S()           { OCR1A  = UART_TIMEOUT_1S_VALUE; } 
#define UART_TIMEOUT_200MS()        { OCR1A  = UART_TIMEOUT_200MS_VALUE; }  

#else

#define UART_NO_TIMEOUT_VALUE       (0xFFFFFFFE )
#define UART_TIMEOUT_VALUE          ( (F_CPU  / 11000) * UART_RX_TIMEOUT )
#define UART_TIMEOUT_200MS_VALUE    ( (F_CPU  / 11000) * 200 )
#define UART_TIMEOUT_1S_VALUE       ( (F_CPU  / 11000) * 1000 )
#define UART_TIMEOUT_ENABLE()       { uart_rx_timeout = UART_TIMEOUT_VALUE; }
#define UART_TIMEOUT_DISABLE()      { uart_rx_timeout = UART_NO_TIMEOUT_VALUE; }
#define UART_TIMEOUT_DEFAULT()      { uart_rx_timeout = UART_TIMEOUT_VALUE; } 
#define UART_TIMEOUT_1S()           { uart_rx_timeout = UART_TIMEOUT_1S_VALUE; } 
#define UART_TIMEOUT_200MS()        { uart_rx_timeout = UART_TIMEOUT_200MS_VALUE; }  

#endif

#if USE_LED_INDICATION == 1
#define TOGGLE_LED()                { PORT_REG(LEDPORT) ^= (1 << LEDPORT_PIN);    }
#define ENABLE_LED()                { DDR_REG(LEDPORT) |= (1 << LEDPORT_PIN);     }
#define DISABLE_LED()               { DDR_REG(LEDPORT) &= (~(1 << LEDPORT_PIN));  }
#define LED_ON()                    { PORT_REG(LEDPORT) |= (1 << LEDPORT_PIN);    }
#define LED_OFF()                   { PORT_REG(LEDPORT) &= (~(1 << LEDPORT_PIN)); }
#else
#define TOGGLE_LED()                { }
#define ENABLE_LED()                { }
#define DISABLE_LED()               { }
#define LED_ON()                    { }
#define LED_OFF()                   { }
#endif

#define ENABLE_LEVEL_PIN()   { DDR_REG(CHECK_LEVEL_PORT) &= (~(1 << CHECK_LEVEL_PIN)); \
                               PORT_REG(CHECK_LEVEL_PORT) |= (1 << CHECK_LEVEL_PIN); \
                             }

#define DISABLE_LEVEL_PIN()  { PORT_REG(CHECK_LEVEL_PORT) &= (~(1 << CHECK_LEVEL_PIN)); }

    

/******************************************************************************************************/
/*                                       FUNCTION PROTOTYPES                                          */
/******************************************************************************************************/

__attribute__((__noinline__)) void sput_char(unsigned char data);
unsigned char sget_char(void);
__attribute__((__noinline__)) void sput_str(const char *str);
void receive_at_command(void);
//void bootloader_init(void);


/************************************************************************************************************/
/*                                          GLOBAL VARIABLES                                                */
/************************************************************************************************************/

// THE BELOW FUSES CORRESPOND TO AVR MEGA 168 ONLY !!!
/*
FUSES = {
           .low = (FUSE_CKSEL0 & FUSE_CKSEL2 & FUSE_CKSEL3 & FUSE_SUT0),
           .high = HFUSE_DEFAULT,
           .extended = (FUSE_BOOTSZ0 & FUSE_BOOTSZ1 & FUSE_BOOTRST),
        };
*/

//define receive buffer which must be equal to flash page size.
#if  X_MODEM_PACKET_LENGTH > SPM_PAGESIZE
unsigned char x_buffer[X_MODEM_PACKET_LENGTH];
#else
unsigned char x_buffer[SPM_PAGESIZE];
#endif

#if USE_TIMER1 == 1

unsigned int uart_rx_timeout = 0 ;

#else

#if CHECK_PIN_LEVEL == 1
unsigned long uart_rx_timeout = UART_TIMEOUT_VALUE ;
#else
unsigned long uart_rx_timeout = UART_TIMEOUT_200MS_VALUE;
#endif

#endif

volatile unsigned char memory_to_write = 0;


/************************************************************************************************************/
/*                                         FUNCTION DEFINITIONS                                             */
/************************************************************************************************************/

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

//send a byte to serial 
__attribute__((__noinline__)) void sput_char(unsigned char data)
{
/*
The UDREn Flag indicates if the transmit buffer (UDRn) is ready to receive new data. If UDREn
is one, the buffer is empty, and therefore ready to be written. 
*/

while( (_UCSRA_ & (1<<_UDRE_)) == 0) ;
_UDR_ = data;
while( (_UCSRA_ & (1<<_UDRE_)) == 0) ;

return;
}

/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
/*2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/

//receive a byte from the UART
__attribute__((__noinline__)) unsigned char sget_char(void)
{
#if USE_TIMER1 == 1
/*
This flag bit is set when there are unread data in the receive buffer and cleared when the receive
buffer is empty (i.e., does not contain any unread data).
*/
TOGGLE_LED();
TCNT1 = 0;

TIFR_REG |= (1 << OCF1A);
while( (_UCSRA_ & (1 << _RXC_)) == 0 )
    {
       if( (TIFR_REG & (1 << OCF1A)) ) { return(0); }
    }

#else

unsigned long timeout = uart_rx_timeout;

while( (_UCSRA_ & (1 << _RXC_)) == 0 )
    {
       timeout--;
       if(timeout == 0){ return(0); }
    }

#endif

return(_UDR_);
}


/*2222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
/*3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333*/

// send a string to uart
void sput_str(const char *str)
{

sput_char('\r'); 
sput_char('\n'); 
while(*str){ sput_char(*str++); }


return;
}

/*3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333*/
/*4444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444*/

void receive_at_command(void)
{

unsigned char received_bytes = 0;
unsigned char rx_char_buffer = 0;
unsigned char x = 0; 
#if defined BOOTLOCK_BIT_SET_SUPPORT && BOOTLOCK_BIT_SET_SUPPORT == 1
unsigned char blb = 0;
unsigned char lock_bits = 0;
#endif
#if defined VERSION_INFO_NEEDED && VERSION_INFO_NEEDED == 1
unsigned int eep_address = 0;
#endif

while(memory_to_write == 0)
    {
      x=_UDR_;  
      x=_UDR_;
      sput_str(prompt);
       do{
             received_bytes = 0; 
             //uart_rx_timeout = UART_TIMEOUT_ABS_MAX_VALUE;
             rx_char_buffer = sget_char();
             if(rx_char_buffer){ sput_char(rx_char_buffer); }
          
             if( rx_char_buffer == 'A' || rx_char_buffer == 'a' )
               {
                  rx_char_buffer = sget_char();
                  if(rx_char_buffer){ sput_char(rx_char_buffer); }
                  if( rx_char_buffer == 'T' || rx_char_buffer == 't' )
                   { 
                      while(1)
                          { 
                             rx_char_buffer = sget_char();
                             sput_char(rx_char_buffer);
                             if(received_bytes > 10 )
                              {
                                received_bytes = 0xff; 
                                sput_str(error_message);
                                break;
                              } 
                             if(rx_char_buffer == '\r' || rx_char_buffer == '\n')
                              {
                                 x_buffer[received_bytes] = '\0'; 
                                 break;
                              }
                             x_buffer[received_bytes++] = rx_char_buffer;   
                             if(rx_char_buffer == 0) { received_bytes = 0; break; }
                          }
                   }

               }else if( rx_char_buffer == '\n' || rx_char_buffer == '\r' ) { sput_str(prompt); } 
              
           }while(received_bytes == 0 ); // end of "while(1)" loop

       if(received_bytes == 0Xff){ continue; } 
       strupr((char*)x_buffer);
       memory_to_write = 0;
       switch(x_buffer[0])
            {    
               case('W'):  switch(x_buffer[1])
                                {
                                   case('F'):  memory_to_write = 1;  break; 
                                   case('E'):  memory_to_write = 2;  break; 
#if defined BOOTLOCK_BIT_SET_SUPPORT && BOOTLOCK_BIT_SET_SUPPORT == 1
                                   case('B'):  blb = 7; lock_bits = 0xFF;
                                               for(x=2; x<=9; x++)
                                                 {
                                                     rx_char_buffer = x_buffer[x]; 
                                                     if(rx_char_buffer == '0'){ lock_bits &= (~(1<<blb)); }
                                                     blb--;
                                                 } 
                                               x_buffer[10] = '\0';
                                               utoa(lock_bits,(char*)x_buffer, 2);
                                               sput_str((char*)x_buffer);
                                               sput_str("Y/N");
                                               rx_char_buffer = sget_char(); 
                                               if(rx_char_buffer == 'Y' || rx_char_buffer == 'y')
                                                {
                                                   boot_lock_bits_set(~lock_bits);
                                                   sput_str(success_message);
                                                }else{ sput_str("Aborted");  }
                                               break;
#endif                                             
                                   default  :  sput_str(error_message); break;  
                                }
                           break;  

#if defined BOOTLOCK_BIT_READ_SUPPORT && BOOTLOCK_BIT_READ_SUPPORT == 1
              case('R'):  switch(x_buffer[1])
                               {
                                  case('B'): utoa(boot_lock_fuse_bits_get(GET_LOCK_BITS), (char*)x_buffer, 2);
                                             sput_str((char*)x_buffer);
                                             break;

                                  case('L'): utoa(boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS), (char*)x_buffer, 2);
                                             sput_str((char*)x_buffer);
                                             break;

                                  case('H'): utoa(boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS), (char*)x_buffer, 2);
                                             sput_str((char*)x_buffer);
                                             break;

                                  case('E'): utoa(boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS), (char*)x_buffer, 2);
                                             sput_str((char*)x_buffer);
                                             break;

                                  default  : sput_str(error_message); break;

                               }
                           
                           break; 
   
#endif //END of #if defined BOOTLOCK_BIT_READ_SUPPORT && BOOTLOCK_BIT_READ_SUPPORT == 1            
              case('I'):   
#if defined VERSION_INFO_NEEDED && VERSION_INFO_NEEDED == 1
                           sput_char('\r'); sput_char('\n');
                           eep_address = (E2END - 31);
                           while(eep_address <= E2END)
                               {
                                  x = eeprom_read_byte((unsigned char*)eep_address++);
                                  if(x < 0xff && x > 0){ sput_char(x); }
                               } 
#endif
                           sput_str("BootLoader V3.2\r\nChris Efstathiou 2009");
                           break;

              default  :   sput_str(error_message);
                           break;
    
            } 

    }//end of "while(memory_to_write == 0)" loop.

return;
}

/************************************************************************************************************/
/*                                           MAIN FUNCTION                                                  */
/************************************************************************************************************/

__attribute__((__noreturn__)) void main(void)
{

#if (FLASHEND > 0xFFFFUL)
unsigned long flash_address = 0;
#else
unsigned int  flash_address = 0;
#endif
#if SPM_PAGESIZE > 128
unsigned int  x=0, y=0;
#else
unsigned char  x=0, y=0;
#endif
unsigned char high_byte_buffer = 0, low_byte_buffer = 0, packet_resend_required = 0;
unsigned char packet_number = 0, error_counter = 0, memory_full = 0;
unsigned int received_bytes = 0, address_buffer = 0, eeprom_address = 0, crc16 = 0, crc_buffer = 0;

EXTRA_STARTUP_COMMANDS();
ENABLE_LEVEL_PIN();
UART_TIMEOUT_ENABLE(); 
sget_char(); // Dummy receive used as a 200 ms delay as the UART is not yet enabled.
if( ((PIN_REG(CHECK_LEVEL_PORT) & (1 << CHECK_LEVEL_PIN))) ){ EXTRA_EXIT_COMMANDS(); asm("jmp 0x0000"); }

/* Initialization commands */
// Setup the UART
_UCSRA_ = 0;
// Enable the UART to receive/transmit and enable UART pins
_UCSRB_ = ( (1 << _RXEN_)|(1 << _TXEN_) );
// Set UART to receive and transmit 8 bits per character, 1 stop bit with no parity.
_UCSRC_ = ( USEURSEL | (1 << _UCSZ1_) | (1 << _UCSZ0_) );
// Set baudrate.
_UBRRH_ = BAUDREG/256; 
_UBRRL_ = BAUDREG%256; 
// Turn on the pull up resistor of the UART rx pin.
DDR_REG(UART_PORT) &= (~(1<<UART_RX_PIN));
PORT_REG(UART_PORT) |= (1<<UART_RX_PIN);
// Make the UART Txd pin an output.
DDR_REG(UART_PORT) |= (1<<UART_TX_PIN);
// Enable the LED indication.
ENABLE_LED();
#if INTIALIZATION_DELAY > 0
sget_char();
#endif
/* MAIN BOOTLOADER LOOP */
while(1)
    {
       // START THE STANDARD 128 BYTE PACKET X MODEM CRC FILE DOWNLOAD    
       // I tried to follow the crc X modem protocol as faithfully as i could   
       LED_ON();
       flash_address = 0;
       eeprom_address = 0;
       packet_number = 0;
       error_counter = 0;
       received_bytes = 0;
       memory_to_write = 0;
       memory_full = 0;
       UART_TIMEOUT_DISABLE(); //Disable uart receive timeout.
       receive_at_command();
       sput_str("START THE X MODEM TRANSFER\r\n");
       UART_TIMEOUT_ENABLE(); //Enable the UART timeout.

       do{
            sput_char(XMODEM_NCG);
         }while(sget_char() != XMODEM_SOH);

       // DOWNLOAD AND VERIFY ONE PACKET AT A TIME UNTILL THE BUFFER IS FULL. The buffer is equal to SPM_PAGESIZE.  
       /*------------------------------------------------------------------------------------------------------*/
       do{
           crc16 = 0;
#if BOOTLOADER_SAVE_CODE_SIZE == 1 
           sget_char();
           sget_char();
#elif BOOTLOADER_SAVE_CODE_SIZE == 0
           packet_number++; 
           packet_resend_required = 0;
           high_byte_buffer =  sget_char();     //get X modem packet number
           low_byte_buffer  =  sget_char();
           // Examine the packet number.
           if( (high_byte_buffer + low_byte_buffer) != 255 ) { packet_resend_required = (1<<2); }
           if( high_byte_buffer == (packet_number-1) ) { packet_resend_required |= (1<<1); }
           if(high_byte_buffer != packet_number) { packet_resend_required |= (1<<0); }
#endif
           // Now receive the packet data and perform the crc16 check.

           for(x = X_MODEM_PACKET_LENGTH; x > 0; x--)
             {
                x_buffer[received_bytes] = sget_char();
                crc16 = ( crc16 ^ ((unsigned int)x_buffer[received_bytes] << 8) );
                for(y = 8; y > 0; y--)
                  {
                     crc_buffer = crc16 << 1;
                     if(crc16 & 0x8000) { crc_buffer = crc_buffer ^ 0x1021; }
                     crc16 = crc_buffer;
                  }
                received_bytes++;
             }  
           high_byte_buffer = sget_char();         //get X modem CRC16
           low_byte_buffer  = sget_char();
#if BOOTLOADER_SAVE_CODE_SIZE == 0
           if( (high_byte_buffer != (crc16 / 256))||(low_byte_buffer != (crc16 % 256)) ){ packet_resend_required |= (1<<3); }
           // Now we must see if the packet was received ok or if not, what went wrong.   
           if(packet_resend_required > 0)
            {
               error_counter++;
               packet_number--;  // Packet number is decremented.
               received_bytes -= X_MODEM_PACKET_LENGTH;
               if( packet_resend_required >= (1<<2) )
                {
                   sput_char(XMODEM_NAK);

                 }else if(packet_resend_required >= (1<<1) )
                        {
                           sput_char(XMODEM_ACK);

                        }else{
                               goto XMODEM_FAILED;
                              } // end of "if( packet_resend_required >= (1<<2) )" statement.
#elif BOOTLOADER_SAVE_CODE_SIZE == 1
           if( (high_byte_buffer != (crc16 / 256))||(low_byte_buffer != (crc16 % 256)) )
            {
               goto XMODEM_FAILED; 
            
#endif
            }else{  // "if(packet_resend_required > 0)" statement.
                   if(memory_to_write == 1)
                    {
                       x = 0;
                       while( received_bytes >= SPM_PAGESIZE )
                           {
                              if(flash_address <= (BOOTLOADER_MEM_START-SPM_PAGESIZE) )
                               {
                                  address_buffer = x; // Store the x_buffer position for the verification process.
                                  y=0;
                                  do{
                                       boot_page_fill_safe( y, x_buffer[x] | (x_buffer[x+1] << 8) );
                                       x+=2; //BYTE COUNTER UP TO "received_bytes"
                                       y+=2; //BYTE COUNTER UP TO SPM_PAGESIZE 
                                    }while(y < SPM_PAGESIZE);
                                  boot_page_erase_safe(flash_address);      //erase one Flash page
                                  boot_page_write_safe(flash_address);      //write buffer to one Flash page
                                  boot_rww_enable_safe();
                                  y=0;
                                  do{
                                      if( pgm_read_byte(flash_address+y) != x_buffer[address_buffer+y] ){ goto XMODEM_FAILED; }
                                      y++;
                                    }while(y < SPM_PAGESIZE);
                                  flash_address += SPM_PAGESIZE; 
                                  received_bytes -= SPM_PAGESIZE;

                               }else{ memory_full = 1; goto XMODEM_FAILED; }
                           }

                    }else if(memory_to_write == 2)
                           {
                             for(x=0; x < X_MODEM_PACKET_LENGTH; x++)
                               { 
                                  if(eeprom_address <= E2END)
                                   { 
                                      eeprom_write_byte((unsigned char*)eeprom_address, x_buffer[x]);
                                      if(eeprom_read_byte((unsigned char*)eeprom_address) != x_buffer[x]){ goto XMODEM_FAILED; }
                                      eeprom_address++;

                                   }else{ memory_full = 1;  }
                               }
                             received_bytes = 0;
                           }
                   sput_char(XMODEM_ACK);
                   //error_counter =0;
                 }// end of "if(packet_resend_required > 0){...}else{...}" statement  

           if(error_counter >= 10){ goto XMODEM_FAILED; }  //Standard X modem max errors is 10, abort update.

         }while(sget_char() != XMODEM_EOT); // End of do{}while loop 
/*----------------------------------------------------------------------------------------------------------*/
       // The flash or Eeprom is written so it is time to exit.
       // exit the X modem transfer.
       sput_char(XMODEM_NAK);
       if(sget_char() == XMODEM_EOT)
        { 
           sput_char(XMODEM_ACK); 
           error_counter = 0;
        }
       else{
             // If the transfer failed send the X modem cancel character 10 times as per X modem specifications.
XMODEM_FAILED:
               y=0;
               do{
                    sput_char(XMODEM_CAN);
                    y++;

                 }while( y<10);
               error_counter = 1;
              
           }

       _UCSRB_ &= (~ (1 << _RXEN_));
       UART_TIMEOUT_1S();
       sget_char();   //dummy receive. It is used as delay.
       _UCSRB_ |= (1 << _RXEN_);
       if(memory_full)
        {
           sput_str("Error memory full");

        }else if(error_counter)
               {
                  sput_str(error_message);

               }else{
                       sput_str(update_success_message);
                    }
       //(*((void(*)(void))(BOOTLOADER_MEM_START)))(); 

    } // END of while(1) main loop.


}


/*###########################################################################################################*/
/*                                              T H E   E N D                                                */
/*###########################################################################################################*/
