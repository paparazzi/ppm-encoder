
/*********************************************************************************************************
 Title  :   C  file for the rc ppm encoder (servo2ppm_v4_3.c)
 Author:    Chris Efstathiou 
 E-mail:    hendrix at vivodinet dot gr
 Homepage:  ........................
 Date:      07/Feb/2010
 Compiler:  AVR-GCC with AVR-AS
 MCU type:  ATmega168 
 Comments:  This software is FREE. Use it at your own risk.
*********************************************************************************************************/


/********************************************************************************************************/
/*                                   PREPROCESSOR DIRECTIVES                                            */
/********************************************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include "servo2ppm_settings.h"




/********************************************************************************************************/
/*             Normaly you shouldn't need to change anything below this line but who knows?             */ 
/********************************************************************************************************/

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

#define RC_SERVO_PORT                 D   /* The port for the servo pulse input. */

#define RC_LED_PORT                   B  /* The port for the PPM waveform and the led. */
#define RC_LED_PIN                    0  /* The led indicator pin. */

#define RC_PPM_PORT                   B
#define RC_PPM_PIN                    2  /* The PPM waveform output pin */

#define RC_SETUP_PORT                 B
#define RC_SETUP_PIN                  4    

#define RC_TIMER0_TIMSK               TIMSK0 /* Timer0 registers */
#define RC_TIMER0_TIFR                TIFR0
#define RC_TIMER0_PRESCALER_REG       TCCR0B

#define RC_TIMER1_TIMSK               TIMSK1 /* Timer1 registers */ 
#define RC_TIMER1_TIFR                TIFR1
#define RC_TIMER1_PRESCALER_REG       TCCR1B
#define RC_TIMER1_MODE_REG            TCCR1A
#define RC_TIMER1_COMP1_REG           OCR1A
#define RC_TIMER1_COMP2_REG           OCR1B

#define RC_PIN_INT_EN_REG             PCICR
#define RC_PIN_INT_EN_BIT             PCIE2
#define RC_PIN_INT_FLAG_REG           PCIFR
#define RC_PIN_INT_FLAG_BIT           PCIF2
#define RC_PIN_INT_MASK_REG           PCMSK2

#define RC_SERVO_INPUT_CHANNELS       8

#else
/*
You can copy the above avr type data and edit them as needed if you plan to use
a different avr type cpu having at least 2k bytes of flash memory.
*/

#error AVR type not suitable
#endif

#define _CONCAT2_(a, b)               a ## b
#define CONCAT2(a, b)                 _CONCAT2_(a, b)

#define RC_SERVO_PORT_OUT_REG         CONCAT2(PORT, RC_SERVO_PORT)
#define RC_SERVO_PORT_DDR_REG         CONCAT2(DDR,  RC_SERVO_PORT)
#define RC_SERVO_PORT_PIN_REG         CONCAT2(PIN,  RC_SERVO_PORT)

#define RC_LED_PORT_OUT_REG           CONCAT2(PORT, RC_LED_PORT)
#define RC_LED_PORT_DDR_REG           CONCAT2(DDR,  RC_LED_PORT)
#define RC_LED_PORT_PIN_REG           CONCAT2(PIN,  RC_LED_PORT)

#define RC_PPM_PORT_OUT_REG           CONCAT2(PORT, RC_PPM_PORT)
#define RC_PPM_PORT_DDR_REG           CONCAT2(DDR,  RC_PPM_PORT)
#define RC_PPM_PORT_PIN_REG           CONCAT2(PIN,  RC_PPM_PORT)

#define RC_SETUP_PORT_OUT_REG         CONCAT2(PORT, RC_SETUP_PORT)
#define RC_SETUP_DDR_REG              CONCAT2(DDR,  RC_SETUP_PORT)
#define RC_SETUP_PIN_REG              CONCAT2(PIN,  RC_SETUP_PORT)


#if RC_PPM_GEN_CHANNELS > 8
#error PPM generator max number of channels is 8 
#endif
#if RC_PPM_GEN_CHANNELS == 0
#error PPM generator channels cannot be zero!
#endif

#if RC_PPM_GEN_CHANNELS < RC_SERVO_INPUT_CHANNELS
#undef RC_SERVO_INPUT_CHANNELS
#define RC_SERVO_INPUT_CHANNELS   RC_PPM_GEN_CHANNELS   
#warning servo channels = PPM generator channels
#endif

#if RC_PPM_RESET_PW == 0 && RC_PPM_FRAME_LENGTH_MS == 0 

#undef RC_PPM_FRAME_LENGTH_MS
#undef RC_PPM_RESET_PW
#define RC_PPM_RESET_PW              5500UL
#define RC_PPM_FRAME_LENGTH_MS       (RC_PPM_RESET_PW + (RC_PPM_GEN_CHANNELS * RC_SERVO_CENTER_PW)) 

#elif RC_PPM_FRAME_LENGTH_MS == 0 && RC_PPM_RESET_PW > 0

#undef RC_PPM_FRAME_LENGTH_MS
#define RC_PPM_FRAME_LENGTH_MS       ((RC_PPM_GEN_CHANNELS * RC_SERVO_MAX_PW)+RC_PPM_RESET_PW)

#elif RC_PPM_FRAME_LENGTH_MS > 0 && RC_PPM_RESET_PW == 0

#undef RC_PPM_RESET_PW
#define RC_PPM_RESET_PW              (RC_PPM_FRAME_LENGTH_MS -(RC_PPM_GEN_CHANNELS * RC_SERVO_MAX_PW)) 

#endif

#if RC_PPM_FRAME_LENGTH_MS > 25000UL
#warning PPM frame period exceeds 25ms 
#endif

#if RC_PPM_RESET_PW <= (5000UL + RC_PPM_CHANNEL_SYNC_PW)
#warning PPM reset period lower than 5ms 
#endif


/* Search for a suitable timer prescaler. Both timers must use the same prescaler factor. */
#if  ((F_CPU * (RC_MAX_TIMEOUT/1000))/1000) < 0xFFF0

#define TIMER0_PRESCALER   1
#define TIMER0_PRESCALER_BITS   ((0<<CS02)|(0<<CS01)|(1<<CS00))
#warning TIMER0 PRESCALER SET TO 1

#elif  (((F_CPU * (RC_PPM_FRAME_TIMEOUT/1000))/1000)/8) < 0xFFF0

#define TIMER0_PRESCALER   8
#define TIMER0_PRESCALER_BITS   ((0<<CS02)|(1<<CS01)|(0<<CS00))
#warning TIMER0 PRESCALER SET TO 8

#elif  (((F_CPU * (RC_PPM_FRAME_TIMEOUT/1000))/1000)/64) < 0xFFF0

#define TIMER0_PRESCALER   64
#define TIMER0_PRESCALER_BITS   ((0<<CS02)|(1<<CS01)|(1<<CS00))
#warning TIMER0 PRESCALER SET TO 64

#endif

#define TIMER1_PRESCALER    TIMER0_PRESCALER

#if TIMER1_PRESCALER == 1

#define TIMER1_PRESCALER_BITS   ((0<<CS12)|(0<<CS11)|(1<<CS10))
#warning TIMER1 PRESCALER SET TO 1

#elif  TIMER1_PRESCALER == 8

#define TIMER1_PRESCALER_BITS   ((0<<CS12)|(1<<CS11)|(0<<CS10))
#warning TIMER1 PRESCALER SET TO 8

#elif  TIMER1_PRESCALER == 64

#define TIMER1_PRESCALER_BITS   ((0<<CS12)|(1<<CS11)|(1<<CS10))
#warning TIMER1 PRESCALER SET TO 64

#else 

#error NO SUITABLE PRESCALER FOR TIMER1 FOUND!

#endif

/* Now that the timer prescalers are known the conversion of microseconds to timer values follows */
#define RC_SERVO_CENTER_PW_VAL          ((((F_CPU/1000) * RC_SERVO_CENTER_PW )/1000)/TIMER0_PRESCALER) 
#define RC_SERVO_MIN_PW_VAL             ((((F_CPU/1000) * RC_SERVO_MIN_PW)/1000)/TIMER0_PRESCALER)
#define RC_SERVO_MAX_PW_VAL             ((((F_CPU/1000) * RC_SERVO_MAX_PW)/1000)/TIMER0_PRESCALER)

#define RC_PPM_OFF_THRESHOLD_VAL        ((((F_CPU/1000) * RC_LOST_THRESHOLD)/1000)/TIMER0_PRESCALER)
#define RC_PPM_OFF_UPPER_WINDOW_VAL     ((((F_CPU/1000) * 1700)/1000)/TIMER0_PRESCALER) //1700 microseconds
#define RC_PPM_OFF_LOWER_WINDOW_VAL     ((((F_CPU/1000) * 1300)/1000)/TIMER0_PRESCALER) //1300 microseconds
#define RC_PPM_OFF_OFFSET_VAL           ((((F_CPU/1000) * RC_THROTTLE_CH_OFFSET_PW)/1000)/TIMER0_PRESCALER)

#define RC_MAX_TIMEOUT_HI_8_VAL         ((((((F_CPU/1000) * RC_MAX_TIMEOUT)/1000)/TIMER0_PRESCALER)/256)+1)
#define RC_MAX_PULSE_TIMEOUT_HI_8_VAL   ((((((F_CPU/1000) * RC_SERVO_MAX_PW)/1000)/TIMER0_PRESCALER)/256)+1)
#define RC_MIN_PULSE_TIMEOUT_HI_8_VAL   ((((((F_CPU/1000) * RC_SERVO_MIN_PW)/1000)/TIMER0_PRESCALER)/256)+1)

#define RC_PPM_FRAME_TIMER_VAL          ((((F_CPU/1000) * RC_PPM_FRAME_LENGTH_MS)/1000)/TIMER1_PRESCALER) 
#define RC_PPM_SYNC_PW_VAL              ((((F_CPU/1000) * RC_PPM_CHANNEL_SYNC_PW)/1000)/TIMER1_PRESCALER) 

#define RC_FS_CH_1_TIMER_VAL            ((((F_CPU/1000) * RC_FAILSAFE_CHANNEL_1)/1000)/TIMER1_PRESCALER)
#define RC_FS_CH_2_TIMER_VAL            ((((F_CPU/1000) * RC_FAILSAFE_CHANNEL_2)/1000)/TIMER1_PRESCALER)
#define RC_FS_CH_3_TIMER_VAL            ((((F_CPU/1000) * RC_FAILSAFE_CHANNEL_3)/1000)/TIMER1_PRESCALER)
#define RC_FS_CH_4_TIMER_VAL            ((((F_CPU/1000) * RC_FAILSAFE_CHANNEL_4)/1000)/TIMER1_PRESCALER)
#define RC_FS_CH_5_TIMER_VAL            ((((F_CPU/1000) * RC_FAILSAFE_CHANNEL_5)/1000)/TIMER1_PRESCALER)
#define RC_FS_CH_6_TIMER_VAL            ((((F_CPU/1000) * RC_FAILSAFE_CHANNEL_6)/1000)/TIMER1_PRESCALER)
#define RC_FS_CH_7_TIMER_VAL            ((((F_CPU/1000) * RC_FAILSAFE_CHANNEL_7)/1000)/TIMER1_PRESCALER)
#define RC_FS_CH_8_TIMER_VAL            ((((F_CPU/1000) * RC_FAILSAFE_CHANNEL_8)/1000)/TIMER1_PRESCALER)
#define RC_RESET_PW_TIMER_VAL           ((((F_CPU/1000) * RC_PPM_RESET_PW)/1000)/TIMER1_PRESCALER) 

#if F_CPU <= 8000000UL
#define  RC_NUMBER_OF_SAMPLES           6
#else
#define  RC_NUMBER_OF_SAMPLES           4
#endif 


//#define RC_TIMER0_VAL_CORRECTION        ((RC_SERVO_CENTER_PW_VAL *100)/RC_SERVO_CENTER_PW )

/* Some error checking */
/* The timer can only count up to 65535 so the high byte can go up to 255 */
#if (RC_MAX_TIMEOUT_HI_8_VAL  > 254)
#error RC_MAX_TIMEOUT  is set too high!
#endif

//It is used when there is no servo signals received so the elapsed time is always RC_MAX_TIMEOUT ms.
//In case the watchdog timer is used then the elapsed time is always the wdt timeout.
#define RC_LED_FREQUENCY_VAL_4HZ	(((1000000/4)/25000)/2) 
#if F_CPU <= 8000000UL
#define RC_LED_FREQUENCY_VAL_1HZ	(((1000000/1)/60000)/2) 
#else
#define RC_LED_FREQUENCY_VAL_1HZ	(((1000000/1)/30000)/2) 
#endif 



/* Macro command definitions. */
#define LED_ON()                        { RC_LED_PORT_OUT_REG |= (1<<RC_LED_PIN); }
#define LED_OFF()                       { RC_LED_PORT_OUT_REG &= (~(1<<RC_LED_PIN)); }
#define TOGGLE_LED()                    { RC_LED_PORT_OUT_REG ^= (1<<RC_LED_PIN); } 

#define STOP_TIMER0()                   { RC_TIMER0_PRESCALER_REG = 0; } 
#define START_TIMER0()                  { RC_TIMER0_PRESCALER_REG = TIMER0_PRESCALER_BITS; }
#define RESET_TIMER0()                  { RC_TIMER0_TIFR |= (1<<TOV1); TCNT0=0; timer0_h8=0; timer0_h8_buf=0; }
#define RESET_START_TIMER0()            { STOP_TIMER0(); RESET_TIMER0(); START_TIMER0(); }  

#if RC_USE_FAILSAFE > 1
#error RC_USE_FAILSAFE can be 0 or 1
#endif 
#if RC_PPM_OUTPUT_TYPE > 1
#error RC_PPM_OUTPUT_TYPE can be 0 or 1
#endif 
#if RC_CONSTANT_PPM_FRAME_TIME > 1
#error RC_CONSTANT_PPM_FRAME_TIME can be 0 or 1
#endif          
#if RC_REDUCE_LATENCY > 2
#error RC_REDUCE_LATENCY can be 0, 1 or 2
#endif 


/********************************************************************************************************/
/*                                   TYPE DEFINITIONS                                                   */
/********************************************************************************************************/







/********************************************************************************************************/
/*                                   LOCAL FUNCTION PROTOTYPES                                          */
/********************************************************************************************************/
void initialize_mcu(void);
void wait_for_rx(void);
unsigned char detect_connected_channels(void);
unsigned int get_channel_pw(unsigned char pin);
void check_for_setup_mode(void);
void write_default_values_to_eeprom(void);
void load_failsafe_values(void);
void load_values_from_eeprom(void);
static inline void ppm_off(void);
static inline void ppm_on(void);
void get_pw_of_connected_channels(unsigned char reset);




/********************************************************************************************************/
/*                                   GLOBAL VARIABLES                                                   */
/********************************************************************************************************/

volatile unsigned int   isr_channel_pw[(RC_PPM_GEN_CHANNELS +1)]; // +1 is for the reset pulse.
/*
unsigned int            fs_channel_pw[8]= { RC_FS_CH_1_TIMER_VAL, RC_FS_CH_2_TIMER_VAL,
                                            RC_FS_CH_3_TIMER_VAL, RC_FS_CH_4_TIMER_VAL,
                                            RC_FS_CH_5_TIMER_VAL, RC_FS_CH_6_TIMER_VAL,
                                            RC_FS_CH_7_TIMER_VAL, RC_FS_CH_8_TIMER_VAL,
                                          };
*/
#if (__GNUC__ == 4 && __GNUC_MINOR__ == 2)

register volatile unsigned char  timer0_l8 asm("r2");
register volatile unsigned char  timer0_h8 asm("r3");
register volatile unsigned int   isr_time_stamp asm("r4");
register volatile unsigned char  pin_interrupt_detected asm("r6");
register volatile unsigned char  sreg_save asm("r7");
register volatile unsigned char  isr_channel_number asm("r8");
register volatile unsigned char  channel_mask_buffer asm("r9"); 
register volatile unsigned char  pin_reg_buffer0 asm("r10");
register volatile unsigned char  pin_reg_buffer1 asm("r11");

#else
#warning GCC versions greater than 4.2.x have optimization problems with register variables. 
#warning No register variables are used.

volatile unsigned char  timer0_l8 = 0 ;
volatile unsigned char  timer0_h8 = 0 ;
volatile unsigned int   isr_time_stamp = 0;
volatile unsigned char  pin_interrupt_detected = 0;
volatile unsigned char  isr_channel_number = 0;
//volatile unsigned char  isr_pin_reg_buf = 0;
volatile unsigned char  channel_mask_buffer = 0; 

#endif

unsigned char                    channels_in_use = 0;
unsigned char                    channel_mask = 0;
volatile unsigned char           timer0_h8_buf = 0;
unsigned char                    servo_signals_lost = 0;
//unsigned char    ppm_status = 0;
volatile unsigned char wdt_timeout = 0;

#if RC_CONSTANT_PPM_FRAME_TIME == 1
volatile unsigned int            reset_pw = 0;
#endif

//unsigned char failsafe_mode = 0;
unsigned int  ppm_off_threshold = RC_PPM_OFF_THRESHOLD_VAL;
unsigned char rc_lost_channel = (RC_LOST_CHANNEL-1);
//Dummy variable array in order not to use the beggining of the eeprom for usefull data.
//The beggining of the eeprom and especially byte 0 is prone to data corruption.
// "EEMEM" is equal to "__attribute__((section(".eeprom")))"
unsigned int dummy_int[10] __attribute__((section(".eeprom")))= { 1, 1 ,1, 1, 1, 1, 1, 1, 1, 1 };
unsigned int EEMEM ppm_off_threshold_e[11] = { RC_PPM_OFF_THRESHOLD_VAL, RC_PPM_OFF_THRESHOLD_VAL, RC_PPM_OFF_THRESHOLD_VAL,
                                               RC_PPM_OFF_THRESHOLD_VAL, RC_PPM_OFF_THRESHOLD_VAL, RC_PPM_OFF_THRESHOLD_VAL,
                                               RC_PPM_OFF_THRESHOLD_VAL, RC_PPM_OFF_THRESHOLD_VAL, RC_PPM_OFF_THRESHOLD_VAL,
                                               RC_PPM_OFF_THRESHOLD_VAL, RC_PPM_OFF_THRESHOLD_VAL  
                                             };

unsigned char  EEMEM rc_lost_channel_e[11] = { RC_LOST_CHANNEL-1, RC_LOST_CHANNEL-1, RC_LOST_CHANNEL-1,
                                               RC_LOST_CHANNEL-1, RC_LOST_CHANNEL-1, RC_LOST_CHANNEL-1,
                                               RC_LOST_CHANNEL-1, RC_LOST_CHANNEL-1, RC_LOST_CHANNEL-1,
                                               RC_LOST_CHANNEL-1, RC_LOST_CHANNEL-1  
                                             };
//unsigned int EEMEM ch_failsafe_pw_e[RC_PPM_GEN_CHANNELS];
const char              version_info[]="Servo2ppm V4.30";

/********************************************************************************************************/
/*                                   LOCAL FUNCTION DEFINITIONS                                         */
/********************************************************************************************************/

/*11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/

void initialize_mcu(void)
{
unsigned char x = 0;
unsigned int  eep_address = 0;


asm("cli");

CLKPR = (1<<CLKPCE);
CLKPR = 0;


STOP_TIMER0();
RESET_TIMER0(); 

/* Enable pwm mode 15 (fast pwm with top=OCR1A) and stop the timer. */
/* The timer compare module must be configured before the DDR register. */
// THE PPM GENERATOR IS CONFIGURED HERE !
#if RC_PPM_OUTPUT_TYPE == 0   // NEGATIVE PULSES
RC_TIMER1_MODE_REG = (1<<COM1B1) | (1<<COM1B0) | (1<<WGM11) | (1<<WGM10);
#warning PPM output type set to NEGATIVE PULSE
#elif RC_PPM_OUTPUT_TYPE == 1   // POSITIVE PULSES
RC_TIMER1_MODE_REG = (1<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (1<<WGM10);
#warning PPM output type set to POSITIVE PULSE
#endif
RC_TIMER1_PRESCALER_REG = (1<<WGM13)|(1<<WGM12);
RC_TIMER1_COMP1_REG = RC_RESET_PW_TIMER_VAL;
RC_TIMER1_COMP2_REG  = RC_PPM_SYNC_PW_VAL;
RC_TIMER1_TIMSK |= (1<<OCIE1B);
RC_TIMER1_TIFR |= ( (1<<OCIE1B)|(1<<TOIE1) );

/*Enable timer0 overflow interupt and timer1 compare B interrupt */
RC_TIMER0_TIMSK |= (1<<TOIE0);
RC_TIMER0_TIFR |= (1<<TOIE0);

RC_LED_PORT_OUT_REG &= (~(1<<RC_LED_PIN));
RC_LED_PORT_DDR_REG |= (1<<RC_LED_PIN);
LED_ON();

// make the setup pin an input and activate the pull up resistor on the setup pin.
RC_SETUP_DDR_REG &= (~(1<<RC_SETUP_PIN));
RC_SETUP_PORT_OUT_REG |= (1<<RC_SETUP_PIN);

/* configure the servo pins as inputs and activate their pull up resistors for noise reduction. */
RC_SERVO_PORT_DDR_REG = 0;
RC_SERVO_PORT_OUT_REG = 0xFF;

isr_channel_number = RC_PPM_GEN_CHANNELS;
rc_lost_channel = (RC_LOST_CHANNEL-1);
ppm_off_threshold = RC_PPM_OFF_THRESHOLD_VAL;
// VERSION CONTROL 
x=0;
eep_address = (E2END - (sizeof(version_info)-1));
while(version_info[x])
    {
       if( (eep_address) < E2END)
        {
           eeprom_write_byte( (unsigned char*)eep_address, version_info[x]);
        }
       eep_address++;
       x++;
    }
eeprom_write_byte((unsigned char*)E2END, '\0');   //Terminate the version control string. 

asm("sei");

/* give some time for the pull up resistors to work. ~30 ms * 3 = 90 ms */
LED_OFF();
RESET_START_TIMER0();
for(x=0; x<5; x++)
  {
    wdt_reset();
    RESET_TIMER0();
    while(timer0_h8_buf < RC_MAX_TIMEOUT_HI_8_VAL ){ timer0_h8_buf = timer0_h8; } 
  }
x = 0;
STOP_TIMER0();
RESET_TIMER0();
LED_ON();
/*
The reason i configure the ppm port here is because i want to give some time
to the timer1 compare module to initialize.
The timer1 compare module as the Mega 168 manual states must be initialized before setting the DDR register
of the OCR1X pins.
*/
//ppm_status = 0;
RC_PPM_PORT_DDR_REG |= (1<<RC_PPM_PIN);

return;
}

/*11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
/*
This function waits untill the receiver has been powered up and running so we can then detect 
the connected channels with certainty.
*/
#if defined(RC_RX_READY_CHANNEL) && RC_RX_READY_CHANNEL > 0

void wait_for_rx(void)
{
unsigned int pw=0;
unsigned char x = 0;
unsigned char servo_connected = 0;


RESET_START_TIMER0();
servo_connected = 0;
do{

        wdt_reset(); 
        RESET_TIMER0();
        x=0;
        while(timer0_h8_buf <= RC_MAX_TIMEOUT_HI_8_VAL )
            {
              timer0_h8_buf = timer0_h8;
              if( (RC_SERVO_PORT_PIN_REG & (1<<(RC_RX_READY_CHANNEL-1))) ) { x=timer0_h8_buf; }
              if( (timer0_h8_buf - x) >= RC_MAX_PULSE_TIMEOUT_HI_8_VAL ){ servo_connected = 1; break; }
            }
 
  }while(servo_connected < 1);

//Now test the found channel for a proper servo pulse.
x = 0;
while(1)
    {
       pw=get_channel_pw( (RC_RX_READY_CHANNEL-1) );
       if( (pw > RC_SERVO_MIN_PW_VAL) && (pw < RC_SERVO_MAX_PW_VAL) ) { x++; }else{ x=0; }
       if(x >= 3) { break; }
    }

return;
}

#else

void wait_for_rx(void)
{
unsigned int pw=0;
unsigned char x = 0;
unsigned char servo_connected = 0;
unsigned char channel = 0;


RESET_START_TIMER0();
do{
    for(channel=0; channel < RC_SERVO_INPUT_CHANNELS; channel++)
      {
        wdt_reset(); 
        RESET_TIMER0();
        servo_connected = 0;
        x=0;
        while(timer0_h8_buf <= RC_MAX_TIMEOUT_HI_8_VAL )
            {
               timer0_h8_buf = timer0_h8; 
               if(RC_SERVO_PORT_PIN_REG & (1<<channel)) { x=timer0_h8_buf; }
               if( (timer0_h8_buf - x) >= RC_MAX_PULSE_TIMEOUT_HI_8_VAL ){ servo_connected = 1; break; }
            }
        if(servo_connected >= 1){ break; } 
      } 

  }while(servo_connected < 1);

//Now test the found channel for a proper servo pulse.
x = 0;
while(1)
    {
       pw=get_channel_pw(channel);
       if( (pw > RC_SERVO_MIN_PW_VAL) && (pw < RC_SERVO_MAX_PW_VAL) ) { x++; }else{ x=0; }
       if(x >= 3) { break; }
    }

return;
}
#endif

/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
/*33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333*/

unsigned char detect_connected_channels(void)
{
unsigned char channel=0;
unsigned char servo_connected = 0;
unsigned char connected_channels = 0;
unsigned char x = 0;
unsigned char y = 0;
#if defined(RC_RX_READY_CHANNEL) && RC_RX_READY_CHANNEL > 0
unsigned int pw = 0;
#endif


/* 
There must be no error in which channels are connected to the encoder
because this will have devastating effects later on.
*/
wdt_reset(); 
RESET_START_TIMER0();

      for(channel=0; channel < RC_SERVO_INPUT_CHANNELS; channel++)
        {
           servo_connected = 0;
           for(y=0; y<5; y++)
             {
                wdt_reset(); 
                RESET_TIMER0();
                x=0;
                while(timer0_h8_buf <= RC_MAX_TIMEOUT_HI_8_VAL )
                    {
                       timer0_h8_buf = timer0_h8;
                       if(RC_SERVO_PORT_PIN_REG & (1<<channel)) { x=timer0_h8_buf; }
                       if( (timer0_h8_buf - x) >= RC_MAX_PULSE_TIMEOUT_HI_8_VAL ){ servo_connected++; break; }
                    }
                if(servo_connected >= 3){ channel_mask |= (1<<channel); connected_channels++; break; } 
             }
        } 
#if defined(RC_RX_READY_CHANNEL) && RC_RX_READY_CHANNEL > 0

connected_channels = 0;
for(channel=0; channel < RC_SERVO_INPUT_CHANNELS; channel++)
  {
    if(channel_mask & (1<<channel) )
     { 
        pw=get_channel_pw(channel);
        if(pw < RC_SERVO_MIN_PW_VAL || pw > RC_SERVO_MAX_PW_VAL)
         {
            channel_mask &= (~(1<<channel));
         }else{ connected_channels++; }
     }
  }
#endif


return(connected_channels);
}

/*33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333*/
/*44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444*/

void write_default_values_to_eeprom(void)
{

unsigned char x = 0;

for(x=0; x<(sizeof(ppm_off_threshold_e)/sizeof(int)); x++)
  { 
     eeprom_write_word(&ppm_off_threshold_e[x], RC_PPM_OFF_THRESHOLD_VAL);
  }
for(x=0; x < (sizeof(rc_lost_channel_e)/sizeof(char)); x++)
  { 
     eeprom_write_byte(&rc_lost_channel_e[x], (RC_LOST_CHANNEL - 1));
  }  
rc_lost_channel = (RC_LOST_CHANNEL - 1);
ppm_off_threshold = RC_PPM_OFF_THRESHOLD_VAL;

return;
}

/*44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444*/
/*55555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555*/

/*
All this code because the Eeprom is prone to data corruption in noisy environments.
11 copies of each variable are read and if more than 50% + 1 values are the same and within limits
this value is taken to be valid.
If not all 11 values are the same then the array is written again with this 50% +1 value. 
*/
void load_values_from_eeprom(void)
{
unsigned int  eeprom_buf_x = 0;
unsigned int  eeprom_buf_y = 0;
unsigned char match = 0;
unsigned char match_upper_limit = 0;
unsigned char match_lower_limit = 0;
unsigned char x = 0;
unsigned char y = 0;

wdt_reset();

/****************************************************************************************************/
/*                 READ WHICH CHANNEL WILL BE USED AS A RX LOST INDICATOR                           */
/****************************************************************************************************/ 
match_upper_limit = (sizeof(rc_lost_channel_e)/sizeof(char));
match_lower_limit = (((sizeof(rc_lost_channel_e)/sizeof(char))/2)+1);

for(x=0; x<match_upper_limit; x++)
  {
    match = 0;
    eeprom_buf_x = eeprom_read_byte(&rc_lost_channel_e[x]); 
    for(y=0; y<match_upper_limit; y++)
      {
         eeprom_buf_y = eeprom_read_byte(&rc_lost_channel_e[y]);
         if(eeprom_buf_y == eeprom_buf_x){ match++; }
      }
    // If 50% +1 or more char values in the array are the same a match has been found. 
    // Now test them to see if they are within limits.
    if(match >= match_lower_limit )
     {
        if( eeprom_buf_x < RC_SERVO_INPUT_CHANNELS )
         {
           rc_lost_channel = eeprom_buf_x; //Load the stored value to throttle_thershold.
           if(match < match_upper_limit)
            {
               for(x=0; x<match_upper_limit; x++)
                 {
                    eeprom_write_byte(&rc_lost_channel_e[x], eeprom_buf_x); 
                 }
            }

         }else{ match = 0; }
        break;
     }
  }

if( match < match_lower_limit ){ write_default_values_to_eeprom();  return; }

/****************************************************************************************************/
/*             NOW READ THE CHANNEL'S PULSE WIDTH SO IT CAN BE USED AS A TRIGGER                    */
/****************************************************************************************************/ 
match_upper_limit = (sizeof(ppm_off_threshold_e)/sizeof(int));
match_lower_limit = (((sizeof(ppm_off_threshold_e)/sizeof(int))/2)+1);

for(x=0; x < match_upper_limit; x++)
  {
    match = 0;
    eeprom_buf_x = eeprom_read_word(&ppm_off_threshold_e[x]); 
    for(y=0; y < match_upper_limit; y++)
      {
         eeprom_buf_y = eeprom_read_word(&ppm_off_threshold_e[y]);
         if(eeprom_buf_y == eeprom_buf_x){ match++; }
      }
    // If 50% +1 or more integer values in the array are the same a match has been found. 
    // Now test them to see if they are within limits.
    if(match >= match_lower_limit )
     {
        if( (eeprom_buf_x >= RC_PPM_OFF_UPPER_WINDOW_VAL) && (eeprom_buf_x <= RC_SERVO_MAX_PW_VAL) )
         {
            ppm_off_threshold = eeprom_buf_x; //Load the stored value to throttle_thershold.
            if(match < match_upper_limit)
             {
               for(x=0; x < match_upper_limit; x++){ eeprom_write_word(&ppm_off_threshold_e[x], eeprom_buf_x); }
             }
        
          }else if( (eeprom_buf_x <= RC_PPM_OFF_LOWER_WINDOW_VAL) && (eeprom_buf_x >= RC_SERVO_MIN_PW_VAL) )
                 {
                    ppm_off_threshold = eeprom_buf_x; //Load the stored value to throttle_thershold.
                    if(match < match_upper_limit)
                     {
                       for(x=0; x < match_upper_limit; x++){ eeprom_write_word(&ppm_off_threshold_e[x], eeprom_buf_x); }
                     }

                 }else{ match = 0; }
        break;
     }
  }

if( match < match_lower_limit ){ write_default_values_to_eeprom();  return; }


return;
}

/*55555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555*/
/*66666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666*/

unsigned int get_channel_pw(unsigned char pin)
{

unsigned int  pw = 0;
unsigned char pw_measurement_started = 0;

wdt_reset();
/* The servo input pins are already configured as inputs with pullup resistors. */ 
// First we must disable the pin interrupt. 
RC_PIN_INT_EN_REG &= (~(1<<RC_PIN_INT_EN_BIT));
//Now we must load the pin interrupt mask register.
RC_PIN_INT_MASK_REG = (1<<pin);
//Clear any pin interrupt flag set. 
RC_PIN_INT_FLAG_REG |= (1<<RC_PIN_INT_FLAG_BIT);
// Clear the pin interrupt ISR detection variable
pin_interrupt_detected = 0;
//Now we can enable the pin interrupt again. 
RC_PIN_INT_EN_REG |= (1<<RC_PIN_INT_EN_BIT); 
// Set and start timer1
RESET_START_TIMER0();
//Turn off any pending pin interrupt
RC_PIN_INT_FLAG_REG |= (1<<RC_PIN_INT_FLAG_BIT);
// clear the pin interrupt flag.
pin_interrupt_detected = 0;
while(1)
    {  
       /* Wait until the pin change state. */
       do{  
            timer0_h8_buf = timer0_h8;
            if( timer0_h8_buf >=  RC_MAX_TIMEOUT_HI_8_VAL ) { return(0);  }
                 
         }while( pin_interrupt_detected == 0 );
         pin_interrupt_detected = 0; 
         if( RC_SERVO_PORT_PIN_REG & (1<<pin) ) /* if the pin is high then give it a time stamp */ 
          {
             pw = isr_time_stamp;
             pw_measurement_started = 1;        /* signal that this channel got it's timer stamp.*/
                                       
          }else{ 
                  // If the pin is low and it already has a time stamp then we are done.
                  if( pw_measurement_started )
                   {
                      pw = isr_time_stamp - pw;
                      break;
                   }
               }
    }  

/*Stop the timer */
STOP_TIMER0(); 
RESET_TIMER0();
   
return((unsigned int)pw);   
}

/*66666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666*/
/*77777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777*/

void check_for_setup_mode(void)
{
unsigned int  pw_buffer = 0;
unsigned int  pw = 0;
unsigned char setup_mode = 0;
unsigned char success = 0;
unsigned char x = 0;
unsigned char y = 0;


//We have to make sure that the setup mode is requested.  
wdt_reset();
x=0;
y=0;
setup_mode = 1;
RESET_START_TIMER0();
do{
    timer0_h8_buf = timer0_h8;
    if( (RC_SETUP_PIN_REG & (1<<RC_SETUP_PIN)) == 0){ x++; }else{ x=0; }
      {
         if(timer0_h8_buf > RC_MAX_TIMEOUT_HI_8_VAL ){setup_mode = 0; break; } 
      }

  }while(x < 100);

if(setup_mode)
 {
/****************************************************************************************************/
/*             FIRST WE MUST FIND WHICH CHANNEL WILL BE USED AS A TX SIGNAL LOST INDICATOR          */
/****************************************************************************************************/
     wdt_reset();
     success = 0;
     if(channels_in_use > 1 )
      {
         if( channel_mask & (1<<(RC_LOST_CHANNEL - 1)) )
          { 
             rc_lost_channel = (RC_LOST_CHANNEL - 1); 
             for(x=0; x < (sizeof(rc_lost_channel_e)/sizeof(char)); x++)
               { 
                  eeprom_write_byte(&rc_lost_channel_e[x], rc_lost_channel);
               } 
             success += 1;
          }
 
      }else if(channels_in_use == 1)
             {
               for(x=0; x < RC_SERVO_INPUT_CHANNELS; x++)
                 {
                   if(channel_mask & (1<<x))
                    { 
                       rc_lost_channel = x;
                       for(x=0; x < (sizeof(rc_lost_channel_e)/sizeof(char)); x++)
                         { 
                            eeprom_write_byte(&rc_lost_channel_e[x], rc_lost_channel);
                         } 
                       success += 1;
                       break;
                    }
                 }
             }


/****************************************************************************************************/
/*   NOW WE NEED TO FIND THE THRESHOLD PULSE WIDTH THAT WILL BE USED AN A TX SIGNAL LOST TRIGGER    */
/****************************************************************************************************/
     if(success == 1)
      {
         wdt_reset();
         y = 0;
         pw_buffer = 0;
         for(x=0; x < 10; x++)
           {
              pw=get_channel_pw(rc_lost_channel);
              if(pw >= RC_SERVO_MIN_PW_VAL && pw <= RC_SERVO_MAX_PW_VAL)
               {
                  pw_buffer += pw;
                  y++;        
               }
           }
         pw_buffer /= y;
         wdt_reset();
         if( (pw_buffer >= RC_PPM_OFF_UPPER_WINDOW_VAL) && (pw_buffer <= RC_SERVO_MAX_PW_VAL) )
          {
             for(x=0; x<(sizeof(ppm_off_threshold_e)/sizeof(int)); x++)
               { 
                 eeprom_write_word(&ppm_off_threshold_e[x], (pw_buffer+RC_PPM_OFF_OFFSET_VAL));
               } 
             success += 1;

         }else if( (pw_buffer <= RC_PPM_OFF_LOWER_WINDOW_VAL) && (pw_buffer >= RC_SERVO_MIN_PW_VAL) )
                {
                   for(x=0; x<(sizeof(ppm_off_threshold_e)/sizeof(int)); x++)
                     { 
                       eeprom_write_word(&ppm_off_threshold_e[x], (pw_buffer-RC_PPM_OFF_OFFSET_VAL));
                     } 
                   success += 1;
                }

      }
           
/****************************************************************************************************/
/*            LASTLY WE MUST INDICATE TO THE USER IF THE SETUP PROCEDURE WAS SUCCESSFUL             */
/****************************************************************************************************/     
     if(success == 2)
      { 
        RC_SETUP_PORT_OUT_REG &= (~(1<<RC_SETUP_PIN));
        while(1)
            {
              LED_ON(); 
              for(x=0; x<3; x++)
                 {
                   wdt_reset();
                   RESET_START_TIMER0();
                   /* delay ~30 ms * 3  = 100 milliseconds */
                   while(timer0_h8_buf < RC_MAX_TIMEOUT_HI_8_VAL ){ timer0_h8_buf = timer0_h8; } 
                 }
               LED_OFF();
               for(x=0; x<30; x++)
                 {
                   wdt_reset();
                   RESET_START_TIMER0();
                   /* delay ~30 ms * 30  = 900 milliseconds */
                   while(timer0_h8_buf < RC_MAX_TIMEOUT_HI_8_VAL ){ timer0_h8_buf = timer0_h8; }  
                 }
            }

      }else{ 
              write_default_values_to_eeprom();
              while(1)
                  {
                     LED_ON(); 
                     for(x=0; x<30; x++)
                       {
                         wdt_reset();
                         RESET_START_TIMER0();
                         /* delay ~30 ms * 30  = 900 milliseconds */
                         while(timer0_h8_buf < RC_MAX_TIMEOUT_HI_8_VAL ){ timer0_h8_buf = timer0_h8; }  
                       }
                     LED_OFF();
                     for(x=0; x<3; x++)
                       {
                          wdt_reset();
                          RESET_START_TIMER0();
                          /* delay ~30 ms * 3  = 100 milliseconds */
                          while(timer0_h8_buf < RC_MAX_TIMEOUT_HI_8_VAL ){ timer0_h8_buf = timer0_h8; }  
                       }
                  }
           }

 } // End of "if(setup_mode)" statement.

return;
}

/*77777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777777*/
/*88888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888*/

void load_failsafe_values(void)
{
wdt_reset();
isr_channel_pw[0] = RC_FS_CH_1_TIMER_VAL;
#if   RC_PPM_GEN_CHANNELS >= 2
isr_channel_pw[1] = RC_FS_CH_2_TIMER_VAL;
#endif
#if RC_PPM_GEN_CHANNELS >= 3
isr_channel_pw[2] = RC_FS_CH_3_TIMER_VAL;
#endif
#if RC_PPM_GEN_CHANNELS >= 4
isr_channel_pw[3] = RC_FS_CH_4_TIMER_VAL;
#endif
#if RC_PPM_GEN_CHANNELS >= 5
isr_channel_pw[4] = RC_FS_CH_5_TIMER_VAL;
#endif
#if RC_PPM_GEN_CHANNELS >= 6
isr_channel_pw[5] = RC_FS_CH_6_TIMER_VAL;
#endif
#if RC_PPM_GEN_CHANNELS >= 7
isr_channel_pw[6] = RC_FS_CH_7_TIMER_VAL;
#endif
#if RC_PPM_GEN_CHANNELS >= 8
isr_channel_pw[7] = RC_FS_CH_8_TIMER_VAL;
#endif
#if RC_PPM_GEN_CHANNELS >= 9
isr_channel_pw[8] = RC_FS_CH_8_TIMER_VAL;
#endif
isr_channel_pw[RC_PPM_GEN_CHANNELS] = RC_RESET_PW_TIMER_VAL;


return;
} 
/*
void load_failsafe_values(void)
{
unsigned char x = 0;

for(x=0; x< RC_PPM_GEN_CHANNELS; x++)
  {
    isr_channel_pw[x] = fs_channel_pw[x];
  }

// LOAD THE PPM FRAME RESET PULSE WIDTH.
isr_channel_pw[RC_PPM_GEN_CHANNELS] = RC_RESET_PW_TIMER_VAL;


return;

} 
*/

/*88888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888888*/
/*99999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999*/

static inline void ppm_on(void)
{

   RC_TIMER1_PRESCALER_REG  &= (~(TIMER1_PRESCALER_BITS));	
   TCNT1 = 0; 
   isr_channel_number = RC_PPM_GEN_CHANNELS;		
   RC_TIMER1_COMP1_REG = RC_RESET_PW_TIMER_VAL;
   RC_TIMER1_COMP2_REG = RC_PPM_SYNC_PW_VAL;	
   RC_TIMER1_TIFR |= ( (1<<OCIE1B)|(1<<TOV1) );			
   RC_TIMER1_PRESCALER_REG |= TIMER1_PRESCALER_BITS;
//   ppm_status = 1;

return;
}

/*99999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999*/
/*10101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010*/

static inline void ppm_off(void)
{
   if(RC_TIMER1_PRESCALER_REG & TIMER1_PRESCALER_BITS)
    {
      while( TCNT1 <= (RC_PPM_SYNC_PW_VAL+(RC_PPM_SYNC_PW_VAL / 10)) );
    } 
   RC_TIMER1_PRESCALER_REG &= (~(TIMER1_PRESCALER_BITS));   
   RC_TIMER1_TIFR |= ( (1<<OCIE1B)|(1<<TOV1) ); 
//   ppm_status = 0;

return;
}

/*10101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010*/
/*11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11*/


/*11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11-11*/
/*12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12-12*/

#if !defined(RC_MIN_LATENCY) || RC_MIN_LATENCY == 0
//Added this to change filter size
#if !defined(PPM_FILTER_WINDOW)
#define PPM_FILTER_WINDOW 3
#endif

void get_pw_of_connected_channels(unsigned char reset)
{

  unsigned short        pw_of_channel[RC_SERVO_INPUT_CHANNELS];
  #if (__GNUC__ != 4 || __GNUC_MINOR__ != 2)
  unsigned char         pin_reg_buffer0 = 0;
  static unsigned char  pin_reg_buffer1 = 0;
  #endif
  unsigned char         channels_to_check=0;
  unsigned char         channel_status = 0;
  unsigned char         channel = 0;
  unsigned int          pw_buffer = 0;
  unsigned int          time_stamp = 0;
  static unsigned char  sample = 0;
  static unsigned short pw_memory[RC_SERVO_INPUT_CHANNELS][PPM_FILTER_WINDOW];

  static signed char    pw_change[RC_SERVO_INPUT_CHANNELS];

  unsigned char         x = 0;
  unsigned char         y = 0;


/*------------------------------------------------------------------------------------------------------*/
/* THIS IS FOR RECEIVERS THAT DO NOT OUTPUT THE SERVO PULSES AT THE SAME TIME AND HAVE A SOME DEAD TIME */
/*------------------------------------------------------------------------------------------------------*/
  if(reset)
  {
    //CHANGED BELOW LINE, NEEDED TO SWAP [y][x] to [x][y] in pw_memory
    for(x=0; x < RC_SERVO_INPUT_CHANNELS; x++)
    {
      for(y=0; y < PPM_FILTER_WINDOW; y++)
      {
        pw_memory[x][y] = 0;
      }
    }
    sample = 0;
    for(x=0; x < RC_SERVO_INPUT_CHANNELS; x++)
    {
      pw_change[x] = 0;
    }
    RESET_START_TIMER0(); 
    pin_reg_buffer1 = RC_SERVO_PORT_PIN_REG & channel_mask;
    RC_PIN_INT_FLAG_REG |= (1<<RC_PIN_INT_FLAG_BIT);
    pin_interrupt_detected = 0;
  }
  wdt_reset();
  channel_mask_buffer = channel_mask;
  channel_status = 0;
  /* This is the main pulse measuring loop. It makes sure that only complete servo pulses are measured. */
  while(channel_mask_buffer)
  { 
    x=0;
    y=1;
    /* Wait until a pin change state. */
    while( pin_interrupt_detected == 0 );
    asm("cli");
    time_stamp = isr_time_stamp;
    pin_interrupt_detected = 0; 
    RC_PIN_INT_FLAG_REG |= (1<<RC_PIN_INT_FLAG_BIT); 
    pin_reg_buffer0 = RC_SERVO_PORT_PIN_REG;
    asm("sei");
    //Only the pins that changed state will be examined.
    pin_reg_buffer0 &= channel_mask_buffer;
    channels_to_check = pin_reg_buffer1 ^ pin_reg_buffer0;
    if(channels_to_check)
    {
      while(x < RC_SERVO_INPUT_CHANNELS)
      {
        if(channels_to_check & y)
        {
          if( (pin_reg_buffer0 & y) ) /* if the pin is high then... */ 
          {
            pw_of_channel[x] = time_stamp;
            channel_status |= y;
          }
          else
          {
            if(channel_status & y)
            { 
              channel_mask_buffer &= (~y);
              pw_of_channel[x] = time_stamp - pw_of_channel[x];
            }
          }   // End of "if( (pin_reg_buffer0 & y) )...else..." statement         
        }	// End of "if(channels_to_check & y)" statement.
        x++;
        y=(y<<1);
      }   //End of while(x < RC_SERVO_INPUT_CHANNELS) loop.
      pin_reg_buffer1 = pin_reg_buffer0; 
    }
  }	// End of "while(channel_mask_buffer)" loop.
  RESET_START_TIMER0();
  if (wdt_timeout){
    WDTCSR |= (1<<WDIE); 
    wdt_reset();
    wdt_timeout = 0; 
    channel_mask_buffer = 0xFF; 
  }  
  /*------------------------------------------------------------------------------------------------------*/
  /*                              CHECK THE RX LOST INDICATOR CHANNEL                                     */
  /*------------------------------------------------------------------------------------------------------*/
#if defined(RC_LOST_CHANNEL) && RC_LOST_CHANNEL > 0
  if( (channel_mask & (1<<rc_lost_channel)) )
  { 
    if(channel_mask_buffer  == 0)
    {
      pw_buffer = pw_of_channel[rc_lost_channel];
      if( (pw_buffer > RC_SERVO_MIN_PW_VAL) && (pw_buffer < RC_SERVO_MAX_PW_VAL)  )
      { 
        if(ppm_off_threshold > RC_SERVO_CENTER_PW_VAL)
        { 
          if(pw_buffer >= ppm_off_threshold)
          {
            channel_mask_buffer = 0xFF;
            return;
          }
        }
        else
        {
          if(pw_buffer <= ppm_off_threshold)
          {
            channel_mask_buffer = 0xFF;
            return;
          }
        }
      }
    }
  }
#endif //defined(RC_LOST_CHANNEL) && RC_LOST_CHANNEL > 0
  /*------------------------------------------------------------------------------------------------------*/
  /*                                        SIMPLE PPM FILTER                                             */
  /*------------------------------------------------------------------------------------------------------*/

  if(servo_signals_lost == 0 && channel_mask_buffer == 0)
  {
    for(channel=0; channel < RC_SERVO_INPUT_CHANNELS; channel++)
    {
      if(channel_mask & (1<<channel))
      {   
        if( (pw_of_channel[channel] > RC_SERVO_MIN_PW_VAL) && (pw_of_channel[channel] < RC_SERVO_MAX_PW_VAL)  )
        { 
          pw_buffer = pw_of_channel[channel];
          x = 1;
          for(y=0; y<PPM_FILTER_WINDOW; y++)
          {
            if(pw_memory[channel][y])
            {
              pw_buffer += pw_memory[channel][y];
              x++;
            }
          }
          pw_buffer /= x;
          pw_memory[channel][sample] = pw_of_channel[channel];
    //      isr_channel_pw[channel]= pw_buffer;
          // The number 10 represent timer ticks and not microseconds.
          if(pw_buffer >= (isr_channel_pw[channel]+10) || pw_buffer <= (isr_channel_pw[channel]-10))
          { 
            isr_channel_pw[channel]= pw_buffer;
            pw_change[channel] = 0;   
          }
          else if(pw_of_channel[channel] > (isr_channel_pw[channel]+1))
          {
            if(pw_change[channel] < 0)
            {
              pw_change[channel] = 0;
            } 
            if(pw_change[channel] >= RC_NUMBER_OF_SAMPLES) //CHANGED THIS TO MATCH THE NO FILTER VERSION, but that one isn't working?
            {
              isr_channel_pw[channel]= pw_buffer;
            }
            else
            {
              pw_change[channel]++;
            } 
          }
          else if(pw_of_channel[channel] < (isr_channel_pw[channel]-1))
          {
            if(pw_change[channel] > 0)
            {
              pw_change[channel] = 0;
            } 
            if(pw_change[channel] <= -(RC_NUMBER_OF_SAMPLES))
            {
              isr_channel_pw[channel]= pw_buffer;
            }
            else
            {
              pw_change[channel]--;
            }
          }
          else
          {
            pw_change[channel] = 0;
          }
        }   //End of "if( (pw_of_channel[channel] > RC_SERVO_MIN_PW_VAL) && (pw_of_channel[channel] < RC_SERVO_MAX_PW_VAL)  )" statement.
      }   //End of " if(channel_mask & (1<<channel))" statement
    }   //End of "for(channel=0; channel < RC_SERVO_INPUT_CHANNELS; channel++)" loop
    sample++;
    if(sample >=PPM_FILTER_WINDOW ){ sample = 0; }
  }   //End of "if(servo_signals_lost == 0)" statement.

  return;
}

#else //#if !defined(RC_MIN_LATENCY) || RC_MIN_LATENCY == 0

#warning PPM filter is OFF

void get_pw_of_connected_channels(unsigned char reset)
{

  unsigned short        pw_of_channel[RC_SERVO_INPUT_CHANNELS];
  static signed char    pw_change[RC_SERVO_INPUT_CHANNELS];
  unsigned char         channel = 0;
  unsigned short        isr_pw_buffer = 0;
  #if (__GNUC__ != 4 || __GNUC_MINOR__ != 2)
  unsigned char         pin_reg_buffer0 = 0;
  static unsigned char  pin_reg_buffer1 = 0;
  #endif
  unsigned char         channels_to_check=0;
  unsigned char         channel_status = 0;
  unsigned int          pw_buffer = 0, time_stamp = 0;
  unsigned char         x = 0;
  unsigned char         y = 0;

  if(reset)
  {
    //added this for loop to make consistent with filtered version
    for(x=0; x < RC_SERVO_INPUT_CHANNELS; x++)
    {
      pw_change[x] = 0;
    }
    RESET_START_TIMER0(); 
    pin_reg_buffer1 = (RC_SERVO_PORT_PIN_REG & channel_mask);
    RC_PIN_INT_FLAG_REG |= (1<<RC_PIN_INT_FLAG_BIT);
    pin_interrupt_detected = 0; 
  }
  wdt_reset();
  channel_mask_buffer = channel_mask;
  channel_status = 0;
  while(channel_mask_buffer)
  { 
    x=0;
    y=1;
    /* Wait until a pin change state. */
    while( pin_interrupt_detected == 0 );
    asm("cli");
    time_stamp = isr_time_stamp;
    pin_interrupt_detected = 0; 
    RC_PIN_INT_FLAG_REG |= (1<<RC_PIN_INT_FLAG_BIT); 
    pin_reg_buffer0 = RC_SERVO_PORT_PIN_REG;
    asm("sei");
    //Only the pins that changed state will be examined.
    pin_reg_buffer0 &= channel_mask_buffer;
    channels_to_check = pin_reg_buffer1 ^ pin_reg_buffer0;
    if(channels_to_check)
    {
      while(x<RC_SERVO_INPUT_CHANNELS)
      {
        if(channels_to_check & y)
        {
          if( (pin_reg_buffer0 & y) )         // if the pin is high then...  
          {
            pw_of_channel[x] = time_stamp;  //Store the time that the servo pulse started.
            channel_status |= y;            //Signal that the servo pulse started. 
          }
          else
          {
  		      // If and only if the channel has been previously high then that is the end of the pulse.
            if(channel_status & y)
            {
              channel_mask_buffer &= (~y);
              pw_buffer = time_stamp - pw_of_channel[x];
              if( (pw_buffer > RC_SERVO_MIN_PW_VAL) && (pw_buffer < RC_SERVO_MAX_PW_VAL) )
              { 
#if defined(RC_LOST_CHANNEL) && RC_LOST_CHANNEL > 0
                if(x == rc_lost_channel)
                {
                  if(ppm_off_threshold > RC_SERVO_CENTER_PW_VAL)
                  { 
                    if(pw_buffer >= ppm_off_threshold)
                    {
                      channel_mask_buffer = 0xFF;
                      return;
                    }
                  }
                  else
                  {
                    if(pw_buffer <= ppm_off_threshold)
                    {
                      channel_mask_buffer = 0xFF;
                      return;
                    }
                  }
                }
#endif
                if (servo_signals_lost == 0)
                {
                  isr_pw_buffer = isr_channel_pw[channel];
                  if (pw_buffer >= (isr_pw_buffer+10) || pw_buffer <= (isr_pw_buffer-10))
                  { 
                    isr_channel_pw[channel]= pw_buffer;
                    pw_change[channel] = 0;   
                  }
                  else if (pw_of_channel[channel] > (isr_pw_buffer+1))
                  {
                    if(pw_change[channel] < 0)
                    {
                      pw_change[channel] = 0;
                    } 
                    if (pw_change[channel] >= RC_NUMBER_OF_SAMPLES)
                    {
                      isr_channel_pw[channel]= pw_buffer;
                    }
                    else
                    {
                      pw_change[channel]++;
                    } 
                  }
                  else if (pw_of_channel[channel] < (isr_pw_buffer-1))
                  {
                    if(pw_change[channel] > 0)
                    {
                      pw_change[channel] = 0;
                    } 
                    if (pw_change[channel] <= -(RC_NUMBER_OF_SAMPLES))
                    {
                      isr_channel_pw[channel]= pw_buffer;          
                    }
                    else
                    {
                      pw_change[channel]--;
                    } 

                  }
                  else
                  {
                    pw_change[channel] = 0;
                  }
                }
              } // End of "if( (pw_buffer > RC_SERVO_MIN_PW_VAL) && (pw_buffer < RC_SERVO_MAX_PW_VAL) )" statement. 
            } // End of "if(channel_status & y)" statement.
          }   // End of "if( (pin_reg_buffer0 & y) )...else..." statement         
        }	// End of "if(channels_to_check & y)" statement.
        x++;
        y=(y<<1);
      }   // End of "while(x<RC_SERVO_INPUT_CHANNELS)" loop.
    }
    pin_reg_buffer1 = pin_reg_buffer0; 
  }// End of "while(channel_mask_buffer)" loop.
  RESET_START_TIMER0();
  if (wdt_timeout){
     WDTCSR |= (1<<WDIE); 
     wdt_reset();
     wdt_timeout = 0; 
     channel_mask_buffer = 0xFF; 
  } 

  return;
}

#endif

/********************************************************************************************************/
/*                                         MAIN FUNCTION                                                */
/********************************************************************************************************/
__attribute__((noreturn)) void main(void)
{

//Variables for the PPM control code.
unsigned char  tx_signal_lost = 0;
unsigned char  tx_signal_detected = 0;
unsigned char  led_frequency = 0;
unsigned char  led_counter = 0;

wdt_disable();
wdt_enable(WDTO_500MS);
wdt_reset();

initialize_mcu(); 
/*Load the values stored in the eeprom like the throttle channel threshold etc. */
load_values_from_eeprom();  
//Load the ISR array. This way if a channel is not connected it will have the failsafe value.
load_failsafe_values(); 
/*
The "wait_for_rx(): function waits untill the receiver has been powered up and running
so we can then detect the connected channels with certainty.
*/
wait_for_rx(); 
channels_in_use = detect_connected_channels();
check_for_setup_mode();
servo_signals_lost = 1;   //Signal that the servo signals have gone missing(no rx signal).
led_frequency = RC_LED_FREQUENCY_VAL_4HZ;

/****************************  SETUP THE PIN INTERRUPT  *************************************************/
wdt_disable();
#if F_CPU <= 8000000UL
wdt_enable(WDTO_60MS);
#else
wdt_enable(WDTO_30MS);
#endif 

WDTCSR |= (1<<WDIE);
wdt_reset();

// Now we must disable the pin interrupt. 
RC_PIN_INT_EN_REG &= (~(1<<RC_PIN_INT_EN_BIT));
//Now we must load the pin interrupt mask register.
RC_PIN_INT_MASK_REG = channel_mask;
//Clear any pin interrupt flag set. 
RC_PIN_INT_FLAG_REG |= (1<<RC_PIN_INT_FLAG_BIT);
// Clear the pin interrupt ISR detection variable
pin_interrupt_detected = 0;
// Finally we can enable the pin interrupt again. 
RC_PIN_INT_EN_REG |= (1<<RC_PIN_INT_EN_BIT); 
// Take a snapshot of the servo pins in order to establish a starting point.

/*------------------------------------------------------------------------------------------------------*/
/*                               PPM OUTPUT AND LED INDICATION CONTROL                                  */
/*------------------------------------------------------------------------------------------------------*/
while(1)
    {

       // Here we decide if a re synchronization with the input is needed. 
       if(servo_signals_lost)
        {
           get_pw_of_connected_channels(1);

        }else{ 
                get_pw_of_connected_channels(0); 
             }

/*------------------------------------------------------------------------------------------------------*/
       // Finally we can control when the PPM generator will be on and off.
       led_counter++; 
       if( led_counter >= led_frequency ){ led_counter = 0; TOGGLE_LED(); } 
       if( channel_mask_buffer == 0  ) //IF ALL CHANNELS HAVE BEEN MEASURED...
        {
           tx_signal_lost = 0;         
           if(servo_signals_lost) //IF PREVIOUSLY THE SERVO SIGNAL WAS LOST...
            {
              tx_signal_detected++;
              if(tx_signal_detected > RC_MAX_BAD_PPM_FRAMES)
               {
                 ppm_on();
                 servo_signals_lost = 0;
                 led_counter = 0;
                 led_frequency = RC_LED_FREQUENCY_VAL_4HZ;
               }
            }
        }
       else{  //IF NOT ALL CHANNELS HAVE BEEN MEASURED...
              tx_signal_detected = 0;
              if(servo_signals_lost == 0)
               {
		   tx_signal_lost++;
                   if(tx_signal_lost > RC_MAX_BAD_PPM_FRAMES)
                    { 
                       servo_signals_lost = 1;
                       led_counter = 0;
                       led_frequency = RC_LED_FREQUENCY_VAL_1HZ;
#if defined(RC_USE_FAILSAFE) && RC_USE_FAILSAFE == 1
                       load_failsafe_values();
#else
                       ppm_off();
#endif
                    } 
               } //end of if(servo_signals_lost == 0) statement.
           } //end of if( x > (channels_in_use/2)  ){...}else{...} statement.
    } //end of while(1) loop.      
  
}
/********************************************************************************************************/
/*                                   INTERRUPT SERVICE ROUTINES                                         */
/********************************************************************************************************/
#if (__GNUC__ == 4 && __GNUC_MINOR__ == 2)
/*
I used this approach because AVR-gcc produces safe code which is far from optimized with a lot of overhead.
For GCC versions greater then 4.2.x global register variables get trashed during optimization
so i need to use normal ram variables instead.
The asm ISR code presented here needs the below variables to be exclusively allocated to predefined registers:

register volatile unsigned char  timer0_l8 asm("r2");
register volatile unsigned char  timer0_h8 asm("r3");
register volatile unsigned int   isr_time_stamp asm("r4");
register volatile unsigned char  pin_interrupt_detected asm("r6");
register volatile unsigned char  sreg_save asm("r7");
Also for clarity:
70-0x20 = TCNT0
59-0x20 = PCIFR and the fourth bit (3) is the PCIF2 flag.
53-0x20 = TIMER0 TIFR  and the first bit (0) is the OVF0 flag.
*/

ISR(TIMER0_OVF_vect, ISR_NAKED)
{
__asm__ __volatile__(
			"in	r7, __SREG__		\n\t"
			"inc	r3			\n\t"
			"sbis	59-0x20, 3		\n\t"    
			"rjmp	.no_external_pin_int	\n\t"  
	                "clr	r2			\n\t"
#if F_CPU <= 8000000UL
	                "inc	r2			\n\t"
#endif
			"movw	r4,r2			\n\t"
     			"mov	r6, r3			\n\t" 
        	        "sbi	59-0x20, 3		\n\t"   
		        ".no_external_pin_int:		\n\t"
			"out	__SREG__, r7		\n\t"
			"reti				\n\t"

                    ::); 
/*
// C EQUIVALENT CODE
sreg_save = SREG;
timer0_h8++;
if( (RC_PIN_INT_FLAG_REG & (1<<RC_PIN_INT_FLAG_BIT)) )
 {
    timer0_l8 = TCNT0;
    isr_time_stamp = (timer0_h8<<8) + timer0_l8;
    pin_interrupt_detected = 1;
    RC_PIN_INT_FLAG_REG = (1<<RC_PIN_INT_FLAG_BIT);
 }
SREG = sreg_save;
*/
}  

/********************************************************************************************************/

ISR(PCINT2_vect, ISR_NAKED)
{
__asm__ __volatile__(
	                "in	r2, 70-0x20		\n\t"	
                        "in	r7,  __SREG__		\n\t"
	                "sbis	53-0x20, 0	     	\n\t"
 	                "rjmp	.no_timer0_ovf		\n\t"
	                "clr	r2			\n\t"	
                        "inc	r3          		\n\t"
                        "sbi	53-0x20, 0   		\n\t"
                         ".no_timer0_ovf: 		\n\t"
                        "movw	r4,r2			\n\t"
                        "clr	r6          		\n\t"
                	"inc	r6			\n\t"
                	"out	__SREG__, r7		\n\t"
                 	"reti                 		\n\t" 
	          ::);
/*
// C EQUIVALENT CODE
timer0_l8 = TCNT0;
sreg_save = SREG;
if( (RC_TIMER0_TIFR & (1<<TOIE0)) )
 {
    timer0_l8 = TCNT0;
    timer0_h8++;
    RC_TIMER0_TIFR = (1<<TOIE0);
 }
isr_time_stamp = (timer0_h8<<8) + timer0_l8;
pin_interrupt_detected = 1;
SREG = sreg_save;
//RC_PIN_INT_FLAG_REG = (1<<RC_PIN_INT_FLAG_BIT);

*/

}

#else 

//For GCC versions greater then 4.2.x global register variables get trashed during optimization
//so i need to use normal ram variables instead. i don't use any register variables here.
ISR(TIMER0_OVF_vect, ISR_NAKED)
{
/*
In order to make the	ISR as fast and accurate it can be, i am forced to use more code but the execution is faster.
It would be nice to have an extra 16 bit timer but i don't, so i am forced to use tricks with an 8 bit timer.
*/ 
__asm__ __volatile__(
			"sbic	59-0x20, 3			\n\t"    
			"rjmp	.external_pin_int0		\n\t"  
			"push	r2				\n\t"
			"push	r3				\n\t"
			"in	r3, __SREG__			\n\t"
			"lds	r2, timer0_h8			\n\t"
			"inc	r2				\n\t"
     			"sts	timer0_h8, r2			\n\t"  
			"out	__SREG__, r3			\n\t"
			"pop	r3				\n\t"
			"pop	r2				\n\t"
			"sbis	59-0x20, 3			\n\t"    
			"rjmp	.no_external_pin_int		\n\t"
			"push	r24				\n\t"
			"lds	r24, timer0_h8			\n\t"
			"sts	(isr_time_stamp)+1, r24		\n\t"
         		"ldi	r24, 1				\n\t" 
			"sts	isr_time_stamp, r24		\n\t"
     			"sts	pin_interrupt_detected, r24	\n\t" 
        	        "sbi	59-0x20, 3			\n\t" 
			"pop	r24				\n\t"           
			".no_external_pin_int:			\n\t"
			"reti					\n\t"

		        ".external_pin_int0:			\n\t"
			"push	r2				\n\t"
			"push	r3				\n\t"
			"in	r3, __SREG__			\n\t"
			"lds	r2, timer0_h8			\n\t"
			"inc	r2				\n\t"
     			"sts	timer0_h8, r2			\n\t"
			"sts	(isr_time_stamp)+1, r2		\n\t"
         		"clr	r2				\n\t" 
			"sts	isr_time_stamp, r2		\n\t"
         		"inc	r2				\n\t" 
     			"sts	pin_interrupt_detected, r2	\n\t" 
        	        "sbi	59-0x20, 3			\n\t" 
			"out	__SREG__, r3			\n\t"
			"pop	r3				\n\t"
			"pop	r2				\n\t"
			"reti					\n\t"

                    ::); 

}

/*
ISR(TIMER0_OVF_vect)
{
timer0_h8++;
if( (RC_PIN_INT_FLAG_REG & (1<<RC_PIN_INT_FLAG_BIT)) )
 {
    isr_time_stamp = (timer0_h8<<8);
    pin_interrupt_detected = 1;
    RC_PIN_INT_FLAG_REG = (1<<RC_PIN_INT_FLAG_BIT);
 }

return;
}  
*/
/********************************************************************************************************/

ISR(PCINT2_vect, ISR_NAKED)
{
__asm__ __volatile__(
			"push	r24				\n\t"
	                "in	r24, 70-0x20			\n\t"	
			"push	r2				\n\t"
			"lds	r2, timer0_h8			\n\t"
	                "sbis	53-0x20, 0	     		\n\t"
 	                "rjmp	.no_timer0_ovf			\n\t"
			"push	r3				\n\t"
                        "in	r3,  __SREG__			\n\t"
			"inc	r2				\n\t"
     			"sts	timer0_h8, r2			\n\t"
			"sts	(isr_time_stamp)+1, r2		\n\t"
    			"sts	pin_interrupt_detected, r2	\n\t" 
			"clr	r2				\n\t" 
			"sts	isr_time_stamp, r2		\n\t"
                        "sbi	53-0x20, 0   			\n\t"
                	"out	__SREG__, r3			\n\t"
			"pop	r3				\n\t"
			"pop	r2				\n\t"
			"pop	r24				\n\t"
			"reti					\n\t"

                        ".no_timer0_ovf: 			\n\t"
			"sts	isr_time_stamp, r24		\n\t"
			"sts	(isr_time_stamp)+1, r2		\n\t"
         		"ldi	r24, 1				\n\t" 
     			"sts	pin_interrupt_detected, r24	\n\t"  
			"pop	r2				\n\t"
			"pop	r24				\n\t"
                 	"reti                 			\n\t" 
	          ::);

}
/*
ISR(PCINT2_vect)
{
timer0_l8 = TCNT0;
if( (RC_TIMER0_TIFR & (1<<TOIE0)) )
 {
    timer0_l8 = TCNT0;
    timer0_h8++;
    RC_TIMER0_TIFR = (1<<TOIE0);
 }
isr_time_stamp = (timer0_h8<<8) + timer0_l8;
pin_interrupt_detected = 1;

return;
}
*/

#endif

/********************************************************************************************************/

/*
By using the fast pwm mode 15 which uses the OCR1A value as TOP value and changing the values within the
OCR1B interrupt, timing is very accurate because the OCR1A and OCR1B registers are double buffered.
This means that although it looks like we modify both registers within the OCR1B interrupt we are actually
modifying their buffers and not the actual registers.
Both actual registers are updated when the timer reaches the OCR1A (TOP) value automatically.
This way the OCR1B interrupt can be delayed as needed without any loss of timing accuracy.
*/
ISR(TIMER1_COMPB_vect, ISR_NOBLOCK)
{
#if RC_CONSTANT_PPM_FRAME_TIME == 1
isr_channel_number++;
if( isr_channel_number >= (RC_PPM_GEN_CHANNELS + 1) ) {isr_channel_number = 0; reset_pw = RC_PPM_FRAME_TIMER_VAL; }

if(isr_channel_number < RC_PPM_GEN_CHANNELS)
 {
    RC_TIMER1_COMP1_REG = isr_channel_pw[isr_channel_number];
    reset_pw -= RC_TIMER1_COMP1_REG;

 }else{
             RC_TIMER1_COMP1_REG = reset_pw;
      } 

#elif RC_CONSTANT_PPM_FRAME_TIME == 0

isr_channel_number++;
if( isr_channel_number >= (RC_PPM_GEN_CHANNELS + 1) ) {isr_channel_number = 0; }
RC_TIMER1_COMP1_REG = isr_channel_pw[isr_channel_number];

#endif

return;
}

/********************************************************************************************************/

ISR(WDT_vect)
{                       
wdt_timeout = 1;
pin_interrupt_detected = 1;
channel_mask_buffer = 0;

return;
}  



/*######################################################################################################*/
/*                                         T H E   E N D                                                */
/*######################################################################################################*/

