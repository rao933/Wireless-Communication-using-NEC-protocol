// Periodic timer example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "tm4c123gh6pm.h"
#include <inttypes.h>
#include "wait.h"
#include "uart0.h"
//  #include "rgb_led.h"


// Pin bitbands
#define GREEN_LED           (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define RED_LED             (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))

#define BLUE_LED            (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

#define PWMPIN_MASK         16               // AS PWM taking pin
#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8


// uint8_t value;
//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
char str[80];
uint32_t time[10];
uint8_t gpio_count = 0;
uint8_t data_value;

uint8_t value1;
uint8_t ARGS1;
#define RECIEVE_BUFFER_LENGTH 16
#define Transmit_BUFFER_LENGTH 16

uint8_t end = 0;
uint8_t flag = 0;
uint8_t idata;
uint8_t iaddress;
uint8_t address1;
uint8_t data1;
uint8_t al_flag;
uint8_t count_buffer = 0;
uint8_t timer_count = 0;
uint8_t rx_flag=0;
uint8_t rx_count=0;
uint8_t dataInfo[16];
// uint8_t Recieve_Buffer[RECIEVE_BUFFER_LENGTH];   // lab 8
// uint8_t Transmit_Buffer[Transmit_BUFFER_LENGTH]; // this store the data to be sent (byte)

uint8_t zyx = 1;
uint8_t zy = 2;

uint8_t R_add, xx, yy;
uint8_t R_iadd;
uint8_t R_data;
uint8_t R_idata;
uint8_t RD_INDEX = 0;
uint8_t WR_INDEX = 0;
uint8_t RD_INDEX_T = 0;
uint8_t WR_INDEX_T = 0;
uint8_t see_count = 0;

uint8_t recieve_buffer_iteration = 0;
uint8_t transmit_buffer_iteration = 0;
bool recieve_flag = false;
bool ready = false;

bool process = false; // for the processing in the main

uint8_t second_flag = 0;

uint8_t  fifo_array[16][8]; // change into define later
uint8_t fifo_recieve_array[16];
uint8_t  fifo_iteration = 0;

uint8_t array_first_arg = 0;
uint8_t array_second_arg = 0;

uint8_t transmit_data_array[16];

uint8_t recieve_byte = 0;

uint8_t tx = 0;

uint8_t ARGS;
int32_t EEPROM_ADD0, EEPROM_ADD1, EEPROM_ADD2 ;// my own address
bool check_flag=false;
bool tx_flag=false;
uint8_t check_sum;
uint8_t R_ON;

bool ackBit;

uint8_t nine = 0, status[2];

uint8_t  dup, dup1, dup2, dup3, dup5 = 0, dup4 = 0, flaggg = 0, galf = 1;
int8_t dup6;
uint8_t summed = 0;
//-----------------------------------------------------------------------------
// BITBANDS AND MASKS
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
//#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

//-----------------------------------------------------------------------------
//                              prototypes

void process_data();

void recieve_key(void);
void sendKey(uint8_t address, uint8_t data);
//-----------------------------------------------------------------------------

#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
//-----------------------------------------------------------------------------
#define MAX_CHARS 80
#define MAX_FIELDS 17
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

uint8_t cmd_Set   =          0x00;
uint8_t cmd_RGB   =          0x01;
uint8_t cmd_get   =          0x10;
uint8_t cmd_Get_Response   = 0x11;
uint8_t cmd_Poll           = 0x20;
uint8_t cmd_Ack =            0x30;
#define DATA_SIZE          10
// Packet information
#define LENGTH_QUEUE        7
//-----------------------------------------------------------------------------
typedef struct struct_packet
{
    uint8_t destAdd;
    uint8_t srcAdd;
    uint8_t command;
    uint8_t checksum;
    uint8_t data_info[DATA_SIZE];
    uint8_t args;
    bool    ack;
} packet;
packet queue[LENGTH_QUEUE];







//-----------------------------------------------------------------------------
// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R5|SYSCTL_RCGCGPIO_R3;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;                       // enable PWM clock M1PWM1 MODULE 1
    SYSCTL_RCGCTIMER_R|=SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCTIMER_R|=SYSCTL_RCGCTIMER_R3;
    _delay_cycles(3);

    GPIO_PORTF_DEN_R |= 8|4;
    GPIO_PORTF_DIR_R |= 8|4;
    // PWM
    GPIO_PORTE_DEN_R    |= PWMPIN_MASK;
    GPIO_PORTE_AFSEL_R  |= PWMPIN_MASK;
    GPIO_PORTE_DR8R_R |= PWMPIN_MASK;
    GPIO_PORTE_PCTL_R   &= ~GPIO_PCTL_PE4_M;
    GPIO_PORTE_PCTL_R   |= GPIO_PCTL_PE4_M0PWM4;
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                             // reset PWM1 module
    SYSCTL_SRPWM_R = 0;
    PWM0_2_CTL_R = 0;                                             // TURN OFF
    PWM0_2_GENA_R |= 0xC0 | 0x8;                                  // when counter = cmp while counting down
    PWM0_2_LOAD_R = 1052;                                         // LOAD VALUE WHICH IS DECREMENTING
    PWM0_2_CMPA_R = 526;                                          // GENERATOR 2, CMPA VALUE
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                              // TURN ON
    PWM0_ENABLE_R = PWM_ENABLE_PWM4EN;
    GPIO_PORTE_DEN_R &= ~PWMPIN_MASK;                             // DEN OFF
    // TIMER2
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                              // disable the timer
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;                        // Configure the timer
    TIMER2_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT;                      // Configure one shot for 9 ms TIMER_TAMR_TAMR_1_SHOT
    TIMER2_TAILR_R = 360000;                                      // for 9 ms timer
    TIMER2_TAMR_R &= ~16;
    TIMER2_IMR_R |= TIMER_IMR_TATOIM;
    NVIC_EN0_R |= 1 << (INT_TIMER2A - 16);                        // turn on timer interrupt

    // Timer 3 for one shot and stopping the burst after recieving it
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                              // disable the timer

    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;                        // Configure the timer
    TIMER3_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT;                      // Configure one shot for 5T
    TIMER3_TAILR_R = 101550;                                      // for 4T + some value
    TIMER3_TAMR_R &= ~16;
    TIMER3_IMR_R |= TIMER_IMR_TATOIM;
    NVIC_EN1_R |= 1 << 3;                        // turn on timer interrupt // wrap arounf nvic_en1


// count down

    // interrupt when hit zero timer_isr3

}
void initRgb()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;

    _delay_cycles(3);

    // Configure three LEDs
    GPIO_PORTF_DIR_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_AFSEL_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    //PWM1_1_CTL_R = 0;

    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 7 on PWM1, gen 3b, cmpb
    //PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;


    PWM1_2_LOAD_R = 256;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 256;
    //PWM1_1_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz


    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off
    //PWM1_1_CMPA_R = 0;                               // Output led off


    //-------------------------------------------------------------
    // Configure PWM module 1 to drive RGB LED
    // OUTPUT   on M1PWM2 (PA6), M1PWM2a
    //SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    //SYSCTL_SRPWM_R = 0;                              // leave reset state
    //PWM1_1_CTL_R = 0;                                // turn-off PWM1 generator 1 (drives out 2)
    //PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 2 on PWM1, gen 1a, cmpa
    //PWM1_1_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

    //PWM1_1_CMPA_R = 0;                               // Output led off
    //PWM1_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 1
    //PWM1_ENABLE_R |= PWM_ENABLE_PWM2EN;
                                                     // enable outputs
    //-------------------------------------------------------------

    PWM1_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 3
   // PWM1_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 1

    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
}

void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
    PWM1_3_CMPB_R = green;
}
void setRgbColor_one(uint16_t red)
{
    PWM1_2_CMPB_R = red;

}
void setRgbColor_blue(uint16_t blue)
{
    PWM1_3_CMPA_R = blue;
}
void setRgbColor_green(uint16_t green)
{
    PWM1_3_CMPB_R = green;
}


void enabletimer()
{

    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                              // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                                             // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR;         // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG;                         // measure time from positive edge to positive edge
    WTIMER1_TAV_R = 0;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                               // enable it

}
void GPIPIN()
{
    GPIO_PORTE_DIR_R &= ~4;
    GPIO_PORTE_DEN_R |= 4;                                          // digital enable the PA1
    GPIO_PORTE_PUR_R |= 4;
    GPIO_PORTE_IS_R &= ~0x4;                                        // edge triggered
    GPIO_PORTE_IEV_R &= ~0x4;                                       // pin working as input
    GPIO_PORTE_IM_R |= 4;


    //TIMER1_CTL_R |= 1;
    NVIC_EN0_R |= 1 << (INT_GPIOE - 16);                            // enable interrupt in nvic
}

// -------------------- lab 5 ------------------------

getsUart0(USER_DATA *data) // LAB 4 FUNCTION
{


    // As losh said pressing backspace as first key is not allowed, here if user presses,
    //it exits the function and starts again, so to the user it appears that backspace is not doing anything, which is
    // kinda expected from the program



    uint8_t count = 0;
    label: ;
    char c = getcUart0();
  //  putcUart0(c);
    if ((c == 127 | 8) & (count > 0)) // backspace and count in acceptable range
        {
            count--;
            goto label;
        }
    else if (c == 13) // return key
        {
            data->buffer[count] = '\0';
            goto exit;
        }
    else if ((c > 31 & c < 127) & (count < MAX_CHARS)) // if buffer not full and printable char
    {
        data->buffer[count] = c;

        putcUart0(c); // just to see the on terminal
        count++;
        goto label;

    }
    else
    {
        data->buffer[count] = '\0'; // this works only in the case when count => max_chars or
        goto exit;
    }
    exit: ;
}

void parsefields(USER_DATA *parse)
{
    uint8_t j = 0;

        uint8_t alpha = 0;  // it will help in if statement to not go again in the if statement
        uint8_t numeric = 0;  // it will help in if statement to not go again in the if statement
        uint8_t i=0;
        parse->fieldCount = 0;
        while(parse->buffer[i]!='\0')
        {
        if ((((parse->buffer[i] > 96) & (parse->buffer[i] < 123)) | ((parse->buffer[i] > 64) & (parse->buffer[i] < 91))) & (alpha == 0)) // if alpha get in
        {
        //putcUart0('a');
        parse->fieldPosition[j] = i; // fill the position array
        parse->fieldType[j] = 'a'; // fill the type array
        parse->fieldCount++;
        j = j + 1;;

        alpha = 1; // set

        }
        else if ((parse->buffer[i]>47  & parse->buffer[i]<58) & (numeric == 0)) // if numeric get in
        {
    //    putsUart0("\n\r");
        //putcUart0('n');
        numeric = 1; //  set
        parse->fieldPosition[j] = i;
        parse->fieldCount++;
        parse->fieldType[j] = 'n';
        j++;

        }
        if(((parse->buffer[i]>=32 & parse->buffer[i]<=47) | (parse->buffer[i]>=58 & parse->buffer[i]<=63) | (parse->buffer[i]>=91 & parse->buffer[i]<=96)) & (numeric==1 | alpha==1)) // thinking of a better way to write this
        {
            parse->buffer[i] = '\0';
            numeric = 0; // reset - very important
            alpha = 0; // reset - very important

        }
        i++;
        }

}
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)  // getting the field position
{
    int32_t help = 0;
    int32_t val = 0;
    char *ptr;
   // uint8_t i = 0;
    if (fieldNumber < 17) // only 5 are allowed
    {
        ptr = &data->buffer[data->fieldPosition[fieldNumber]];
        while (ptr[help] != '\0')
        {
            val = ((val * 10) + (*(ptr + help) - '0'));
            help++;
        }
        return val;
    }
    else
        return -9999;

}
char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    return &(data->buffer[data->fieldPosition[fieldNumber]]); // return the adress of the start of selected field

}
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    uint8_t a = data->fieldPosition[0]; // set or any command entered position
    uint8_t b = 0;
    if ((data->fieldCount) >= minArguments)
    {
    bool check;
    while(strCommand[a] != '\0')
    {
        if (strCommand[b] == data->buffer[b])
        {
            check = 1;
        }
        else
        {
            check = 0;
            break;
        }
      a++;
      b++;
     }

    if (check == 1)
    {
        return true;
    }

    else
    {
    return false;
    }
    }
    return false;
}
bool custom_strcmp(char *source, char *destination)
{
    bool Verify;
    Verify = 1;
    uint8_t i = 0;
    while(source[i] != '\0')
    {
    if (source[i] != destination[i])
    {
        Verify = 0;
        break;
    }
    i++;
}
    return Verify;
}
// -----------------------------lab 5 ends-----------------------------------------
void recieve_buffer(uint8_t input)
{

    dup4++;
    fifo_recieve_array[WR_INDEX] = input;
    WR_INDEX++;
    if (WR_INDEX == 16)
        WR_INDEX = 0;

}
void initEEprom()
{
    SYSCTL_RCGCEEPROM_R=1;
    _delay_cycles(3);
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}
void writeEeprom(uint8_t eadd,uint8_t edata)
{
    EEPROM_EEBLOCK_R=eadd>>4;
    EEPROM_EEOFFSET_R=eadd&0xf;
    EEPROM_EERDWR_R=edata;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}
uint32_t readEeprom(uint8_t eadd)
{
    EEPROM_EEBLOCK_R = eadd>>4;
    EEPROM_EEOFFSET_R = eadd & 0xf;
    return EEPROM_EERDWR_R;
}

void transmit_buffer_funtion(char* data)
{

    // 2D ARRAY//

        uint8_t j;
        uint8_t k = 0;

        while(data[k] != '\0')
        {
            for (j = 0; j < 8; j++)
            {
                fifo_array[fifo_iteration][j] = data[k] & 1;
                data[k] = data[k] >> 1;
            }
            WR_INDEX_T++;
            fifo_iteration++;
            if(fifo_iteration == 16){fifo_iteration = 0;}
            if(WR_INDEX_T == 16){WR_INDEX_T = 0;}
            k++;
        }

    // 2D ARRAY //


            sendKey(0, 0); // prime the bump

}
void transmit_buffer_funtion_2()
{

    // 2D ARRAY//

        uint8_t j;
        uint8_t k = 0, m = 0;

        {
            for (j = 0; j < 8; j++)
            {
                fifo_array[fifo_iteration][j] = queue[k].destAdd & 1;
                queue[k].destAdd = queue[k].destAdd >> 1;
            }
            WR_INDEX_T++;
            fifo_iteration++;
            if(fifo_iteration == 16){fifo_iteration = 0;}
            if(WR_INDEX_T == 16){WR_INDEX_T = 0;}
            // k++;

            for (j = 0; j < 8; j++)
            {
                fifo_array[fifo_iteration][j] = queue[k].srcAdd & 1;
                queue[k].srcAdd = queue[k].srcAdd >> 1;
            }
            WR_INDEX_T++;
            fifo_iteration++;
            if(fifo_iteration == 16){fifo_iteration = 0;}
            if(WR_INDEX_T == 16){WR_INDEX_T = 0;}
            // k++;

            for (j = 0; j < 8; j++)
            {
                fifo_array[fifo_iteration][j] = queue[k].command & 1;
                queue[k].command = queue[k].command >> 1;
            }
            WR_INDEX_T++;
            fifo_iteration++;
            if(fifo_iteration == 16){fifo_iteration = 0;}
            if(WR_INDEX_T == 16){WR_INDEX_T = 0;}
            // k++;

            while (ARGS > 0)
            {
                for (j = 0; j < 8; j++)
                {
                    fifo_array[fifo_iteration][j] = queue[k].data_info[m] & 1;
                    queue[k].data_info[m] = queue[k].data_info[m] >> 1;
                }
                WR_INDEX_T++;
                fifo_iteration++;
                if(fifo_iteration == 16){fifo_iteration = 0;}
                if(WR_INDEX_T == 16){WR_INDEX_T = 0;}
                m++;
                ARGS--;
            }
            for (j = 0; j < 8; j++)
            {
                fifo_array[fifo_iteration][j] = queue[k].checksum & 1;
                queue[k].checksum = queue[k].checksum >> 1;
            }
            WR_INDEX_T++;
            fifo_iteration++;
            if(fifo_iteration == 16){fifo_iteration = 0;}
            if(WR_INDEX_T == 16){WR_INDEX_T = 0;}

            //k++;
        }


    // 2D ARRAY //

        {
            sendKey(0, 0); // prime the bump
        }





}

void gpioIsr(void)
{
    uint8_t xcv = (uint8_t)readEeprom(0);
    uint8_t xyz = ((uint8_t)readEeprom(0));

    yy = xyz+zyx;

     xx = zy + xyz;
    uint8_t x, value1 = 1;
    time[gpio_count] = WTIMER1_TAV_R;

    GPIO_PORTE_ICR_R |= 4; // clear

  //  TIMER3_CTL_R &= ~TIMER_CTL_TAEN;
  //  TIMER3_TAILR_R = 360000;
  //  TIMER3_CTL_R |= TIMER_CTL_TAEN;

    if (gpio_count == 0)
    {
        WTIMER1_TAV_R = 0;                                          // reset counter for next period
        time[gpio_count] = WTIMER1_TAV_R;
        gpio_count++;
    }
    else if (gpio_count == 1)
    {

        if (((time[1] - time[0]) > 520000) && ((time[1] - time[0]) < 560000))
        {
            gpio_count++;
            GREEN_LED = 1;
        }
        else
        {
            gpio_count = 0;
        }
    }
    else if (gpio_count > 1 && gpio_count < 9)
    {

        TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                              // disable the timer 3

       if (((time[gpio_count] - time[gpio_count - 1]) > 33720 && (time[gpio_count] - time[gpio_count - 1]) < 56200) || (time[gpio_count] - time[gpio_count - 1] ) > 78680 && (time[gpio_count] - time[gpio_count - 1] < 101160))
       {
           gpio_count++;
           GREEN_LED = 0;
       }
       else
       {
           gpio_count = 0;
       }
    }
    else if (gpio_count == 9)
    {
        gpio_count = 2;

        TIMER3_TAV_R = 101550;
        TIMER3_CTL_R |= TIMER_CTL_TAEN;

        for (x = 1; x < 9; x++)// it gives binary to decimal for any byte that comes
        {
            if ((time[x + 1] - time[x]) > 78680 && (time[x + 1] - time[x]) < 101160)
            {
                value1 = value1 << (x - 1);
                recieve_byte = value1 + recieve_byte;
                value1 = 1;

            }
        }

        if (galf)
            {
        if ((recieve_byte == xcv) || (recieve_byte == xcv + 1) || (recieve_byte == xcv + 2) || (flaggg == 1) || (recieve_byte == 255))
        {
        recieve_buffer(recieve_byte); // we recieve regardless
        rx_count++;
        flaggg = 1;
        galf = 1;
        }
        else
        {
            flaggg = 0;
            galf = 0;
        }
            }
        recieve_byte = 0;
        WTIMER1_TAV_R = 540000;
        time[1] = WTIMER1_TAV_R;


    }

}

void process_data()
{
    process = 0;
    summed = 0;
    uint8_t xyzz;
    gpio_count = 0;

    if ((fifo_recieve_array[RD_INDEX] == (uint8_t)readEeprom(0)) || (fifo_recieve_array[RD_INDEX] == 255) || (fifo_recieve_array[RD_INDEX] == yy) || (fifo_recieve_array[RD_INDEX] == xx))
    {


      // dup4 = RD_INDEX;
      for (xyzz = 0; xyzz < dup4 - 1; xyzz++)
      {
         summed = summed + fifo_recieve_array[dup5++];
         if(dup5 == 16){dup5 = 0;}
         if(dup5 == 17){dup5 = 1;}
         if(dup5 == 18){dup5 = 2;}
         if(dup5 == 19){dup5 = 3;}
         if(dup5 == 20){dup5 = 4;}
         if(dup5 == 21){dup5 = 5;}
         if(dup5 == 22){dup5 = 6;}
         if(dup5 == 23){dup5 = 7;}

      }
      dup4 = 0; // no issue
     summed = ~ summed;
     rx_count = 0;
      dup5++;
      dup6 = WR_INDEX - 1;
      if(dup6 == -1){dup6 = 15;}
      if (summed == fifo_recieve_array[dup6])
              {
   // dontopen();

    dup = RD_INDEX + 2;
    dup1 = RD_INDEX + 3;
    dup2 = RD_INDEX + 4;
    dup3 = RD_INDEX + 5;


   if(RD_INDEX == 13){dup1 = 0;}
   if(RD_INDEX == 14){dup1 = 1;}
   if(RD_INDEX == 15){dup1 = 2;}
   //
   if(RD_INDEX == 12){dup2 = 0;}
   if(RD_INDEX == 13){dup2 = 1;}
   if(RD_INDEX == 14){dup2 = 2;}
   if(RD_INDEX == 15){dup2 = 3;}
   //
   if(RD_INDEX == 11){dup3 = 0;}
   if(RD_INDEX == 12){dup3 = 1;}
   if(RD_INDEX == 13){dup3 = 2;}
   if(RD_INDEX == 14){dup3 = 3;}
   if(RD_INDEX == 15){dup3 = 4;}
   //
  // if(RD_INDEX == 14){dup2 = 3;}
   if(dup5 == 16){dup5 = 0;}
   if(dup5 == 17){dup5 = 1;}
   if(dup5 == 18){dup5 = 2;}
   if(dup5 == 19){dup5 = 3;}
   if(dup5 == 20){dup5 = 4;}
   if(dup5 == 21){dup5 = 5;}
   if(dup5 == 22){dup5 = 6;}
   if(dup5 == 23){dup5 = 7;}
   //




   if (dup == 16)
   {
       dup = 0;
   }
   if (dup == 17)
   {
       dup = 1;
   }




        if((fifo_recieve_array[dup] == cmd_RGB) || (fifo_recieve_array[dup] == (cmd_RGB + 128 ))) // RGB
         {

             setRgbColor(fifo_recieve_array[dup1], fifo_recieve_array[dup2], fifo_recieve_array[dup3]);


             if (fifo_recieve_array[dup] == (cmd_RGB +128))
             {
                 send_packet(fifo_recieve_array[RD_INDEX + 1], fifo_recieve_array[dup], cmd_Ack , 0 , status , 0);
                 transmit_buffer_funtion_2();
             }



             //print_recieve_buffer();
             RD_INDEX = WR_INDEX;

         }


         else if(fifo_recieve_array[dup] == cmd_Set || (fifo_recieve_array[dup] == (cmd_Set + 128))) // SET
         {
             if(fifo_recieve_array[RD_INDEX] == (yy)){
             setRgbColor_blue(fifo_recieve_array[dup1]);}
             if(fifo_recieve_array[RD_INDEX] == (xx)){
                 setRgbColor_green(fifo_recieve_array[dup1]);
             }
             if(fifo_recieve_array[RD_INDEX] == (((uint8_t)readEeprom(0)) + 0)){
             setRgbColor_one(fifo_recieve_array[dup1]);
             }
             if (fifo_recieve_array[dup] == (cmd_Set + 128))
             {
              send_packet(fifo_recieve_array[RD_INDEX + 1], fifo_recieve_array[dup], cmd_Ack , 0 , status , 0);
              transmit_buffer_funtion_2();
             }



             //print_recieve_buffer();
             RD_INDEX = WR_INDEX;

         }

         else if((fifo_recieve_array[dup] == cmd_get) || (fifo_recieve_array[dup] == (cmd_get + 128))) // GET
         {
             putsUart0("Recieved Get Response from ");
           //  putcUart0((char) fifo_recieve_array[RD_INDEX + 1]);
             char strg[5];
             snprintf(strg, sizeof(strg), "Data:               %7"PRIu32"   \t", (char) fifo_recieve_array[RD_INDEX + 1]);
             putsUart0(strg);

             putsUart0("\n\n\r");

             waitMicrosecond(1000000);

             // status[0] = 77; // status of your push button
             send_packet(fifo_recieve_array[RD_INDEX + 1], fifo_recieve_array[RD_INDEX], cmd_Get_Response , 0 , status , 1);
             ARGS = 1;
             transmit_buffer_funtion_2();

             // print_recieve_buffer();
             putsUart0("Response Sent\n\n\r");
             RD_INDEX = WR_INDEX;
         }

         else if((fifo_recieve_array[dup] == cmd_Get_Response) || (fifo_recieve_array[dup] == (cmd_Get_Response + 128))) // GET RESPONSE
          {

             putsUart0("Response Sent ");
             putsUart0("\n\n\r");

          print_recieve_buffer();

          }

         else if((fifo_recieve_array[dup] == cmd_Poll) || (fifo_recieve_array[dup] == (cmd_Poll + 128))) // POLL
          {

          putsUart0("Recieved Poll from  ");
          char strg[5];
          snprintf(strg, sizeof(strg), "Data:               %7"PRIu32"            \t", (char) fifo_recieve_array[RD_INDEX + 1]);
          putsUart0(strg);
          putsUart0("\n\n\r");

          waitMicrosecond(1000000);
          send_packet(fifo_recieve_array[RD_INDEX + 1], (uint8_t)readEeprom(0) , cmd_Ack, 0 , status , 0);
          transmit_buffer_funtion_2();
          putsUart0("Response Sent\n\n\r");
          //print_recieve_buffer();
          RD_INDEX = WR_INDEX;
          }

         else if((fifo_recieve_array[dup] == cmd_Ack) || (fifo_recieve_array[dup] == (cmd_Ack + 128))) // ACK
          {
          print_recieve_buffer();
          }

        /*
                  RD_INDEX = 0;
                  WR_INDEX = 0;
                  dup5 = 0;
                  uint8_t flush = 0;
                  for (flush = 0; flush < 16; flush++)
                  {
                      fifo_recieve_array[flush] = '\0';
                  }
        */
        }
      else
      {
          RD_INDEX = 0;
          WR_INDEX = 0;
          dup5 = 0;
          uint8_t flush = 0;
          for (flush = 0; flush < 16; flush++)
          {
              fifo_recieve_array[flush] = '\0';
          }
      }
        }
    else
    {
        RD_INDEX = 0;
        WR_INDEX = 0;
        dup5 = 0;
        uint8_t flush = 0;
        for (flush = 0; flush < 16; flush++)
        {
            fifo_recieve_array[flush] = '\0';
        }
    }

}
void TIMER3_ISR()
{
    gpio_count = 0;
    TIMER3_ICR_R |= TIMER_ICR_TATOCINT;
    process = 1;
    flaggg = 0;
    galf = 1;
}

void TIMER2_ISR()
{

        if (timer_count == 55)
        {
            GPIO_PORTE_DEN_R &= ~PWMPIN_MASK;
            timer_count = 0;
            array_second_arg = 0;
            al_flag = 99;
            flag = 0;
            end = 0;
            TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

        }
        else  if(flag == 0)                                  // this whole is for creating the burst
        {
            GPIO_PORTE_DEN_R &= ~PWMPIN_MASK;
            TIMER2_TAILR_R = 180000;                          // 4.5 ms delay
            TIMER2_CTL_R |= TIMER_CTL_TAEN;                   // Timer starts
            al_flag = 1;
            flag = 99;
        }
        else if(al_flag == 1)
       {
            see_count++;
            GPIO_PORTE_DEN_R |= PWMPIN_MASK;
            TIMER2_TAILR_R = 22500; // always 562 us
            TIMER2_CTL_R |= TIMER_CTL_TAEN;  // Timer starts
            al_flag = 99;
            flag = 2;
            // hoice_state = true;
       }
        else if(flag == 2)
        {
            if (array_second_arg == 8)
            {
                array_second_arg = 0;
                RD_INDEX_T++;
                array_first_arg++;
                if(RD_INDEX_T == 16){RD_INDEX_T = 0;}
            }
            if(RD_INDEX_T == WR_INDEX_T)
            {
                end = 1;
                GPIO_PORTE_DEN_R &= ~PWMPIN_MASK;

                array_second_arg = 0;
                al_flag = 99;
                flag = 0;

                TIMER2_CTL_R &= ~TIMER_CTL_TAEN;

            }
          //  if(RD_INDEX_T == 16){RD_INDEX_T = 0;} // REDUNDANT

            if (array_first_arg == 16){array_first_arg = 0;}

            if((fifo_array[array_first_arg][array_second_arg] == 0))
           {
                if(end == 0){
               GPIO_PORTE_DEN_R &= ~PWMPIN_MASK; // PWM STARTS FOR 9 ms
               TIMER2_TAILR_R = 22500;
               TIMER2_CTL_R |= TIMER_CTL_TAEN;  // Timer starts
               al_flag = 1;
               array_second_arg++;}
           }
           else
           {
               if (end == 0)
               {
               GPIO_PORTE_DEN_R &= ~PWMPIN_MASK;
               TIMER2_TAILR_R = (22500 * 3);
               TIMER2_CTL_R |= TIMER_CTL_TAEN;  // Timer starts
               al_flag = 1;
               array_second_arg++;
               }
           }
        }
        end = 0;

        BLUE_LED = 0;

       TIMER2_ICR_R |= TIMER_ICR_TATOCINT;

}
void sendKey(uint8_t address, uint8_t data)
{
   /* address1 = address;
    data1 = data;
    iaddress = ~address;
    idata = ~data;
    uint8_t w = 0, x = 0, y = 0, z = 0, l = 1; // for loop variables
    count_buffer = 0;


  */

   // transbuffer[count_buffer] = 1;


    BLUE_LED = 1;
    GPIO_PORTE_DEN_R |= PWMPIN_MASK;    // PWM STARTS FOR 9 ms
    TIMER2_TAILR_R = 360000;            // 9 ms
    TIMER2_CTL_R |= TIMER_CTL_TAEN;     // Timer starts

}


void print_recieve_buffer()
{
    char str[50];
    while(RD_INDEX != WR_INDEX)
    {
       snprintf(str, sizeof(str), "Data:               %7"PRIu32"            \t\n\r", fifo_recieve_array[RD_INDEX]);
       putsUart0(str);
       RD_INDEX++;
       if (RD_INDEX == 16) {
           RD_INDEX = 0;

       }
    }

}

//----------------------------------------

void send_packet(uint8_t dest, uint8_t src_add, uint8_t cmd, bool ack, uint8_t datainfo[DATA_SIZE], uint8_t args)
{

    uint8_t dt;
    uint8_t sum=0;



    queue[tx].destAdd = dest;
    queue[tx].srcAdd = src_add;
    queue[tx].command = cmd;
    queue[tx].args = args;


    if(ackBit == true)
    {
        queue[tx].command = cmd|(1<<7);
    }
    else
    {
        queue[tx].command = cmd;

    }

    for(dt=0;dt<args;dt++)
    {
        queue[tx].data_info[dt]=datainfo[dt];
        sum=sum+datainfo[dt];
    }
    if (ackBit){
    cmd = cmd | (1<<7);
    queue[tx].checksum = ~ (dest + src_add + cmd + sum);
    check_sum=queue[tx].checksum;
    }
    if (!ackBit)
    {
        queue[tx].checksum = ~ (dest+src_add+cmd+sum);
        check_sum=queue[tx].checksum;
    }
    // tx++;
   // sendKey(0, 0);

}

//--------------------------



//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{


    initEEprom();
    uint8_t dest_address;


 //   char strr[100];
    USER_DATA data;
    uint8_t i, j;
   // char str[100];
   // uint8_t x;

    initHw();
    initRgb();
    GPIPIN();
    enabletimer();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    putsUart0("\t\t\t\t CSE 5400 Project \n\r\r");
    putsUart0("First Set the source address \n\r\r");
    putsUart0("If an Ack is request the (CMD_X_VAL or CMD_X_VAL+ 128) will be sent back \n\n\r\r");

while(1)
{

if( kbhitUart0() )

{
           getsUart0(&data);
           putsUart0("\n\r");
           parsefields(&data);
           putsUart0("\n\r");


          for (i = 0; i < data.fieldCount; i++)
          {
             putsUart0(&data.buffer[data.fieldPosition[i]]);
             putcUart0('\t');
             putcUart0(data.fieldType[i]);
             //putsUart0('\n\r');
             putsUart0("\n\r");

          }

          if (isCommand(&data, "recieve", 1))
          {
                      print_recieve_buffer();
          }

          if (isCommand(&data, "transmit", 1))
          {
              tx_flag=true;
              if (data.fieldType[1] == 'a')
              {
                  char* str = getFieldString(&data, 1);
                  if (custom_strcmp(str, str))
                  {
                  transmit_buffer_funtion(str);
                  }
              }
              else

              for (j = 1; j <= data.fieldCount - 1; j++)
              {
                  transmit_data_array[j - 1]  = ((getFieldInteger(&data, j)));
              }
              transmit_buffer_funtion((char*) transmit_data_array);
             // setRgbColor(0,0,0);
          }
          if(isCommand(&data,"rgb", 4))
            {
                dest_address=getFieldInteger(&data,1);

                ARGS = 3;

                dataInfo[0]=getFieldInteger(&data,2);
                dataInfo[1]=getFieldInteger(&data,3);
                dataInfo[2]=getFieldInteger(&data,4);

                send_packet(dest_address, (uint8_t) readEeprom(0), cmd_RGB , ackBit , dataInfo , ARGS);
              //  tx_flag=false;// write this in every IS command except from transmit command
                transmit_buffer_funtion_2();
               // rgb_on();
            }
          if(isCommand(&data,"set", 2))
         {
             dest_address=getFieldInteger(&data,1);
             ARGS = 1;

             dataInfo[0]=getFieldInteger(&data,2);
             send_packet(dest_address, (uint8_t) readEeprom(0), cmd_Set , ackBit , dataInfo , ARGS);
           //  tx_flag=false;// write this in every IS command except from transmit command
             transmit_buffer_funtion_2();
            // rgb_on();
         }
          if(isCommand(&data,"get", 1))
          {
              dest_address=getFieldInteger(&data,1);
              ARGS = 0;
              send_packet(dest_address, (uint8_t) readEeprom(0), cmd_get , ackBit , dataInfo , ARGS);
              transmit_buffer_funtion_2();
          }
          if(isCommand(&data,"poll", 0))
        {
              dest_address = 255;
              ARGS = 0;
              send_packet(dest_address, (uint8_t) readEeprom(0), cmd_Poll , ackBit , dataInfo , ARGS);
              transmit_buffer_funtion_2();
        }
          if(isCommand(&data,"address", 1))
          {
              writeEeprom(0, (uint8_t) getFieldInteger(&data,1));

          }
          if(isCommand(&data, "ack", 1))
          {
              char* str9 = getFieldString(&data, 1);
              if (custom_strcmp(str9, "ON"))
              {
                  ackBit = 1;
              }
              if (custom_strcmp(str9, "OFF"))
              {
                  ackBit = 0;
              }

          }
}

         if (process)
         {
              process_data();
         }


    }








}
