#include "main.h"
#include "hal_LCD.h"

//MAX_CNT is 2^n - 1
#define FIFO_ADDR_MODED_ADD(CNT, MAX_CNT) ((++CNT) & (MAX_CNT))

#define MAX_RX_FIFO_CNT 0x0F
#define MAX_TX_FIFO_CNT 0x0F

volatile unsigned short RX_FIFO[16];
volatile unsigned short TX_FIFO[16];

volatile unsigned short RX_CLK_CNT = 0;
volatile unsigned short IS_UP_STROKE_FLAG = 0;

volatile unsigned short RX_DATA = 0x00;
volatile unsigned short RX_FIFO_WR_CNT = 0;
volatile unsigned short RX_FIFO_RD_CNT = 0;


volatile unsigned short TX_CONN_INIT_CNT = 0;
volatile unsigned short TX_CONN_INIT_WAIT_FLG = 0;
volatile unsigned short TX_CLK_WAIT_FLG = 0;
volatile unsigned short TX_DATA_TRANSMIT_FLG = 0;

volatile unsigned short DEBUG_CNT = 0;

// Backup Memory variables to track states through LPM3.5
// BAKMEM0 ~ BAKMEM15(16bit int)
// + _L _H (8bit char)
volatile unsigned char * rx_fifo_remain = &BAKMEM0_L;  // # of rx fifo in remain data.
volatile unsigned char * tx_fifo_remain = &BAKMEM0_H;  // # of tx fifo in remain data.
volatile unsigned char * mode = &BAKMEM1_L;  // mode flag ( 0:rx, 1:tx)

// mode list
#define RX_MODE 0
#define TX_MODE 1

// sleep mode
// --if rx_fifo is null, run timer until xxsec.
//      if rx_fifo is not null when timer is running, timer is clear & stop.
//      else enter LPM3.5(need Backup Memory)

int main(){
    unsigned short data; //process tx & rx fifo data

    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    //fifo clear
    *rx_fifo_remain = 0;
    *tx_fifo_remain = 0;

    Init_GPIO();
    Init_Timer();
    Init_RTC();
    Init_LCD();

    //interrupt clear
    P1IFG = P2IFG = 0; // remove start-up garbage interrupt

    __enable_interrupt();

    while(1)
    {

        LCD_E_selectDisplayMemory(LCD_E_BASE, LCD_E_DISPLAYSOURCE_MEMORY);

        switch(*mode){
            case RX_MODE:
                //fifo check
                while(*rx_fifo_remain)
                {
                    //pull fifo
                    data = RX_FIFO[RX_FIFO_RD_CNT];
                    RX_FIFO_RD_CNT = FIFO_ADDR_MODED_ADD(RX_FIFO_RD_CNT, MAX_RX_FIFO_CNT);

                    //print & output data
                    clearLCD();
                    showHex(data, 0);
                    showChar(*rx_fifo_remain + '0', pos6);

                    //loop until fifo is null
                    __delay_cycles(20000);
                    *rx_fifo_remain = (RX_FIFO_WR_CNT - RX_FIFO_RD_CNT) & 0xF;
                }
                break;
            case TX_MODE:
                //fifo check
                //push fifo
                break;
        }

        __low_power_mode_3();
        __no_operation();
    }
}


/*
 * GPIO Initialization
 */
void Init_GPIO()
{
    //Set all GPIO pins to output low to prevent floating input and reduce power consumption
    P1OUT = P2OUT = P3OUT = P4OUT = P5OUT = P6OUT = P7OUT = P8OUT = 0;
    P1DIR = P2DIR = P3DIR = P4DIR = P5DIR = P6DIR = P7DIR = P8DIR = 0xFF;
    P1DIR &= ~0x20;
    P2DIR &= ~0x80;


    //interrupt clear
    P1IFG = P2IFG = 0;
    P1IE  = P2IE  = 0;
    P1IES = P2IES = 0;

    // P1.0 DEBUG out(LED)
    // P1.2 SW1 in(RX_CLR) - pull-up
    // P1.5 CLK in(RX) - negedge
    // P1.7 TA01_OUT(PWM)
    P1IES |= 0x20;
    P1DIR &= ~0x24;
    P1REN |= 0x04;
    P1OUT |= 0x04;
    P1SEL0 |= 0x80;
    P1IFG = 0;
    P1IE |= 0x20;


    // P2.6 SW2 in() - pull-up
    // P2.7 DATA IN(RX)
    P2IES |= 0x40;
    P2DIR &= ~0xC0;
    P2REN |= 0x40;
    P2OUT |= 0x40;
    P2IFG = 0;
    //P2IE |= 0x40;

    // P4.0 DEBUG out(LED)
    //P4DIR |= 0x01;

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}
/*
 * Timer_A Initialization
 */
void Init_Timer()
{
    TA0CCR0 = 80; // 12.5khz
    TA0CCR1 = TA0CCR0 >> 1;  // toggle

    TA1CCR0 = 16383; // xxms

    /* TAxCCTLn
     * [15:14] capture mode
     *         0:no_cap 1:cap_rz 2:cap_fl 3:cap_rz_fl
     * [13:12] cap/comp input
     *         0:CCIxA 1:CCIxB 2:GND 3:VCC
     * [11]    sync cap source
     *         0:Async 1:Sync
     * [10]    Syncd cap/comp input
     *         0
     * [8]     cap mode
     *         0:comp 1:cap
     * [7:5]   output mode
     *         0:OUT 1:set 2:toggle/rst 3:set/rst
     *         4:toggle 5:rst 6:toggle/set 7:rst/set
     * [4]     cap/comp interrupt enable
     * [3]     cap/comp input(ro)
     * [2]     OUT(output data) - rw
     * [1]     cap overflow
     * [0]     cap/comp interrupt flag
     */
    TA0CCTL0 = 0x0000;
    TA0CCTL1 = 0x0000;

    TA1CCTL0 = 0x0010;

    /* TAxCTL
     * [9:8] clock_source
     *       0:TAxCLK 1:ACLK 2:SMCLK 3:INCLK
     * [7:6] input devider
     *       0:/1 1:/2 2:/4 3:/8
     * [5:4] mode control
     *       0:stop 1:up 2:continue 3:up/down
     * [2]   timer clear(wo)
     * [1]   interrupt enable
     * [0]   TA interrupt flag
     */
    TA0CTL = 0x0000;
    TA1CTL = 0x02D0;

}

/*
 * Real Time Clock counter Initialization
 */
void Init_RTC()
{
    //set 4.1, 4.2 for XT1 IN, OUT
    P4SEL0 |= 0x06;

    // Intializes the XT1 crystal oscillator
    do{
        CSCTL7 &= ~(XT1OFFG | DCOFFG);
        SFRIFG1 &= ~OFIFG;
    }while(SFRIFG1 & OFIFG);

    // Set RTC modulo to 327-1 to trigger interrupt every ~10 ms
    RTCMOD = 327 - 1;
    RTCCTL = 0x0040;    //RTC clear
    RTCCTL = 0x2000;
}


/*
//RX clk generator
#pragma vector=TIMER0_A0_VECTOR
__interrupt void ta00_isr(void){
    RX_DATA_TEMP = P1IN; // for minimal time after interrupt call
    if(++DEBUG_CNT == 10)
        DEBUG_CNT;
    if(RX_CLK_GEN_EN){
        if(++RX_CLK_GEN_CNT == 12){
            TA0CTL &= ~0x0030; //timer stop
            TA0CTL |= 0x0004; //timer clear
            TA0CCTL1 &= ~0x0004; //clk output reset(reset 0)
            RX_CLK_GEN_EN = 0;
            RX_CLK_GEN_CNT = 0;
            P1IE |= 0x20; // DATA - 1.5
            DEBUG_CNT = 0;
            P1OUT ^= 0x01; //debug - pin 1.0 LED
        }
        else
            //port1.5 is data input
            RX_DATA |= ((RX_DATA_TEMP & 0x20) && 1) << DATA_CNT;
    }
    TA0CCTL0 &= ~0x0001; //interrupt flag down
}
*/


// PS/2 protocol check timer
#pragma vector=TIMER1_A0_VECTOR
__interrupt void ta10_isr(void){
    if(*rx_fifo_remain == 0)
    {
        RX_CLK_CNT = 0;
        RX_DATA = 0x00;
    }
    TA1CCTL0 &= ~0x0001; //interrupt flag down
    /*
    // tx init clk-down 100us check
    if(TX_CONN_INIT_WAIT_FLG && ++TX_CONN_INIT_CNT == 1){ // 100us timer timeout
        //PxOUT |= 0xXX; //tx data out assert
        //PxOUT & ~0xXX; //tx clk  out reset
        TX_CONN_INIT_WAIT_FLG = 0;
        TX_CLK_WAIT_FLG = 1;
    }
    // tx clk(generated by rx) timeout(2ms) check
    else if (TX_CLK_WAIT_FLG && ++TX_CONN_INIT_CNT == 20){ // 2ms timer timeout
        TX_CLK_WAIT_FLG = 0;
        tx_done(-1);
    }
    // tx transmit timeout(15ms) check
    else if (TX_DATA_TRANSMIT_FLG && ++TX_CONN_INIT_CNT == 150){ //15ms timer timeout

        //at first, TX_CONN_INIT_CNT must be reset to 0 by transmit start step


        TX_DATA_TRANSMIT_FLG = 0;
        tx_done(-2);
    }
    P5OUT ^= 0x01; // P5.0 debug out
    TA1CCTL0 &= ~0x0001; //interrupt flag down
    */
}

/*
// TX port
#pragma vector=PORT2_VECTOR
__interrupt void p2_isr(void){
    switch(P2IV){
        //port1.7 - DATA_IN start(negedge)
        case 0x10:
            if(!(P2IN & 0x20)){ // CLK_in == 0
                P2IE &= ~0x80; // DATA - INT 2.7 disable
                RX_CLK_GEN_EN = 1;
                RX_CLK_CNT = 0;
                TA0CCTL0 &= ~0x0001; //interrupt flag down
                TA0CTL |= 0x0014; //clk_gen timer clear & on(up mode)
             }
            break;
        default:
            break;
    }
}
*/

// RX port
#pragma vector=PORT1_VECTOR
__interrupt void p1_isr(void){
    switch(P1IV){
        //port1.5 - RX_CLK(negedge)
        case 0x0C:
               if(RX_CLK_CNT++ == 10)
               {
                   RX_DATA &= 0xFF;
                   RX_FIFO[RX_FIFO_WR_CNT] = IS_UP_STROKE_FLAG ? RX_DATA | 0x8000 : RX_DATA;

                   if((IS_UP_STROKE_FLAG = (RX_DATA == 0xF0)) == 0)
                   {
                       RX_FIFO_WR_CNT = FIFO_ADDR_MODED_ADD(RX_FIFO_WR_CNT, MAX_RX_FIFO_CNT);
                       *rx_fifo_remain = FIFO_ADDR_MODED_ADD(*rx_fifo_remain, MAX_RX_FIFO_CNT);
                   }

                   RX_DATA = 0x00;
                   RX_CLK_CNT = 0;

                   ////__bic_SR_register_on_exit(LPM3_bits);
                   __low_power_mode_off_on_exit();
               }
               else
                   //port2.7 is data input(If device is not LaunchPad, I'll use port8.7.)
                   //this process(RX data read from PS/2 protocol) timing is critical.
                   //do not add additional process.
                   RX_DATA = (RX_DATA >> 1) | ((P2IN & 0x80) << 1);
        break;
    default:
        break;
    }
}
