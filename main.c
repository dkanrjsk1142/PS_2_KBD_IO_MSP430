#include <msp430.h>
#include <intrinsics.h>

#define CCR1INTERVAL 32768
#define CCR2INTERVAL 8192

volatile unsigned short DATA_CNT=0xFFFE;
volatile unsigned short DATA_RECV_DONE_FLG = 1;

volatile unsigned short RECV_CLK_EN = 0;
volatile unsigned short RECV_CLK_CNT = 0;
volatile unsigned short RECV_DATA = 0x00;
volatile unsigned short RECV_DATA_READ_DONE_FLG = 0;


volatile unsigned short DEBUG_CNT = 0;

//clk generator timer
#pragma vector=TIMER0_A0_VECTOR
__interrupt void ta00_isr(void){
    if(++RECV_CLK_CNT == 12) {
    //if(RECV_CLK_CNT++ == 6000) {
        TA0CTL &= ~0x0030; //timer stop
        TA0CTL |= 0x0004; //timer clear
        TA0CCTL1 &= ~0x0004; //clk output reset(reset 0)
        RECV_CLK_EN = 0;
        //RECV_CLK_CNT = 0;
        P1OUT ^= 0x01; //debug - pin 1.0 LED
    }
    TA0CCTL0 &= ~0x0001; //interrupt flag down
}

//watch dog(PS/2 timeout) timer
#pragma vector=TIMER1_A0_VECTOR
__interrupt void ta10_isr(void){
    //timeout
    TA1CTL &= ~0x0030; //timer stop
    TA1CCTL0 &= ~0x0001; //interrupt flag down
    if(!DATA_RECV_DONE_FLG)
        P1OUT ^= 0x08; //DEBUG 1.3

    //state reset
    DATA_RECV_DONE_FLG = 1;
    RECV_DATA_READ_DONE_FLG = 1;
    P2IE |= 0x80;  // DATA - INT 2.7 enable
    P2IE &= ~0x20; // CLK  - INT 2.5 disable
    TA0CCTL1 &= ~0x0004; //clk output reset(reset 0)

    TA1CCR0 = 150; // 2clk(data_income timeout)
    TA1CTL |= 0x0014; //timer clear & on
}


#pragma vector=PORT2_VECTOR
__interrupt void p2_isr(void){
    switch(P2IV){
        //port2.7 - DATA_IN start(negedge)
        case 0x10:
            if(!(P2IN & 0x20)){ // CLK_in == 0
                TA1CTL &= ~0x0030; //timer stop
                TA1CCTL0 &= ~0x0001; //interrupt flag down
                P4OUT ^= 0x01; //debug - pin 4.0 LED
                P2IE &= ~0x80; // DATA - INT 2.7 disable
                P2IE |= 0x20;  // CLK  - INT 2.5 enable

                //activate RECV_CLK
                RECV_CLK_EN = 1;
                RECV_CLK_CNT = 0;
                TA0CCTL0 &= ~0x0001; //interrupt flag down
                TA0CTL |= 0x0014; //clk_gen timer clear & on(up mode)

                //initial RECV_DATA
                DATA_CNT = 0xFFFE; // -2. start bit
                DATA_RECV_DONE_FLG = 0;
                RECV_DATA = 0;

                TA1CCR0 = 15000; // 15ms(init_clk_wait timeout)
                TA1CTL |= 0x0014; //timer clear & on
            }
            break;
        //port2.5 - CLK_IN(negedge)
        case 0x0C:
            TA1CTL &= ~0x0030; //timer stop
            TA1CCTL0 &= ~0x0001; //interrupt flag down
            //TA0CTL |= 0x0004; //timer clear
            if(++DEBUG_CNT == 8)
                DEBUG_CNT;
            if(!DATA_RECV_DONE_FLG){
                if(++DATA_CNT == 10){
                    P2IE |= 0x80; // DATA - 1.4
                    DATA_RECV_DONE_FLG = 1;
                    RECV_DATA_READ_DONE_FLG = 0;
                    DEBUG_CNT = 0;
                    P2IE &= ~0x20; // CLK  - INT 2.5 disable
                }
                else
                    //port2.7 is data input
                    RECV_DATA |= ((P2IN & 0x80) && 1) << DATA_CNT;
            }
            TA1CCR0 = 150; // 2clk(data_income timeout)
            TA1CTL |= 0x0014; //timer clear & on
            break;
        default:
            break;
    }
}

int main(){
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    P4SEL0 |= 0x06;
    do{
        CSCTL7 &=~(XT1OFFG | DCOFFG);
        SFRIFG1 &= ~OFIFG;
    }while(SFRIFG1 & OFIFG);
    PM5CTL0 &= ~LOCKLPM5;

    // P1.0 DEBUG  out
    // P1.3 DEBUG  out
    // P1.7 CLK    out
    P1DIR |= 0x89;
    // P2.5 CLK  in
    // P2.7 DATA in
    P2DIR = 0x00;
    // P4.0 DEBUG out
    P4DIR |= 0x01;

    // P1.5 TA0CLK
    // P1.7 TA0.1(PWM)
    P1SEL0 |= 0x80;

    P2IE |= 0xA0; // DATA - 2.7, CLK - 2.5negedge
    P2IES = 0xA0; // 2.7-negedge(for connection initialize), 2.5-negedge(for pick incoming data)

    P1OUT = 0;
    P2OUT = 0;
    P4OUT = 0;

    RTCCTL = 0x0040;

    TA0CCR0 = 80; // 12.5khz
    TA0CCR1 = TA0CCR0 >> 1;  // toggle

    TA1CCR0 = 150; // 2clk(timeout)

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
    TA0CCTL0 = 0x0010;
//    TA0CCTL0 = 0x0000;
    TA0CCTL1 = 0x0060;

    TA1CCTL0 = 0x0010;

    __enable_interrupt();

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
    TA0CTL = 0x0204;
    TA1CTL = 0x0214;

    //TA0CTL = 0x0210;


    __get_interrupt_state();
    //__low_power_mode_3();
}
