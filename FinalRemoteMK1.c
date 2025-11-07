#include <msp430.h>
#include <stdint.h>

#define F_CARRIER   38000UL         // 38 kHz carrier
#define CYCLE_US    (1000000UL / F_CARRIER)  // ≈ 26.3 µs per cycle


static void sendBurst(uint16_t cycles);

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;       // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // Unlock GPIO (FRAM devices)

    // ===== Set DCO = 1 MHz =====
    CSCTL0_H = CSKEY >> 8;          // Unlock clock registers
    CSCTL1 = DCOFSEL_0;             // DCO = 1 MHz
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK;  // MCLK & SMCLK = DCO
    CSCTL3 = DIVM__1 | DIVS__1;     // No dividers
    CSCTL0_H = 0;                   // Lock clock system

    // ===== Configure P2.7 for TB0.6 output =====
    P2DIR  |= BIT7;                 // P2.7 as output
    P2SEL0 |= BIT7;                 // Select TB0.6 function
    P2SEL1 &= ~BIT7;                // Primary module function (Timer_B)
     

    // ===== Configure Timer_B0 for 38 kHz =====
    // fPWM = SMCLK / (TB0CCR0 + 1)
    TB0CCR0  = 25;                  // (1 MHz / (25 + 1)) ≈ 38 kHz
    TB0CCR6  = 13;                  // 50% duty cycle
    TB0CCTL6 = OUTMOD_7;            // Reset/Set mode
    TB0CTL   = TBSSEL__SMCLK | MC__UP | TBCLR;  // Use SMCLK, up mode

    P3DIR &= ~(BIT0 | BIT3 | BIT6 | BIT7);    // inputs
    P3REN |=  (BIT0 | BIT3 | BIT6 | BIT7); 
    P3OUT |=  (BIT0 | BIT3 | BIT6 | BIT7);    // pull-ups
    

    P1DIR |= BIT0;              // Set P1.0 as output (LED)
    P1OUT &= ~BIT0;             // Start with LED off


    while (1)
    {
        // Button pressed? (active low)
        while (!(P3IN & BIT0))
        {
            sendBurst(20);
        }
        while (!(P3IN & BIT3))
        {
            sendBurst(40);
        }
        while (!(P3IN & BIT6))
        {
            sendBurst(60);
        }
        while (!(P3IN & BIT7))
        {
            sendBurst(80);
        }
    }

}
static void delay_us(uint32_t us)
{
    while (us--) __delay_cycles(1);  // 1 µs delay per loop at 1 MHz
}

// cycles = number of carrier cycles
static void sendBurst(uint16_t cycles)
{
    uint32_t burst_us = cycles * CYCLE_US;
    uint32_t gap_us   = (cycles) ? burst_us * 4 : burst_us * 3;

    TB0CCTL6 |= OUTMOD_7;                     // enable modulation
    delay_us(burst_us);               // 1 µs per cycle at 1MHz
    TB0CCTL6 &= ~OUTMOD_7;                    // stop modulation
    delay_us(gap_us);                   // enforce gap
}
