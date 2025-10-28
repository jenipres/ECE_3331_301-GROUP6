#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

/* ========= User config ========= */
#define SMCLK_HZ     1000000UL      // Use 1 MHz SMCLK for simple µs math
#define IR_F_CARRIER 38000UL        // 38 kHz
#define IR_DUTY_NUM  1              // ~33% duty = 1/3
#define IR_DUTY_DEN  3

/* P3.6 → TB0.2 output (MSP430FR6989 pin mapping) */
#define IR_PORT_DIR  P3DIR
#define IR_PORT_SEL0 P3SEL0
#define IR_PORT_SEL1 P3SEL1
#define IR_PIN_BIT   BIT6

/* ========= Globals ========= */
static volatile bool ta1_done = false;

/* ========= Prototypes ========= */
static void init_clocks_1MHz(void);
static void init_carrier_tb0(void);
static void init_wait_timer_ta1(void);
static void ir_on(void);
static void ir_off(void);
static void wait_us(uint32_t us);

void ir_mark_us(uint32_t us);
void ir_space_us(uint32_t us);

/* ========= Main ========= */
int main(void)
{
    WDTCTL  = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;                 // unlock GPIOs (FRAM devices)

    init_clocks_1MHz();                   // SMCLK = 1 MHz
    init_carrier_tb0();                   // TB0.2 = 38 kHz PWM, gated
    init_wait_timer_ta1();                // TA1 used for µs waits

    __enable_interrupt();

    while (1) {
        /* ---- Demo pattern: three different "marks" with gaps ----
           Replace with your own send_cmd() that uses ir_mark_us/ir_space_us
         */
        ir_mark_us(560);   // ~0.56 ms mark (NEC short)
        ir_space_us(560);

        ir_mark_us(1600);  // ~1.6 ms mark
        ir_space_us(1600);

        ir_mark_us(2400);  // ~2.4 ms mark
        ir_space_us(5000); // idle gap before repeating
    }
}

/* ========= Init: 1 MHz DCO / SMCLK ========= */
static void init_clocks_1MHz(void)
{
    // Default on FR6989 is 1 MHz DCO; keep it simple.
    // If you’ve changed CS elsewhere, ensure SMCLK = 1 MHz, sourced from DCO.
    CSCTL0_H = CSKEY_H;                       // Unlock CS registers
    CSCTL1   = DCOFSEL_0;                     // DCO ~1 MHz
    CSCTL2   = SELS__DCOCLK | SELM__DCOCLK;   // SMCLK, MCLK from DCO
    CSCTL3   = DIVS__1 | DIVM__1;             // /1
    CSCTL0_H = 0;                             // Lock CS
}

/* ========= Init: Timer_B0 for 38 kHz PWM on P3.6/TB0.2 ========= */
static void init_carrier_tb0(void)
{
    /* Route TB0.2 to pin */
    IR_PORT_DIR  |= IR_PIN_BIT;
    IR_PORT_SEL0 |= IR_PIN_BIT;
    IR_PORT_SEL1 &= ~IR_PIN_BIT;

    /* Set PWM period and duty */
    uint16_t ccr0 = (uint16_t)((SMCLK_HZ / IR_F_CARRIER) - 1);  // carrier period
    TB0CCR0 = ccr0;

    // ~33% duty => CCR2 = (CCR0+1) * 1/3
    uint16_t duty = (uint16_t)(((uint32_t)(ccr0 + 1) * IR_DUTY_NUM) / IR_DUTY_DEN);
    if (duty == 0) duty = 1;
    TB0CCR2 = duty;

    TB0CCTL2 = OUTMOD_0;                     // start "off" (output low)
    TB0CTL   = TASSEL__SMCLK | MC__UP | TBCLR;
}

/* ========= Init: Timer_A1 for µs one-shot waits ========= */
static void init_wait_timer_ta1(void)
{
    TA1CTL = TASSEL__SMCLK | MC__STOP | TACLR;  // SMCLK, stopped
    TA1CCTL0 = 0;
}

/* ========= Carrier gate control =========
   OUTMOD_7 (Reset/Set) toggles the PWM; OUTMOD_0 holds OUT bit (we force low).
*/
static void ir_on(void)
{
    TB0CCTL2 = OUTMOD_7;        // enable PWM on TB0.2
}

static void ir_off(void)
{
    TB0CCTL2 = OUTMOD_0;        // force static output
    TB0CCTL2 &= ~OUT;           // hold low while "off"
}

/* ========= µs wait using TA1 CCR0 interrupt (no busy-wait) ========= */
static void wait_us(uint32_t us)
{
    if (us == 0) return;

    // For long waits, split into chunks <= 0xFFFF µs
    while (us) {
        uint16_t this_chunk = (us > 0xFFFFUL) ? 0xFFFFu : (uint16_t)us;
        ta1_done = false;

        TA1CCR0  = this_chunk - 1;               // N µs at 1 MHz
        TA1CCTL0 = CCIE;
        TA1CTL   = TASSEL__SMCLK | MC__UP | TACLR;

        // Sleep until ISR wakes us
        __bis_SR_register(LPM0_bits | GIE);
        // (on exit, ISR stopped TA1 and cleared LPM0)

        us -= this_chunk;
    }
}

/* ========= Public helpers ========= */
void ir_mark_us(uint32_t us)
{
    ir_on();
    wait_us(us);
    ir_off();                   // ensure clean end of mark
}

void ir_space_us(uint32_t us)
{
    ir_off();
    wait_us(us);
}

/* ========= TA1 CCR0 ISR ========= */
#pragma vector = TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
    TA1CTL   &= ~MC__UP;        // stop
    TA1CCTL0  = 0;              // disable CCIE
    ta1_done  = true;
    __bic_SR_register_on_exit(LPM0_bits);  // wake main
}
