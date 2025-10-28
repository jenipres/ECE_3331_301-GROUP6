#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

/* ===== User config ===== */
#define SMCLK_HZ     1000000UL      // We force SMCLK to 1 MHz
#define IR_F_CARRIER 38000UL        // 38 kHz
#define IR_DUTY_NUM  1              // ~33% duty = 1/3
#define IR_DUTY_DEN  3

/* P3.6 → TB0.2 output (FR6989) */
#define IR_PORT_DIR  P3DIR
#define IR_PORT_SEL0 P3SEL0
#define IR_PORT_SEL1 P3SEL1
#define IR_PIN_BIT   BIT6

static volatile bool ta1_done = false;

/* ===== Prototypes ===== */
static void force_SMCLK_1MHz(void);
static void init_carrier_tb0(void);
static void init_wait_timer_ta1(void);
static void ir_on(void);
static void ir_off(void);
static void wait_us(uint32_t us);

/* Public helpers */
void ir_mark_us(uint32_t us);
void ir_space_us(uint32_t us);

int main(void)
{
    WDTCTL  = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    force_SMCLK_1MHz();   // <— lock SMCLK to 1 MHz
    init_carrier_tb0();   // 38 kHz, ~33% duty on P3.6 (TB0.2)
    init_wait_timer_ta1();// µs waits via TA1 CCR0
    __enable_interrupt();

    while (1) {
        // Demo pattern (replace with your send_cmd logic)
        ir_mark_us(560);   ir_space_us(560);
        ir_mark_us(1600);  ir_space_us(1600);
        ir_mark_us(2400);  ir_space_us(5000);
    }
}

/* ===== Force SMCLK = 1 MHz from DCO ===== */
static void force_SMCLK_1MHz(void)
{
    CSCTL0_H = CSKEY_H;                        // unlock
    CSCTL1   = DCOFSEL_0;                      // DCO ~1 MHz
    CSCTL2   = SELS__DCOCLK | SELM__DCOCLK;    // SMCLK, MCLK from DCO
    CSCTL3   = DIVS__1 | DIVM__1;              // /1
    CSCTL0_H = 0;                              // lock
}

/* ===== Timer_B0: 38 kHz PWM on TB0.2 (P3.6) ===== */
static void init_carrier_tb0(void)
{
    IR_PORT_DIR  |= IR_PIN_BIT;
    IR_PORT_SEL0 |= IR_PIN_BIT;
    IR_PORT_SEL1 &= ~IR_PIN_BIT;

    uint16_t ccr0 = (uint16_t)((SMCLK_HZ / IR_F_CARRIER) - 1);  // 1e6/38k - 1 = 26
    TB0CCR0 = ccr0;

    // ~33% duty = (CCR0+1) * 1/3  (round to at least 1)
    uint16_t duty = (uint16_t)(((uint32_t)(ccr0 + 1) * IR_DUTY_NUM) / IR_DUTY_DEN);
    if (duty == 0) duty = 1;
    TB0CCR2  = duty;

    TB0CCTL2 = OUTMOD_0;                       // start "off" (low)
    TB0CTL   = TASSEL__SMCLK | MC__UP | TBCLR; // SMCLK, up mode
}

/* ===== Timer_A1: µs one-shot waiter ===== */
static void init_wait_timer_ta1(void)
{
    TA1CTL   = TASSEL__SMCLK | MC__STOP | TACLR;  // SMCLK, stopped
    TA1CCTL0 = 0;
}

/* ===== Carrier gate ===== */
static void ir_on(void)
{
    TB0CCTL2 = OUTMOD_7;        // enable PWM (reset/set)
}
static void ir_off(void)
{
    TB0CCTL2 = OUTMOD_0;        // static output
    TB0CCTL2 &= ~OUT;           // hold low
}

/* ===== µs wait (splits long waits into <= 65535 µs chunks) ===== */
static void wait_us(uint32_t us)
{
    while (us) {
        uint16_t chunk = (us > 0xFFFFUL) ? 0xFFFFu : (uint16_t)us;
        ta1_done = false;

        TA1CCR0  = chunk - 1;                   // N µs @ 1 MHz
        TA1CCTL0 = CCIE;
        TA1CTL   = TASSEL__SMCLK | MC__UP | TACLR;

        __bis_SR_register(LPM0_bits | GIE);     // sleep until ISR
        us -= chunk;
    }
}

/* ===== Public helpers ===== */
void ir_mark_us(uint32_t us)  { ir_on();  wait_us(us); ir_off(); }
void ir_space_us(uint32_t us) { ir_off(); wait_us(us); }

/* ===== TA1 CCR0 ISR ===== */
#pragma vector = TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
    TA1CTL   &= ~MC__UP;        // stop timer
    TA1CCTL0  = 0;              // disable CCIE
    ta1_done  = true;
    __bic_SR_register_on_exit(LPM0_bits); // wake main
}

