#include <msp430fr6989.h>
#include <stdint.h>
#include <stdbool.h>

/* --------- Clock / rates --------- */
#define F_CPU_HZ            8000000UL        /* DCO ~8 MHz */
#define PWM_FREQ_HZ         20000UL          /* ~20 kHz */
#define TA0_TICKS_PER_PWM   (F_CPU_HZ / PWM_FREQ_HZ) /* 400 at 8 MHz */
#define SYSTICK_HZ          1000UL           /* 1 kHz periodic ISR */
#define OC_WINDOW_MS        50               /* trip if >thr for >=50 ms */

/* --------- Pins (edit to suit your wiring) --------- */
/* PWM: P1.2=TA0.1, P1.3=TA0.2 */
#define PWM_A_BIT           BIT2
#define PWM_B_BIT           BIT3

/* Direction pins on P2.x */
#define IN1_A               BIT0
#define IN2_A               BIT1
#define IN3_B               BIT2
#define IN4_B               BIT3

/* LED and Button */
#define LED_BIT             BIT0        /* P1.0 LED */
#define BTN_BIT             BIT1        /* P1.1 button (active low) */

/* ADC12_B inputs: P3.0=A12, P3.1=A13 */
#define ADC_CH_A_MCTL       ADC12INCH_12     /* A12 -> ADC12MCTLx INCH=12 */
#define ADC_CH_B_MCTL       ADC12INCH_13     /* A13 -> ADC12MCTLx INCH=13 */

/* ---- Overcurrent thresholds (ADC counts, 12-bit: 0..4095) ----
   Set after measuring I_load and converting 1.5 * I_load -> V_sense -> counts.
   counts = (V_sense / Vref) * 4095, default Vref = 3.3V. */
#define ADC_THRESH_A        1200      /* EDIT after measuring */
#define ADC_THRESH_B        1200      /* EDIT after measuring */

/* Duty (0..CCR0) */
static volatile uint16_t dutyA = (TA0_TICKS_PER_PWM * 40) / 100;
static volatile uint16_t dutyB = (TA0_TICKS_PER_PWM * 40) / 100;

typedef enum { SYS_OK = 0, SYS_FAULT = 1 } sys_state_t;
static volatile sys_state_t sys_state = SYS_OK;
static volatile uint16_t oc_ticks_A = 0, oc_ticks_B = 0;
static volatile uint16_t clear_hold_ms = 0;
static volatile uint32_t fault_count = 0;

/* ------------ Helpers ------------ */
static inline void set_dir_A_forward(void){ P2OUT = (P2OUT & ~(IN1_A|IN2_A)) | IN1_A; }
static inline void set_dir_A_reverse(void){ P2OUT = (P2OUT & ~(IN1_A|IN2_A)) | IN2_A; }
static inline void set_dir_B_forward(void){ P2OUT = (P2OUT & ~(IN3_B|IN4_B)) | IN3_B; }
static inline void set_dir_B_reverse(void){ P2OUT = (P2OUT & ~(IN3_B|IN4_B)) | IN4_B; }

static inline void pwm_apply(void){
    TA0CCR1 = dutyA;  /* TA0.1 -> P1.2 */
    TA0CCR2 = dutyB;  /* TA0.2 -> P1.3 */
}

static inline void pwm_enable(bool en){
    if(en){
        TA0CCTL1 = OUTMOD_7;  /* reset/set */
        TA0CCTL2 = OUTMOD_7;
    } else {
        TA0CCTL1 = OUTMOD_0;  /* force OUT bit state */
        TA0CCTL2 = OUTMOD_0;
        P1OUT &= ~(PWM_A_BIT | PWM_B_BIT);
    }
}

/* Single-conversion read on ADC12_B, returns 12-bit result */
static uint16_t adc12_read(uint8_t mctl_inch)
{
    ADC12CTL0 &= ~ADC12ENC;
    ADC12MCTL0 = mctl_inch;          /* select channel, Vref=AVCC/AVSS */
    ADC12CTL0 |= ADC12ENC | ADC12SC; /* enable + start */
    while (ADC12CTL1 & ADC12BUSY);
    return ADC12MEM0;
}

/* ------------ Init ------------ */
static void clock_init_8MHz(void){
    /* Unlock CS registers */
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_6;       /* DCO ~8 MHz */
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__VLOCLK;
    CSCTL3 = DIVM__1 | DIVS__1 | DIVA__1;
    CSCTL0_H = 0;             /* Lock CS */
}

static void gpio_init(void){
    /* LED out */
    P1DIR |= LED_BIT;  P1OUT &= ~LED_BIT;

    /* Button in with pull-up */
    P1DIR &= ~BTN_BIT;
    P1REN |= BTN_BIT;
    P1OUT |= BTN_BIT;

    /* Direction pins out low */
    P2DIR |= (IN1_A|IN2_A|IN3_B|IN4_B);
    P2OUT &= ~(IN1_A|IN2_A|IN3_B|IN4_B);

    /* PWM pins to TA0 function: P1.2/P1.3
       FR devices use SEL0/SEL1 mux: primary TA0.1/TA0.2 is SEL0=1, SEL1=0 */
    P1DIR |= (PWM_A_BIT | PWM_B_BIT);
    P1SEL0 |= (PWM_A_BIT | PWM_B_BIT);
    P1SEL1 &= ~(PWM_A_BIT | PWM_B_BIT);

    /* ADC inputs: P3.0 (A12), P3.1 (A13) to analog */
    P3SEL0 |= (BIT0 | BIT1);
    P3SEL1 |= (BIT0 | BIT1); /* both 1 -> analog on FR6989 */
}

static void timerA0_pwm_init(void){
    TA0CCR0  = TA0_TICKS_PER_PWM - 1; /* period */
    TA0CCR1  = 0;
    TA0CCR2  = 0;
    TA0CCTL1 = OUTMOD_7;  /* reset/set */
    TA0CCTL2 = OUTMOD_7;
    TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;
}

static void timerA1_systick_init(void){
    TA1CCR0  = (F_CPU_HZ / SYSTICK_HZ) - 1; /* 8e6/1000 - 1 = 7999 */
    TA1CCTL0 = CCIE;
    TA1CTL   = TASSEL__SMCLK | MC__UP | TACLR;
}

static void adc12_init(void){
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;         /* 16 cycles sample, ON */
    ADC12CTL1 = ADC12SHP;                       /* SAMPCON from timer; here use pulse mode */
    ADC12CTL2 = ADC12RES_2;                     /* 12-bit conversion */
    /* Vref = AVCC/AVSS default; MCTL will set channel */
}

/* ------------ Fault handling ------------ */
static void enter_fault(void){
    sys_state = SYS_FAULT;
    pwm_enable(false);
    P1OUT |= LED_BIT;
    fault_count++;
}
static void clear_fault(void){
    sys_state = SYS_OK;
    oc_ticks_A = oc_ticks_B = 0;
    clear_hold_ms = 0;
    pwm_enable(true);
    P1OUT &= ~LED_BIT;
}

/* ------------ Main ------------ */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   /* Stop WDT */

    clock_init_8MHz();
    gpio_init();
    adc12_init();
    timerA0_pwm_init();
    timerA1_systick_init();

    __enable_interrupt();

    set_dir_A_forward();
    set_dir_B_forward();

    pwm_apply();
    pwm_enable(true);

    while(1){
        /* Idle; wake each 1 ms in TA1 ISR */
        __bis_SR_register(LPM0_bits | GIE);
        __no_operation();
    }
}

/* ------------ 1 kHz SysTick ISR ------------ */
void __attribute__((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR(void)
{
    /* Sample currents */
    uint16_t iA = adc12_read(ADC_CH_A_MCTL);
    uint16_t iB = adc12_read(ADC_CH_B_MCTL);

    /* Overcurrent windows */
    oc_ticks_A = (iA > ADC_THRESH_A) ? (oc_ticks_A + 1) : 0;
    oc_ticks_B = (iB > ADC_THRESH_B) ? (oc_ticks_B + 1) : 0;

    if (sys_state == SYS_OK) {
        if (oc_ticks_A >= OC_WINDOW_MS || oc_ticks_B >= OC_WINDOW_MS) {
            enter_fault();
        }
    } else {
        /* Require 1 s button hold to clear */
        bool btn_down = ((P1IN & BTN_BIT) == 0);
        if (btn_down) {
            if (clear_hold_ms < 1200) clear_hold_ms++;
            if (clear_hold_ms >= 1000) clear_fault();
        } else {
            clear_hold_ms = 0;
        }
    }

    /* Exit LPM0 on ISR return so main() can run (optional) */
    __bic_SR_register_on_exit(LPM0_bits);
}

/* Optional: duty setters */
void set_duty_A_percent(uint8_t pct){
    if (pct > 100) pct = 100;
    dutyA = (uint16_t)((uint32_t)TA0_TICKS_PER_PWM * pct / 100U);
    pwm_apply();
}
void set_duty_B_percent(uint8_t pct){
    if (pct > 100) pct = 100;
    dutyB = (uint16_t)((uint32_t)(8000000UL / 20000UL) * pct / 100U);
    pwm_apply();
}
