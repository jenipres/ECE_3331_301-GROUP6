#include <driverlib.h>
#include <msp430fr6989.h>
#include <stdint.h>
#include <stdbool.h>

#define F_CPU_HZ              8000000UL   // DCO 8 MHz 
#define PWM_FREQ_HZ           19000UL     // ~20 kHz PWM 
#define SYSTICK_HZ            1000UL      // 1 kHz system tick 
#define OC_WINDOW_MS          50          // overcurrent window: trip if >thr for >= this ms 
#define CLEAR_HOLD_MS         1000        // button hold to clear fault 
#define NO_SIGNAL_MS          200
 

//OC Threshold
#define ADC_THRESH_A          507//506.9999999999999699
#define ADC_THRESH_B          507//506.9999999999999699

#define PWM_TICKS             (F_CPU_HZ / PWM_FREQ_HZ)     //tick rate, used for duty cycle

//state defs
typedef enum { SYS_OK=0, SYS_FAULT=1 } sys_state_t;
static volatile sys_state_t sys_state = SYS_OK;
static volatile uint16_t oc_ticks_A = 0, oc_ticks_B = 0;
static volatile uint16_t clear_hold_ms = 0;
static volatile uint32_t fault_count = 0;
static volatile uint16_t dutyA = (PWM_TICKS * 50) / 100;
static volatile uint16_t dutyB = (PWM_TICKS * 50) / 100;

volatile uint16_t lastCap = 0;
volatile uint16_t pulseWidth = 0;
volatile uint8_t  gotPulse = 0;
volatile uint16_t riseTime = 0;
volatile uint16_t fallTime = 0;
volatile uint8_t  edge = 0;

volatile uint16_t ms_since_pulse = 0;

void initCapture(void)
{
    // Configure P2.7 as TB0.6 capture input
    P3DIR &= ~BIT3;             // Input
    P3SEL1 |= BIT3;             // Select TB0.6 function
    P3SEL0 &= ~BIT3;

    // Configure Timer_B0 in continuous mode using SMCLK
    TA1CTL = TASSEL__SMCLK | MC__CONTINUOUS | TACLR;

    // Capture on BOTH edges, synchronize, interrupt enabled
    TA1CCTL1 = CM_2 | CCIS_0 | CAP | SCS | CCIE;
}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
{
    switch (TA1IV)
    {
        case TA1IV_TACCR1:
        {
            uint16_t now = TA1CCR1;

            if (edge == 0)
            {
                // Rising edge detected
                fallTime = now;
                edge = 1;

                // Switch to capture falling edge next
                TA1CCTL1 = CM_1 | CCIS_0 | CAP | SCS | CCIE;
            }
            else
            {
                // Falling edge detected
                riseTime = now;
                pulseWidth = (uint16_t)(riseTime - fallTime);  // Auto-wrap works fine
                gotPulse = 1;
                edge = 0;

                // Switch back to rising edge for next pulse
                TA1CCTL1 = CM_2 | CCIS_0 | CAP | SCS | CCIE;
            }

            break;
        }
        default:
            break;
    }
}

//helpers
static inline void dir_A_forward(void){  // IN1=P1.3 high, IN2=P3.0 low 
    P1OUT |= BIT3;
    P3OUT &= ~BIT0;
}
static inline void dir_A_reverse(void){  // IN1=low, IN2=high 
    P1OUT &= ~BIT3;
    P3OUT |= BIT0;
}
static inline void dir_B_forward(void){  // IN3=P3.1 high, IN4=P2.3 low 
    P3OUT |= BIT1;
    P2OUT &= ~BIT3;
}
static inline void dir_B_reverse(void){  // IN3=low, IN4=high 
    P3OUT &= ~BIT1;
    P2OUT |= BIT3;
}
static inline void stop_motors(void){
    set_speed_A_percent(0);
    set_speed_B_percent(0);
    // Force direction outputs low for safety
    P1OUT &= ~BIT3;
    P3OUT &= ~BIT0;
    P3OUT &= ~BIT1;
    P2OUT &= ~BIT3;
}
static inline void pwm_apply(void){
    TB0CCR6 = dutyA;  // TA1.1 -P3.3 
    TB0CCR2 = dutyB;  // TB0.2 -P3.6 
}

static inline void pwm_enable(bool en){
    if (en){
        TB0CCTL2 = OUTMOD_7;
        TB0CCTL6 = OUTMOD_7;
    } else {
        TB0CCTL6 = OUTMOD_0;
        TB0CCTL2 = OUTMOD_0;
        P3OUT &= ~BIT6; // force ENA/ENB low
        P2OUT &= ~BIT7;
    }
}

static uint16_t adc12_read(uint8_t mctl_inch) {
    ADC12CTL0 &= ~ADC12ENC;            // disable ADC to configure channel
    ADC12MCTL0 = mctl_inch;            // select channel
    ADC12CTL0 |= ADC12ENC | ADC12SC;   // enable and start conversion
    while (ADC12CTL1 & ADC12BUSY);     // wait until done
    return ADC12MEM0;                  // return result
}

//Initiate clock and gpios
static void clock_init_8MHz(void){
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_6;
    CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__VLOCLK;
    CSCTL3 = DIVM__1 | DIVS__1 | DIVA__1;
    CSCTL0_H = 0;
}

static void gpio_init(void){
    /* LED (P1.0) */
    P1DIR |= BIT0;  
    P1OUT &= ~BIT0;

    // Button (P1.1 pull-up) 
    P1DIR &= ~BIT1;
    P1REN |= BIT1;
    P1OUT |= BIT1;

    // Direction pins as outputs low 
    P1DIR |= BIT3; P1OUT &= ~BIT3;  // IN1 
    P3DIR |= BIT0; P3OUT &= ~BIT0;  // IN2 
    P3DIR |= BIT1; P3OUT &= ~BIT1;  // IN3
    P2DIR |= BIT3; P2OUT &= ~BIT3;  // IN4 

    //PWM pins: P3.3=TA1.1, P3.6=TB0.2 (SEL1=1, SEL0=0)
    P3DIR  |= BIT6;
    P3SEL1 |= BIT6;
    P3SEL0 &= ~BIT6;

    P2DIR |= BIT7;
    P2SEL0 |= BIT7;
    P2SEL1 &= ~BIT7;

    // ADC analog pins: P8.6=A5, P8.7=A4
    P8SEL0 |= BIT6 | BIT7;   // select peripheral function
    P8SEL1 &= ~(BIT6 | BIT7); // make sure it's primary (not secondary/tertiary)


}

static void timer_pwm_init(void){
    TB0CCR0  = PWM_TICKS - 1;
    TB0CCR6  = 0;
    TB0CCTL6 = OUTMOD_7;
    TB0CTL   = TBSSEL__SMCLK | MC__UP | TBCLR;

    TB0CCR0  = PWM_TICKS - 1;
    TB0CCR2  = 0;
    TB0CCTL2 = OUTMOD_7;
    TB0CTL   = TBSSEL__SMCLK | MC__UP | TBCLR;
}


static void adc12_init(void){
    ADC12CTL0 = ADC12SHT0_2 | ADC12ON;
    ADC12CTL1 = ADC12SHP;
    ADC12CTL2 = ADC12RES_2;
}

// Fault functions
static void enter_fault(void){
    if (sys_state == SYS_FAULT) return;
    sys_state = SYS_FAULT;
    pwm_enable(false);
    P1OUT |= BIT0; /* LED on */
    fault_count++;
}

static void clear_fault(void){
    sys_state = SYS_OK;
    oc_ticks_A = oc_ticks_B = 0;
    clear_hold_ms = 0;
    P1OUT &= ~BIT0;
    pwm_enable(true);
}
//Speed control functions
void set_speed_A_percent(uint8_t pct){
    if (pct > 100) pct = 100;
    dutyA = (uint16_t)((uint32_t)PWM_TICKS * pct / 100U);
    pwm_apply();
}
void set_speed_B_percent(uint8_t pct){
    if (pct > 100) pct = 100;
    dutyB = (uint16_t)((uint32_t)PWM_TICKS * pct / 100U);
    pwm_apply();
}

// DEMO---------------------



// Main----------------------
int main(void){
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    P2DIR |= BIT1 | BIT2 | BIT3 | BIT4;   // Set P2.1–P2.4 as outputs
    P2OUT &= ~(BIT1 | BIT2 | BIT3 | BIT4);

    initCapture();
    clock_init_8MHz();
    gpio_init();
    adc12_init();
    timer_pwm_init();

    __enable_interrupt();

    pwm_apply();
    pwm_enable(true);


    while (1){
        if (gotPulse) {
            gotPulse = 0;
            uint16_t us = pulseWidth;
            ms_since_pulse = 0; 

            if ( us > 30000 && us < 33500)
            {
                dir_A_forward(); dir_B_forward();
                set_speed_A_percent(35); set_speed_B_percent(35);
                
            }
            else if ( us > 34000 && us < 35000)
            {
                dir_A_reverse(); dir_B_forward();
                set_speed_A_percent(35); set_speed_B_percent(35);
                
            }
            else if ( us > 35000 && us <36000)
            {
                dir_A_forward(); dir_B_reverse();
                set_speed_A_percent(35); set_speed_B_percent(35);
                
            }
            else if ( us > 36000 && us <38000)
            {
                dir_A_reverse(); dir_B_reverse();
                set_speed_A_percent(35); set_speed_B_percent(35);
                
            }

        }
        else {
        // crude 1 ms tick — interrupts still run
        __delay_cycles(F_CPU_HZ/1000); // 1 ms delay (approx, F_CPU_HZ must be correct)
            if (++ms_since_pulse >= NO_SIGNAL_MS){
            stop_motors();
            }      
        }
    }
}

//Fault detection and state control
void __attribute__((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR(void)
{
    uint16_t iA = adc12_read(ADC12INCH_6); // P8.6- reads analog signal
    uint16_t iB = adc12_read(ADC12INCH_7); // P8.7- reads analog signal

    oc_ticks_A = (iA > ADC_THRESH_A) ? (oc_ticks_A + 1) : 0;
    oc_ticks_B = (iB > ADC_THRESH_B) ? (oc_ticks_B + 1) : 0;//checks if current is above threshold, increments if so

    if (sys_state == SYS_OK)//ensures system state is okay
    {
        if (oc_ticks_A >= OC_WINDOW_MS || oc_ticks_B >= OC_WINDOW_MS){// checks if overcurrent was sustained longer than 50 ms
            enter_fault();
        } 
        else{
            return;
        }
    } else {
        bool btn_down = ((P1IN & BIT1) == 0);// clears fault if button P1.1 pressed
        if (btn_down){
            if (clear_hold_ms < (CLEAR_HOLD_MS + 200)) clear_hold_ms++;
            if (clear_hold_ms >= CLEAR_HOLD_MS){
                clear_fault();
            }
        } else {
            clear_hold_ms = 0;
        }
    }

    static uint16_t hb = 0;
    if (sys_state == SYS_OK){
        if (++hb >= 250){
            hb = 0;
            P1OUT ^= BIT0;
        }
    }

    __bic_SR_register_on_exit(LPM0_bits);
}
