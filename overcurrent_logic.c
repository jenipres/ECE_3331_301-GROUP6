/* ---- Public prototypes you can call from your code ---- */
void OC_Config(uint16_t trip_counts, uint16_t trip_ms, uint16_t clear_ms);
void OC_SetCallbacks(void (*cut_cb)(void), void (*resume_cb)(void));
void OC_Init_A4A5(void);
void OC_1msTask(bool clear_button_pressed);
bool     OC_IsFault(void);
uint16_t OC_LatestAdcA(void);
uint16_t OC_LatestAdcB(void);

/* ---- Internal state ---- */
enum { ST_NORMAL=0, ST_OVER, ST_FAULT };
static uint8_t  oc_state;
static uint16_t TRIP_MS      = 50;     // sustain time (ms)
static uint16_t CLEAR_MS     = 1000;   // button hold (ms)
static uint16_t over_ms, btn_ms;
static uint16_t adcA, adcB;            // latest raw reads (for telemetry)
static void (*cut_cb)(void);
static void (*resume_cb)(void);

/* ---- Per-channel integer thresholds (edit these if you like) ---- */
static uint16_t ADC_TRESH_A = 500;     // set per your testing
static uint16_t ADC_TRESH_B = 500;

/* ---- Tiny ADC helpers (blocking, no ISRs) ---- */
static inline void adc_wait_done(void){ while (ADC12CTL1 & ADC12BUSY); }
static uint16_t adc_read_inch(uint8_t inch){
  ADC12CTL0 &= ~ADC12ENC;
  ADC12MCTL0 = (inch & 0x0F) | ADC12VRSEL_1;   // VR+=Vref(1.2V), VR-=AVss
  ADC12CTL0 |= ADC12ENC | ADC12SC;
  adc_wait_done();
  return ADC12MEM0;
}

/* ---- API ---- */
void OC_Config(uint16_t trip_counts, uint16_t trip_ms, uint16_t clear_ms){
  // For backward compatibility: apply same trip to both channels if provided
  if (trip_counts){ ADC_TRESH_A = trip_counts; ADC_TRESH_B = trip_counts; }
  if (trip_ms)     TRIP_MS     = trip_ms;
  if (clear_ms)    CLEAR_MS    = clear_ms;
}

void OC_SetCallbacks(void (*cut)(void), void (*resume)(void)){
  cut_cb = cut; resume_cb = resume;
}

void OC_Init_A4A5(void){
  /* Route P8.7 (A4) & P8.6 (A5) to analog */
  P8SEL1 |= (BIT7 | BIT6);
  P8SEL0 |= (BIT7 | BIT6);

  /* Internal 1.2 V reference on */
  PMMCTL0_H = PMMPW_H;
  REFCTL0   = REFON | REFVSEL_0;   // 1.2 V
  __delay_cycles(40000);           // settle

  /* ADC12_B: single-channel template (we switch INCH in software) */
  ADC12CTL0 = ADC12SHT0_2 | ADC12ON;
  ADC12CTL1 = ADC12SHP;            // sampling timer
  ADC12CTL2 = ADC12RES_2;          // 12-bit

  oc_state = ST_NORMAL;
  over_ms = btn_ms = 0;
  adcA = adcB = 0;
}

/* Simple 2-sample mini-average to reduce PWM-phase jitter */
static inline uint16_t read_avg(uint8_t inch){
  uint32_t acc = 0;
  acc += adc_read_inch(inch);
  acc += adc_read_inch(inch);
  return (uint16_t)(acc >> 1);   // /2
}

void OC_1msTask(bool clear_button_pressed){
  /* Sample A4 then A5 (each a few tens of µs) with tiny averaging */
  adcA = read_avg(4);   // A4 -> P8.7 (Motor A sense)
  adcB = read_avg(5);   // A5 -> P8.6 (Motor B sense)

  // Integer-only compare; use >= (one-count predictable behavior)
  bool over_now = (adcA >= ADC_TRESH_A) || (adcB >= ADC_TRESH_B);

  switch(oc_state){
    case ST_NORMAL:
      if (over_now){ over_ms = 1; oc_state = ST_OVER; }
      break;

    case ST_OVER:
      if (!over_now){ over_ms = 0; oc_state = ST_NORMAL; }
      else if (++over_ms >= TRIP_MS){
        if (cut_cb) cut_cb();       // -> your 20 kHz PWM layer: set duty=0 / EN low
        btn_ms = 0;
        oc_state = ST_FAULT;
      }
      break;

    case ST_FAULT:
      btn_ms = clear_button_pressed ? (btn_ms < 0xFFFF ? btn_ms + 1 : btn_ms) : 0;
      if (btn_ms >= CLEAR_MS){
        if (resume_cb) resume_cb(); // -> restore PWM safely
        btn_ms = over_ms = 0;
        oc_state = ST_NORMAL;
      }
      break;
  }
}

bool     OC_IsFault(void){ return oc_state == ST_FAULT; }
uint16_t OC_LatestAdcA(void){ return adcA; }
uint16_t OC_LatestAdcB(void){ return adcB; }

/* ================== Minimal 1 ms tick (hook into your project) ==================
   If you already have a 1 ms system tick: just call OC_1msTask() there and
   delete this Timer_A block. Otherwise this gives you a tiny tick generator.
*/
static void oc_start_1ms_tick(void){
  // TA0 up mode @ SMCLK ~1 MHz -> 1 ms period
  TA0CTL   = TASSEL__SMCLK | MC__UP | TACLR;
  TA0CCR0  = 999;
  TA0CCTL0 = CCIE;
}
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void){
  bool btn = (P1IN & BIT3) == 0;         // active-low clear button on P1.3
  OC_1msTask(btn);
}
