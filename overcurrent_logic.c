#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

/* === Overcurrent Protection API (formerly in header) === */

/* Configure thresholds (ADC counts & ms). Defaults are typical for 0.53 V trip @ 1.2Vref. */
void OC_Config(uint16_t trip_counts, uint16_t trip_ms, uint16_t clear_ms);

/* Provide callbacks. I’ll call cut_cb() when fault latches, resume_cb() when cleared. */
void OC_SetCallbacks(void (*cut_cb)(void), void (*resume_cb)(void));

/* One-time init: routes P8.7/P8.6 to analog, powers 1.2 V ref, sets up ADC12_B (blocking mode). */
void OC_Init_A4A5(void);

/* Call this exactly every 1 ms (from YOUR system tick). Pass true while the clear-fault button is held. */
void OC_1msTask(bool clear_button_pressed);

/* Status / telemetry (optional). */
bool     OC_IsFault(void);
uint16_t OC_LatestAdcA(void);   // A4 (P8.7)
uint16_t OC_LatestAdcB(void);   // A5 (P8.6)

/* === Implementation === */

enum { ST_NORMAL=0, ST_OVER, ST_FAULT };
static uint8_t  state;
static uint16_t TRIP_COUNTS = 1800;   // ~0.53 V @ 1.2 V ref
static uint16_t TRIP_MS      = 50;    // sustain time
static uint16_t CLEAR_MS     = 1000;  // button hold time

static uint16_t over_ms, btn_ms;
static uint16_t adcA, adcB;
static void (*cut_cb)(void);
static void (*resume_cb)(void);

/* --- tiny ADC helpers (blocking, no ISRs) --- */
static inline void adc_wait_done(void){ while (ADC12CTL1 & ADC12BUSY); }
static uint16_t adc_read_inch(uint8_t inch){
  ADC12CTL0 &= ~ADC12ENC;
  ADC12MCTL0 = (inch & 0x0F) | ADC12VRSEL_1;    // VR+=Vref(1.2V), VR-=AVss
  ADC12CTL0 |= ADC12ENC | ADC12SC;              // start
  adc_wait_done();
  return ADC12MEM0;
}

/* --- public API --- */
void OC_Config(uint16_t trip_counts, uint16_t trip_ms, uint16_t clear_ms){
  TRIP_COUNTS = trip_counts ? trip_counts : TRIP_COUNTS;
  TRIP_MS     = trip_ms     ? trip_ms     : TRIP_MS;
  CLEAR_MS    = clear_ms    ? clear_ms    : CLEAR_MS;
}
void OC_SetCallbacks(void (*cut)(void), void (*resume)(void)){ cut_cb = cut; resume_cb = resume; }

void OC_Init_A4A5(void){
  /* Route P8.7 (A4) & P8.6 (A5) to analog */
  P8SEL1 |= (BIT7 | BIT6);
  P8SEL0 |= (BIT7 | BIT6);

  /* Internal 1.2 V reference on */
  PMMCTL0_H = PMMPW_H;
  REFCTL0   = REFON | REFVSEL_0;
  __delay_cycles(40000);

  /* ADC12_B basic single-channel config (we’ll switch INCH in software) */
  ADC12CTL0 = ADC12SHT0_2 | ADC12ON;     // 16 cyc sample, ADC on
  ADC12CTL1 = ADC12SHP;                  // sampling timer
  ADC12CTL2 = ADC12RES_2;                // 12-bit

  state = ST_NORMAL;
  over_ms = btn_ms = 0;
  adcA = adcB = 0;
}

void OC_1msTask(bool clear_button_pressed){
  /* Sample A4 then A5 (blocking). Each conversion ~20–30 µs at typical clocks. */
  adcA = adc_read_inch(4);   // A4 (P8.7)
  adcB = adc_read_inch(5);   // A5 (P8.6)
  bool over_now = (adcA > TRIP_COUNTS) || (adcB > TRIP_COUNTS);

  switch(state){
    case ST_NORMAL:
      if (over_now){ over_ms = 1; state = ST_OVER; }
      break;

    case ST_OVER:
      if (!over_now){ over_ms = 0; state = ST_NORMAL; }
      else if (++over_ms >= TRIP_MS){
        if (cut_cb) cut_cb();  // cut PWM
        state = ST_FAULT;
        btn_ms = 0;
      }
      break;

    case ST_FAULT:
      btn_ms = clear_button_pressed ? (btn_ms < 0xFFFF ? btn_ms + 1 : btn_ms) : 0;
      if (btn_ms >= CLEAR_MS){
        if (resume_cb) resume_cb();   // restore PWM
        btn_ms = over_ms = 0;
        state = ST_NORMAL;
      }
      break;
  }
}

bool     OC_IsFault(void){ return state == ST_FAULT; }
uint16_t OC_LatestAdcA(void){ return adcA; }
uint16_t OC_LatestAdcB(void){ return adcB; }
