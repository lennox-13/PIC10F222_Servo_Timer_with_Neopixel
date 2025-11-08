/* 
 PIC10F222 – Servo Timer with Battery Status (WS2812 LED)

 Function:
 - At power-up, sample internal 0.6 V reference (VBG) to estimate VDD and show battery status on WS2812 (GP2):
   green = high , Purple = medium, red = low.
 - Potentiometer on GP1/AN1 sets countdown time from 10.seconds to 13. minutes, then LED blinks during countdown.
 - Servo on GP0 goes from most left position to most right position after countdown end.
 - Button on GP3  starts countdown; pressing again returns to and move servo to most left position

 Control:
 1) Power on: LED shows battery color (based on single VBG read).
 2) Press button: read potentiometer, start countdown LED blinks.
 3) When time elapses: LED turns solid blue, servo switches to 2 ms. Press button to return to WAIT.

 Pins:
 - GP0: Servo signal (OUT)
 - GP1/AN1: Potentiometer wiper (IN)
 - GP2: WS2812 data (OUT)
 - GP3: Button input (IN, pull-up)

 Notes:
 - Battery level is read once at startup; timing is ratiometric (pot vs VDD), so it does not drift with battery voltage.
*/

#include <xc.h>

// CONFIGURATION (PIC10F222)
#pragma config IOSCFS = 8MHZ      // Internal oscillator frequency select 
#pragma config WDTE   = OFF       // Watchdog Timer disabled
#pragma config CP     = OFF       // Code protection disabled
#pragma config MCLRE  = OFF       // MCLR pin function is digital input 

// System clock frequency 
#define _XTAL_FREQ 8000000UL

// Internal ~0.6 V reference and battery voltage thresholds (single measurement at start)
// VBG_MV is the approximate internal band-gap voltage.
// VBAT_HIGH_MV / VBAT_LOW_MV define voltage thresholds for LED color decisions.
#define VBG_MV        600UL
#define VBAT_HIGH_MV  3600UL
#define VBAT_LOW_MV   3300UL
// COUNT_FROM_MV converts a target VDD into an ADC code when measuring VBG: code = 255 * VBG / VDD
#define COUNT_FROM_MV(vbat_mv) ((uint8_t)((VBG_MV*255UL + ((vbat_mv)/2)) / (vbat_mv)))
#define CT_HIGH  COUNT_FROM_MV(VBAT_HIGH_MV) // Lower ADC code (higher VDD)
#define CT_LOW   COUNT_FROM_MV(VBAT_LOW_MV)  // Higher ADC code (lower VDD)

// Servo pulses:
// SERVO_1MS: 1 ms HIGH then 19 ms LOW  -> ~20 ms period
// SERVO_2MS: 2 ms HIGH then 18 ms LOW  -> ~20 ms period
#define SERVO_1MS() do { GP0=1; __delay_us(1000); GP0=0; __delay_ms(19); } while(0)
#define SERVO_2MS() do { GP0=1; __delay_us(2000); GP0=0; __delay_ms(18); } while(0)

// WS2812 (NeoPixel) bit-bang driver on GP2.
static void WS2812_Send_Byte(uint8_t b) {
    for (uint8_t i = 8; i; i--) {
        if (b & 0x80) { asm("bsf GPIO,2\nnop\nbcf GPIO,2"); } // '1' bit waveform (longer HIGH)
        else          { asm("bsf GPIO,2\nbcf GPIO,2"); }      // '0' bit waveform (shorter HIGH)
        b <<= 1;
    }
}
static void WS2812_Send_RGB(uint8_t r, uint8_t g, uint8_t b) {
    WS2812_Send_Byte(r); WS2812_Send_Byte(g); WS2812_Send_Byte(b);
}
static void WS2812_Latch(void) { GP2 = 0; __delay_us(300); } // Reset/latch time

// LED color based on battery ADC code 'c':
// Lower 'c'    => higher VDD => green
// Middle range => purple
// Higher 'c'   => low battery => red
#define WS_SEND_COLOR_BY_C(cval) do{ \
    if ((cval) <= CT_HIGH)      { WS2812_Send_RGB(0,25,0); } \
    else if ((cval) <= CT_LOW)  { WS2812_Send_RGB(30,0,11); } \
    else                        { WS2812_Send_RGB(80,0,0);} \
    WS2812_Latch(); \
}while(0)
#define WS_SEND_OFF()  do{ WS2812_Send_RGB(0,0,0);  WS2812_Latch(); }while(0) // LED OFF macro
#define WS_SEND_BLUE() do{ WS2812_Send_RGB(0,0,40); WS2812_Latch(); }while(0) // Solid Blue Macro

// Blinking during countdown: toggle every ~500 ms (25 servo frames * ~20 ms)
#define BLINK_TOGGLE_FRAMES 25u

// State machine:
// STATE_WAIT      -> waiting for button press, showing battery color, servo at 1 ms
// STATE_COUNTDOWN -> blinking LED and counting down time, servo at 1 ms
// STATE_ACTIVE    -> final state, solid blue LED, servo at 2 ms pulse
typedef enum { STATE_WAIT=0, STATE_COUNTDOWN, STATE_ACTIVE } state_e;

void main(void)
{
    OSCCAL   = 0b00110000;  // Oscillator calibration 
    OPTION   = 0b11000000;  // Prescaler to WDT, internal pull-ups - disabled
    TRISGPIO = 0b00001010;  // GP3 IN (button), GP2 OUT (NeoPixel), GP1 IN (potentiometer), GP0 OUT (servo)
    GP0 = 1; // Servo idle high 
    GP2 = 0; // WS2812 data line low before first transmission
    
    // State variables 
    uint8_t  state = STATE_WAIT;
    uint16_t frames_left_total = 0u;
    uint8_t  btn_stable = 1u;          // Debounced button state (1 = not pressed due to pull-up)
    uint8_t  btn_cnt = 0u;             // Debounce counter
    uint8_t  blink_frames = BLINK_TOGGLE_FRAMES; // Initialize blink frame countdown (phase bit7 = 0 => start OFF)
    
    // Power stabilization loop after startup 
    // 60000 iterations * 8 cycles -> rough settling time for supply before ADC battery measurement.
    for (frames_left_total = 60000u; frames_left_total; frames_left_total--) {
        asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
        asm("nop"); asm("nop"); asm("nop"); asm("nop");
    }

    // Battery measurement using internal reference (CHS=10).
    ADCON0=0b00001001;             // ADCS=00 (Fosc/2), ANS1=0, ANS0=0 (digital pins), CHS=10 (VBG), GO=0, ADON=1.
    __delay_us(50);                // Acquisition delay
    ADCON0bits.GO_nDONE = 1;       // Start conversion
    __delay_us(50);                // Small delay for settling 
    while (ADCON0bits.GO_nDONE) { }// Wait for completion
    uint8_t c = ADRES;             // Battery voltage ratio code ≈ 255 * 0.6 / VDD

    // WS2812 reset 100 ms needed for Neopixel correct bootup 
    for (frames_left_total = 50000u; frames_left_total; frames_left_total--) {
        asm("nop"); asm("nop"); asm("nop"); asm("nop");
    }
    // Display initial battery color in WAIT state
    WS_SEND_COLOR_BY_C(c);

    while (1) {
        // Button debounce on GP3:
        // Stable-count require 5 consecutive differing samples to accept change.
        uint8_t btn_raw = (GPIO & 0x08) ? 1u : 0u; // Read GP3
        uint8_t btn_edge_fall = 0u;                // Set to 1 on detected falling edge (press)
        if (btn_raw != btn_stable) {
            if (++btn_cnt >= 5u) {                 // Debounce threshold
                uint8_t old = btn_stable;
                btn_stable = btn_raw;
                btn_cnt = 0u;
                if (old==1u && btn_stable==0u) btn_edge_fall = 1u; // Falling edge detection
            }
        } else {
            btn_cnt = 0u;                          // Reset counter if stable
        }

        switch (state) {
            case STATE_WAIT:
                if (btn_edge_fall) {
                    // Switch ADC to external AN1 (potentiometer) after internal reference was used.
                    // ADCS=10 (Fosc/32 slower clock), ANS1=0? (as coded), ANS0=0, CHS=01 (AN1), ADON=1.
                    ADCON0=0b10000101; // Switch from internal ref to external channel AN1
                    __delay_us(50);
                    ADCON0bits.GO_nDONE = 1;
                    __delay_us(50);
                    while (ADCON0bits.GO_nDONE) { }

                    // Store ADC result (0-255) into blink_frames (reuse 8-bit var to save RAM).
                    blink_frames = ADRES;

                    // Time mapping: frames_left_total = 500 + 155*ADC (implemented via shifts).
                    frames_left_total  = 512u;  // Base = 500 (512 - 8 - 4)
                    frames_left_total -= 8u;
                    frames_left_total -= 4u;
                    frames_left_total += (uint16_t)blink_frames << 7; // *128
                    frames_left_total += (uint16_t)blink_frames << 4; // *16
                    frames_left_total += (uint16_t)blink_frames << 3; // *8
                    frames_left_total += (uint16_t)blink_frames << 1; // *2
                    frames_left_total += (uint16_t)blink_frames;      // *1

                    // If zero (won't be, min is 500), go directly ACTIVE, else start countdown.
                    if (frames_left_total == 0u) {
                        WS_SEND_BLUE();
                        state = STATE_ACTIVE;
                    } else {
                        WS_SEND_OFF(); // Start with LED off before blinking
                        blink_frames = (uint8_t)(0x80u | BLINK_TOGGLE_FRAMES); // Set phase bit7=1 (next ON) + reload countdown
                        state = STATE_COUNTDOWN;
                    }
                }
                SERVO_1MS(); // Maintain 1 ms servo pulse in WAIT
                break;

            case STATE_COUNTDOWN: {
                SERVO_1MS(); // Servo continues 1 ms pulse during countdown

                // Blink handler: decrement internal blink frame counter; toggle phase when it reaches 0.
                uint8_t t = (uint8_t)(blink_frames & 0x7Fu); // Lower 7 bits = remaining frames in current blink phase
                if (t) {
                    t--;
                    blink_frames = (uint8_t)((blink_frames & 0x80u) | t); // Preserve phase bit7
                } else {
                    // Time to switch LED state (color vs OFF) based on phase bit7
                    if (blink_frames & 0x80u) WS_SEND_COLOR_BY_C(c);
                    else                      WS_SEND_OFF();
                    // Flip phase bit and reload blink countdown
                    blink_frames = (uint8_t)(((blink_frames ^ 0x80u) & 0x80u) | BLINK_TOGGLE_FRAMES);
                }

                // Main time countdown (one decrement per ~20 ms frame)
                if (frames_left_total) {
                    frames_left_total--;
                    if (frames_left_total == 0u) {
                        WS_SEND_BLUE(); // Transition to ACTIVE state when finished
                        state = STATE_ACTIVE;
                    }
                }
            } break;

            case STATE_ACTIVE:
                // Active state: LED solid blue, servo pulses change to 2 ms.
                // Button press returns to WAIT and restores battery color.
                if (btn_edge_fall) {
                    WS_SEND_COLOR_BY_C(c); // Restore battery status color
                    state = STATE_WAIT;
                }
                SERVO_2MS(); // 2 ms servo pulse while active
                break;
        }
    }
}