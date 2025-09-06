/*
THE PLAGUE INTERPRETER
A cellular automata based sound synthesizer

This program implements various cellular automata algorithms to create experimental
sound synthesis. It uses an ATMega128 microcontroller with the following features:

Hardware Setup:
- 3 Potentiometers for control:
  * Left: CPU step & instruction set selection
  * Middle: Hardware routing & filter configuration
  * Right: Plague step & process selection, filter modulation
- Audio output via PWM (Pin D6)
- Filter clock output (Pin B1)
- Routing switches for audio path configuration

Core Concepts:
1. Cell Space:
   - 256 byte array representing the cellular automata grid
   - Can be divided into two 128-byte spaces for double-buffering

2. Instruction Sets:
   - First: Basic operations (26 instructions)
   - Plague: Infection spreading algorithms (8 instructions)
   - Brainfuck: Minimalist instruction set (9 instructions)
   - SIR: Susceptible-Infected-Recovered model (6 instructions)
   - Redcode: Core War instruction set (11 instructions)
   - Biota: 2D grid movement operations (10 instructions)
   - Red Death: Special plague spreading algorithms (7 instructions)

3. Plague Functions:
   Different cellular automata algorithms that modify the cell space:
   - Mutate: Random mutations based on ADC input
   - SIR: Epidemic spreading model
   - Hodge: Complex neighbor-based evolution
   - Cel: Rule-based cell state changes
   - Life: Conway's Game of Life variant

4. Hardware Control:
   - Filter frequency modulation via PWM
   - Audio signal routing through different paths
   - Feedback loop configuration
   - Multiple clock divider settings

Based on original work by microresearch (http://1010.co.uk/)
Modified and documented by circuitnoise
*/

#define DEBUG 0
#define F_CPU 16000000UL

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
// #include <math.h>            // entfällt (kein floor() mehr)
#include <stdbool.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/atomic.h>

/* --- Consistent 16x16 Layout ----------------------------------------- */
#define GRID_W 16
#define GRID_H 16
#define CELLLEN GRID_W
#define CELLS_LEN (GRID_W * GRID_H) /* 256: free 8-bit wraps */

/* Compile-time Guard (if preprocessor available) */
#if (GRID_W != 16) || (GRID_H != 16) || (CELLLEN != 16) || (CELLS_LEN != 256)
#error "GRID/CELLLEN/CELLS_LEN must be 16/16/16/256 for 8-bit wrapping semantics."
#endif

/* --- Constants/Macros -------------------------------------------------- */

#define MAX_SAM 255 // Maximum number of samples (historical)
#define BV(bit) (1 << (bit))
#define cbi(reg, bit) reg &= ~(BV(bit))
#define sbi(reg, bit) reg |= (BV(bit))
#define HEX__(n) 0x##n##UL
#define B8__(x) ((x & 0x0000000FLU) ? 1 : 0) + ((x & 0x000000F0LU) ? 2 : 0) + ((x & 0x00000F00LU) ? 4 : 0) + ((x & 0x0000F000LU) ? 8 : 0) + ((x & 0x000F0000LU) ? 16 : 0) + ((x & 0x00F00000LU) ? 32 : 0) + ((x & 0x0F000000LU) ? 64 : 0) + ((x & 0xF0000000LU) ? 128 : 0)
#define B8(d) ((unsigned char)B8__(HEX__(d)))
#define low(port, pin) (port &= ~_BV(pin))
#define high(port, pin) (port |= _BV(pin))
#define PI 3.1415926535897932384626433832795
#define BET(A, B, C) (((A >= B) && (A <= C)) ? 1 : 0) /* a between [b,c] */
#define NSTEPS 10000
#define recovered 129
#define dead 255
#define susceptible 0
#define tau 2

/* --- Global Variables -------------------------------------------------- */
int8_t insdir = 1, dir = 1; /* signed! */
uint8_t filterk = 0, cpu = 0, plague = 0, step = 0;
uint8_t hardk = 0, fhk = 0, instruction = 0;
uint8_t instructionp = 0, IP = 0, controls = 0;
uint8_t hardware = 0, samp = 0, count = 0, qqq = 0;
uint8_t btdir = 0, dcdir = 0;
uint8_t clock = 0;
static bool insdir_modified = false;

int8_t cycle = -1; // signiert!
uint8_t ostack[20];

/* Complete cell memory: 256 */
static uint8_t cells_buf[CELLS_LEN];

uint8_t stack[20];
static uint8_t omem; /* remains 8-bit: Mod 256 free */

static uint8_t last_cpu = 0xFF; // impossible start value => first run triggers optional

/* --- Safe-Index + Wrap Helper Functions --------------------------------- */
static inline uint16_t wrap_u16(int32_t x, uint16_t mod)
{
  int32_t r = x % mod;
  return (uint16_t)(r < 0 ? r + mod : r);
}
#define SAFE_IDX(i) (wrap_u16((int32_t)(i), CELLS_LEN))

static inline uint8_t CGET(const uint8_t *c, int32_t i) { return c[SAFE_IDX(i)]; }
static inline void CSET(uint8_t *c, int32_t i, uint8_t v) { c[SAFE_IDX(i)] = v; }

static inline uint8_t clamp_filterk(uint8_t k) { return (k > 8) ? 8 : k; }

/* --- Schnelle 8-Bit-Wrap-Helper für IP-Nachbarn ------------------------- */
static inline uint8_t add_u8(uint8_t base, int16_t delta) { return (uint8_t)(base + delta); }
#define IP_LEFT(ip) (add_u8((ip), -1))
#define IP_RIGHT(ip) (add_u8((ip), +1))

/* --- 2D Helper Functions for 16x16 grid -------------------------------- */
static inline uint8_t idx2d(int x, int y)
{
  x = (x % GRID_W + GRID_W) % GRID_W;
  y = (y % GRID_H + GRID_H) % GRID_H;
  return (uint8_t)(y * GRID_W + x);
}
static inline uint8_t omem_move(uint8_t om, int dx, int dy)
{
  int x = om % GRID_W, y = om / GRID_W;
  x = (x + dx + GRID_W) % GRID_W;
  y = (y + dy + GRID_H) % GRID_H;
  return (uint8_t)(y * GRID_W + x);
}

/* --- omem increment/decrement (Mod 256 free) --------------------------- */
static inline void omem_inc(void) { omem = (uint8_t)(omem + 1); }
static inline void omem_dec(void) { omem = (uint8_t)(omem - 1); }

/* ---------------------------------------------------------------------- */
/* ADC                                                                    */
/* ---------------------------------------------------------------------- */

/*
Initialize Analog Digital Converter (ADC)
- Sets reference voltage to AVCC
- Configures for 8-bit left-adjusted results
- Enables ADC with 128 prescaler
- Sets up Port C for analog inputs
*/
void adc_init(void)
{
  // Reference voltage: AVCC
  cbi(ADMUX, REFS1);
  sbi(ADMUX, REFS0);

  // Result left-justified -> ADCH contains upper 8 bits
  sbi(ADMUX, ADLAR);

  // ADC Prescaler = 128 (16 MHz / 128 = 125 kHz)
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  // Enable ADC
  sbi(ADCSRA, ADEN);

  // Set PortC as input (for ADC channels)
  DDRC = 0x00;
  PORTC = 0x00;
}

/*
Read ADC value from specified channel
Parameters:
- channel: ADC input channel (0-7)
Returns: 8-bit conversion result
*/
uint8_t adcread(uint8_t channel)
{
  if (!(ADCSRA & (1 << ADEN)) || channel > 7)
    return 0;

  // Kanal setzen (REFSx/ADLAR bleiben erhalten)
  ADMUX = (ADMUX & 0xF8) | (channel & 0x07);

  // ADIF vor Messablauf löschen (write-1-to-clear)
  ADCSRA |= (1 << ADIF);

  // Dummy conversion after MUX change (discard first sample)
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
  {
  }
  (void)ADCH; // Discard dummy value

  // Actual measurement
  ADCSRA |= (1 << ADSC);
  uint16_t timeout = 10000;
  while ((ADCSRA & (1 << ADSC)) && --timeout)
  {
  }
  if (!timeout)
    return 0;

  // ADIF nach Abschluss löschen (optional, aber sauber)
  ADCSRA |= (1 << ADIF);

  // left-justified -> upper byte
  return ADCH;
}

/*
Create array of values from output signal (adcread(3)) as sample storage
→ Now: initialize all 256 cells
*/
void initcell(uint8_t *cells)
{
  for (uint16_t x = 0; x < CELLS_LEN; x++)
  {
    cells[x] = adcread(3); // get output signal
  }
}

/* ---------------------------------------------------------------------- */
/* Filter Functions                                                      */
/* ---------------------------------------------------------------------- */

void leftsh(unsigned int cel)
{
  uint8_t k = clamp_filterk(filterk);
  OCR1A = cel << k;
}
void rightsh(unsigned int cel)
{
  uint8_t k = clamp_filterk(filterk);
  OCR1A = cel >> k;
}
void mult(unsigned int cel)
{
  uint8_t k = clamp_filterk(filterk);
  OCR1A = cel * k;
}
void divvv(unsigned int cel)
{
  uint8_t k = clamp_filterk(filterk);
  OCR1A = cel / (k + 1);
}

/* Pointer to filter functions */
void (*filtermod[])(unsigned int cel) = {leftsh, rightsh, mult, divvv};

/* ---------------------------------------------------------------------- */
/* instructionsetfirst                                                    */
/* ---------------------------------------------------------------------- */

uint8_t outff(uint8_t *cells, uint8_t IP)
{
  (*filtermod[qqq])((int)cells[omem]);
  return (uint8_t)(IP + insdir);
}

uint8_t outpp(uint8_t *cells, uint8_t IP)
{
  OCR0A = omem;
  return (uint8_t)(IP + insdir);
}

uint8_t finc(uint8_t *cells, uint8_t IP)
{
  omem_inc();
  return (uint8_t)(IP + insdir);
}

uint8_t fdec(uint8_t *cells, uint8_t IP)
{
  omem_dec();
  return (uint8_t)(IP + insdir);
}

uint8_t fincm(uint8_t *cells, uint8_t IP)
{
  cells[omem]++;
  return (uint8_t)(IP + insdir);
}

uint8_t fdecm(uint8_t *cells, uint8_t IP)
{
  cells[omem]--;
  return (uint8_t)(IP + insdir);
}

/* get omem from Output*/
uint8_t fin1(uint8_t *cells, uint8_t IP)
{
  omem = adcread(3); // get output signal
  return (uint8_t)(IP + insdir);
}

/* get omem from Poti 3 */
uint8_t fin2(uint8_t *cells, uint8_t IP)
{
  omem = adcread(2);
  return (uint8_t)(IP + insdir);
}
/* get IP from Poti 3 */
uint8_t fin3(uint8_t *cells, uint8_t IP)
{
  IP = adcread(2);
  return (uint8_t)(IP + insdir);
}
/**/
uint8_t fin4(uint8_t *cells, uint8_t IP)
{
  if (omem < CELLS_LEN)
  {
    cells[omem] = adcread(3); // get output signal
  }
  return (uint8_t)(IP + insdir);
}

uint8_t outf(uint8_t *cells, uint8_t IP)
{
  (*filtermod[qqq])((int)cells[omem]);
  return (uint8_t)(IP + insdir);
}

uint8_t outp(uint8_t *cells, uint8_t IP)
{
  OCR0A = cells[omem];
  return (uint8_t)(IP + insdir);
}

uint8_t plus(uint8_t *cells, uint8_t IP)
{
  CSET(cells, IP, (uint8_t)(CGET(cells, IP) + 1));
  return (uint8_t)(IP + insdir);
}

uint8_t minus(uint8_t *cells, uint8_t IP)
{
  CSET(cells, IP, (uint8_t)(CGET(cells, IP) - 1));
  return (uint8_t)(IP + insdir);
}

uint8_t bitshift1(uint8_t *cells, uint8_t IP)
{
  CSET(cells, IP, (uint8_t)(CGET(cells, IP) << 1));
  return (uint8_t)(IP + insdir);
}

uint8_t bitshift2(uint8_t *cells, uint8_t IP)
{
  CSET(cells, IP, (uint8_t)(CGET(cells, IP) << 2));
  return (uint8_t)(IP + insdir);
}

uint8_t bitshift3(uint8_t *cells, uint8_t IP)
{
  CSET(cells, IP, (uint8_t)(CGET(cells, IP) << 3));
  return (uint8_t)(IP + insdir);
}

uint8_t branch(uint8_t *cells, uint8_t IP)
{
  if (CGET(cells, IP_RIGHT(IP)) == 0)
    IP = CGET(cells, omem);
  return (uint8_t)(IP + insdir);
}

uint8_t jump(uint8_t *cells, uint8_t IP)
{
  uint8_t off = CGET(cells, IP_RIGHT(IP));
  if (off < 128)
    return (uint8_t)(IP + off);
  else
    return (uint8_t)(IP + insdir);
}

uint8_t infect(uint8_t *cells, uint8_t IP)
{
  uint8_t left = IP_LEFT(IP);
  if (CGET(cells, left) < 128)
    CSET(cells, IP_RIGHT(IP), CGET(cells, IP));
  return (uint8_t)(IP + insdir);
}

uint8_t store(uint8_t *cells, uint8_t IP)
{
  uint8_t addr = CGET(cells, IP_RIGHT(IP));
  CSET(cells, IP, CGET(cells, addr));
  return (uint8_t)(IP + insdir);
}

uint8_t writeknob(uint8_t *cells, uint8_t IP)
{
  CSET(cells, IP, adcread(2));
  return (uint8_t)(IP + insdir);
}

uint8_t writesamp(uint8_t *cells, uint8_t IP)
{
  CSET(cells, IP, adcread(3)); // get output signal
  return (uint8_t)(IP + insdir);
}

uint8_t skip(uint8_t *cells, uint8_t IP)
{
  return (uint8_t)(IP + insdir);
}

// Sets direction
uint8_t direction(uint8_t *cells, uint8_t IP)
{
  if (dir < 0)
    dir = 1;
  else
    dir = -1;
  return (uint8_t)(IP + insdir);
}

// do nothing
uint8_t die(uint8_t *cells, uint8_t IP)
{
  return (uint8_t)(IP + insdir);
}

/* ---------------------------------------------------------------------- */
/* instructionsetplague                                                   */
/* ---------------------------------------------------------------------- */

uint8_t ploutf(uint8_t *cells, uint8_t IP)
{
  (*filtermod[qqq])((int)cells[omem]);
  return (uint8_t)(IP + insdir);
}

uint8_t ploutp(uint8_t *cells, uint8_t IP)
{
  uint8_t a = CGET(cells, (int32_t)IP + 1);
  uint8_t b = CGET(cells, (int32_t)IP - 1);
  OCR0A = (uint8_t)(a + b);
  return (uint8_t)(IP + insdir);
}

uint8_t plenclose(uint8_t *cells, uint8_t IP)
{
  CSET(cells, IP, 255);
  CSET(cells, (int32_t)IP + 1, 255);
  return (uint8_t)(IP + 2);
}

uint8_t plinfect(uint8_t *cells, uint8_t IP)
{
  uint8_t cur = CGET(cells, IP);
  if (cur < 128)
  {
    CSET(cells, (int32_t)IP + 1, cur);
    CSET(cells, (int32_t)IP - 1, cur);
  }
  return (uint8_t)(IP + insdir);
}

uint8_t pldie(uint8_t *cells, uint8_t IP)
{
  CSET(cells, (int32_t)IP - 1, 0);
  CSET(cells, (int32_t)IP + 1, 0);
  return (uint8_t)(IP + insdir);
}

uint8_t plwalk(uint8_t *cells, uint8_t IP)
{
  if (dir < 0 && (CGET(cells, IP) & 0x03) == 1)
    dir = +1;
  else if (dir > 0 && (CGET(cells, IP) & 0x03) == 0)
    dir = -1;
  else
  {
    insdir = (dir >= 0 ? +1 : -1) * (CGET(cells, IP) / 16);
    if (insdir == 0)
      insdir = dir;
    insdir_modified = true;
  }
  return (uint8_t)(IP + insdir);
}

/* ---------------------------------------------------------------------- */
/* instructionsetbf                                                       */
/* ---------------------------------------------------------------------- */

uint8_t bfinc(uint8_t *cells, uint8_t IP)
{
  omem_inc();
  return ++IP;
}

uint8_t bfdec(uint8_t *cells, uint8_t IP)
{
  omem_dec();
  return ++IP;
}

uint8_t bfincm(uint8_t *cells, uint8_t IP)
{
  cells[omem]++;
  return ++IP;
}

uint8_t bfdecm(uint8_t *cells, uint8_t IP)
{
  cells[omem]--;
  return ++IP;
}

uint8_t bfoutf(uint8_t *cells, uint8_t IP)
{
  (*filtermod[qqq])((int)cells[omem]);
  return ++IP;
}

uint8_t bfoutp(uint8_t *cells, uint8_t IP)
{
  OCR0A = cells[omem];
  return ++IP;
}

uint8_t bfin(uint8_t *cells, uint8_t IP)
{
  if (omem < CELLS_LEN)
  {
    cells[omem] = adcread(3); // get output signal
  }
  return ++IP;
}

uint8_t bfbrac1(uint8_t *cells, uint8_t IP)
{
  if (cycle < 19)
  {
    cycle++;
    ostack[cycle] = IP;
  }
  return ++IP;
}
uint8_t bfbrac2(uint8_t *cells, uint8_t IP)
{
  if (cycle >= 0 && cells[omem] != 0)
    return (uint8_t)(ostack[cycle]);
  if (cycle >= 0)
    cycle--;
  return ++IP;
}

/* ---------------------------------------------------------------------- */
/* instructionsetSIR                                                      */
/* ---------------------------------------------------------------------- */

uint8_t SIRoutf(uint8_t *cells, uint8_t IP)
{
  (*filtermod[qqq])((int)CGET(cells, (int32_t)IP + 1) + (int)CGET(cells, (int32_t)IP - 1));
  return (uint8_t)(IP + insdir);
}

uint8_t SIRoutp(uint8_t *cells, uint8_t IP)
{
  OCR0A = (uint8_t)(CGET(cells, (int32_t)IP + 1) + CGET(cells, (int32_t)IP - 1));
  return (uint8_t)(IP + insdir);
}

/* --------- Leichtgewichtiger PRNG (ersetzt rand()) --------- */
static uint8_t lfsr = 0xA5;
static inline uint8_t prng8(void)
{
  uint8_t x = lfsr;
  x ^= (uint8_t)(x << 3);
  x ^= (uint8_t)(x >> 5);
  x ^= (uint8_t)(x << 1);
  lfsr = x;
  return x;
}

uint8_t SIRincif(uint8_t *cells, uint8_t IP)
{
  if ((CGET(cells, (int32_t)IP + 1) > 0 && CGET(cells, (int32_t)IP + 1) < 128))
    CSET(cells, IP, (uint8_t)(CGET(cells, IP) + 1));
  return (uint8_t)(IP + insdir);
}

uint8_t SIRdieif(uint8_t *cells, uint8_t IP)
{
  if ((CGET(cells, (int32_t)IP + 1) > 0 && CGET(cells, (int32_t)IP + 1) < 128))
  {
    if ((prng8() % 10) < 4)
      CSET(cells, IP, dead);
  }
  return (uint8_t)(IP + insdir);
}

uint8_t SIRrecif(uint8_t *cells, uint8_t IP)
{
  if (CGET(cells, (int32_t)IP + 1) >= 128)
    CSET(cells, IP, recovered);
  return (uint8_t)(IP + insdir);
}

uint8_t SIRinfif(uint8_t *cells, uint8_t IP)
{
  if (CGET(cells, (int32_t)IP + 1) == 0)
  {
    if ((CGET(cells, (int32_t)IP - 1) > 0 && CGET(cells, (int32_t)IP - 1) < 128) ||
        (CGET(cells, (int32_t)IP + 1) > 0 && CGET(cells, (int32_t)IP + 1) < 128))
    {
      if ((prng8() % 10) < 4)
        CSET(cells, IP, 1);
    }
  }
  return (uint8_t)(IP + insdir);
}

/* ---------------------------------------------------------------------- */
/* instructionsetredcode                                                  */
/* ---------------------------------------------------------------------- */

uint8_t rdmov(uint8_t *cells, uint8_t IP)
{
  uint8_t off1 = CGET(cells, IP + 1);
  uint8_t off2 = CGET(cells, IP + 2);
  uint8_t src = CGET(cells, (int32_t)IP + off1);
  CSET(cells, (int32_t)IP + off2, src);
  return (uint8_t)(IP + 3);
}

uint8_t rdadd(uint8_t *cells, uint8_t IP)
{
  uint8_t off1 = CGET(cells, IP + 1);
  uint8_t off2 = CGET(cells, IP + 2);
  uint8_t dstv = CGET(cells, (int32_t)IP + off2);
  uint8_t srcv = CGET(cells, (int32_t)IP + off1);
  CSET(cells, (int32_t)IP + off2, (uint8_t)(dstv + srcv));
  return (uint8_t)(IP + 3);
}

uint8_t rdsub(uint8_t *cells, uint8_t IP)
{
  uint8_t off1 = CGET(cells, IP + 1);
  uint8_t off2 = CGET(cells, IP + 2);
  uint8_t dstv = CGET(cells, (int32_t)IP + off2);
  uint8_t srcv = CGET(cells, (int32_t)IP + off1);
  CSET(cells, (int32_t)IP + off2, (uint8_t)(dstv - srcv));
  return (uint8_t)(IP + 3);
}

uint8_t rdjmp(uint8_t *cells, uint8_t IP)
{
  uint8_t off = CGET(cells, IP + 1);
  return (uint8_t)(IP + off);
}

uint8_t rdjmz(uint8_t *cells, uint8_t IP)
{
  uint8_t off2 = CGET(cells, IP + 2);
  if (CGET(cells, (int32_t)IP + off2) == 0)
    return CGET(cells, IP + 1);
  else
    return (uint8_t)(IP + 3);
}

uint8_t rdjmg(uint8_t *cells, uint8_t IP)
{
  uint8_t off2 = CGET(cells, IP + 2);
  if (CGET(cells, (int32_t)IP + off2) > 0)
    return CGET(cells, IP + 1);
  else
    return (uint8_t)(IP + 3);
}

uint8_t rddjz(uint8_t *cells, uint8_t IP)
{
  uint8_t off2 = CGET(cells, IP + 2);
  int32_t x = (int32_t)IP + off2;
  uint8_t xv = CGET(cells, x);
  xv = (uint8_t)(xv - 1);
  CSET(cells, x, xv);
  if (xv == 0)
    return CGET(cells, IP + 1);
  else
    return (uint8_t)(IP + 3);
}

uint8_t rddat(uint8_t *cells, uint8_t IP)
{
  IP += 3;
  return IP;
}

uint8_t rdcmp(uint8_t *cells, uint8_t IP)
{
  uint8_t off1 = CGET(cells, IP + 1);
  uint8_t off2 = CGET(cells, IP + 2);
  if (CGET(cells, (int32_t)IP + off2) != CGET(cells, (int32_t)IP + off1))
    return (uint8_t)(IP + 6);
  else
    return (uint8_t)(IP + 3);
}

uint8_t rdoutf(uint8_t *cells, uint8_t IP)
{
  (*filtermod[qqq])((int)CGET(cells, IP + 1));
  return (uint8_t)(IP + 3);
}

uint8_t rdoutp(uint8_t *cells, uint8_t IP)
{
  OCR0A = CGET(cells, IP + 2);
  return (uint8_t)(IP + 3);
}

/* ---------------------------------------------------------------------- */
/* instructionsetbiota                                                    */
/* ---------------------------------------------------------------------- */

uint8_t btempty(uint8_t *cells, uint8_t IP)
{
  // turn around
  if (btdir == 0)
    btdir = 1;
  else if (btdir == 1)
    btdir = 0;
  else if (btdir == 2)
    btdir = 3;
  else if (btdir == 3)
    btdir = 2;
  return IP;
}

uint8_t btoutf(uint8_t *cells, uint8_t IP)
{
  (*filtermod[qqq])((int)cells[omem]);
  return IP;
}

uint8_t btoutp(uint8_t *cells, uint8_t IP)
{
  OCR0A = cells[omem];
  return IP;
}

uint8_t btstraight(uint8_t *cells, uint8_t IP)
{
  if (dcdir == 0)
    omem = omem_move(omem, +1, 0);
  else if (dcdir == 1)
    omem = omem_move(omem, -1, 0);
  else if (dcdir == 2)
    omem = omem_move(omem, 0, +1);
  else if (dcdir == 3)
    omem = omem_move(omem, 0, -1);

  if (cells[omem] == 0)
  { // change dir
    if (btdir == 0)
      btdir = 1;
    else if (btdir == 1)
      btdir = 0;
    else if (btdir == 2)
      btdir = 3;
    else if (btdir == 3)
      btdir = 2;
  }
  return IP;
}

uint8_t btbackup(uint8_t *cells, uint8_t IP)
{
  if (dcdir == 0)
    omem = omem_move(omem, -1, 0);
  else if (dcdir == 1)
    omem = omem_move(omem, +1, 0);
  else if (dcdir == 2)
    omem = omem_move(omem, 0, -1);
  else if (dcdir == 3)
    omem = omem_move(omem, 0, +1);
  if (cells[omem] == 0)
  {
    if (btdir == 0)
      btdir = 1;
    else if (btdir == 1)
      btdir = 0;
    else if (btdir == 2)
      btdir = 3;
    else if (btdir == 3)
      btdir = 2;
  }
  return IP;
}

uint8_t btturn(uint8_t *cells, uint8_t IP)
{
  if (dcdir == 0)
    omem = omem_move(omem, 0, +1);
  else if (dcdir == 1)
    omem = omem_move(omem, 0, -1);
  else if (dcdir == 2)
    omem = omem_move(omem, +1, 0);
  else if (dcdir == 3)
    omem = omem_move(omem, -1, 0);
  return IP;
}

uint8_t btunturn(uint8_t *cells, uint8_t IP)
{
  if (dcdir == 0)
    omem = omem_move(omem, 0, -1);
  else if (dcdir == 1)
    omem = omem_move(omem, 0, +1);
  else if (dcdir == 2)
    omem = omem_move(omem, -1, 0);
  else if (dcdir == 3)
    omem = omem_move(omem, +1, 0);
  return IP;
}

uint8_t btg(uint8_t *cells, uint8_t IP)
{
  uint8_t x = 0;
  while (x < 20 && cells[omem] != 0)
  {
    if (dcdir == 0)
      omem = omem_move(omem, +1, 0);
    else if (dcdir == 1)
      omem = omem_move(omem, -1, 0);
    else if (dcdir == 2)
      omem = omem_move(omem, 0, +1);
    else if (dcdir == 3)
      omem = omem_move(omem, 0, -1);
    x++;
  }
  return IP;
}

uint8_t btclear(uint8_t *cells, uint8_t IP)
{
  if (cells[omem] == 0)
  {
    if (btdir == 0)
      btdir = 1;
    else if (btdir == 1)
      btdir = 0;
    else if (btdir == 2)
      btdir = 3;
    else if (btdir == 3)
      btdir = 2;
  }
  else
    CSET(cells, omem, 0);
  return IP;
}

uint8_t btdup(uint8_t *cells, uint8_t IP)
{
  if (cells[omem] == 0 || CGET(cells, (int32_t)omem - 1) != 0)
  {
    if (btdir == 0)
      btdir = 1;
    else if (btdir == 1)
      btdir = 0;
    else if (btdir == 2)
      btdir = 3;
    else if (btdir == 3)
      btdir = 2;
  }
  else
    CSET(cells, (int32_t)omem - 1, cells[omem]);
  return IP;
}

/* ---------------------------------------------------------------------- */
/* instructionsetreddeath                                                 */
/* ---------------------------------------------------------------------- */

uint8_t redplague(uint8_t *cells, uint8_t IP)
{
  if (clock == 12)
  {
    clock = 12;
    CSET(cells, IP_RIGHT(IP), CGET(cells, IP));
    if (IP == 255)
      clock = 13;
    return (uint8_t)(IP + 1);
  }
  else
    return (uint8_t)(IP + insdir);
}

uint8_t reddeath(uint8_t *cells, uint8_t IP)
{
  if (clock == 13)
  {
    clock = 13;
    count = (uint8_t)((count + 1) % CELLS_LEN);
    CSET(cells, (int32_t)IP + count, adcread(3)); // get output signal
    return IP;                                    // just keeps on going
  }
  else
    return (uint8_t)(IP + insdir);
}

uint8_t redclock(uint8_t *cells, uint8_t IP)
{
  clock++;
  if (clock % 60 == 0)
  {
    OCR0A ^= 255;
    return IP; // everyone stops
  }
  else
    return (uint8_t)(IP + insdir);
}

uint8_t redrooms(uint8_t *cells, uint8_t IP)
{
  switch (IP % 7)
  {
  case 0:
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
    filterk = 8;
    break;
  case 1:
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
    break;
  case 2:
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS11); // divide by 8
    filterk = 8;
    break;
  case 3:
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS11); // divide by 8
    break;
  case 4:
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // divide by 64
    break;
  case 5:
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS12); // 256
    break;
  case 6:
    cbi(DDRB, PORTB1); // filter off
  }
  return (uint8_t)(IP + insdir);
}

uint8_t redunmask(uint8_t *cells, uint8_t IP)
{
  uint8_t vL = CGET(cells, (int32_t)IP - 1) ^ 255;
  uint8_t vR = CGET(cells, (int32_t)IP + 1) ^ 255;
  CSET(cells, (int32_t)IP - 1, vL);
  CSET(cells, (int32_t)IP + 1, vR);
  return (uint8_t)(IP + insdir);
}

uint8_t redprospero(uint8_t *cells, uint8_t IP)
{
  uint8_t dirrr = adcread(3) % 4; // get output signal
  if (dirrr == 0)
    omem = omem_move(omem, +1, 0);
  else if (dirrr == 1)
    omem = omem_move(omem, -1, 0);
  else if (dirrr == 2)
    omem = omem_move(omem, 0, +1);
  else if (dirrr == 3)
    omem = omem_move(omem, 0, -1);

  OCR0A = cells[omem];
  return (uint8_t)(IP + insdir);
}

uint8_t redoutside(uint8_t *cells, uint8_t IP)
{
  CSET(cells, (int32_t)omem + 1, adcread(3)); // get output signal
  (*filtermod[qqq])((int)cells[omem]);
  return (uint8_t)(IP + insdir);
}

/* ---------------------------------------------------------------------- */
/* Plague Function Group (Algorithmen)                                    */
/* ---------------------------------------------------------------------- */

/*
  Plaque Mutate changes cell values to values from Filter Output until cells[0] value is reached
*/
void mutate(uint8_t *cells)
{
  uint8_t x, y;
  for (y = 0; y < cells[0]; y++)
  {
    x = adcread(3);         // Read output signal
    cells[x] ^= (x & 0x0f); // 0b00001111
  }
}

/*
  Plague Hodge Implementation
  - nutzt jetzt CELLS_LEN/2 (=128) als Halbraum
  - sichere Grenzen (CoreCellx: 17..110)
  - keine Floats / floor()
*/
void hodge(uint8_t *cellies)
{
  int sum = 0, numill = 0, numinf = 0;
  uint8_t q, k1, k2, g;
  static uint8_t CoreCellx = CELLLEN + 1; // 17..110
  static uint8_t flag = 0;                // Toggle Flag
  static uint8_t *newcells, *cells, *swap;

  const uint16_t HALF = (CELLS_LEN / 2); // 128

  // Swap wo die Daten liegen
  if ((flag & 0x01) == 0)
  {
    cells = cellies;
    newcells = &cellies[HALF];
  }
  else
  {
    cells = &cellies[HALF];
    newcells = cellies;
  }

  // Referenzen
  q = cells[0];
  k1 = cells[1];
  if (k1 == 0)
    k1 = 1;
  k2 = cells[2];
  if (k2 == 0)
    k2 = 1;
  g = cells[3];

  // Neighbors (CoreCellx in safe range 17..110)
  sum = cells[CoreCellx] + cells[CoreCellx - 1] + cells[CoreCellx + 1] + cells[CoreCellx - CELLLEN] + cells[CoreCellx + CELLLEN] + cells[CoreCellx - CELLLEN - 1] + cells[CoreCellx - CELLLEN + 1] + cells[CoreCellx + CELLLEN - 1] + cells[CoreCellx + CELLLEN + 1];

  // Counters
  numill = 0;
  numinf = 0;
  {
    uint16_t idx = CoreCellx - 1;
    if (cells[idx] == (uint8_t)(q - 1))
      numill++;
    else if (cells[idx] > 0)
      numinf++;
  }
  {
    uint16_t idx = CoreCellx + 1;
    if (cells[idx] == (uint8_t)(q - 1))
      numill++;
    else if (cells[idx] > 0)
      numinf++;
  }
  {
    uint16_t idx = CoreCellx - CELLLEN;
    if (cells[idx] == (uint8_t)(q - 1))
      numill++;
    else if (cells[idx] > 0)
      numinf++;
  }
  {
    uint16_t idx = CoreCellx + CELLLEN;
    if (cells[idx] == (uint8_t)(q - 1))
      numill++;
    else if (cells[idx] > 0)
      numinf++;
  }
  {
    uint16_t idx = CoreCellx - CELLLEN - 1;
    if (cells[idx] == q)
      numill++;
    else if (cells[idx] > 0)
      numinf++;
  }
  {
    uint16_t idx = CoreCellx - CELLLEN + 1;
    if (cells[idx] == q)
      numill++;
    else if (cells[idx] > 0)
      numinf++;
  }
  {
    uint16_t idx = CoreCellx + CELLLEN - 1;
    if (cells[idx] == q)
      numill++;
    else if (cells[idx] > 0)
      numinf++;
  }
  {
    uint16_t idx = CoreCellx + CELLLEN + 1;
    if (cells[idx] == q)
      numill++;
    else if (cells[idx] > 0)
      numinf++;
  }

  // Integer arithmetic (equivalent to floor for >=0)
  if (cells[CoreCellx] == 0)
    newcells[CoreCellx] = (uint8_t)((numinf / k1) + (numill / k2));
  else if (cells[CoreCellx] < (uint8_t)(q - 1))
    newcells[CoreCellx] = (uint8_t)((sum / (numinf + 1)) + g);
  else
    newcells[CoreCellx] = 0;

  if (newcells[CoreCellx] > (uint8_t)(q - 1))
    newcells[CoreCellx] = (uint8_t)(q - 1);

  CoreCellx++;
  const uint8_t CORE_MAX = (uint8_t)(HALF - CELLLEN - 2); // 128-16-2 = 110
  if (CoreCellx > CORE_MAX)
  {
    CoreCellx = CELLLEN + 1;
    swap = cells;
    cells = newcells;
    newcells = swap;
    flag ^= 0x01; // Toggle
  }
}

/*
  Plague Cel Algorithm
*/
void cel(uint8_t *cells)
{
  static uint8_t l = 0;
  uint8_t cell, state, res;
  uint8_t rule = cells[0];
  res = 0;
  l++;
  l %= CELLLEN;

  for (cell = 1; cell < CELLLEN; cell++)
  {
    state = 0;
    if (cells[cell + 1 + (l * CELLLEN)] > 128)
      state |= 0x4;
    if (cells[cell + (CELLLEN * l)] > 128)
      state |= 0x2;
    if (cells[cell - 1 + (CELLLEN * l)] > 128)
      state |= 0x1;

    if ((rule >> state) & 1)
    {
      res += 1;
      cells[cell + (((l + 1) % CELLLEN) * CELLLEN)] = 255;
    }
    else
    {
      cells[cell + (((l + 1) % CELLLEN) * CELLLEN)] = 0;
    }
  }
}

/*
  Plague SIR Algorithm
  → Halbraum nun CELLS_LEN/2 (=128)
*/
void SIR(uint8_t *cellies)
{
  uint16_t x = 0;
  static uint8_t flag = 0; // Toggle Flag
  uint8_t *newcells, *cells;
  uint8_t kk = cellies[0], p = cellies[1];

  const uint16_t HALF = (CELLS_LEN / 2); // 128

  if ((flag & 0x01) == 0)
  {
    cells = cellies;
    newcells = &cellies[HALF];
  }
  else
  {
    cells = &cellies[HALF];
    newcells = cellies;
  }

  for (x = CELLLEN; x < (HALF - CELLLEN); x++)
  {
    uint8_t cell = cells[x];
    newcells[x] = cell;
    if (cell >= kk)
      newcells[x] = recovered;
    else if ((cell > 0 && cell < kk))
    {
      newcells[x]++;
    }
    else if (cell == susceptible)
    {
      if ((cells[x - CELLLEN] > 0 && cells[x - CELLLEN] < kk) ||
          (cells[x + CELLLEN] > 0 && cells[x + CELLLEN] < kk) ||
          (cells[x - 1] > 0 && cells[x - 1] < kk) ||
          (cells[x + 1] > 0 && cells[x + 1] < kk))
      {
        if ((prng8() % 10) < p)
          newcells[x] = 1;
      }
    }
  }
  flag ^= 0x01;
}

/*
  Plague Life Algorithm
  → Halbraum nun CELLS_LEN/2 (=128)
*/
void life(uint8_t *cellies)
{
  uint16_t x;
  uint8_t sum;
  static uint8_t flag = 0;
  uint8_t *newcells, *cells;

  const uint16_t HALF = (CELLS_LEN / 2); // 128

  if ((flag & 0x01) == 0)
  {
    cells = cellies;
    newcells = &cellies[HALF];
  }
  else
  {
    cells = &cellies[HALF];
    newcells = cellies;
  }

  for (x = CELLLEN + 1; x < (HALF - CELLLEN - 1); x++)
  {
    sum = cells[x] % 2 + cells[x - 1] % 2 + cells[x + 1] % 2 +
          cells[x - CELLLEN] % 2 + cells[x + CELLLEN] % 2 +
          cells[x - CELLLEN - 1] % 2 + cells[x - CELLLEN + 1] % 2 +
          cells[x + CELLLEN - 1] % 2 + cells[x + CELLLEN + 1] % 2;
    sum = (uint8_t)(sum - (cells[x] % 2));
    if (sum == 3 || (uint8_t)(sum + (cells[x] % 2)) == 3)
      newcells[x] = 255;
    else
      newcells[x] = 0;
  }

  flag ^= 0x01;
}

/* Seed für PRNG (LFSR) aus ADC-Rauschen */
void seed_rng(void)
{
  uint8_t seed = 0;
  for (int i = 0; i < 16; i++)
  {
    seed ^= adcread(3);
    _delay_us(50);
  }
  if (seed == 0)
    seed = 0xA5; // Fallback
  lfsr ^= seed;
}

/* --------------------------------------------------------------------------
   ADC-Cache (polling, throttled)
   -------------------------------------------------------------------------- */
typedef struct
{
  uint8_t ch0, ch1, ch2, ch3;
} AdcSnapshot;
static AdcSnapshot g_adc;       // Current values
static uint16_t g_adc_tick = 0; // Clock for throttling

// Update all potentiometers every 16 iterations
static inline void adc_poll_throttled(void)
{
  if ((g_adc_tick++ & 0x0F) == 0)
  {
    g_adc.ch0 = adcread(0);
    g_adc.ch1 = adcread(1);
    g_adc.ch2 = adcread(2);
    g_adc.ch3 = adcread(3);
  }
}

/* ---------------------------------------------------------------------- */
/* main                                                                   */
/* ---------------------------------------------------------------------- */

int main(void)
{
  seed_rng();

  uint8_t *cells = cells_buf;

  // CPU Functions // Instruction Groups
  uint8_t (*instructionsetfirst[])(uint8_t *cells, uint8_t IP) =
      {outff, outpp, finc, fdec, fincm, fdecm, fin1, fin2, fin3, fin4, outf, outp, plus, minus, bitshift1, bitshift2, bitshift3, branch, jump, infect, store, writeknob, writesamp, skip, direction, die}; // 26

  uint8_t (*instructionsetplague[])(uint8_t *cells, uint8_t IP) =
      {writeknob, writesamp, ploutf, ploutp, plenclose, plinfect, pldie, plwalk}; // 8

  uint8_t (*instructionsetbf[])(uint8_t *cells, uint8_t IP) =
      {bfinc, bfdec, bfincm, bfdecm, bfoutf, bfoutp, bfin, bfbrac1, bfbrac2}; // 9

  uint8_t (*instructionsetSIR[])(uint8_t *cells, uint8_t IP) =
      {SIRoutf, SIRoutp, SIRincif, SIRdieif, SIRrecif, SIRinfif}; // 6

  uint8_t (*instructionsetredcode[])(uint8_t *cells, uint8_t IP) =
      {rdmov, rdadd, rdsub, rdjmp, rdjmz, rdjmg, rddjz, rddat, rdcmp, rdoutf, rdoutp}; // 11

  uint8_t (*instructionsetbiota[])(uint8_t *cells, uint8_t IP) =
      {btempty, btoutf, btoutp, btstraight, btbackup, btturn, btunturn, btg, btclear, btdup}; // 10

  uint8_t (*instructionsetreddeath[])(uint8_t *cells, uint8_t IP) =
      {redplague, reddeath, redclock, redrooms, redunmask, redprospero, redoutside}; // 7

  // Plague Function Group
  void (*plag[])(uint8_t *cells) = {mutate, SIR, hodge, cel, hodge, SIR, life, mutate};

  adc_init();      // Initialize ADC
  initcell(cells); // Initialize Array of Cells for Sound Storage (jetzt 256 Zellen)

  sbi(DDRD, PORTD0); // PinD0 as out -> Switch1 -> IC40106(OSC) to filter
  sbi(DDRD, PORTD1); // PinD1 as out -> Switch2 -> pwm to filter  (PinD6 to Filter)
  sbi(DDRD, PORTD2); // PinD2 as out -> Switch3 -> Feedback on/off
  sbi(DDRD, PORTD6); // PinD6 as out -> Audio Signal Output
  sbi(DDRB, PORTB1); // Set Filter Clock via OCR1A

  // Set Timer
  TCCR1A = (1 << COM1A0);              // Toggle OCR1A/OCR1B on Compare Match
  TCCR1B = (1 << WGM12) | (1 << CS11); // CTC, /8

  // Audio Output (maintain; check hardware if COM0A0 is correct)
  TCCR0A = (1 << COM0A0) | (1 << WGM01) | (1 << WGM00); // Toggle OC0A on Compare Match // Fast PWM
  TCCR0B |= (1 << CS00) | (1 << CS02) | (1 << WGM02);   // prescaler /1024 // WGM02=1 >> Fast PWM

  cbi(PORTD, PORTD0); // IC40106 not to filter
  sbi(PORTD, PORTD1); // pwm to filter
  cbi(PORTD, PORTD2); // no feedback

  instructionp = 0; // InstructionPointer
  insdir = 1;       // Step size
  dir = 1;          // Direction
  btdir = 0;
  dcdir = 0;

  while (1)
  {
    // --- ADC-Cache ---
    adc_poll_throttled();
    IP = g_adc.ch0;       // Poti 1
    hardware = g_adc.ch1; // Poti 2
    controls = g_adc.ch2; // Poti 3

    if (hardware == 0)
      hardware = instructionp;
    if (controls == 0)
      controls = instructionp;

    qqq = controls % 4;         // Filtertyp
    cpu = IP >> 5;              // 8 CPUs
    step = (controls % 32) + 1; // 1-32
    plague = controls >> 5;

    count++;

    // every 1-32 steps run an algorithm
    if (count % ((IP % 32) + 1) == 0)
    {
      switch (cpu)
      {
      case 0:
        instruction = cells[instructionp];
        instructionp = (*instructionsetfirst[instruction % 26])(cells, instructionp);
        insdir = dir;
        break;
      case 1:
        instruction = cells[instructionp];
        instructionp = (*instructionsetplague[instruction % 8])(cells, instructionp);
        insdir = dir;
        if (cells[instructionp] == 255 && dir < 0)
          dir = 1;
        else if (cells[instructionp] == 255 && dir > 0)
          dir = -1;
        break;
      case 2:
        instruction = cells[instructionp];
        instructionp = (*instructionsetbf[instruction % 9])(cells, instructionp);
        insdir = dir;
        break;
      case 3:
        instruction = cells[instructionp];
        instructionp = (*instructionsetSIR[instruction % 6])(cells, instructionp);
        insdir = dir;
        break;
      case 4:
        instruction = cells[instructionp];
        instructionp = (*instructionsetredcode[instruction % 11])(cells, instructionp);
        insdir = dir;
        break;
      case 5:
        instruction = cells[instructionp];
        OCR0A = instruction;
        instructionp += dir; // intentional wrap
        break;
      case 6:
        instruction = cells[instructionp];
        instructionp = (*instructionsetreddeath[instruction % 7])(cells, instructionp);
        insdir = dir;
        break;
      case 7:
        instruction = cells[instructionp];
        instructionp = (*instructionsetbiota[instruction % 10])(cells, instructionp);
        if (btdir == 0)
          instructionp += 1;
        else if (btdir == 1)
          instructionp -= 1;
        else if (btdir == 2)
          instructionp += 16;
        else if (btdir == 3)
          instructionp -= 16;
        break;
      }
      if (!insdir_modified)
        insdir = dir;
      else
        insdir_modified = false;
    }

    // Is it time for a new plaque?
    if (count % step == 0)
    {
      (*plag[plague])(cells);
    }

    // Hardware Routing (atomar)
    hardk = hardware % 8;
    switch (hardk)
    {
    case 0:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { cbi(PORTD, PORTD2); } // no feedback
      break;
    case 1:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { PORTD |= (1 << PORTD2); } // feedback
      break;
    case 2:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        sbi(PORTD, PORTD0); // IC40106 to filter
        cbi(PORTD, PORTD1); // pwm to filter = NO!
      }
      break;
    case 3:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        cbi(PORTD, PORTD0); // not IC40106 to filter
        sbi(PORTD, PORTD1); // pwm to filter
      }
      break;
    case 4:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        PORTD |= (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD2);
      }
      break;
    case 5:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        if ((instructionp & 0x01) == 0x01)
        {
          cbi(PORTD, PORTD2);
        } // no feedback
        else
        {
          sbi(PORTD, PORTD2);
        } // feedback
      }
      break;
    case 6:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        PORTD |= (1 << PORTD0) | (1 << PORTD1); // IC40106 + PWM to filter
      }
      break;
    case 7:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        PORTD ^= (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD2); // Toggle routing
      }
      break;
    }

    // Filter Configuration
    fhk = hardware >> 4;
    switch (fhk)
    {
    case 0:
      cbi(DDRB, PORTB1); // filter off
      break;
    case 1:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
      filterk = 8;
      break;
    case 2:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
      filterk = 4;
      break;
    case 3:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
      filterk = 2;
      break;
    case 4:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
      break;
    case 5:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS11); // /8
      filterk = 8;
      break;
    case 6:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS11); // /8
      filterk = 4;
      break;
    case 7:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS11); // /8
      filterk = 2;
      break;
    case 8:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS11); // /8
      break;
    case 9:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // /64
      filterk = 8;
      break;
    case 10:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // /64
      filterk = 4;
      break;
    case 11:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // /64
      filterk = 2;
      break;
    case 12:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // /64
      break;
    case 13:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS12); // 256
      filterk = 8;
      break;
    case 14:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS12); // 256
      filterk = 6;
      break;
    case 15:
      sbi(DDRB, PORTB1);
      TCCR1B = (1 << WGM12) | (1 << CS12); // 256
      filterk = 4;
    }
  }
  return 0;
}