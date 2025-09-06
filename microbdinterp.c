/*

- THE PLAGUE INTERPRETER

... to bring back to all of us a natural, occult equivalent of the
dogma we no longer believe. [Antonin Artaud]

Returning the body, electronics, and dystopic code to the earth,
revived and decoded years later as "yersinia pestis".

- knobs:

left: cpu step, select instruction set (>>x)
mid: hardware and filter
right: controls - plague step and plague process select, filter modifier!

Based on the programming of microresearch - http://1010.co.uk/

Modified by circuitnoise

~ more documentation
~ code refactoring
~ merge changes of orig. code from Jan 23, 2013

TODO:

** crashing still!!!!

*/
#define DEBUG 0
#define F_CPU 16000000UL

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define CELLLEN 16

#define MAX_SAM 255                     // Maximum quantity of samples // Define that cells[] is 255 max
#define BV(bit) (1 << (bit))            // Byte Value => converts bit into a byte value. One at bit location.
#define cbi(reg, bit) reg &= ~(BV(bit)) // Clears the corresponding bit in register reg
#define sbi(reg, bit) reg |= (BV(bit))  // Sets the corresponding bit in register reg
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

// Initialisierung der globalen Variablen
signed char insdir = 1, dir = 1;
unsigned char filterk = 0, cpu = 0, plague = 0, step = 0;
unsigned char hardk = 0, fhk = 0, instruction = 0;
unsigned char instructionp = 0, IP = 0, controls = 0;
unsigned char hardware = 0, samp = 0, count = 0, qqq = 0;
unsigned char btdir = 0, dcdir = 0;
unsigned char clock = 0;
static bool insdir_modified = false;

int8_t cycle = -1; // signiert!
uint8_t ostack[20];

static unsigned char xxx[MAX_SAM + 12];

unsigned char stack[20], omem;

static inline uint8_t clamp_filterk(uint8_t k) { return (k > 8) ? 8 : k; }

static uint8_t last_cpu = 0xFF; // unmöglicher Startwert => erster Durchlauf triggert optional

// Obergrenze dem realen Array anpassen:
#define CELLS_LEN (MAX_SAM + 12) // 267

static inline uint16_t wrap_u16(int32_t x, uint16_t mod)
{
  int32_t r = x % mod;
  return (uint16_t)(r < 0 ? r + mod : r);
}

static inline uint8_t CGET(unsigned char *cells, int32_t i)
{
  return cells[wrap_u16(i, CELLS_LEN)];
}
static inline void CSET(unsigned char *cells, int32_t i, uint8_t v)
{
  cells[wrap_u16(i, CELLS_LEN)] = v;
}

/*
Initialize Analog Digital Converter (ADC)
Reference Voltage to AVCC
Single Conversion Mode
Left Adjusted Results
Activate ADC
*/
void adc_init(void)
{
  // Referenzspannung: AVCC
  cbi(ADMUX, REFS1);
  sbi(ADMUX, REFS0);

  // Ergebnis linksjustiert → ADCH enthält oberes 8-Bit
  sbi(ADMUX, ADLAR);

  // ADC Prescaler = 128 (16 MHz / 128 = 125 kHz)
  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  // ADC einschalten
  sbi(ADCSRA, ADEN);

  // PortC als Eingang (für die ADC-Kanäle)
  DDRC = 0x00;
  PORTC = 0x00;
}

/*
Sets Channel, Read and Return ADC Results
Voraussetzung: ADLAR=1 (linksjustiert) -> Rückgabe aus ADCH (8 Bit)
*/
unsigned char adcread(unsigned char channel)
{
  if (!(ADCSRA & (1 << ADEN)) || channel > 7)
    return 0;

  // Kanal setzen (REFSx/ADLAR bleiben erhalten)
  ADMUX = (ADMUX & 0xF8) | (channel & 0x07);

  // ADIF vor Messablauf löschen (write-1-to-clear)
  ADCSRA |= (1 << ADIF);

  // Dummy-Konversion nach MUX-Wechsel (erstes Sample verwerfen)
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
  {
  }
  (void)ADCH; // Dummy-Wert verwerfen

  // Echte Messung
  ADCSRA |= (1 << ADSC);
  uint16_t timeout = 10000;
  while ((ADCSRA & (1 << ADSC)) && --timeout)
  {
  }
  if (!timeout)
    return 0;

  // ADIF nach Abschluss löschen (optional, aber sauber)
  ADCSRA |= (1 << ADIF);

  return ADCH; // linksjustiert → oberes Byte
}

/*
Create a array of values from output signal(acdread(3)) as sample storage
*/
void initcell(unsigned char *cells)
{
  unsigned char x;
  for (x = 0; x < MAX_SAM; x++)
  {
    cells[x] = adcread(3); // get output signal
  }
}

/*Filter Function: Shift Left */
void leftsh(unsigned int cel)
{
  uint8_t k = clamp_filterk(filterk);
  OCR1A = cel << k;
}
/*Filter Function: Shift Right */
void rightsh(unsigned int cel)
{
  uint8_t k = clamp_filterk(filterk);
  OCR1A = cel >> k;
}
/*Filter Function: Multiply */
void mult(unsigned int cel)
{
  uint8_t k = clamp_filterk(filterk);
  OCR1A = cel * k;
}
/*Filter Function: Division */
void divvv(unsigned int cel)
{
  uint8_t k = clamp_filterk(filterk);
  OCR1A = cel / (k + 1);
}
/*
Pointer Function for Filter Assignments
Functions modify Clock Frequenz of Filter Max7400
*/
void (*filtermod[])(unsigned int cel) = {leftsh, rightsh, mult, divvv};

// first attempt - add in DATA POINTER= omem

/* instructionsetfirst */

/*Modify Filter Frequenz of Max7400 Filter*/
unsigned char outff(unsigned char *cells, unsigned char IP)
{
  //  OCR1A=(int)omem<<filterk;
  (*filtermod[qqq])((int)cells[omem]);
  return IP + insdir;
}

unsigned char outpp(unsigned char *cells, unsigned char IP)
{
  OCR0A = omem;
  return IP + insdir;
}

unsigned char finc(unsigned char *cells, unsigned char IP)
{
  /* [1.1] omem innerhalb der Zellfläche wrappen */
  omem = (uint8_t)wrap_u16((int32_t)omem + 1, CELLS_LEN);
  return IP + insdir;
}

unsigned char fdec(unsigned char *cells, unsigned char IP)
{
  /* [1.1] omem innerhalb der Zellfläche wrappen */
  omem = (uint8_t)wrap_u16((int32_t)omem - 1, CELLS_LEN);
  return IP + insdir;
}

unsigned char fincm(unsigned char *cells, unsigned char IP)
{
  cells[omem]++;
  return IP + insdir;
}

unsigned char fdecm(unsigned char *cells, unsigned char IP)
{
  cells[omem]--;
  return IP + insdir;
}

/* get omem from Output*/
unsigned char fin1(unsigned char *cells, unsigned char IP)
{
  omem = adcread(3); // get output signal
  return IP + insdir;
}

/*get omem from Poti 3 */
unsigned char fin2(unsigned char *cells, unsigned char IP)
{
  omem = adcread(2);
  return IP + insdir;
}
/*get IP from Poti 3*/
unsigned char fin3(unsigned char *cells, unsigned char IP)
{
  IP = adcread(2);
  return IP + insdir;
}
/**/
unsigned char fin4(unsigned char *cells, unsigned char IP)
{
  if (omem < MAX_SAM)
  {
    cells[omem] = adcread(3); // get output signal
  }
  return IP + insdir;
}

unsigned char outf(unsigned char *cells, unsigned char IP)
{
  //  OCR1A=(int)cells[omem]<<filterk;
  (*filtermod[qqq])((int)cells[omem]);
  return IP + insdir;
}

unsigned char outp(unsigned char *cells, unsigned char IP)
{
  OCR0A = cells[omem];
  return IP + insdir;
}

unsigned char plus(unsigned char *cells, unsigned char IP)
{
  cells[IP] += 1;
  return IP + insdir;
}

unsigned char minus(unsigned char *cells, unsigned char IP)
{
  cells[IP] -= 1;
  return IP + insdir;
}

unsigned char bitshift1(unsigned char *cells, unsigned char IP)
{
  cells[IP] = cells[IP] << 1;
  return IP + insdir;
}

unsigned char bitshift2(unsigned char *cells, unsigned char IP)
{
  cells[IP] = cells[IP] << 2;
  return IP + insdir;
}

unsigned char bitshift3(unsigned char *cells, unsigned char IP)
{
  cells[IP] = cells[IP] << 3;
  return IP + insdir;
}

unsigned char branch(unsigned char *cells, unsigned char IP)
{
  if (cells[IP + 1] == 0)
    IP = cells[omem];
  return IP + insdir;
}

unsigned char jump(unsigned char *cells, unsigned char IP)
{
  if (cells[(IP + 1)] < 128)
    return IP + cells[(IP + 1)];
  else
    return IP + insdir;
}

unsigned char infect(unsigned char *cells, unsigned char IP)
{
  int x = IP - 1;
  if (x < 0)
    x = MAX_SAM;
  if (cells[x] < 128)
    cells[(IP + 1)] = cells[IP];
  return IP + insdir;
}

unsigned char store(unsigned char *cells, unsigned char IP)
{
  cells[IP] = cells[cells[IP + 1]];
  return IP + insdir;
}

unsigned char writeknob(unsigned char *cells, unsigned char IP)
{
  cells[IP] = adcread(2);
  return IP + insdir;
}

unsigned char writesamp(unsigned char *cells, unsigned char IP)
{
  cells[IP] = adcread(3); // get output signal
  return IP + insdir;
}

unsigned char skip(unsigned char *cells, unsigned char IP)
{
  return IP + insdir;
}

// Sets direction
unsigned char direction(unsigned char *cells, unsigned char IP)
{
  if (dir < 0)
    dir = 1;
  else
    dir = -1;
  return IP + insdir;
}

// do nothing
unsigned char die(unsigned char *cells, unsigned char IP)
{
  return IP + insdir;
}

/* instructionsetplague */
/* Plague Algorithms*/

unsigned char ploutf(unsigned char *cells, unsigned char IP)
{
  //  OCR1A=((int)cells[IP+1]+(int)cells[IP-1])<<filterk;
  (*filtermod[qqq])((int)cells[omem]);

  return IP + insdir;
}

unsigned char ploutp(unsigned char *cells, unsigned char IP)
{
  OCR0A = cells[IP + 1] + cells[IP - 1];
  return IP + insdir;
}

unsigned char plenclose(unsigned char *cells, unsigned char IP)
{
  cells[IP] = 255;
  cells[IP + 1] = 255;
  return IP + 2;
}

unsigned char plinfect(unsigned char *cells, unsigned char IP)
{

  if (cells[IP] < 128)
  {
    cells[IP + 1] = cells[IP];
    cells[IP - 1] = cells[IP];
  }
  return IP + insdir;
}

unsigned char pldie(unsigned char *cells, unsigned char IP)
{
  cells[IP - 1] = 0;
  cells[IP + 1] = 0;
  return IP + insdir;
}

unsigned char plwalk(uint8_t *cells, uint8_t IP)
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

/* instructionsetbf */
/* instructionsetplague */
/* Brainfuck*/

unsigned char bfinc(unsigned char *cells, unsigned char IP)
{
  /* [1.1] omem innerhalb der Zellfläche wrappen */
  omem = (uint8_t)wrap_u16((int32_t)omem + 1, CELLS_LEN);
  return ++IP;
}

unsigned char bfdec(unsigned char *cells, unsigned char IP)
{
  /* [1.1] omem innerhalb der Zellfläche wrappen */
  omem = (uint8_t)wrap_u16((int32_t)omem - 1, CELLS_LEN);
  return ++IP;
}

unsigned char bfincm(unsigned char *cells, unsigned char IP)
{
  cells[omem]++;
  return ++IP;
}

unsigned char bfdecm(unsigned char *cells, unsigned char IP)
{
  cells[omem]--;
  return ++IP;
}

unsigned char bfoutf(unsigned char *cells, unsigned char IP)
{
  //  OCR1A=(int)cells[omem]<<filterk;
  (*filtermod[qqq])((int)cells[omem]);
  return ++IP;
}

unsigned char bfoutp(unsigned char *cells, unsigned char IP)
{
  OCR0A = cells[omem];
  return ++IP;
}

unsigned char bfin(unsigned char *cells, unsigned char IP)
{
  if (omem < MAX_SAM)
  {
    cells[omem] = adcread(3); // get output signal
  }
  return ++IP;
}

unsigned char bfbrac1(uint8_t *cells, uint8_t IP)
{
  if (cycle < 19)
  {
    cycle++;
    ostack[cycle] = IP;
  }
  return ++IP;
}
unsigned char bfbrac2(uint8_t *cells, uint8_t IP)
{
  if (cycle >= 0 && cells[omem] != 0)
    return (uint8_t)(ostack[cycle]);
  if (cycle >= 0)
    cycle--;
  return ++IP;
}

/* instructionsetSIR */
// SIR: inc if , die if, recover if, getinfected if

unsigned char SIRoutf(unsigned char *cells, unsigned char IP)
{
  //  OCR1A=((int)cells[(IP+1)]+(int)cells[IP-1])<<filterk;
  (*filtermod[qqq])((int)cells[(IP + 1)] + (int)cells[IP - 1]);

  return IP + insdir;
}

unsigned char SIRoutp(unsigned char *cells, unsigned char IP)
{
  OCR0A = cells[(IP + 1)] + cells[IP - 1]; // neg?
  return IP + insdir;
}

unsigned char SIRincif(unsigned char *cells, unsigned char IP)
{
  if ((cells[(IP + 1)] > 0 && cells[(IP + 1)] < 128))
    cells[IP]++;
  return IP + insdir;
}

unsigned char SIRdieif(unsigned char *cells, unsigned char IP)
{

  if ((cells[(IP + 1)] > 0 && cells[(IP + 1)] < 128))
  {
    if (rand() % 10 < 4)
      cells[IP] = dead;
  }
  return IP + insdir;
}

unsigned char SIRrecif(unsigned char *cells, unsigned char IP)
{
  if (cells[(IP + 1)] >= 128)
    cells[IP] = recovered;
  return IP + insdir;
}

unsigned char SIRinfif(unsigned char *cells, unsigned char IP)
{

  if (cells[(IP + 1)] == 0)
  {

    if ((cells[IP - 1] > 0 && cells[IP - 1] < 128) ||
        (cells[(IP + 1)] > 0 && cells[(IP + 1)] < 128))
    {
      if (rand() % 10 < 4)
        cells[IP] = 1;
    }
  }
  return IP + insdir;
}

/* instructionsetredcode */
// red code

unsigned char rdmov(unsigned char *cells, unsigned char IP)
{
  // cells[(IP + cells[IP + 2])] = cells[(IP + cells[IP + 1])];
  uint8_t off1 = CGET(cells, IP + 1);
  uint8_t off2 = CGET(cells, IP + 2);
  uint8_t src = CGET(cells, (int32_t)IP + off1);
  CSET(cells, (int32_t)IP + off2, src);
  return IP + 3;
}

unsigned char rdadd(unsigned char *cells, unsigned char IP)
{
  // cells[(IP + cells[IP + 2])] = cells[(IP + cells[IP + 2])] + cells[(IP + cells[IP + 1])];
  uint8_t off1 = CGET(cells, IP + 1);
  uint8_t off2 = CGET(cells, IP + 2);
  uint8_t dstv = CGET(cells, (int32_t)IP + off2);
  uint8_t srcv = CGET(cells, (int32_t)IP + off1);
  CSET(cells, (int32_t)IP + off2, (uint8_t)(dstv + srcv));
  return IP + 3;
}

unsigned char rdsub(unsigned char *cells, unsigned char IP)
{
  // cells[(IP + cells[IP + 2])] = cells[(IP + cells[IP + 2])] - cells[(IP + cells[IP + 1])];
  uint8_t off1 = CGET(cells, IP + 1);
  uint8_t off2 = CGET(cells, IP + 2);
  uint8_t dstv = CGET(cells, (int32_t)IP + off2);
  uint8_t srcv = CGET(cells, (int32_t)IP + off1);
  CSET(cells, (int32_t)IP + off2, (uint8_t)(dstv - srcv));
  return IP + 3;
}

unsigned char rdjmp(unsigned char *cells, unsigned char IP)
{
  // IP = IP + cells[IP + 1];
  uint8_t off = CGET(cells, IP + 1);
  return (uint8_t)(IP + off);
}

unsigned char rdjmz(unsigned char *cells, unsigned char IP)
{
  // if (cells[(IP + cells[IP + 2])] == 0) IP = cells[IP + 1]; else IP += 3;
  uint8_t off2 = CGET(cells, IP + 2);
  if (CGET(cells, (int32_t)IP + off2) == 0)
    return CGET(cells, IP + 1);
  else
    return (uint8_t)(IP + 3);
}

unsigned char rdjmg(unsigned char *cells, unsigned char IP)
{
  // if (cells[(IP + cells[IP + 2])] >= 0) ...  // >=0 immer wahr bei uint8_t → >0
  uint8_t off2 = CGET(cells, IP + 2);
  if (CGET(cells, (int32_t)IP + off2) > 0)
    return CGET(cells, IP + 1);
  else
    return (uint8_t)(IP + 3);
}

unsigned char rddjz(unsigned char *cells, unsigned char IP)
{
  // x = (IP + cells[IP + 2]); cells[x] = cells[x] - 1; if (cells[x] == 0) ...
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

unsigned char rddat(unsigned char *cells, unsigned char IP)
{
  IP += 3;
  return IP;
}

unsigned char rdcmp(unsigned char *cells, unsigned char IP)
{
  // if (cells[(IP + cells[IP + 2])] != cells[(IP + cells[IP + 1])]) IP += 6; else IP += 3;
  uint8_t off1 = CGET(cells, IP + 1);
  uint8_t off2 = CGET(cells, IP + 2);
  if (CGET(cells, (int32_t)IP + off2) != CGET(cells, (int32_t)IP + off1))
    return (uint8_t)(IP + 6);
  else
    return (uint8_t)(IP + 3);
}

unsigned char rdoutf(unsigned char *cells, unsigned char IP)
{
  // (*filtermod[qqq])((int)cells[IP + 1]);
  (*filtermod[qqq])((int)CGET(cells, IP + 1));
  return (uint8_t)(IP + 3);
}

unsigned char rdoutp(unsigned char *cells, unsigned char IP)
{
  OCR0A = CGET(cells, IP + 2);
  return (uint8_t)(IP + 3);
}

/* instructionsetbiota */

// BIOTA!

unsigned char btempty(unsigned char *cells, unsigned char IP)
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

unsigned char btoutf(unsigned char *cells, unsigned char IP)
{
  //  OCR1A=(int)cells[omem]<<filterk;
  (*filtermod[qqq])((int)cells[omem]);

  return IP;
}

unsigned char btoutp(unsigned char *cells, unsigned char IP)
{
  OCR0A = cells[omem];
  return IP;
}

unsigned char btstraight(unsigned char *cells, unsigned char IP)
{
  if (dcdir == 0)
    omem += 1;
  else if (dcdir == 1)
    omem -= 1;
  else if (dcdir == 2)
    omem += 16;
  else if (dcdir == 3)
    omem -= 16;

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

unsigned char btbackup(unsigned char *cells, unsigned char IP)
{
  if (dcdir == 0)
    omem -= 1;
  else if (dcdir == 1)
    omem += 1;
  else if (dcdir == 2)
    omem -= 16;
  else if (dcdir == 3)
    omem += 16;
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

unsigned char btturn(unsigned char *cells, unsigned char IP)
{
  if (dcdir == 0)
    omem += 16;
  else if (dcdir == 1)
    omem -= 16;
  else if (dcdir == 2)
    omem += 1;
  else if (dcdir == 3)
    omem -= 1;
  return IP;
}

unsigned char btunturn(unsigned char *cells, unsigned char IP)
{
  if (dcdir == 0)
    omem -= 16;
  else if (dcdir == 1)
    omem += 16;
  else if (dcdir == 2)
    omem -= 1;
  else if (dcdir == 3)
    omem += 1;
  return IP;
}

unsigned char btg(unsigned char *cells, unsigned char IP)
{
  unsigned char x = 0;
  while (x < 20 && cells[omem] != 0)
  {
    if (dcdir == 0)
      omem += 1;
    else if (dcdir == 1)
      omem -= 1;
    else if (dcdir == 2)
      omem += 16;
    else if (dcdir == 3)
      omem -= 16;
    x++;
  }
  return IP;
}

unsigned char btclear(unsigned char *cells, unsigned char IP)
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
    cells[omem] = 0;
  return IP;
}

unsigned char btdup(unsigned char *cells, unsigned char IP)
{
  if (cells[omem] == 0 || cells[omem - 1] != 0)
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
    cells[omem - 1] = cells[omem];
  return IP;
}

/* instructionsetreddeath */

// 1- the plague within (12 midnight) - all the cells infect

unsigned char redplague(unsigned char *cells, unsigned char IP)
{
  if (clock == 12)
  {
    clock = 12;
    cells[IP + 1] = cells[IP];
    if (IP == 255)
      clock = 13;
    return IP + 1;
  }
  else
    return IP + insdir;
}

// 3- death - one by one fall dead
unsigned char reddeath(unsigned char *cells, unsigned char IP)
{
  if (clock == 13)
  {
    clock = 13;
    // cells[IP + count] = adcread(3); count++;
    count = (uint8_t)((count + 1) % CELLS_LEN);
    CSET(cells, (int32_t)IP + count, adcread(3)); // get output signal
    return IP;                                    // just keeps on going
  }
  else
    return IP + insdir;
}

// 2- clock every hour - instruction counter or IP -some kind of TICK
unsigned char redclock(unsigned char *cells, unsigned char IP)
{
  clock++;
  if (clock % 60 == 0)
  {
    OCR0A ^= 255;
    return IP; // everyone stops
  }
  else
    return IP + insdir;
}

// 4- seven rooms: divide cellspace into 7 - 7 layers with filter each
unsigned char redrooms(unsigned char *cells, unsigned char IP)
{
  switch (IP % 7)
  {
  case 0:
    // blue
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
    filterk = 8;
    break;
  case 1:
    // purple
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
    break;
  case 2:
    // green
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS11); // divide by 8
    filterk = 8;
    break;
  case 3:
    // orange
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS11); // divide by 8
    break;
  case 4:
    // white
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // divide by 64
    break;
  case 5:
    // violet
    sbi(DDRB, PORTB1);
    TCCR1B = (1 << WGM12) | (1 << CS12); // 256
    break;
  case 6:
    // black
    cbi(DDRB, PORTB1); // filter off
  }
  return IP + insdir;
}

// 5- unmasking (change neighbouring cells)

unsigned char redunmask(unsigned char *cells, unsigned char IP)
{
  cells[IP - 1] ^= 255;
  cells[IP + 1] ^= 255;
  return IP + insdir;
}
// 6- the prince (omem) - the output! walking through 7 rooms

unsigned char redprospero(unsigned char *cells, unsigned char IP)
{

  unsigned char dirrr;
  // prince/omem moves at random through rooms
  dirrr = adcread(3) % 4; // get output signal
  if (dirrr == 0)
    omem = omem + 1;
  else if (dirrr == 1)
    omem = omem - 1;
  else if (dirrr == 2)
    omem = omem + 16;
  else if (dirrr == 3)
    omem = omem - 16;

  // output
  OCR0A = cells[omem];
  return IP + insdir;
}

// 7- the outside - the input!
unsigned char redoutside(unsigned char *cells, unsigned char IP)
{

  // input sample to cell (which one neighbour to omem)
  cells[omem + 1] = adcread(3); // get output signal

  // output to filter
  (*filtermod[qqq])((int)cells[omem]);
  return IP + insdir;
}

/* plag - Plague Function Group
   instructions for plague CPUs! */

/*
  Plaque Mutate changes cell values to values from Filter Output until cells[0] value is reached
*/
void mutate(unsigned char *cells)
{
  unsigned char x, y;
  for (y = 0; y < cells[0]; y++)
  {
    x = adcread(3);         // Read output signal
    cells[x] ^= (x & 0x0f); // copies the bit if it is set in one operand (but not both) 0b00001111
  }
}
/*
  Plague Hodge Implementation
  Switches every 110 Cycles the cells array with newcells array

*/

void hodge(unsigned char *cellies)
{
  int sum = 0, numill = 0, numinf = 0; // max value 32767
  unsigned char q, k1, k2, g;
  static unsigned char CoreCellx = CELLLEN + 1;  // raises every time function is called
  static unsigned char flag = 0;                 // Toggle Flag
  static unsigned char *newcells, *cells, *swap; // Changed variables to static

  // Swap where the cellies go
  if ((flag & 0x01) == 0)
  {
    cells = cellies;
    newcells = &cellies[MAX_SAM / 2];
  }
  else
  {
    cells = &cellies[MAX_SAM / 2];
    newcells = cellies;
  }

  // Sets Reference Values
  q = cells[0];
  k1 = cells[1];
  k2 = cells[2];
  g = cells[3];
  if (k1 == 0)
    k1 = 1;
  if (k2 == 0)
    k2 = 1;

  // Calculate sum of 3 neighbor cells values
  sum = cells[CoreCellx] + cells[CoreCellx - 1] + cells[CoreCellx + 1] + cells[CoreCellx - CELLLEN] + cells[CoreCellx + CELLLEN] + cells[CoreCellx - CELLLEN - 1] + cells[CoreCellx - CELLLEN + 1] + cells[CoreCellx + CELLLEN - 1] + cells[CoreCellx + CELLLEN + 1];

  // Decide which one is infected or ill.
  if (cells[CoreCellx - 1] == (q - 1))
    numill++;
  else if (cells[CoreCellx - 1] > 0)
    numinf++;
  if (cells[CoreCellx + 1] == (q - 1))
    numill++;
  else if (cells[CoreCellx + 1] > 0)
    numinf++;
  if (cells[CoreCellx - CELLLEN] == (q - 1))
    numill++;
  else if (cells[CoreCellx - CELLLEN] > 0)
    numinf++;
  if (cells[CoreCellx + CELLLEN] == (q - 1))
    numill++;
  else if (cells[CoreCellx + CELLLEN] > 0)
    numinf++;
  if (cells[CoreCellx - CELLLEN - 1] == q)
    numill++;
  else if (cells[CoreCellx - CELLLEN - 1] > 0)
    numinf++;
  if (cells[CoreCellx - CELLLEN + 1] == q)
    numill++;
  else if (cells[CoreCellx - CELLLEN + 1] > 0)
    numinf++;
  if (cells[CoreCellx + CELLLEN - 1] == q)
    numill++;
  else if (cells[CoreCellx + CELLLEN - 1] > 0)
    numinf++;
  if (cells[CoreCellx + CELLLEN + 1] == q)
    numill++;
  else if (cells[CoreCellx + CELLLEN + 1] > 0)
    numinf++;

  // Sets the Values of Cells[0-127]
  if (cells[CoreCellx] == 0)
    // there is a slight chance cell value will raise up to 2
    // sets the lowest integral number -maximal value is 2 = 1(numinf/k1) + 1 (numill/k2)
    newcells[CoreCellx % 128] = floor(numinf / k1) + floor(numill / k2);
  else if (cells[CoreCellx] < q - 1) // if cells[CoreCellx]<cells[0]+1

    newcells[CoreCellx % 128] = floor(sum / (numinf + 1)) + g;
  else
    newcells[CoreCellx % 128] = 0;

  if (newcells[CoreCellx % 128] > q - 1)
    newcells[CoreCellx % 128] = q - 1;

  CoreCellx++; // next time take the next cell

  // if CoreCellx reaches 110, reset CoreCellx, swap cells and newcells;
  if (CoreCellx > ((MAX_SAM / 2) - CELLLEN - 1))
  {
    CoreCellx = CELLLEN + 1;
    //    swap = cells; cells = newcells; newcells = swap;
    swap = cells;
    cells = newcells;
    newcells = swap;

    flag ^= 0x01; // Toggle Flag
  }
}
/*
  Plague Cel Algorithm
  Changes up to 16 Cells to 0 or 255
*/
void cel(unsigned char *cells)
{

  static unsigned char l = 0;
  unsigned char cell, state, res;
  unsigned char rule = cells[0];
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
      res += 1; // Todo:Warum wird res benutzt?
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
  Sets value of cells to recovered(129), susceptible(0) or 1
*/
void SIR(unsigned char *cellies)
{
  unsigned char cell, x = 0;
  static unsigned char flag = 0; // Toggle Flag
  unsigned char *newcells, *cells;
  unsigned char kk = cellies[0], p = cellies[1];

  if ((flag & 0x01) == 0)
  {
    cells = cellies;
    newcells = &cellies[MAX_SAM / 2];
  }
  else
  {
    cells = &cellies[MAX_SAM / 2];
    newcells = cellies;
  }

  for (x = CELLLEN; x < ((MAX_SAM / 2) - CELLLEN); x++)
  {
    cell = cells[x];
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
        if (rand() % 10 < p)
          newcells[x] = 1;
      }
    }
  }
  flag ^= 0x01;
}
/*
  Plague Life Algorithm
*/
void life(unsigned char *cellies)
{
  unsigned char x, sum;

  static unsigned char flag = 0;
  unsigned char *newcells, *cells;

  if ((flag & 0x01) == 0)
  {
    cells = cellies;
    newcells = &cellies[MAX_SAM / 2];
  }
  else
  {
    cells = &cellies[MAX_SAM / 2];
    newcells = cellies;
  }

  for (x = CELLLEN + 1; x < ((MAX_SAM / 2) - CELLLEN - 1); x++)
  {
    sum = cells[x] % 2 + cells[x - 1] % 2 + cells[x + 1] % 2 + cells[x - CELLLEN] % 2 + cells[x + CELLLEN] % 2 + cells[x - CELLLEN - 1] % 2 + cells[x - CELLLEN + 1] % 2 + cells[x + CELLLEN - 1] % 2 + cells[x + CELLLEN + 1] % 2;
    sum = sum - cells[x] % 2;
    if (sum == 3 || (sum + (cells[x] % 2) == 3))
      newcells[x] = 255;
    else
      newcells[x] = 0;
  }

  // swapping
  flag ^= 0x01;
}

void seed_rng(void)
{
  uint16_t seed = 0;
  for (int i = 0; i < 16; i++)
  {
    seed = (seed << 1) ^ adcread(3);
    _delay_us(50);
  }
  srand(seed);
}

int main(void)
{

  seed_rng();

  unsigned char *cells = xxx;

  // CPU Functions // Instruction Groups
  unsigned char (*instructionsetfirst[])(unsigned char *cells, unsigned char IP) = {outff, outpp, finc, fdec, fincm, fdecm, fin1, fin2, fin3, fin4, outf, outp, plus, minus, bitshift1, bitshift2, bitshift3, branch, jump, infect, store, writeknob, writesamp, skip, direction, die}; // 26 instructions

  unsigned char (*instructionsetplague[])(unsigned char *cells, unsigned char IP) = {writeknob, writesamp, ploutf, ploutp, plenclose, plinfect, pldie, plwalk}; // 8

  unsigned char (*instructionsetbf[])(unsigned char *cells, unsigned char IP) = {bfinc, bfdec, bfincm, bfdecm, bfoutf, bfoutp, bfin, bfbrac1, bfbrac2}; // 9

  unsigned char (*instructionsetSIR[])(unsigned char *cells, unsigned char IP) = {SIRoutf, SIRoutp, SIRincif, SIRdieif, SIRrecif, SIRinfif}; // 6

  unsigned char (*instructionsetredcode[])(unsigned char *cells, unsigned char IP) = {rdmov, rdadd, rdsub, rdjmp, rdjmz, rdjmg, rddjz, rddat, rdcmp, rdoutf, rdoutp}; // 11

  unsigned char (*instructionsetbiota[])(unsigned char *cells, unsigned char IP) = {btempty, btoutf, btoutp, btstraight, btbackup, btturn, btunturn, btg, btclear, btdup}; // 10

  unsigned char (*instructionsetreddeath[])(unsigned char *cells, unsigned char IP) = {redplague, reddeath, redclock, redrooms, redunmask, redprospero, redoutside}; // 7

  // Plague Function Group
  void (*plag[])(unsigned char *cells) = {mutate, SIR, hodge, cel, hodge, SIR, life, mutate};

  adc_init(); // Initialize Analog Digital Converter

  initcell(cells); // Initialize Array of Cells for Sound Storage

  sbi(DDRD, PORTD0); // PinD0 as out -> Switch1 -> IC40106(OSC) to filter
  sbi(DDRD, PORTD1); // PinD1 as out -> Switch2 -> pwm to filter  (PinD6 to Filter)
  sbi(DDRD, PORTD2); // PinD2 as out -> Switch3 -> Feedback on/off
  sbi(DDRD, PORTD6); // PinD6 as out -> Audio Signal Output > immer wenn OCR0A gleich TCNT0 ist
  sbi(DDRB, PORTB1); // Set Filter Clock via OCR1A

  // Set Timer
  TCCR1A = (1 << COM1A0);              // Toggle OCR1A/OCR1B on Compare Match
  TCCR1B = (1 << WGM12) | (1 << CS11); // Sets WaveGenerationMode to "clear timer on compare match" Top is OCR1A // Clock Select prescaler /8

  // Configure Audio Output
  // TCCR0A Sets WaveGernationMode to Fast PWM TOP is OCRA
  TCCR0A = (1 << COM0A0) | (1 << WGM01) | (1 << WGM00); // Toggle OC0A on Compare Match // Fast PWM
  TCCR0B |= (1 << CS00) | (1 << CS02) | (1 << WGM02);   // Clock Select prescaler /1024 // WGM02=1 >> Fast PWM

  cbi(PORTD, PORTD0); // IC40106 not to filter
  sbi(PORTD, PORTD1); // pwm to filter
  cbi(PORTD, PORTD2); // no feedback

  instructionp = 0; // InstructionPointer selects cell value is used for the next instruction select
  insdir = 1;       // Step size for instruction Pointer - only changes in plwalk()
  dir = 1;          // Direction for the next step
  btdir = 0;
  dcdir = 0;

  while (1)
  {

    IP = adcread(0);       // read Poti 1 top    /  left of panel mount jack
    hardware = adcread(1); // read Poti 2 middle /   top of panel mount jack
    controls = adcread(2); // read Poti 3 buttom / right of panel mount jack

    if (hardware == 0)
      hardware = instructionp;
    if (controls == 0)
      controls = instructionp;

    qqq = controls % 4; // Sets the filtertyp in filtermod()

    cpu = IP >> 5;              // 8 CPUs  // cpu sets 1 of 8 instruction groups/algorithm
    step = (controls % 32) + 1; // sets step to 1-32 // decided than a new plague will be created

    plague = controls >> 5; // Sets plague Function

    count++;

    // every 1-32 steps run an algorithm
    if (count % ((IP % 32) + 1) == 0)
    {

      // Which instruction group/algorithm is used?
      switch (cpu)
      {
      case 0:
        //
        instruction = cells[instructionp];
        instructionp = (*instructionsetfirst[instruction % 26])(cells, instructionp); // mistake before as was instruction%INSTLEN in last instance
        //      insdir=dir*(IP%16)+1; // prev mistake as just got exponentially larger
        insdir = dir; // set direction for next instruction
        break;
      case 1:
        // Plague Alogrithms
        instruction = cells[instructionp];
        instructionp = (*instructionsetplague[instruction % 8])(cells, instructionp);
        //	    insdir=dir*(IP%16)+1;
        insdir = dir;
        if (cells[instructionp] == 255 && dir < 0)
          dir = 1;
        else if (cells[instructionp] == 255 && dir > 0)
          dir = -1; // barrier
        break;
      case 2:
        // Brain Fuck Algorithms
        instruction = cells[instructionp];
        instructionp = (*instructionsetbf[instruction % 9])(cells, instructionp);
        //	    insdir=dir*(IP%16)+1;
        insdir = dir;
        break;
      case 3:
        // SIR (susceptible, infected, recovered) Algorithms
        instruction = cells[instructionp];
        instructionp = (*instructionsetSIR[instruction % 6])(cells, instructionp);
        //	    insdir=dir*(IP%16)+1;
        insdir = dir;
        break;
      case 4:
        // Red Code Algorithms
        instruction = cells[instructionp];
        instructionp = (*instructionsetredcode[instruction % 11])(cells, instructionp);
        //	    insdir=dir*(IP%16)+1;
        insdir = dir;
        break;
      case 5:
        // direct output
        instruction = cells[instructionp];
        OCR0A = instruction;
        instructionp += dir; // changed from insdir
        break;
      case 6:
        // Red Death Algorithms
        instruction = cells[instructionp];
        instructionp = (*instructionsetreddeath[instruction % 7])(cells, instructionp);
        //	    insdir=dir*(IP%16)+1;
        insdir = dir;
        break;
      case 7:
        // la biota Algorithms
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

    // Is is time for a new plaque?
    if (count % step == 0)
    { // was instructionp%step
      (*plag[plague])(cells);
    }

    // Filter or Feedback required?
    hardk = hardware % 8;

    switch (hardk)
    {
    case 0:
      cbi(PORTD, PORTD2); // no feedback
      break;
    case 1:
      cli();                  // Disable interrupts
      PORTD |= (1 << PORTD2); // feedback
      sei();                  // Enable interrupts
      break;
    case 2:
      sbi(PORTD, PORTD0); // IC40106 to filter
      cbi(PORTD, PORTD1); // pwm to filter = NO!
      break;
    case 3:
      cbi(PORTD, PORTD0); // not IC40106 to filter
      sbi(PORTD, PORTD1); // pwm to filter
      break;
    case 4:
      // All to filter with feedback
      PORTD |= (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD2);
      // PORTD|=instructionp&0x07;
      break;
    case 5:
      if ((instructionp & 0x01) == 0x01)
      {                     // Toggle feedback through instructionp
        cbi(PORTD, PORTD2); // no feedback
      }
      else
      {
        sbi(PORTD, PORTD2); // feedback
      }
      break;
    case 6:
      // Activate IC40106 and PWM to filter
      PORTD |= (1 << PORTD0) | (1 << PORTD1);
      break;
    case 7:
      // Toggle Routing to Filter and Feedback
      PORTD ^= (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD2);
      // PORTD^=instructionp&0x07;
    }

    // Filter Configuration
    fhk = hardware >> 4;

    switch (fhk)
    {
    case 0:
      cbi(DDRB, PORTB1); // filter off
      break;
    case 1:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
      filterk = 8;
      break;
    case 2:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
      filterk = 4;
      break;
    case 3:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
      filterk = 2;
      break;
    case 4:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS10); // no divider
      break;
    case 5:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS11); // divide by 8
      filterk = 8;
      break;
    case 6:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS11); // divide by 8
      filterk = 4;
      break;
    case 7:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS11); // divide by 8
      filterk = 2;
      break;
    case 8:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS11); // divide by 8
      break;
    case 9:
      sbi(DDRB, PORTB1);                                 // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // divide by 64
      filterk = 8;
      break;
    case 10:
      sbi(DDRB, PORTB1);                                 // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // divide by 64
      filterk = 4;
      break;
    case 11:
      sbi(DDRB, PORTB1);                                 // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // divide by 64
      filterk = 2;
      break;
    case 12:
      sbi(DDRB, PORTB1);                                 // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // divide by 64
      break;
    case 13:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS12); // 256
      filterk = 8;
      break;
    case 14:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS12); // 256
      filterk = 6;
      break;
    case 15:
      sbi(DDRB, PORTB1);                   // Filter on
      TCCR1B = (1 << WGM12) | (1 << CS12); // 256
      filterk = 4;
    }
  }
  return 0;
}
