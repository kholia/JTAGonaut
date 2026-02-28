/*
 JTAGenum
 Given a Arduino compatible microcontroller JTAGenum scans
 pins[] for basic JTAG functionality. After programming
 your microcontroller open a serial terminal with 115200
 baud and send 'h' to see usage information.

 SETUP:
 Define the pins[] and pinnames[] for your microcontroller.

 If after finding the JTAG pins of target you would like
 to run the IRenum and BYPASS scans, then set the
 TCK, TMS, TDO, TDI, TRST #define's

 Further documentation:
 http://github.com/cyphunk/JTAGenum/
 http://deadhacker.com/2010/02/03/jtag-enumeration/

 This license for this code is whatever you want it to be

 user@zion:~/repos/openocd/tcl$ grep -r "irlen [0-9]" . | sed 's/.*irlen/irlen/'| awk '{print $1" "$2}' | sort | uniq -c | sort -n
      1 irlen 11
      1 irlen 18
      1 irlen 19
      1 irlen 2
      1 irlen 22
      1 irlen 24
      1 irlen 32
      1 irlen 9
      2 irlen 14
      2 irlen 16
      3 irlen 12
      3 irlen 7
      5 irlen 1
      7 irlen 38
     14 irlen 10
     22 irlen 6
     27 irlen 8
     79 irlen 5
    245 irlen 4

user@zion:~/repos/openocd/tcl$ date
Sat Feb 28 10:32:43 IST 2026

*/

//needed to put help strings into flash
#ifdef ARDUINO_ARCH_AVR
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif

#include "fpga_db.h"

#ifdef ESP32
#include <esp_task_wdt.h>
#endif

const char* fpga_lookup(uint32_t idcode) {
  uint32_t masked_id = idcode & 0x0FFFFFFF;
  static char buf[128];

  for (int i = 0; i < FPGA_COUNT; i++) {
    if (fpga_table[i].idcode == masked_id) {
      snprintf(buf, sizeof(buf), "%s %s %s (IR:%d)",
               fpga_table[i].mfg, fpga_table[i].family,
               fpga_table[i].model, fpga_table[i].irlen);
      return buf;
    }
  }

  uint16_t mfg_id = (idcode >> 1) & 0x7FF;
  for (int i = 0; i < MFG_COUNT; i++) {
    if (mfg_table[i].id == mfg_id) {
      return mfg_table[i].name;
    }
  }

  return "Unknown";
}


// #define DEBUGTAP
// #define DEBUGIR
// #define DEBUG_FIXED_TCK 18  // Index of GP18 in pins[] for RP2040 - FOR DEBUGGING ONLY!

/*
 * BEGIN USER DEFINITIONS
 */
//#define HALFCLOCK // uncomment for AVR (arduino/teensy) running at 3.3v

/*
 * SETUP TARGET PINS
 * determine your microcontroller platform mand set pins accordingly
 * pins[] should be valid pins of your microcontroller
 * pinnames[] can be any string you choose
 * when in doubt comment out all but one pin[] pinnames[] definition
 */
#if defined(TEENSY_40)  // Teensy v4 usable digital are: A0-A9; A0-A9 are always digital 14-23, for Arduino compatibility
byte pins[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9 };
const char* pinnames[] = { "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8", "A9" };
#elif defined(KINETISK)             // Teensy v3 usable digital are: A0-A7. 13=LED
byte pins[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
const char* pinnames[] = { "A0", "A1", "A2", "A3", "A4", "A5", "A6", "A7" };
#elif defined(CORE_TEENSY)          // Teensy v2
byte pins[] = { PIN_B2, PIN_B3, PIN_B6, PIN_B4, PIN_B1 };
const char* pinnames[] = { "B2", "B3", "B6", "B4", "B1" };
#elif defined(ENERGIA)              // TI Launchpad Tiva C
byte pins[] = { PA_5, PB_4, PE_5, PE_4, PB_1 };
const char* pinnames[] = { "PA_5", "PB_4", "PE_5", "PE_4", "PB_1" };
#elif defined(STM32)                // STM32 bluepill, pinout is here: https://wiki.stm32duino.com/index.php?title=File:Bluepillpinout.gif. See also instructions to get it running with the Arduino IDE here: http://www.zoobab.com/bluepill-arduinoide
byte pins[] = { 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22 };
const char* pinnames[] = { "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "21", "22" };
#elif defined(STM32Nucleo)          // STM32 Nucleo, pinout is here: https://os.mbed.com/platforms/ST-Nucleo-L152RE/.
byte pins[] = { D2, D3, D4, D5, D6 };
const char* pinnames[] = { "D2", "D3", "D4", "D5", "D6" };
#elif defined(ESP32)                // nano esp32 - not sure if correct pinout
byte pins[] = { D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12 };
const char* pinnames[] = { "D2", "D3", "D4", "D5", "D6", "D7", "D8", "D9", " D10", "D11", "D12" };
#elif defined(ESP_H)                // ESP8266 Wemos D1 Mini. if properly not set may trigger watchdog
byte pins[] = { D1, D2, D3, D4, D5, D6, D7, D8 };
const char* pinnames[] = { "D1", "D2", "D3", "D4", "D5", "D6", "D7", "D8" };
#elif defined(ARDUINO_ARCH_RP2040)  // Raspberry Pi Pico
byte pins[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 };
const char* pinnames[] = { "GP0", "GP1", "GP2", "GP3", "GP4", "GP5", "GP6", "GP7", "GP8", "GP9", "GP10", "GP11", "GP12", "GP13", "GP14", "GP15", "GP16", "GP17", "GP18", "GP19", "GP20" };
#include <EEPROM.h>
#define EEPROMSTORE
#else  // DEFAULT \
       // Arduino Pro. usable digital 2-12,14-10. 13=LED 0,1=serial
byte pins[] = { 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
const char* pinnames[] = { "DIG_2", "DIG_3", "DIG_4", "DIG_5", "DIG_6",
                           "DIG_7", "DIG_8", "DIG_9", "DIG_10", "DIG_11" };
#include <EEPROM.h>
#define EEPROMSTORE
#endif

// Once you have found the JTAG pins you can define
// the following to allow for the boundary scan and
// irenum functions to be run.
//
// The value is the index to the pin in pins[] array
// eg. TCK=3 is used as pins[TCK] pins[3]
#define PIN_NOT_SET 0xff
const char* jtagpinnames[] = { "TCK", "TMS", "TDO", "TDI", "TRST" };

byte TCK = PIN_NOT_SET;
byte TMS = PIN_NOT_SET;
byte TDO = PIN_NOT_SET;
byte TDI = PIN_NOT_SET;
byte TRST = PIN_NOT_SET;

// Pattern used for scan() and loopback() tests
#define PATTERN_LEN 64
// Use something random when trying find JTAG lines:
static char pattern[PATTERN_LEN] = "0110011101001101101000010111001001";
// Use something more determinate when trying to find
// length of the DR register:
//static char pattern[PATTERN_LEN] = "1000000000000000000000000000000000";

// Max. number of JTAG enabled chips (MAX_DEV_NR) and length
// of the DR register together define the number of
// iterations to run for scan_idcode():
#define MAX_DEV_NR 8
#define IDCODE_LEN 32

// Target specific, check your documentation or guess
#define SCAN_LEN 1890  // used for IR enum. bigger the better
#define IR_LEN 5
// IR registers must be IR_LEN wide:
#define IR_IDCODE "01100"  // always 011
#define IR_SAMPLE "10100"  // always 101
#define IR_PRELOAD IR_SAMPLE

/*
 * END USER DEFINITIONS
 */



// TAP TMS states we care to use. NOTE: MSB sent first
// Meaning ALL TAP and IR codes have their leftmost
// bit sent first. This might be the reverse of what
// documentation for your target(s) show.
#define TAP_RESET "11111"  // looping 1 will return \
                           // IDCODE if reg available
#define TAP_SHIFTDR "111110100"
#define TAP_SHIFTIR "1111101100"  // -11111> Reset -0> Idle -1> SelectDR \
                                  // -1> SelectIR -0> CaptureIR -0> ShiftIR

// Ignore TCK, TMS use in loopback check:
#define IGNOREPIN 0xFFFF
// Flags configured by UI:
#define TRUE 255
#define FALSE 0
boolean VERBOSE = FALSE;
boolean DELAY = FALSE;
long DELAYUS = 50;
boolean JTAG_PULLUP = TRUE;


const byte pinslen = sizeof(pins) / sizeof(pins[0]);
byte active_pins[32];
byte active_pins_count = 0;

// For 3.3v AVR boards. Cuts clock in half. Also see cmd in setup()
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))


/*
* Return a pointer to the jtag pins given an index.
*
*/
byte* jtag_ptr_from_idx(const int idx) {
  byte* curr = NULL;
  switch (idx) {
    case 0:
      curr = &TCK;
      break;
    case 1:
      curr = &TMS;
      break;
    case 2:
      curr = &TDO;
      break;
    case 3:
      curr = &TDI;
      break;
    case 4:
      curr = &TRST;
      break;
  }
  return curr;
}



void setup(void) {
#ifdef HALFCLOCK
  // for 3.3v boards. Cuts clock in half
  // normally only on avr based arduino & teensy hardware
  CPU_PRESCALE(0x01);
#endif
#if defined(ARDUINO_ARCH_RP2040)
  EEPROM.begin(512);
  while (!Serial && millis() < 5000)
    ;
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  Serial.begin(115200);

  active_pins_count = pinslen;
  if (active_pins_count > 32) active_pins_count = 32;
  for (int i = 0; i < active_pins_count; i++) active_pins[i] = i;

  byte* curr;
  for (int i = 0; i < 5; ++i) {
    curr = jtag_ptr_from_idx(i);
    if (*curr == PIN_NOT_SET) {
#ifdef EEPROMSTORE
      // if we are on arduino we can save/restore jtag pins from
      // the eeprom
      *curr = EEPROM.read(i);
#endif
      if (*curr < 0 || *curr >= pinslen) {
        *curr = i;
      }
    }
  }
}


/*
 * Set the JTAG TAP state machine
 */
void tap_state(const char* tap_state, int tck, int tms) {
#ifdef DEBUGTAP
  Serial.print("tap_state: tms set to: ");
#endif
  int tap_state_length = strlen(tap_state);
  for (int i = 0; i < tap_state_length; i++) {
    if (DELAY) delayMicroseconds(DELAYUS);
    digitalWrite(tck, LOW);
    digitalWrite(tms, tap_state[i] - '0');  // conv from ascii pattern
#ifdef DEBUGTAP
    Serial.print(tap_state[i] - '0', DEC);
#endif
    digitalWrite(tck, HIGH);  // rising edge shifts in TMS
  }
#ifdef DEBUGTAP
  Serial.println();
#endif
}

#ifdef DEBUGTAP
static void pulse_tms(int tck, int tms, int s_tms) {
  if (tck == IGNOREPIN) return;
  digitalWrite(tck, LOW);
  digitalWrite(tms, s_tms);
  digitalWrite(tck, HIGH);
}
#endif
static void pulse_tdi(int tck, int tdi, int s_tdi) {
  if (DELAY) delayMicroseconds(DELAYUS);
  if (tck != IGNOREPIN) digitalWrite(tck, LOW);
  digitalWrite(tdi, s_tdi);
  if (tck != IGNOREPIN) digitalWrite(tck, HIGH);
}
byte pulse_tdo(int tck, int tdo) {
  byte tdo_read;
  if (DELAY) delayMicroseconds(DELAYUS);
  digitalWrite(tck, LOW);  // read in TDO on falling edge
  tdo_read = digitalRead(tdo);
  digitalWrite(tck, HIGH);
  return tdo_read;
}

/*
 * Reset all configured pins to INPUT state
 */
void reset_all_pins() {
  for (int i = 0; i < pinslen; i++) {
    pinMode(pins[i], INPUT);
    // internal pullups default to logic 1:
    if (JTAG_PULLUP) digitalWrite(pins[i], HIGH);
  }
}

/*
 * Initialize specific pins to a default state
 * default with no arguments: all pins as INPUTs
 */
void init_pins(int tck = IGNOREPIN, int tms = IGNOREPIN, int tdi = IGNOREPIN, int ntrst = IGNOREPIN, bool reset = true) {
#if defined(ESP32)
  esp_task_wdt_reset();  //reset timer (feed watchdog)
#elif defined(ESP8266) || defined(ESP_H)
  ESP.wdtFeed();  //feed watchdog
#endif
  // default all to INPUT state
  if (reset) {
    reset_all_pins();
  }
  // TCK = output
  if (tck != IGNOREPIN) pinMode(tck, OUTPUT);
  // TMS = output
  if (tms != IGNOREPIN) pinMode(tms, OUTPUT);
  // tdi = output
  if (tdi != IGNOREPIN) pinMode(tdi, OUTPUT);
  // ntrst = output, fixed to 1
  if (ntrst != IGNOREPIN) {
    pinMode(ntrst, OUTPUT);
    digitalWrite(ntrst, HIGH);
  }
}


/*
 * send pattern[] to TDI and check for output on TDO
 * This is used for both loopback, and Shift-IR testing, i.e.
 * the pattern may show up with some delay.
 * return: 0 = no match
 *       1 = match
 *       2 or greater = no pattern found but line appears active
 *
 * if retval == 1, *reglen returns the length of the register
 */
static int check_data(char pattern[], int iterations, int tck, int tdi, int tdo,
                      int* reg_len) {
  int i;
  int w = 0;
  int plen = strlen(pattern);
  char tdo_read;
  char tdo_prev;
  int nr_toggle = 0;  // count how often tdo toggled
  /* we store the last plen (<=PATTERN_LEN) bits,
   *  rcv[0] contains the oldest bit */
  char rcv[PATTERN_LEN];

  tdo_prev = '0' + (digitalRead(tdo) == HIGH);

  for (i = 0; i < iterations; i++) {
    if ((i & 0x0F) == 0) yield();
    /* output pattern and incr write index */
    pulse_tdi(tck, tdi, pattern[w++] - '0');
    if (!pattern[w])
      w = 0;

    /* read from TDO and put it into rcv[] */
    tdo_read = '0' + (digitalRead(tdo) == HIGH);

    nr_toggle += (tdo_read != tdo_prev);
    tdo_prev = tdo_read;

    if (i < plen)
      rcv[i] = tdo_read;
    else {
      memmove(rcv, rcv + 1, plen - 1);
      rcv[plen - 1] = tdo_read;
    }

    /* check if we got the pattern in rcv[] */
    if (i >= (plen - 1)) {
      if (!memcmp(pattern, rcv, plen)) {
        *reg_len = i + 1 - plen;
        return 1;
      }
    }
  } /* for(i=0; ... ) */

  *reg_len = 0;
  return nr_toggle > 1 ? nr_toggle : 0;
}

static void print_pins(int tck, int tms, int tdo, int tdi, int ntrst) {
#if defined(ARDUINO_ARCH_RP2040) || defined(ESP32) || defined(ESP8266)
  if (ntrst != IGNOREPIN) Serial.printf(" ntrst:%s", pinnames[ntrst]);
  Serial.printf(" tck:%s tms:%s tdo:%s", pinnames[tck], pinnames[tms], pinnames[tdo]);
  if (tdi != IGNOREPIN) Serial.printf(" tdi:%s", pinnames[tdi]);
#else
  if (ntrst != IGNOREPIN) {
    Serial.print(" ntrst:");
    Serial.print(pinnames[ntrst]);
  }
  Serial.print(" tck:");
  Serial.print(pinnames[tck]);
  Serial.print(" tms:");
  Serial.print(pinnames[tms]);
  Serial.print(" tdo:");
  Serial.print(pinnames[tdo]);
  if (tdi != IGNOREPIN) {
    Serial.print(" tdi:");
    Serial.print(pinnames[tdi]);
  }
#endif
}

/*
 * Shift JTAG TAP to ShiftIR state. Send pattern to TDI and check
 * for output on TDO
 */
static void scan() {
  int tck, tms, tdo, tdi, ntrst;
  int checkdataret = 0;
  int reg_len;
  printProgStr(PSTR("================================\r\n"
                    "Starting scan for pattern:"));
  Serial.println(pattern);

  reset_all_pins();

  for (int i_ntrst = 0; i_ntrst < active_pins_count; i_ntrst++) {
    ntrst = active_pins[i_ntrst];
    // Setup nTRST
    pinMode(pins[ntrst], OUTPUT);
    digitalWrite(pins[ntrst], HIGH);

#ifdef DEBUG_FIXED_TCK
    for (tck = DEBUG_FIXED_TCK; tck <= DEBUG_FIXED_TCK; tck++) {
#else
    for (int i_tck = 0; i_tck < active_pins_count; i_tck++) {
      tck = active_pins[i_tck];
#endif
      if (tck == ntrst) continue;

      // Setup TCK
      pinMode(pins[tck], OUTPUT);

      for (int i_tms = 0; i_tms < active_pins_count; i_tms++) {
        tms = active_pins[i_tms];
        if (tms == ntrst || tms == tck) continue;
        yield();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Toggle LED for progress

        // Setup TMS
        pinMode(pins[tms], OUTPUT);

        for (int i_tdo = 0; i_tdo < active_pins_count; i_tdo++) {
          tdo = active_pins[i_tdo];
          if (tdo == ntrst || tdo == tck || tdo == tms) continue;
          // tdo is already INPUT from reset_all_pins

          for (int i_tdi = 0; i_tdi < active_pins_count; i_tdi++) {
            tdi = active_pins[i_tdi];
            if (tdi == ntrst || tdi == tck || tdi == tms || tdi == tdo) continue;

            // Setup TDI
            pinMode(pins[tdi], OUTPUT);

            tap_state(TAP_SHIFTIR, pins[tck], pins[tms]);
            checkdataret = check_data(pattern, (2 * PATTERN_LEN),
                                      pins[tck], pins[tdi], pins[tdo], &reg_len);

            if (checkdataret == 1 && reg_len > 0) {
              Serial.printf("FOUND! ntrst:%s tck:%s tms:%s tdo:%s tdi:%s IRlen:%d\r\n",
                            pinnames[ntrst], pinnames[tck], pinnames[tms], pinnames[tdo], pinnames[tdi], reg_len);
              Serial.flush();
              delay(10);
            }

            // Revert TDI
            pinMode(pins[tdi], INPUT);
            if (JTAG_PULLUP) digitalWrite(pins[tdi], HIGH);
          }
        }
        // Revert TMS
        pinMode(pins[tms], INPUT);
        if (JTAG_PULLUP) digitalWrite(pins[tms], HIGH);
      }
      // Revert TCK
      pinMode(pins[tck], INPUT);
      if (JTAG_PULLUP) digitalWrite(pins[tck], HIGH);
    }
    // Revert nTRST
    pinMode(pins[ntrst], INPUT);
    if (JTAG_PULLUP) digitalWrite(pins[ntrst], HIGH);
  }
  printProgStr(PSTR("================================\r\n"));
  Serial.flush();
  delay(100);
  while (Serial.available()) Serial.read();  // Clear any stray input during scan
}

void select_scan_pins() {
  Serial.println("Enter pin indices to scan (e.g. 18 19 17 16 0 1 2), end with '.' or newline:");
  active_pins_count = 0;
  while (active_pins_count < 32) {
    while (!Serial.available())
      ;
    char c = Serial.peek();
    if (c == '\n' || c == '\r' || c == '.') {
      Serial.read();  // consume terminator
      break;
    }
    int p = Serial.parseInt();
    if (p >= 0 && p < pinslen) {
      active_pins[active_pins_count++] = p;
      Serial.printf("Added %s (index %d)\r\n", pinnames[p], p);
    }
    // Skip spaces/commas
    while (Serial.available() && (Serial.peek() == ' ' || Serial.peek() == ',')) Serial.read();
  }
  Serial.printf("Selected %d pins for scanning.\r\n", active_pins_count);
}

/*
 * Check for pins that pass pattern[] between tdi and tdo
 * regardless of JTAG TAP state (tms, tck ignored).
 *
 * TDO, TDI pairs that match indicate possible shorts between
 * pins. Pins that do not match but are active might indicate
 * that the patch cable used is not shielded well enough. Run
 * the test again without the cable connected between controller
 * and target. Run with the verbose flag to examine closely.
 */
static void loopback_check() {
  int tdo, tdi;
  int checkdataret = 0;
  int reg_len;

  printProgStr(PSTR("================================\r\n"
                    "Starting loopback check...\r\n"));
  for (tdo = 0; tdo < pinslen; tdo++) {
    for (tdi = 0; tdi < pinslen; tdi++) {
      if (tdi == tdo) continue;

      if (VERBOSE) {
        Serial.print(" tdo:");
        Serial.print(pinnames[tdo]);
        Serial.print(" tdi:");
        Serial.print(pinnames[tdi]);
        Serial.print("    ");
      }
      init_pins(IGNOREPIN /*tck*/, IGNOREPIN /*tms*/, pins[tdi], IGNOREPIN /*ntrst*/);
      checkdataret = check_data(pattern, (2 * PATTERN_LEN), IGNOREPIN, pins[tdi], pins[tdo], &reg_len);
      if (checkdataret == 1) {
        Serial.print("FOUND! ");
        Serial.print(" tdo:");
        Serial.print(pinnames[tdo]);
        Serial.print(" tdi:");
        Serial.print(pinnames[tdi]);
        Serial.print(" reglen:");
        Serial.println(reg_len);
      } else if (checkdataret > 1) {
        Serial.print("active ");
        Serial.print(" tdo:");
        Serial.print(pinnames[tdo]);
        Serial.print(" tdi:");
        Serial.print(pinnames[tdi]);
        Serial.print("  bits toggled:");
        Serial.println(checkdataret);
      } else if (VERBOSE) Serial.println();
    }
  }
  printProgStr(PSTR("================================\r\n"));
}

static void list_pin_names() {
  int pin;
  Serial.print("The configured pins are:\r\n");
  for (pin = 0; pin < pinslen; pin++) {
    Serial.print(pinnames[pin]);
    Serial.print(" ");
  }
  Serial.println();
}

/*
 * Scan TDO for IDCODE. Handle MAX_DEV_NR many devices.
 * We feed zeros into TDI and wait for the first 32 of them to come out at TDO (after n * 32 bit).
 * As IEEE 1149.1 requires bit 0 of an IDCODE to be a "1", we check this bit.
 * We record the first bit from the idcodes into bit0.
 * (oppposite to the old code).
 * If we get an IDCODE of all ones, we assume that the pins are wrong.
 * This scan assumes IDCODE is the default DR between TDI and TDO.
 */
static void scan_idcode() {
  int tck, tms, tdo, tdi, ntrst;
  int i, j;
  int tdo_read;
  uint32_t idcodes[MAX_DEV_NR];
  printProgStr(PSTR("================================\r\n"
                    "Starting scan for IDCODE...\r\n"
                    "(assumes IDCODE default DR)\r\n"));

  reset_all_pins();

  for (int i_ntrst = 0; i_ntrst < active_pins_count; i_ntrst++) {
    ntrst = active_pins[i_ntrst];
    pinMode(pins[ntrst], OUTPUT);
    digitalWrite(pins[ntrst], HIGH);

    for (int i_tck = 0; i_tck < active_pins_count; i_tck++) {
      tck = active_pins[i_tck];
      if (tck == ntrst) continue;
      pinMode(pins[tck], OUTPUT);

      for (int i_tms = 0; i_tms < active_pins_count; i_tms++) {
        tms = active_pins[i_tms];
        if (tms == ntrst || tms == tck) continue;
        yield();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        pinMode(pins[tms], OUTPUT);

        for (int i_tdo = 0; i_tdo < active_pins_count; i_tdo++) {
          tdo = active_pins[i_tdo];
          if (tdo == ntrst || tdo == tck || tdo == tms) continue;

          for (int i_tdi = 0; i_tdi < active_pins_count; i_tdi++) {
            tdi = active_pins[i_tdi];
            if (tdi == ntrst || tdi == tck || tdi == tms || tdi == tdo) continue;
            pinMode(pins[tdi], OUTPUT);

            if (VERBOSE) {
              Serial.printf("ntrst:%s tck:%s tms:%s tdo:%s tdi:%s    ",
                            pinnames[ntrst], pinnames[tck], pinnames[tms], pinnames[tdo], pinnames[tdi]);
            }

            /* we hope that IDCODE is the default DR after reset */
            tap_state(TAP_RESET, pins[tck], pins[tms]);
            tap_state(TAP_SHIFTDR, pins[tck], pins[tms]);

            /* j is the number of bits we pulse into TDI and read from TDO */
            for (i = 0; i < MAX_DEV_NR; i++) {
              idcodes[i] = 0;
              for (j = 0; j < IDCODE_LEN; j++) {
                /* we send '0' in */
                pulse_tdi(pins[tck], pins[tdi], 0);
                tdo_read = digitalRead(pins[tdo]);
                if (tdo_read)
                  idcodes[i] |= ((uint32_t)1) << j;

                if (VERBOSE)
                  Serial.print(tdo_read, DEC);
              } /* for(j=0; ... ) */
              if (VERBOSE) {
                Serial.printf(" 0x%08X\r\n", idcodes[i]);
              }
              /* save time: break at the first idcode with bit0 != 1 */
              if (!(idcodes[i] & 1) || idcodes[i] == 0xffffffff)
                break;
            } /* for(i=0; ...) */

            if (i > 0) {
              Serial.printf("ntrst:%s tck:%s tms:%s tdo:%s tdi:%s  devices: %d\r\n",
                            pinnames[ntrst], pinnames[tck], pinnames[tms], pinnames[tdo], pinnames[tdi], i);
              for (j = 0; j < i; j++) {
                Serial.printf("  0x%08X (%s)\r\n", idcodes[j], fpga_lookup(idcodes[j]));
              }
              Serial.flush();
              delay(10);
            } /* if (i > 0) */

            pinMode(pins[tdi], INPUT);
            if (JTAG_PULLUP) digitalWrite(pins[tdi], HIGH);
          }
        }
        pinMode(pins[tms], INPUT);
        if (JTAG_PULLUP) digitalWrite(pins[tms], HIGH);
      }
      pinMode(pins[tck], INPUT);
      if (JTAG_PULLUP) digitalWrite(pins[tck], HIGH);
    }
    pinMode(pins[ntrst], INPUT);
    if (JTAG_PULLUP) digitalWrite(pins[ntrst], HIGH);
  }

  printProgStr(PSTR("================================\r\n"));
  Serial.flush();
  delay(100);
  while (Serial.available()) Serial.read();
}


static void shift_bypass() {
  int tdi, tdo, tck;
  int checkdataret;
  int reg_len;

  printProgStr(PSTR("================================\r\n"
                    "Starting shift of pattern through bypass...\r\n"
                    "Assumes bypass is the default DR on reset.\r\n"
                    "Hence, no need to check for TMS. Also, currently\r\n"
                    "not checking for nTRST, which might not work\r\n"));
  for (tck = 0; tck < pinslen; tck++) {
    for (tdi = 0; tdi < pinslen; tdi++) {
      if (tdi == tck) continue;
      for (tdo = 0; tdo < pinslen; tdo++) {
        if (tdo == tck) continue;
        if (tdo == tdi) continue;
        if (VERBOSE) {
          Serial.printf(" tck:%s tdi:%s tdo:%s    ", pinnames[tck], pinnames[tdi], pinnames[tdo]);
        }

        init_pins(pins[tck], IGNOREPIN /*tms*/, pins[tdi], IGNOREPIN /*ntrst*/);
        // if bypass is default on start, no need to init TAP state
        checkdataret = check_data(pattern, (2 * PATTERN_LEN), pins[tck], pins[tdi], pins[tdo], &reg_len);
        if (checkdataret == 1) {
          Serial.printf("FOUND!  tck:%s tdo:%s tdi:%s\r\n", pinnames[tck], pinnames[tdo], pinnames[tdi]);
        } else if (checkdataret > 1) {
          Serial.printf("active  tck:%s tdo:%s tdi:%s  bits toggled:%d\r\n", pinnames[tck], pinnames[tdo], pinnames[tdi], checkdataret);
        } else if (VERBOSE) Serial.println();
      }
    }
  }
  printProgStr(PSTR("================================\r\n"));
}
/* ir_state()
 * Set TAP to Reset then ShiftIR.
 * Shift in state[] as IR value.
 * Switch to ShiftDR state and end.
 */
void ir_state(const char* state, int tck, int tms, int tdi) {
#ifdef DEBUGIR
  Serial.println("ir_state: set TAP to ShiftIR:");
#endif
  tap_state(TAP_SHIFTIR, tck, tms);
#ifdef DEBUGIR
  Serial.print("ir_state: pulse_tdi to: ");
#endif
  for (int i = 0; i < IR_LEN; i++) {
    if (DELAY) delayMicroseconds(DELAYUS);
    // TAP/TMS changes to Exit IR state (1) must be executed
    // at same time that the last TDI bit is sent:
    if (i == IR_LEN - 1) {
      digitalWrite(tms, HIGH);  // ExitIR
#ifdef DEBUGIR
      Serial.print(" (will be in ExitIR after next bit) ");
#endif
    }
    pulse_tdi(tck, tdi, state[i] - '0');
#ifdef DEBUGIR
    Serial.print(state[i] - '0', DEC);
#endif
    // TMS already set to 0 "shiftir" state to shift in bit to IR
  }
#ifdef DEBUGIR
  Serial.println("\r\nir_state: Change TAP from ExitIR to ShiftDR:");
#endif
  // a reset would cause IDCODE instruction to be selected again
  tap_state("1100", tck, tms);  // -1> UpdateIR -1> SelectDR -0> CaptureDR -0> ShiftDR
}
static void sample(int iterations, int tck, int tms, int tdi, int tdo, int ntrst = IGNOREPIN) {
  printProgStr(PSTR("================================\r\n"
                    "Starting sample (boundary scan)...\r\n"));
  init_pins(tck, tms, tdi, ntrst);

  // send instruction and go to ShiftDR
  ir_state(IR_SAMPLE, tck, tms, tdi);

  // Tell TAP to go to shiftout of selected data register (DR)
  // is determined by the instruction we sent, in our case
  // SAMPLE/boundary scan
  for (int i = 0; i < iterations; i++) {
    // no need to set TMS. It's set to the '0' state to
    // force a Shift DR by the TAP
    Serial.print(pulse_tdo(tck, tdo), DEC);
    if (i % 32 == 31) Serial.print(" ");
    if (i % 128 == 127) Serial.println();
  }
}

char ir_buf[IR_LEN + 1];
static void brute_ir(int iterations, int tck, int tms, int tdi, int tdo, int ntrst = IGNOREPIN) {
  printProgStr(PSTR("================================\r\n"
                    "Starting brute force scan of IR instructions...\r\n"
                    "NOTE: If Verbose mode is off output is only printed\r\n"
                    "      after activity (bit changes) are noticed and\r\n"
                    "      you might not see the first bit of output.\r\n"
                    "IR_LEN set to "));
  Serial.println(IR_LEN, DEC);

  init_pins(tck, tms, tdi, ntrst);
  int iractive;
  byte tdo_read;
  byte prevread;
  for (uint32_t ir = 0; ir < (1UL << IR_LEN); ir++) {
    iractive = 0;
    // send instruction and go to ShiftDR (ir_state() does this already)
    // convert ir to string.
    for (int i = 0; i < IR_LEN; i++)
      ir_buf[i] = bitRead(ir, i) + '0';
    ir_buf[IR_LEN] = 0;  // terminate
    ir_state(ir_buf, tck, tms, tdi);
    // we are now in TAP_SHIFTDR state

    prevread = pulse_tdo(tck, tdo);
    for (int i = 0; i < iterations - 1; i++) {
      // no need to set TMS. It's set to the '0' state to force a Shift DR by the TAP
      tdo_read = pulse_tdo(tck, tdo);
      if (tdo_read != prevread) iractive++;

      if (iractive || VERBOSE) {
        Serial.print(prevread, DEC);
        if (i % 16 == 15) Serial.print(" ");
        if (i % 128 == 127) Serial.println();
      }
      prevread = tdo_read;
    }
    if (iractive || VERBOSE) {
      Serial.print(prevread, DEC);
      Serial.print("  Ir ");
      Serial.print(ir_buf);
      Serial.print("  bits changed ");
      Serial.println(iractive, DEC);
    }
  }
}

void set_pattern() {
  int i;
  char c;

  Serial.print("Enter new pattern of 1's or 0's (terminate with new line or '.'):\r\n"
               "> ");
  i = 0;
  while (1) {
    c = Serial.read();
    switch (c) {
      case '0':
      case '1':
        if (i < (PATTERN_LEN - 1)) {
          pattern[i++] = c;
          Serial.print(c);
        }
        break;
      case '\n':
      case '\r':
      case '.':  // bah. for the arduino serial console which does not pass us \n
        pattern[i] = 0;
        Serial.println();
        Serial.print("new pattern set [");
        Serial.print(pattern);
        Serial.println("]");
        return;
    }
  }
}

void configure_pins() {
  Serial.println("Available pins, the index is based on them");
  for (int pin = 0; pin < pinslen; pin++) {
    Serial.print(pinnames[pin]);
    Serial.print("[");
    Serial.print(pin);
    Serial.print("]");
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("Current pin configuration");
  print_pins(TCK, TMS, TDO, TDI, TRST);
  byte* curr = NULL;
  for (int i = 0; i < 5; ++i) {
    curr = jtag_ptr_from_idx(i);
    do {
      // Print current value
      Serial.println();
      Serial.print(jtagpinnames[i]);
      Serial.print("(");
      Serial.print(*curr, DEC);
      Serial.print(") = ");
      // Read the new pin configuration
      while (!Serial.available())
        ;
      *curr = Serial.parseInt();
    } while (*curr < 0 || *curr >= pinslen);
    Serial.print(*curr);
#ifdef EEPROMSTORE
    // Save to eeprom
    EEPROM.write(i, *curr);
#endif
  }
#if defined(ARDUINO_ARCH_RP2040)
  EEPROM.commit();
#endif
  Serial.print("\nConfiguration saved\n");
}

// given a PROGMEM string, use Serial.print() to send it out
void printProgStr(const char* str) {
  if (!str) return;
#ifdef ARDUINO_ARCH_AVR
  char c;
  while ((c = pgm_read_byte(str++)))
    Serial.print(c);
#else
  Serial.print(str);
#endif
}

void help() {
  printProgStr(PSTR("Short and long form commands can be used.\r\n\r\nSCANS\r\n-----\r\n"));
  printProgStr(PSTR("s > pattern scan\r\n   Scans for all JTAG pins. Attempts to set TAP state to\r\n"));
  printProgStr(PSTR("   DR_SHIFT and then shift the pattern through the DR.\r\n"));
  printProgStr(PSTR("p > pattern set\r\n   currently: ["));
  Serial.print(pattern);
  printProgStr(PSTR("]\r\n\r\ni > idcode scan\r\n   Assumes IDCODE is default DR on reset. Ignores TDI.\r\n"));
  printProgStr(PSTR("   Sets TAP state to DR_SHIFT and prints TDO to console\r\n"));
  printProgStr(PSTR("   when TDO appears active. Human examination required to\r\n"));
  printProgStr(PSTR("   determine if actual IDCODE is present. Run several times.\r\n"));
  printProgStr(PSTR("\r\nb > bypass scan\r\n   Assumes BYPASS is default DR on reset. Ignores TMS.\r\n"));
  printProgStr(PSTR("\r\nERATTA\r\n------\r\nl > loopback check\r\n   connected there is a short or a false-positive\r\n"));
  printProgStr(PSTR("r > pullups\r\n   internal pullups on inputs, on/off.\r\n"));
  printProgStr(PSTR("v > verbose\r\n   on/off. print tdo bits to console. If on, this slows scan.\r\n"));
  printProgStr(PSTR("d > delay\r\n   on/off. will slow down scan.\r\n"));
  printProgStr(PSTR("c > configure pin\r\n   configure jtag pins\r\n"));
  printProgStr(PSTR("k > select scan pins\r\n   manually select/order pins to scan\r\n"));
  printProgStr(PSTR("h > help\r\n"));
  printProgStr(PSTR("n > list pin names\r\n\r\nOTHER JTAG TESTS\r\n----------------\r\n"));
  printProgStr(PSTR("x > boundary scan\r\n   checks code defined tdo for 4000+ bits.\r\n"));
  printProgStr(PSTR("y > irenum\r\n   sets every possible IR and checks DR output.\r\n"));
  Serial.flush();
}
/*
 * main()
 */
#define CMDLEN 20
char command[CMDLEN];
int dummy;
void loop() {
  if (Serial.available()) {
    // READ COMMAND
    delay(5);  // hoping read buffer is idle after 5 ms
    int i = 0;
    while (Serial.available() && i < CMDLEN - 1)
      command[i++] = Serial.read();

    while (Serial.available()) Serial.read();  // Clear remaining buffer
    command[i] = 0;                            // terminate string
    Serial.println(command);                   // echo back

    // EXECUTE COMMAND
    if (strcmp(command, "pattern scan") == 0 || strcmp(command, "s") == 0)
      scan();
    else if (strcmp(command, "pattern scan single") == 0 || strcmp(command, "1") == 0) {
      Serial.print("pins");
      print_pins(TCK, TMS, TDO, TDI, TRST);
      init_pins(pins[TCK], pins[TMS], pins[TDI], pins[TRST] /*ntrst*/);
      tap_state(TAP_SHIFTIR, pins[TCK], pins[TMS]);
      if (check_data(pattern, (2 * PATTERN_LEN), pins[TCK], pins[TDI], pins[TDO], &dummy))
        Serial.println("found pattern or other");
      else
        Serial.println("no pattern found");
    } else if (strcmp(command, "pattern set") == 0 || strcmp(command, "p") == 0)
      set_pattern();
    else if (strcmp(command, "loopback check") == 0 || strcmp(command, "l") == 0)
      loopback_check();
    else if (strcmp(command, "idcode scan") == 0 || strcmp(command, "i") == 0)
      scan_idcode();
    else if (strcmp(command, "bypass scan") == 0 || strcmp(command, "b") == 0)
      shift_bypass();
    else if (strcmp(command, "boundary scan") == 0 || strcmp(command, "x") == 0) {
      Serial.print("pins");
      print_pins(TCK, TMS, TDO, TDI, TRST);
      Serial.println();
      sample(SCAN_LEN + 100, pins[TCK], pins[TMS], pins[TDI], pins[TDO], pins[TRST]);
    } else if (strcmp(command, "irenum") == 0 || strcmp(command, "y") == 0)
      brute_ir(SCAN_LEN, pins[TCK], pins[TMS], pins[TDI], pins[TDO], pins[TRST]);
    else if (strcmp(command, "verbose") == 0 || strcmp(command, "v") == 0) {
      if (VERBOSE == FALSE) {
        VERBOSE = TRUE;
      } else {
        VERBOSE = FALSE;
      }
      Serial.println(VERBOSE ? "Verbose ON" : "Verbose OFF");
    } else if (strcmp(command, "delay") == 0 || strcmp(command, "d") == 0) {
      if (DELAY == FALSE) {
        DELAY = TRUE;
      } else {
        DELAY = FALSE;
      }
      Serial.println(DELAY ? "Delay ON" : "Delay OFF");
    } else if (strcmp(command, "delay -") == 0 || strcmp(command, "-") == 0) {
      Serial.print("Delay microseconds: ");
      if (DELAYUS != 0 && DELAYUS > 1000) DELAYUS -= 1000;
      else if (DELAYUS != 0 && DELAYUS > 100) DELAYUS -= 100;
      else if (DELAYUS != 0) DELAYUS -= 10;
      Serial.println(DELAYUS, DEC);
    } else if (strcmp(command, "delay +") == 0 || strcmp(command, "+") == 0) {
      Serial.print("Delay microseconds: ");
      if (DELAYUS < 100) DELAYUS += 10;
      else if (DELAYUS <= 1000) DELAYUS += 100;
      else DELAYUS += 1000;
      Serial.println(DELAYUS, DEC);
    } else if (strcmp(command, "pullups") == 0 || strcmp(command, "r") == 0) {
      if (JTAG_PULLUP == FALSE) {
        JTAG_PULLUP = TRUE;
      } else {
        JTAG_PULLUP = FALSE;
      }
      Serial.println(JTAG_PULLUP ? "Pullups ON" : "Pullups OFF");
    } else if (strcmp(command, "help") == 0 || strcmp(command, "h") == 0)
      help();
    else if (strcmp(command, "list pin names") == 0 || strcmp(command, "n") == 0)
      list_pin_names();
    else if (strcmp(command, "select scan pins") == 0 || strcmp(command, "k") == 0)
      select_scan_pins();
    else if (strcmp(command, "configure pin") == 0 || strcmp(command, "c") == 0)
      configure_pins();
    else {
      Serial.println("unknown command");
      help();
    }
    Serial.print("\n> ");
  }
}
