/* Sega MegaDrive/Genesis Speed&Language Control 
 *  
 * The Sega MegaDrive motherboards are generally the same for all regions.  Refresh Rate (50hz/60hz or PAL/NTSC) and language (Japanese/Export) are controlled by 4 jumpers.
 * There are quite a few articles/project available that describe how to modify a MegaDrive to allow changing these properties at will, either by adding physical switches to
 * the case or switchless by using a microcontroller/Arduino.
 * 
 * This Arduino sketch is nothing special. There are more advanced implementations available. I just wanted to write my own version because I never worked with an Arduino before
 * and though it would be a nice starter experiment. Tested on an Arduino Nano (and clones).
 *  
 * Copyright (c) 2019 - Johan Smet - Licensed under the BSD-3-CLAUSE license (reproduced below)
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Installation :
 * (NOTE: Only tested with an "IC BD MD5 PAL" board but should work with other revisions. Jumpers might be located somewhere else. Earlier revision use active-high reset line!)
 * 
 *  JP1-4 are on the right side of the board and will be connected differently according to the original region of the Megadrive.
 *       European                     American                   Japanse
 *     50Hz / English             60 Hz / English            60 Hz / Japanese
 *    
 *  (A) O     O (GND) JP1      (A) O     O (GND) JP1      (A) O --- O (GND) JP1
 *      |                          |                          |
 *      O --- O (+5V) JP2          O --- O (+5V) JP2          O     O (+5V) JP2
 *  
 *  (B) O --- O (GND) JP3      (B) O     O (GND) JP3          O     O (GND) JP3
 *      |                          |                          |
 *      O     O (+5V) JP4          O --- O (+5V) JP4          O --- O (+5V) JP4
 *  
 *  Cut the traces between the jumpers and connect D5 of the Arduino to (A) and D6 to (B).
 *  
 *  To intercept the reset switch: on the bottom of the PCB cut the trace between C80 and the reset button.
 *  Connect D12 to the reset button and D11 to C80.
 *  
 *  Optional: attach an RGB LED (common cathode) to D2/D3/D4 and GND to display the current region of your MegaDrive.
 *    D2 = American / D3 = European / D4 = Japanese
 */

/* TODO:
 *   - detect reset active level at startup to support earlier board revisions
 *   - add support for common anode status LED
 */

#include <EEPROM.h>

/* pin definitions */
const int PIN_LED_B = 2;    // Blue
const int PIN_LED_G = 3;    // Green
const int PIN_LED_R = 4;    // Red

const int PIN_LANG = 5;     // pin to control language
const int PIN_SPEED = 6;    // pin to control refresh rate

const int PIN_RESET_IN = 12;    // pin to read reset button
const int PIN_RESET_OUT = 11;   // pin to reset console

/* general configuration */
const unsigned long DEBOUNCE_DELAY = 20;    // in ms
const unsigned long MD_STATE_DELAY = 1000;  // shorter = reset - longer = state change (in ms)
const unsigned long RESET_DURATION = 500;   // how long reset will be asserted (in ms)
const byte RESET_ACTIVE_LEVEL = LOW;        // reset is active low on later revision but active high on earlier boards
const int EEPROM_ADDRESS = 63;              // eeprom address where mode will be saved
const unsigned long EEPROM_DELAY = 10000;   // how long region must be stable before it is saved to eeprom (in ms) - to avoid unnecessary writes to eeprom while cycling regions

const byte NUM_REGIONS = 3;

/* led colors per region */
const byte LED_STATES[NUM_REGIONS][3] = {
// B     G     R
  {HIGH, LOW,  LOW},  // US
  {LOW,  HIGH, LOW},  // EU
  {LOW,  LOW,  HIGH}  // JP
};

/* jumper values per region */
const byte JUMPER_STATES[NUM_REGIONS][2] {
// lang  speed
  {HIGH, HIGH},       // US
  {HIGH, LOW},        // EU
  {LOW,  HIGH}        // JP
};

/* global variables */
int megadrive_region = 0;
unsigned long megadrive_pending_save = 0;

/* helper functions */
void output_jumpers() {
  digitalWrite(PIN_LANG, JUMPER_STATES[megadrive_region][0]);
  digitalWrite(PIN_SPEED, JUMPER_STATES[megadrive_region][1]);
}

void output_status_led() {
  digitalWrite(PIN_LED_B, LED_STATES[megadrive_region][0]); 
  digitalWrite(PIN_LED_G, LED_STATES[megadrive_region][1]);
  digitalWrite(PIN_LED_R, LED_STATES[megadrive_region][2]);
}

void clear_status_led() {
  digitalWrite(PIN_LED_B, LOW); 
  digitalWrite(PIN_LED_G, LOW);
  digitalWrite(PIN_LED_R, LOW);
}

void megadrive_next_region() {
  megadrive_region = (megadrive_region + 1) % NUM_REGIONS;
  megadrive_pending_save = millis() + EEPROM_DELAY;
}

void megadrive_reset() {
  digitalWrite(PIN_RESET_OUT, RESET_ACTIVE_LEVEL);
  delay(RESET_DURATION);
  digitalWrite(PIN_RESET_OUT, !RESET_ACTIVE_LEVEL);
}

void handle_reset_input() {
  static unsigned long last_debounce_time = 0;
  static unsigned long reset_on_time = 0;
  static bool last_reset_value = false;
  static bool reset_value = false;

  bool cur_value = digitalRead(PIN_RESET_IN);

  if (cur_value != last_reset_value) {
    last_debounce_time = millis();
  } 
  
  if ((millis() - last_debounce_time) > DEBOUNCE_DELAY) {
    // pin is stable - check if reset value has been changed
    if (cur_value != reset_value) {
      reset_value = cur_value;

      if (reset_value == RESET_ACTIVE_LEVEL) {  
        // reset pressed: keep track of time
        reset_on_time = millis();
        Serial.print("reset pressed at ");
        Serial.println(reset_on_time);
      } else {
        // reset released: action depends on length of press
        if ((millis() - reset_on_time) < MD_STATE_DELAY) {
          megadrive_reset();
        }
      }
    } 

    // as soon as reste down time exceeds MD_STATE_DELAY: change state of megadrive
    if (reset_value == RESET_ACTIVE_LEVEL && millis() > (reset_on_time + MD_STATE_DELAY)) {
      Serial.print("next_state: ");
      Serial.print(millis());
      Serial.print(" ... ");
      megadrive_next_region();
      reset_on_time = millis() + 3000;
      Serial.println(reset_on_time);
      
    }   
  }

  last_reset_value = cur_value;
}

void save_persistent_data() {
  if (megadrive_pending_save > 0 && millis() > megadrive_pending_save) {
    EEPROM.update(EEPROM_ADDRESS, megadrive_region);
    megadrive_pending_save = 0;
  }
}

/* device setup */
void setup() {
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_RESET_IN, INPUT_PULLUP);
  pinMode(PIN_RESET_OUT, OUTPUT);
  pinMode(PIN_LANG, OUTPUT);
  pinMode(PIN_SPEED, OUTPUT);

  clear_status_led();
  digitalWrite(PIN_RESET_OUT, !RESET_ACTIVE_LEVEL);

  megadrive_region = EEPROM.read(EEPROM_ADDRESS) % NUM_REGIONS;
}

/* main loop */
void loop() {
  handle_reset_input();
  output_jumpers();
  output_status_led();
  save_persistent_data();
}
