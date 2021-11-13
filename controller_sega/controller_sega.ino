/* controller_sega - sketch to turn an Arduino into a tester for Sega MasterSystem and
					 MegaDrive (Genesis) controllers
 *
 * Copyright (c) 2021 - Johan Smet - Licensed under the BSD-3-CLAUSE license (reproduced below)

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Based upon information published by Jon Thysell at
 * https://github.com/jonthysell/SegaController/wiki/How-To-Read-Sega-Controllers
 */

/* Please NOTE: not tested with a 6-button MegaDrive controller; I do not currently have one available. */

/* Arduino pin definitions */
const uint8_t PIN_DB1 = 22;
const uint8_t PIN_DB2 = 24;
const uint8_t PIN_DB3 = 26;
const uint8_t PIN_DB4 = 28;
// DB5 = +5V
const uint8_t PIN_DB6 = 30;
const uint8_t PIN_DB7 = 32;
// DB8 = GND
const uint8_t PIN_DB9 = 34;

/* controller button flags */
const uint16_t BTN_NONE  = 0;
const uint16_t BTN_UP	 = 1 << 0;
const uint16_t BTN_DOWN  = 1 << 1;
const uint16_t BTN_LEFT  = 1 << 2;
const uint16_t BTN_RIGHT = 1 << 3;
const uint16_t BTN_B	 = 1 << 4;
const uint16_t BTN_C	 = 1 << 5;
const uint16_t BTN_A	 = 1 << 6;
const uint16_t BTN_X	 = 1 << 7;
const uint16_t BTN_Y	 = 1 << 8;
const uint16_t BTN_Z	 = 1 << 9;
const uint16_t BTN_START = 1 << 10;
const uint16_t BTN_MODE  = 1 << 11;

const uint16_t BTN_MD3   = 1 << 14;		// flags to indicate that either a MegaDrive/Genesis
const uint16_t BTN_MD6   = 1 << 15;		//		3-button or 6-button controller was detected

const uint16_t MASK_CONTROLLER = 0xC000;
const uint16_t MASK_BUTTONS	   = 0x3fff;

const uint8_t  MS_BTN_COUNT = 6;		// Sega Master System
const uint8_t  MD_BTN_COUNT = 12;		// Sega MegaDrive/Genesis (6-button controller)

const char *MS_BTN_NAMES[MS_BTN_COUNT] = {
	"Up",
	"Down",
	"Left",
	"Right",
	"1/Start",
	"2"
};

const char *MD_BTN_NAMES[MD_BTN_COUNT] = {
	"Up",
	"Down",
	"Left",
	"Right",
	"B",
	"C",
	"A",
	"X",
	"Y",
	"Z",
	"Start",
	"Mode"
};

/* input mappings */
typedef struct {
	uint8_t		pin;
	uint16_t	button;
} sega_ms_input_t;

sega_ms_input_t ms_input[] = {
	{ PIN_DB1, BTN_UP },
	{ PIN_DB2, BTN_DOWN },
	{ PIN_DB3, BTN_LEFT },
	{ PIN_DB4, BTN_RIGHT },
	{ PIN_DB6, BTN_B },
	{ PIN_DB9, BTN_C },
};

typedef struct {
	uint8_t		pin;
	uint16_t	button[3];
} sega_md_input_t;

sega_md_input_t md_input[] = {
	{ PIN_DB1, {BTN_NONE,	BTN_UP,		BTN_Z}},
	{ PIN_DB2, {BTN_NONE,	BTN_DOWN,	BTN_Y}},
	{ PIN_DB3, {BTN_NONE,	BTN_LEFT,	BTN_X}},
	{ PIN_DB4, {BTN_NONE,	BTN_RIGHT,	BTN_MODE}},
	{ PIN_DB6, {BTN_A,		BTN_B,		BTN_NONE}},
	{ PIN_DB9, {BTN_START,	BTN_C,		BTN_NONE}},
};

/* global variables */
static uint16_t current_state = 0;

/* helper functions */
void print_state_buttons(uint16_t state, uint8_t count, const char **names) {
	for (uint8_t btn = 0; btn < count; ++btn) {
		if (state & (1 << btn)) {
			Serial.print(names[btn]);
			Serial.print(" ");
		}
	}
}

void print_state(uint16_t state) {

	if ((state & MASK_CONTROLLER) != (current_state & MASK_CONTROLLER)) {
		if (state & BTN_MD6) {
			Serial.println("Sega MegaDrive/Genesis 6-button controller detected");
		} else if (state & BTN_MD3) {
			Serial.println("Sega MegaDrive/Genesis 3-button controller detected");
		} else {
			Serial.println("Sega MasterSystem or no controller present");
		}
	}

	if ((state & MASK_BUTTONS) == 0) {
		return;
	}

	if (state & BTN_MD3) {
		print_state_buttons(state, MD_BTN_COUNT, MD_BTN_NAMES);
	} else {
		print_state_buttons(state, MS_BTN_COUNT, MS_BTN_NAMES);
	}

	Serial.println();
}

uint16_t read_master_system_controller() {
	uint16_t state = 0;

	for (int idx = 0; idx < sizeof(ms_input) / sizeof(ms_input[0]); ++idx) {
		if (digitalRead(ms_input[idx].pin) == LOW) {
			state |= ms_input[idx].button;
		}
	}

	return state;
}

uint16_t read_megadrive_controller(int phase) {
	uint16_t state = 0;

	for (int idx = 0; idx < sizeof(md_input) / sizeof(md_input[0]); ++idx) {
		if (digitalRead(md_input[idx].pin) == LOW) {
			state |= md_input[idx].button[phase];
		}
	}

	return state;
}

uint16_t read_controller() {

	// CYCLE 1 - SELECT LOW
	digitalWrite(PIN_DB7, LOW);

	// >> pins 3 & 4 are low when a MegaDrive/Genesis controller (either 3 or 6 buttons) is connected
	bool md_present = !digitalRead(PIN_DB3) && !digitalRead(PIN_DB4);

	if (!md_present) {
		// either a MasterSystem or no controller connected
		return read_master_system_controller();
	}

	// CYCLE 1 - SELECT HIGH
	digitalWrite(PIN_DB7, HIGH);

	// CYCLE 2 - SELECT LOW
	digitalWrite(PIN_DB7, LOW);
	uint16_t state = BTN_MD3 | read_megadrive_controller(0);

	// CYCLE 2 - SELECT HIGH
	digitalWrite(PIN_DB7, HIGH);
	state |= read_megadrive_controller(1);

	// CYCLE 3 - SELECT LOW
	digitalWrite(PIN_DB7, LOW);

	// >> pins 1 & 2 are low when a MegaDrive/Genesis 6 button controller is connected
	bool md6_present = !digitalRead(PIN_DB1) && !digitalRead(PIN_DB2);

	if (!md6_present) {
		return state;
	}

	// CYCLE 3 - SELECT HIGH
	digitalWrite(PIN_DB7, HIGH);
	state |= BTN_MD6 | read_megadrive_controller(3);

	// CYCLE 4 - SELECT LOW
	digitalWrite(PIN_DB7, LOW);

	// CYCLE 4 - SELECT HIGH
	digitalWrite(PIN_DB7, HIGH);

	return state;
}

/* device setup */
void setup() {
	/* enable serial I/O with host computer */
	Serial.begin(9600);

	/* setup input pins */
	pinMode(PIN_DB1, INPUT_PULLUP);
	pinMode(PIN_DB2, INPUT_PULLUP);
	pinMode(PIN_DB3, INPUT_PULLUP);
	pinMode(PIN_DB4, INPUT_PULLUP);
	pinMode(PIN_DB6, INPUT_PULLUP);
	pinMode(PIN_DB9, INPUT_PULLUP);

	pinMode(PIN_DB7, OUTPUT);
	digitalWrite(PIN_DB7, HIGH);
}

/* main loop */
void loop() {
	uint16_t state = read_controller();

	if (state != current_state) {
		print_state(state);
		current_state = state;
	}

	delay(16);		// poll controller approx. 60 times a second
}
