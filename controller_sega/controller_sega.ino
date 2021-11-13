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
 */

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
const uint16_t BTN_UP	 = 1 << 0;
const uint16_t BTN_DOWN  = 1 << 1;
const uint16_t BTN_LEFT  = 1 << 2;
const uint16_t BTN_RIGHT = 1 << 3;
const uint16_t BTN_A	 = 1 << 4;
const uint16_t BTN_B	 = 1 << 5;

const uint8_t  MS_BTN_COUNT = 6;	// Sega Master System

const char *MS_BTN_NAMES[MS_BTN_COUNT] = {
	"Up",
	"Down",
	"Left",
	"Right",
	"1/Start",
	"2"
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
	{ PIN_DB6, BTN_A },
	{ PIN_DB9, BTN_B },
};

/* global variables */
static uint16_t current_state = 0;

/* helper functions */
void print_state(uint16_t state) {
	if (state == 0) {
		return;
	}

	for (uint8_t btn = 0; btn < MS_BTN_COUNT; ++btn) {
		if (state & (1 << btn)) {
			Serial.print(MS_BTN_NAMES[btn]);
			Serial.print(" ");
		}
	}

	Serial.println();
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
	uint16_t state = 0;

	for (int idx = 0; idx < sizeof(ms_input) / sizeof(ms_input[0]); ++idx) {
		if (digitalRead(ms_input[idx].pin) == LOW) {
			state |= ms_input[idx].button;
		}
	}

	if (state != current_state) {
		print_state(state);
		current_state = state;
	}

	delay(16);		// poll controller approx. 60 times a second
}
