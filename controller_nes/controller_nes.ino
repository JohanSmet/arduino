/* controller_nes - sketch to turn an Arduibo into a tester for a Nintendo Entertainment System controller
 *
 * Copyright (c) 2022 - Johan Smet - Licensed under the BSD-3-CLAUSE license (reproduced below)

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

/* Arduino pin definitions */
const uint8_t PIN_SHIFT_CLOCK = 22;
const uint8_t PIN_SHIFT_LATCH = 24;
const uint8_t PIN_DATA = 26;
const uint8_t PIN_DATA_D3 = 28;			// zapper - light sensor
const uint8_t PIN_DATA_D4 = 30;			// zapper - trigger pull

/* controller button flags - in reverse order of how they are transmitted by the controller */
const uint16_t BTN_NONE    = 0;
const uint16_t BTN_RIGHT   = 1 << 0;
const uint16_t BTN_LEFT    = 1 << 1;
const uint16_t BTN_DOWN    = 1 << 2;
const uint16_t BTN_UP	   = 1 << 3;
const uint16_t BTN_START   = 1 << 4;
const uint16_t BTN_SELECT  = 1 << 5;
const uint16_t BTN_B	   = 1 << 6;
const uint16_t BTN_A	   = 1 << 7;
const uint8_t  NES_BTN_COUNT = 8;

const char *NES_BTN_NAMES[NES_BTN_COUNT] = {
	"Right",
	"Left",
	"Down",
	"Up",
	"Start",
	"Select",
	"B",
	"A"
};

/* helper functions */
void print_state(uint16_t state) {
	for (uint8_t btn = 0; btn < NES_BTN_COUNT; ++btn) {
		if (state & (1 << btn)) {
			Serial.print(NES_BTN_NAMES[btn]);
			Serial.print(" ");
		}
	}
	Serial.println();
}

uint16_t read_controller() {

	// tell controller to load the state of the buttons into the latch
	digitalWrite(PIN_SHIFT_LATCH, HIGH);
	delayMicroseconds(12);
	digitalWrite(PIN_SHIFT_LATCH, LOW);

	uint16_t state = 0;

	for (int i = 0; i < NES_BTN_COUNT; ++i) {
		// read a databit from the controller
		state <<= 1;
		state |= !digitalRead(PIN_DATA);

		// tell the controller to send the next bit
		digitalWrite(PIN_SHIFT_CLOCK, HIGH);
		delayMicroseconds(12);
		digitalWrite(PIN_SHIFT_CLOCK, LOW);
	}

	return state;
}

/* device setup */
void setup() {
	/* enable serial I/O with host computer */
	Serial.begin(9600);

	/* setup pins: input */
	pinMode(PIN_DATA,		 INPUT_PULLUP);
	pinMode(PIN_DATA_D3,	 INPUT_PULLUP);
	pinMode(PIN_DATA_D4,	 INPUT_PULLUP);

	/* setup pins: output */
	pinMode(PIN_SHIFT_CLOCK, OUTPUT);
	digitalWrite(PIN_SHIFT_CLOCK, LOW);
	pinMode(PIN_SHIFT_LATCH, OUTPUT);
	digitalWrite(PIN_SHIFT_LATCH, LOW);
}

/* main loop */
void loop() {
	uint16_t current_state = 0;
	uint16_t state = read_controller();

	if (state != current_state) {
		print_state(state);
		current_state = state;
	}

	delay(16);		// poll controller approx. 60 times a second
}
