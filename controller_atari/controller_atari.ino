/* controller_atari - sketch to turn an Arduibo into a tester for a Atari-style joysticks
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
 */

/* Pinout of the atari joystick/paddle (looking at the front of the female connector)

        5   4   3   2   1            1 = Joystick UP
     ___|___|___|___|___|__          2 = Joystick DOWN
	 \  o   o   o   o   o /          3 = Joystick LEFT
	  \   o   o   o   o  /           4 = Joystick RIGHT
       ---|---|---|---|--            5 = Paddle-B Pot
	      9   8   7   6              6 = Fire button
		                             7 = +5V
									 8 = GND
									 9 = Paddle-A Pot

	The paddle-pots (pin 5 and 9) should be connected through a little RC-circuit:

    arduino pin ----- R ------------ atari controller pin     with R = 100 Ohm and C = 1nF
	                        |
							C
							|
						   GND

	The "value" of the paddle reported is the "time" it takes for the capacitor to be
	charged by the controller.
*/

/* Please NOTE: Atari driving controller is not supported (because I don't have access to one) */

/* Arduino pin definitions */
const uint8_t PIN_UP	= 22;
const uint8_t PIN_DOWN	= 24;
const uint8_t PIN_LEFT	= 26;
const uint8_t PIN_RIGHT	= 28;
const uint8_t PIN_FIRE	= 30;

const uint8_t PIN_PADDLEA = 34;
const uint8_t PIN_PADDLEB = 32;

/* controller button flags */
const uint16_t BTN_NONE    = 0;
const uint16_t BTN_UP	   = 1 << 0;
const uint16_t BTN_DOWN    = 1 << 1;
const uint16_t BTN_LEFT    = 1 << 2;
const uint16_t BTN_RIGHT   = 1 << 3;
const uint16_t BTN_FIRE    = 1 << 4;
const uint8_t  BTN_COUNT   = 5;

const uint8_t  PADDLE_A	   = 1 << 0;
const uint8_t  PADDLE_B	   = 1 << 1;

static const char *BTN_NAMES[BTN_COUNT] = {
	"Up",
	"Down",
	"Left",
	"Right",
	"Fire",
};

static const uint8_t BTN_PIN_MAP[BTN_COUNT] = {
	PIN_UP,
	PIN_DOWN,
	PIN_LEFT,
	PIN_RIGHT,
	PIN_FIRE
};

// values for arduino mega 2560
#define PADDLE_MAX_VALUE	1000
#define PADDLE_A_SHIFT		3		// pin 34 = PC3
#define PADDLE_B_SHIFT		5		// pin 32 = PC5
#define PADDLE_DDR			DDRC
#define PADDLE_PORT			PORTC
#define PADDLE_PIN			PINC
#define PADDLE_DEADZONE		10

// global variables
static uint16_t button_state = 0;

static uint8_t  paddle_state = 0;
static int		paddle_a_value = 0;
static int		paddle_b_value = 0;

/* helper functions */
void discharge_capacitors() {
	// discharge capacitor by connecting pin to ground
	PADDLE_DDR |= ((1 << PADDLE_A_SHIFT) | (1 << PADDLE_B_SHIFT));
	PADDLE_PORT &= ~((1 << PADDLE_A_SHIFT) | (1 << PADDLE_B_SHIFT));
}

void disconnect_capacitors() {
	// disconnect arduino from the capacitors so the paddle controller can charge it
	PADDLE_DDR &= ~((1 << PADDLE_B_SHIFT) | (1 << PADDLE_A_SHIFT));
}

void print_joystick_state() {
	static uint16_t previous_button_state = BTN_NONE;

	if (button_state == previous_button_state) {
		return;
	}
	previous_button_state = button_state;

	if (button_state == BTN_NONE) {
		Serial.println("No buttons pressed");
		return;
	}

	for (uint8_t btn = 0; btn < BTN_COUNT; ++btn) {
		if (button_state & (1 << btn)) {
			Serial.print(BTN_NAMES[btn]);
			Serial.print(" ");
		}
	}
	Serial.println();
}

void print_paddle_state() {

	static uint16_t prev_button_state = BTN_NONE;
	static int		prev_paddle_a = -1;
	static int		prev_paddle_b = -1;

	bool new_line = false;

	if (button_state != prev_button_state) {
		if (button_state & BTN_LEFT) {
			Serial.print("Paddle-1 button ");
			new_line = true;
		}
		if (button_state & BTN_RIGHT) {
			Serial.print("Paddle-2 button ");
			new_line = true;
		}
		prev_button_state = button_state;
	}

	if (abs(paddle_a_value - prev_paddle_a) > PADDLE_DEADZONE) {
		if (prev_paddle_a >= 0) {
			Serial.print("Paddle-1 = ");
			Serial.print(paddle_a_value);
			Serial.print(" ");
			new_line = true;
		}
		prev_paddle_a = paddle_a_value;
	}

	if (abs(paddle_b_value - prev_paddle_b) > PADDLE_DEADZONE) {
		if (prev_paddle_b >= 0) {
			Serial.print("Paddle-2 = ");
			Serial.print(" ");
			Serial.print(paddle_b_value);
			new_line = true;
		}
		prev_paddle_b = paddle_b_value;
	}

	if (new_line) {
		Serial.println();
	}
}

void print_paddle_connected() {

	static uint8_t previous_state = 0;

	if (previous_state == paddle_state) {
		return;
	}
	previous_state = paddle_state;

	switch (paddle_state) {
		case 0:
			Serial.println("No paddles detected");
			return;
		case PADDLE_A:
			Serial.println("Only paddle-A detected");
			return;
		case PADDLE_B:
			Serial.println("Only paddle-B detected");
			return;
		case PADDLE_A | PADDLE_B:
			Serial.println("Paddles detected");
			return;
		default:
			Serial.println("Invalid paddle state");
	}
}

void print_controller_state() {

	print_paddle_connected();

	if (paddle_state > 0) {
		print_paddle_state();
	} else {
		print_joystick_state();
	}
}

uint16_t read_controller() {

	uint16_t state = 0;

	for (int btn = 0; btn < BTN_COUNT; ++btn) {
		if (!digitalRead(BTN_PIN_MAP[btn])) {
			state |= 1 << btn;
		}
	}

	return state;
}


int8_t read_paddles() {
	const int MASK_A = 1 << PADDLE_A_SHIFT;
	const int MASK_B = 1 << PADDLE_B_SHIFT;

	disconnect_capacitors();

	paddle_a_value = 0;
	paddle_b_value = 0;

	for (int i = 0; i < PADDLE_MAX_VALUE; ++i) {
		int input = ~PADDLE_PIN;
		paddle_a_value += (input & MASK_A) >> PADDLE_A_SHIFT;
		paddle_b_value += (input & MASK_B) >> PADDLE_B_SHIFT;
	}

	discharge_capacitors();

	return ((paddle_a_value < PADDLE_MAX_VALUE) ? PADDLE_A : 0) |
		    ((paddle_b_value < PADDLE_MAX_VALUE) ? PADDLE_B : 0);
}


/* device setup */
void setup() {
	/* enable serial I/O with host computer */
	Serial.begin(9600);

	/* setup pins: input */
	for (int btn = 0; btn < BTN_COUNT; ++btn) {
		pinMode(BTN_PIN_MAP[btn], INPUT_PULLUP);
	}

	/* setup paddles */
	discharge_capacitors();
	delay(1);
}

/* main loop */
void loop() {
	button_state = read_controller();
	paddle_state = read_paddles();

	print_controller_state();

	delay(16);		// poll controller approx. 60 times a second
}
