/* cpu8_register - test fixture to validate a register-board for a custom 74-logic family
 * 8-bit cpu / computer.
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

/* pin definitions */
const int PIN_RES_B = 29;
const int PIN_CLK   = 27;
const int PIN_WE_B  = 25;
const int PIN_OE_B  = 23;

const int PIN_DB7	= 31;
const int PIN_DB6	= 33;
const int PIN_DB5	= 35;
const int PIN_DB4	= 37;
const int PIN_DB3	= 39;
const int PIN_DB2	= 41;
const int PIN_DB1	= 43;
const int PIN_DB0	= 45;

const int PIN_R0	= 22;
const int PIN_R1	= 24;
const int PIN_R2	= 26;
const int PIN_R3	= 28;
const int PIN_R4	= 30;
const int PIN_R5	= 32;
const int PIN_R6	= 34;
const int PIN_R7	= 36;

/* global data */
bool g_test_ok = true;

/* utility functions */
void data_mode_input() {
	pinMode(PIN_DB0, INPUT);
	pinMode(PIN_DB1, INPUT);
	pinMode(PIN_DB2, INPUT);
	pinMode(PIN_DB3, INPUT);
	pinMode(PIN_DB4, INPUT);
	pinMode(PIN_DB5, INPUT);
	pinMode(PIN_DB6, INPUT);
	pinMode(PIN_DB7, INPUT);
}

void data_mode_output() {
	pinMode(PIN_DB0, OUTPUT);
	pinMode(PIN_DB1, OUTPUT);
	pinMode(PIN_DB2, OUTPUT);
	pinMode(PIN_DB3, OUTPUT);
	pinMode(PIN_DB4, OUTPUT);
	pinMode(PIN_DB5, OUTPUT);
	pinMode(PIN_DB6, OUTPUT);
	pinMode(PIN_DB7, OUTPUT);
}

void alu_data_mode_input() {
	pinMode(PIN_R0, INPUT);
	pinMode(PIN_R1, INPUT);
	pinMode(PIN_R2, INPUT);
	pinMode(PIN_R3, INPUT);
	pinMode(PIN_R4, INPUT);
	pinMode(PIN_R5, INPUT);
	pinMode(PIN_R6, INPUT);
	pinMode(PIN_R7, INPUT);
}

void write_data(uint8_t data) {
	digitalWrite(PIN_DB0, (data >> 0) & 0x01);
	digitalWrite(PIN_DB1, (data >> 1) & 0x01);
	digitalWrite(PIN_DB2, (data >> 2) & 0x01);
	digitalWrite(PIN_DB3, (data >> 3) & 0x01);
	digitalWrite(PIN_DB4, (data >> 4) & 0x01);
	digitalWrite(PIN_DB5, (data >> 5) & 0x01);
	digitalWrite(PIN_DB6, (data >> 6) & 0x01);
	digitalWrite(PIN_DB7, (data >> 7) & 0x01);
}

uint8_t read_data() {
	uint8_t result = (digitalRead(PIN_DB0) << 0) |
					 (digitalRead(PIN_DB1) << 1) |
					 (digitalRead(PIN_DB2) << 2) |
					 (digitalRead(PIN_DB3) << 3) |
					 (digitalRead(PIN_DB4) << 4) |
					 (digitalRead(PIN_DB5) << 5) |
					 (digitalRead(PIN_DB6) << 6) |
					 (digitalRead(PIN_DB7) << 7);
	return result;
}

uint8_t read_alu_data() {
	uint8_t result = (digitalRead(PIN_R0) << 0) |
					 (digitalRead(PIN_R1) << 1) |
					 (digitalRead(PIN_R2) << 2) |
					 (digitalRead(PIN_R3) << 3) |
					 (digitalRead(PIN_R4) << 4) |
					 (digitalRead(PIN_R5) << 5) |
					 (digitalRead(PIN_R6) << 6) |
					 (digitalRead(PIN_R7) << 7);
	return result;
}

void output_byte(uint8_t data) {
	data_mode_output();
	write_data(data);
	digitalWrite(PIN_WE_B, LOW);
	digitalWrite(PIN_CLK,  LOW);
	delayMicroseconds(10);
	digitalWrite(PIN_CLK,  HIGH);
	digitalWrite(PIN_WE_B, HIGH);
	delayMicroseconds(10);
	data_mode_input();
}

uint8_t input_byte() {
	data_mode_input();

	digitalWrite(PIN_OE_B, LOW);
	digitalWrite(PIN_CLK,  LOW);
	uint8_t result = read_data();
	delayMicroseconds(10);
	digitalWrite(PIN_CLK,  HIGH);
	digitalWrite(PIN_OE_B, HIGH);
	delayMicroseconds(10);

	return result;
}

void check_value(uint8_t value) {
	Serial.print("Writing databus value 0x");
	Serial.println(value, HEX);
	output_byte(value);
	delay(100);

	uint8_t check = input_byte();
	Serial.print("Read databus value 0x");
	Serial.println(check, HEX);
	delay(100);

	if (check != value) {
		g_test_ok = false;
		Serial.println("----- TEST FAILED -----");
	}

	check = read_alu_data();

	Serial.print("Read alu-bus value 0x");
	Serial.println(check, HEX);
	delay(100);

	if (check != value) {
		g_test_ok = false;
		Serial.println("----- TEST FAILED -----");
	}
}

/* main functions */
void setup() {
	/* enable serial I/O with host computer */
	Serial.begin(9600);

	/* pin mode - control signals */
	pinMode(PIN_RES_B, OUTPUT);
	pinMode(PIN_CLK,   OUTPUT);
	pinMode(PIN_WE_B,  OUTPUT);
	pinMode(PIN_OE_B,  OUTPUT);

	/* pin mode - data pins */
	data_mode_input();
	alu_data_mode_input();

	/* set default control signals */
	digitalWrite(PIN_RES_B, HIGH);
	digitalWrite(PIN_CLK,   HIGH);
	digitalWrite(PIN_WE_B,  HIGH);
	digitalWrite(PIN_OE_B,  HIGH);

	/* reset target device */
	digitalWrite(PIN_RES_B, LOW);
	delay(500);
	digitalWrite(PIN_RES_B, HIGH);
}

void loop() {
	for (uint8_t i = 0; g_test_ok && i <= 8; ++i) {
		check_value(1 << i);
	}

	for (uint8_t i = 0; g_test_ok && i <= 255; ++i) {
		check_value(i);
	}
}
