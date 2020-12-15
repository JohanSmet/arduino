/* quick sketch to test a cheap logic analyzer I'm trying out */

/* pin definitions */
const int NUM_PINS = 8;
const int LA_PINS[NUM_PINS] = {22, 24, 26, 28, 30, 32, 34, 36};

void output(int target) {
	for (int i = 0; i < NUM_PINS; ++i) {
		digitalWrite(LA_PINS[i], (target == i) ? HIGH : LOW);
	}
}

/* device setup */
void setup() {
	for (int i = 0; i < NUM_PINS; ++i) {
		pinMode(LA_PINS[i], OUTPUT);
	}
}

void loop() {

	for (int i = 0; i < NUM_PINS; ++i) {
		output(-1);
		delay(100);
		output(i);
		delay(100);
	}
}
