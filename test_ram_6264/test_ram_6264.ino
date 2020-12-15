/* Quick sketch to test a 6264 SRAM chip
	(using 5V Ardiuno, connect chip to correct pins)
*/

/* pin definitions */
const int RAM_PIN_D0 = 22;
const int RAM_PIN_D1 = 23;
const int RAM_PIN_D2 = 24;
const int RAM_PIN_D3 = 25;
const int RAM_PIN_D4 = 26;
const int RAM_PIN_D5 = 27;
const int RAM_PIN_D6 = 28;
const int RAM_PIN_D7 = 29;

const int RAM_PIN_A0 = 30;
const int RAM_PIN_A1 = 31;
const int RAM_PIN_A2 = 32;
const int RAM_PIN_A3 = 33;
const int RAM_PIN_A4 = 34;
const int RAM_PIN_A5 = 35;
const int RAM_PIN_A6 = 36;
const int RAM_PIN_A7 = 37;
const int RAM_PIN_A8 = 38;
const int RAM_PIN_A9 = 39;
const int RAM_PIN_A10 = 40;
const int RAM_PIN_A11 = 41;
const int RAM_PIN_A12 = 42;

const int RAM_PIN_WE = 44;
const int RAM_PIN_OE = 45;

const int R_DELAY = 1;
const int W_DELAY = 1;

void set_data_pinMode(int mode) {
	for (int pin = RAM_PIN_D0; pin <= RAM_PIN_D7; ++pin) {
		pinMode(pin, mode);
	}
}

void set_address_output(int address) {
	for (int pin = RAM_PIN_A0; pin <= RAM_PIN_A12; ++pin) {
		digitalWrite(pin, address & 1);
		address = address >> 1;
	}
}

void set_data_output(int data) {
	set_data_pinMode(OUTPUT);

	for (int pin = RAM_PIN_D0; pin <= RAM_PIN_D7; ++pin) {
		digitalWrite(pin, data & 1);
		data = data >> 1;
	}
}

int read_data_pins() {

	int result = 0;

	set_data_pinMode(INPUT);

	for (int pin = RAM_PIN_D7; pin >= RAM_PIN_D0; --pin) {
		int value = digitalRead(pin);
		result = (result << 1) | value;
	}

	return result;
}

void write_data(int address, int data) {

	/*Serial.print("Write to address ");
	Serial.print(address, HEX);
	Serial.print(" - value = ");
	Serial.println(data, HEX); */

	digitalWrite(RAM_PIN_WE, HIGH);
	digitalWrite(RAM_PIN_OE, HIGH);

	set_address_output(address);
	delay(W_DELAY);
	set_data_output(data);
	delay(W_DELAY);
	digitalWrite(RAM_PIN_WE, LOW);
	delay(W_DELAY);
	digitalWrite(RAM_PIN_WE, HIGH);
}

int read_data(int address) {

	digitalWrite(RAM_PIN_WE, HIGH);
	digitalWrite(RAM_PIN_OE, HIGH);

	set_address_output(address);
	delay(R_DELAY);
	digitalWrite(RAM_PIN_OE, LOW);
	delay(R_DELAY);

	int result = read_data_pins();
	delay(R_DELAY);
	digitalWrite(RAM_PIN_OE, HIGH);

	/*Serial.print("Read from address ");
	Serial.print(address, HEX);
	Serial.print(" - value = ");
	Serial.println(result, HEX); */

	return result;
}

/* device setup */
void setup() {
	for (int pin = RAM_PIN_A0; pin <= RAM_PIN_A12; ++pin) {
		pinMode(pin, OUTPUT);
	}

	set_data_pinMode(OUTPUT);
	pinMode(RAM_PIN_WE, OUTPUT);
	pinMode(RAM_PIN_OE, OUTPUT);

	digitalWrite(RAM_PIN_WE, HIGH);
	digitalWrite(RAM_PIN_OE, HIGH);
  
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(9600);
}

int address = 0x0000;

void loop() {
	int test_data[] = {0x55, 0xaa, 0xff, 0x00};

	if (address % 0x100 == 0) {
		Serial.print("At address 0x");
		Serial.println(address, HEX);
	}

	for (int i = 0; i < 4; ++i) {

		write_data(address, test_data[i]);
		int result = read_data(address);

		if (result != test_data[i]) {
			Serial.print("ERROR at address 0x");
			Serial.print(address, HEX);
			Serial.print(" using value 0x");
			Serial.print(test_data[i], HEX);
			Serial.print(" received value 0x");
			Serial.println(result, HEX);
		}
	}

	address = (address + 1) & 0x1fff;
}
