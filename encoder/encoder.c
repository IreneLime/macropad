#include <avr/io.h>

void init(void) {
  // Set LED pins as output
  DDRD |= (1 << 5);  // PD5
  DDRB |= (1 << 0);  // PB0

  // Set encoder pins as input
  DDRB &= ~(1 << 2);  // PB2
  DDRB &= ~(1 << 6);  // PB6

  // Enable pull-up resistors on encoder pins
  PORTB |= (1 << 2);  // PB2
  PORTB |= (1 << 6);  // PB6
}

void illuminate_right_led() {
  PORTD |= (1 << 5);  // Turn off LED on PD5
  PORTB &= ~(1 << 0); // Turn on LED on PB0
}

void illuminate_left_led() {
  PORTB |= (1 << 0);  // Turn off LED on PB0
  PORTD &= ~(1 << 5); // Turn on LED on PD5
}

void leds_off() {
  PORTB |= (1 << 0);  // Turn off LED on PB0
  PORTD |= (1 << 5);  // Turn off LED on PD5
}

int main(void) {
  init();
  leds_off();

  // Read initial state of PB2, store in bit 0
  uint8_t last_state = (PINB & (1 << 2)) >> 2;

  while (1) {
    // Read current state of PB2, store in bit 0
    uint8_t current_state = (PINB & (1 << 2)) >> 2;

    // Has A's state changed?
    if (current_state != last_state) {
      // Read current state of PB6, store in bit 0
      uint8_t current_b_state = (PINB & (1 << 6)) >> 6;
      // is B at the same state as A?
      if (current_b_state == current_state) {
	// Counterclockwise rotation
	illuminate_left_led();
      } else {
	// Clockwise rotation
	illuminate_right_led();
      }
    }

    last_state = current_state;
  }

  return 0;
}
