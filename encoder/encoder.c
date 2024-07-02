#include <avr/io.h>

void init(void) {
  // Set LED pins as output
  DDRD |= (1 << 5);  // PD5
  DDRB |= (1 << 0);  // PB0

  // Set encoder pins as input
  DDRB &= ~(1 << 2);  // PB2
  DDRB &= ~(1 << 6);  // PB6

  // Enable pull-up resistors on encoder pins
  // Set the state as high (pull-up) for encoder to register
  PORTB |= (1 << 2);  // PB2
  PORTB |= (1 << 6);  // PB6
}

// LED illumination on microcontroller
// 5 v to power the LED
// 0 V to shut off the LED
void illuminate_right_led() {
  PORTD |= (1 << 5);  // Turn off LED on PD5 
  PORTB &= ~(1 << 0); // Turn on LED on PB0
}

void illuminate_left_led() {
  PORTB |= (1 << 0);  // Turn off LED on PB0
  PORTD &= ~(1 << 5); // Turn on LED on PD5
}

void leds_off(){
    PORTB |= (1 << 0); //Turn off LED on PB0
    PORTD |= (1 << 5); // Turn off LED on PD5
}

int main(void){
    init();
    leds_off();

    // Read initial state of PB2 (pin A rotary encoder 1), store in bit 0
    uint8_t last_state = (PINB & (1<<2)) >> 2; //shift back 2 to put it in the least significant bit

    while(1) {
        // Read current state of PB2, store in bit 0
        uint8_t current_state = (PINB & (1<<2)) >> 2;

        //Check if the previous state == current state
        if (current_state != last_state) {
            // Check the state of the next pin (read current state of PB2, store in bit 0)
            uint8_t current_b_state = (PINB & (1 << 6)) >> 6;
            // If the same are the same -> counterclockwise
            if (current_b_state == current_state) { //Compare bit 0
                illuminate_left_led();
            }
            // If the states are not the same -> clockwise
            else {
                illuminate_right_led();
            }
        }
        // Push current state to last state to read a new current state next time
        last_state = current_state;
    }

    return 0;
}