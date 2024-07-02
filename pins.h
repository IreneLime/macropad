#pragma once

typedef struct {
  volatile uint8_t* port;
  volatile uint8_t* pin;
  uint8_t pin_num;
} pin_t;
