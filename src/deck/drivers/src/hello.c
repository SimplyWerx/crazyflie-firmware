#define DEBUG_MODULE "HelloDeck"
#include "debug.h"

#include "deck.h"

#include "hello.h"

bool isFlyMode(void) {
  return digitalRead(DECK_GPIO_IO4) == LOW;
}

void onErrorLed(void) {
  digitalWrite(DECK_GPIO_IO1, HIGH);
}

void offErrorLed(void) {
  digitalWrite(DECK_GPIO_IO1, LOW);
}

static void helloInit()
{
  DEBUG_PRINT("Hello Crazyflie 2.1 deck world!\n");
  // setup error LED
  pinMode(DECK_GPIO_IO1, OUTPUT);     // Set my error Led pin to output
  digitalWrite(DECK_GPIO_IO1, LOW);   // Off
  // setup switch input
  pinMode(DECK_GPIO_IO4, INPUT_PULLUP);      //  Set my switch pin to input
}

static bool helloTest()
{
  DEBUG_PRINT("Hello test passed!\n");
  return true;
}

static const DeckDriver helloDriver = {
  .vid = 0,
  .pid = 0,

  .usedGpio = (DECK_USING_IO_1 | DECK_USING_IO_4),

  .name = "myHello",
  .init = helloInit,
  .test = helloTest,
};

DECK_DRIVER(helloDriver);