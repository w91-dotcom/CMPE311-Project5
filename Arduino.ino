#include <Arduino.h>
#include <limits.h>
#include <stdlib.h>
#include "src/rtos/FreeRTOS.h"
#include "src/rtos/task.h"

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error "This sketch requires an ATmega328P Arduino Uno R3 at 16 MHz."
#endif

/* Custom register definitions for the ATmega328P */
#define MY_TCCR2A (*(volatile unsigned char *)0xB0)
#define MY_TCCR2B (*(volatile unsigned char *)0xB1)
#define MY_TCNT2  (*(volatile unsigned char *)0xB2)
#define MY_OCR2A  (*(volatile unsigned char *)0xB3)
#define MY_TIMSK2 (*(volatile unsigned char *)0x70)
#define MY_TIFR2  (*(volatile unsigned char *)0x37)
#define MY_SREG   (*(volatile unsigned char *)0x5F)

/* Custom Timer1 registers for fan PWM on pin 9 */
#define MY_TCCR1A (*(volatile unsigned char *)0x80)
#define MY_TCCR1B (*(volatile unsigned char *)0x81)
#define MY_TCNT1L (*(volatile unsigned char *)0x84)
#define MY_TCNT1H (*(volatile unsigned char *)0x85)
#define MY_OCR1AL (*(volatile unsigned char *)0x88)
#define MY_OCR1AH (*(volatile unsigned char *)0x89)
#define MY_TIMSK1 (*(volatile unsigned char *)0x6F)
#define MY_PORTB  (*(volatile unsigned char *)0x25)

/* Timer0 supplies the FreeRTOS scheduler's separate 1 ms tick */
#define MY_TCCR0A (*(volatile unsigned char *)0x44)
#define MY_TCCR0B (*(volatile unsigned char *)0x45)
#define MY_TCNT0  (*(volatile unsigned char *)0x46)
#define MY_OCR0A  (*(volatile unsigned char *)0x47)
#define MY_TIMSK0 (*(volatile unsigned char *)0x6E)
#define MY_TIFR0  (*(volatile unsigned char *)0x35)

volatile unsigned long myMilliCounter = 0;

int selectedLED = -1;

/* Shared between the serial task and the LED tasks */
volatile unsigned long blinkRateOne = 0;
volatile unsigned long blinkRateTwo = 0;
volatile unsigned long blinkTrackerOne = 0;
volatile unsigned long blinkTrackerTwo = 0;

unsigned long inputValue = 0;
int inputStarted = 0;
int inputInvalid = 0;

/* Fan PWM output and button input */
const int fanPin = 9;
const int buttonPin = 4;

/* OFF, LOW, MEDIUM, HIGH, MEDIUM, LOW; then repeat */
const int fanSpeeds[6] = {0, 85, 170, 255, 170, 85};
int fanStep = 0;

/* Button debouncing */
int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long buttonTracker = 0;
const unsigned long debounceTime = 30;

/* FreeRTOS task declarations */
void ledOneTask(void *parameters);
void ledTwoTask(void *parameters);
void serialTask(void *parameters);
void fanTask(void *parameters);
void stopProgram(void);

/* Custom Timer2 compare-match interrupt */
extern "C" void __vector_7(void)
    __attribute__((signal, used, externally_visible));

void __vector_7(void)
{
  myMilliCounter++;
}

/*
 * FreeRTOS's AVR port saves and restores the task context.
 * A naked ISR must contain only the assembly trampoline below.
 */
extern "C" void vPortYieldFromTick(void);
extern "C" void __vector_14(void)
    __attribute__((naked, used, externally_visible));

void __vector_14(void)
{
  __asm__ __volatile__("call vPortYieldFromTick\n\treti" ::: "memory");
}

/* Called by the FreeRTOS AVR port when the scheduler starts */
extern "C" void prvSetupTimerInterrupt(void)
{
  MY_TCCR0B = 0;
  MY_TIMSK0 = 0;
  MY_TCCR0A = 0x02;   /* CTC mode */
  MY_TCNT0 = 0;
  MY_OCR0A = 249;     /* 16 MHz / 64 / 250 = 1000 ticks per second */
  MY_TIFR0 = 0x07;
  MY_TIMSK0 = 0x02;   /* Compare-match A interrupt */
  MY_TCCR0B = 0x03;   /* Prescaler 64 */
}

/* Returns a copy of the millisecond */
unsigned long myMilli(void)
{
  unsigned char savedStatus;
  unsigned long currentTime;

  savedStatus = MY_SREG;
  __asm__ __volatile__("cli" ::: "memory");
  currentTime = myMilliCounter;
  MY_SREG = savedStatus;
  __asm__ __volatile__("" ::: "memory");

  return currentTime;
}

/* Configures Timer2 to interrupt once every millisecond */
void setupMyMilli(void)
{
  unsigned char savedStatus;

  savedStatus = MY_SREG;
  __asm__ __volatile__("cli" ::: "memory");

  MY_TCCR2B = 0;       /* Stop Timer2 */
  MY_TIMSK2 = 0;       /* Disable Timer2 interrupts */
  MY_TCCR2A = 0x02;    /* Clear timer on compare match */
  MY_TCNT2 = 0;
  MY_OCR2A = 249;      /* 250 counts per interrupt */
  MY_TIFR2 = 0x07;     /* Clear old interrupt flags */

  myMilliCounter = 0;

  MY_TIMSK2 = 0x02;    /* Enable compare-match A interrupt */
  MY_TCCR2B = 0x04;    /* Start Timer2 with prescaler 64 */

  MY_SREG = savedStatus;
  __asm__ __volatile__("" ::: "memory");
}

/* Processes a complete number after Enter is pressed */
void processInput(void)
{
  unsigned long currentTime;

  if (inputInvalid) {
    Serial.println(F("Invalid input. Enter a whole number."));
  }
  else if (selectedLED == -1) {
    if (inputValue == 1 || inputValue == 2) {
      selectedLED = (int)inputValue;
    }
    else {
      Serial.println(F("Please choose LED 1 or 2."));
    }
  }
  else {
    /* Keep rate, tracker, and output updates together across task switches */
    taskENTER_CRITICAL();
    currentTime = myMilli();

    if (selectedLED == 1) {
      blinkRateOne = inputValue;
      blinkTrackerOne = currentTime;
      digitalWrite(2, LOW);
    }
    else {
      blinkRateTwo = inputValue;
      blinkTrackerTwo = currentTime;
      digitalWrite(3, LOW);
    }

    taskEXIT_CRITICAL();
    selectedLED = -1;
  }

  inputValue = 0;
  inputStarted = 0;
  inputInvalid = 0;

  if (selectedLED == -1) {
    Serial.println(F("What LED? (1 or 2):"));
  }
  else {
    Serial.println(F("What interval (in msec):"));
  }
}

/* Reads one character at a time so LEDs keep blinking */
void taskSerial(void)
{
  char character;
  unsigned long digit;

  if (Serial.available() <= 0) {
    return;
  }

  character = (char)Serial.read();

  if (character == '\r' || character == '\n') {
    if (inputStarted) {
      processInput();
    }
  }
  else if (character >= '0' && character <= '9') {
    inputStarted = 1;
    digit = (unsigned long)(character - '0');

    /* Prevent the input number from overflowing */
    if (inputValue > (ULONG_MAX - digit) / 10UL) {
      inputInvalid = 1;
    }
    else {
      inputValue = inputValue * 10UL + digit;
    }
  }
  else {
    inputStarted = 1;
    inputInvalid = 1;
  }
}

/* Sets fan output: 0 = OFF, 255 = fully ON */
void setFanSpeed(int speed)
{
  unsigned char savedStatus;

  savedStatus = MY_SREG;
  __asm__ __volatile__("cli" ::: "memory");

  /* Write the high byte before the low byte */
  MY_OCR1AH = 0;
  MY_OCR1AL = (unsigned char)speed;

  if (speed == 0) {
    /* Disconnect PWM and hold pin 9 LOW */
    MY_TCCR1A = 0x01;
    MY_PORTB &= 0xFD;
  }
  else if (speed == 255) {
    /* Disconnect PWM and hold pin 9 HIGH */
    MY_TCCR1A = 0x01;
    MY_PORTB |= 0x02;
  }
  else {
    /* Connect non-inverting PWM to pin 9 */
    MY_TCCR1A = 0x81;
  }

  MY_SREG = savedStatus;
  __asm__ __volatile__("" ::: "memory");
}

/* Configures Timer1 for 8-bit phase-correct PWM */
void setupFan(void)
{
  unsigned char savedStatus;

  savedStatus = MY_SREG;
  __asm__ __volatile__("cli" ::: "memory");

  MY_TCCR1B = 0;       /* Stop Timer1 during configuration */
  MY_TIMSK1 = 0;       /* Hardware PWM needs no Timer1 ISR */
  MY_TCCR1A = 0x01;    /* 8-bit phase-correct mode; output disconnected */

  /* Clear the counter and duty-cycle registers */
  MY_TCNT1H = 0;
  MY_TCNT1L = 0;
  MY_OCR1AH = 0;
  MY_OCR1AL = 0;

  MY_PORTB &= 0xFD;    /* Start with pin 9 LOW: fan OFF */
  MY_TCCR1B = 0x03;    /* Start Timer1 with prescaler 64 */

  MY_SREG = savedStatus;
  __asm__ __volatile__("" ::: "memory");
}

/* Changes fan speed once per debounced button press */
void taskFan(void)
{
  int buttonReading;
  unsigned long currentTime;

  buttonReading = digitalRead(buttonPin);
  currentTime = myMilli();

  /* Restart debounce timing whenever the input changes */
  if (buttonReading != lastButtonReading) {
    buttonTracker = currentTime;
  }

  /* Accept the new state after it stays unchanged for 30 ms */
  if (currentTime - buttonTracker >= debounceTime) {
    if (buttonReading != stableButtonState) {
      stableButtonState = buttonReading;

      /* INPUT_PULLUP makes a pressed button read LOW */
      if (stableButtonState == LOW) {
        fanStep++;

        if (fanStep >= 6) {
          fanStep = 0;
        }

        setFanSpeed(fanSpeeds[fanStep]);
      }
    }
  }

  lastButtonReading = buttonReading;
}

/* FreeRTOS now schedules the LED 1 activity */
void ledOneTask(void *parameters)
{
  (void)parameters;

  for (;;) {
    unsigned long currentTime;
    unsigned long halfInterval;

    taskENTER_CRITICAL();
    currentTime = myMilli();

    /* LED 1 */
    if (blinkRateOne > 0) {
      halfInterval = blinkRateOne / 2;

      if (halfInterval == 0) {
        halfInterval = 1;
      }

      if (currentTime - blinkTrackerOne >= halfInterval) {
        digitalWrite(2, !digitalRead(2));
        blinkTrackerOne = currentTime;
      }
    }
    taskEXIT_CRITICAL();

    vTaskDelay(1);  /* Block for one scheduler tick: 1 ms */
  }
}

/* FreeRTOS now schedules the LED 2 activity */
void ledTwoTask(void *parameters)
{
  (void)parameters;

  for (;;) {
    unsigned long currentTime;
    unsigned long halfInterval;

    taskENTER_CRITICAL();
    currentTime = myMilli();

    /* LED 2 */
    if (blinkRateTwo > 0) {
      halfInterval = blinkRateTwo / 2;

      if (halfInterval == 0) {
        halfInterval = 1;
      }

      if (currentTime - blinkTrackerTwo >= halfInterval) {
        digitalWrite(3, !digitalRead(3));
        blinkTrackerTwo = currentTime;
      }
    }
    taskEXIT_CRITICAL();

    vTaskDelay(1);
  }
}

/* Only this task accesses Serial after the scheduler starts */
void serialTask(void *parameters)
{
  (void)parameters;

  for (;;) {
    taskSerial();
    vTaskDelay(1);
  }
}

void fanTask(void *parameters)
{
  (void)parameters;

  for (;;) {
    taskFan();
    vTaskDelay(1);
  }
}

/* Halt with the fan OFF and built-in LED ON if RTOS setup fails */
void stopProgram(void)
{
  __asm__ __volatile__("cli" ::: "memory");
  MY_TCCR1A = 0x01;
  MY_PORTB &= 0xFD;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (;;) {
  }
}

extern "C" void vApplicationMallocFailedHook(void)
{
  stopProgram();
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
  (void)task;
  (void)name;
  stopProgram();
}

void setup(void)
{
  Serial.begin(9600);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);

  /* Configure custom PWM and start the fan OFF */
  setupFan();
  setupMyMilli();

  Serial.println(F("What LED? (1 or 2):"));
  Serial.flush();

  /* Fix the heap boundary before task stacks are allocated; reserve 128 bytes */
  __malloc_heap_end = (char *)(0x08FF - 128);

  /* Stack depths are bytes on this AVR port; all application tasks use priority 1 */
  if (xTaskCreate(ledOneTask, "LED1", 160, NULL, 1, NULL) != pdPASS) {
    stopProgram();
  }
  if (xTaskCreate(ledTwoTask, "LED2", 160, NULL, 1, NULL) != pdPASS) {
    stopProgram();
  }
  if (xTaskCreate(serialTask, "Serial", 320, NULL, 1, NULL) != pdPASS) {
    stopProgram();
  }
  if (xTaskCreate(fanTask, "Fan", 160, NULL, 1, NULL) != pdPASS) {
    stopProgram();
  }

  vTaskStartScheduler();

  /* Reaching here means the scheduler could not start */
  stopProgram();
}

void loop(void)
{
  /* FreeRTOS tasks perform the application work. */
}
