/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-button-long-press-short-press
 */

// constants won't change. They're used here to set pin numbers:
const int BUTTON_PIN = 7; // the number of the pushbutton pin
const int LONG_PRESS_TIME  = 1000; // 1000 milliseconds

// Variables will change:
int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;


void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_PIN);

  if(lastState == HIGH && currentState == LOW)        // button is pressed
    pressedTime = millis();
  else if(lastState == LOW && currentState == HIGH) { // button is released
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration > LONG_PRESS_TIME )
      Serial.println("A long press is detected");
  }

  // save the the last state
  lastState = currentState;
}

#include "project.h"
#include "stdio.h"

#define STR_LEN 64
char str[STR_LEN+1] ;
void print(char *str)
{
    UART_PutString(str) ;
}

void cls(void)
{
    print("\033c") ; /* reset */
    CyDelay(100) ;
    print("\033[2J") ; /* clear screen */
    CyDelay(100) ;
}

void splash(char *title)
{
    cls() ;
    if (title && *title) {
        print(title) ;
        print(" ") ;
    }
    snprintf(str, STR_LEN, "(%s %s)\n\r", __DATE__, __TIME__) ;
    print(str) ;
}

volatile uint32_t tick_count = 0 ;

CY_ISR(tick_callback)
{
    tick_count++ ;
}

int find_empty_slot(void)
{
    int result = -1 ;
    uint32_t i ;
    for (i = 0 ; i < CY_SYS_SYST_NUM_OF_CALLBACKS ; i++ ) {
        if (CySysTickGetCallback(i) == NULL) {
            result = i ;
            break ;
        }
    }
    return(result) ;
}
        
void show_delta_time(void)
{
    static uint32_t prev_tick = 0 ;
    uint32_t new_tick = 0 ;
    uint32_t delta = 0 ;
    
    new_tick = tick_count ;
    delta = new_tick - prev_tick ;
    prev_tick = new_tick ;
    snprintf(str, STR_LEN, "Current: %d -> %d ms passed\n\r", new_tick, delta) ;
    print(str) ;

}

void clear_millis(void)
{
    tick_count = 0 ;
}

uint32_t millis(void)
{
    return( tick_count ) ;
}

void print_time(uint32_t value)
{
    char buf[65] ;
    
    snprintf(buf, 64, "millis: %u\n\r", value) ;
    print(buf) ;
}

int main(void)
{   
    uint32_t time = 0 ;
    int sys_tick_slot = 0 ;
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    UART_Start() ;

    sys_tick_slot = find_empty_slot() ;
    if (sys_tick_slot < 0) {
        print("Sorry No empty SysTick Slot available\n\r") ;
        while(1) { } /* halting here */
    } else {
        CySysTickStart() ;
        CySysTickSetCallback(sys_tick_slot, tick_callback) ;
    }
    
    splash("millis() test with SysTick") ;
    
    clear_millis() ; // optional
    
    for(;;)
    {
        time = millis() ;
        print_time(time) ;
        
        CyDelay(1000) ; 
    }
}


#include <ezButton.h>

const int SHORT_PRESS_TIME = 1000; // 1000 milliseconds
const int LONG_PRESS_TIME  = 1000; // 1000 milliseconds

ezButton button(7);  // create ezButton object that attach to pin 7;

unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;

void setup() {
  Serial.begin(9600);
  button.setDebounceTime(50); // set debounce time to 50 milliseconds
}

void loop() {
  button.loop(); // MUST call the loop() function first

  if(button.isPressed())
    pressedTime = millis();

  if(button.isReleased()) {
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration < SHORT_PRESS_TIME )
      Serial.println("A short press is detected");

    if( pressDuration > LONG_PRESS_TIME )
      Serial.println("A long press is detected");
  }
}