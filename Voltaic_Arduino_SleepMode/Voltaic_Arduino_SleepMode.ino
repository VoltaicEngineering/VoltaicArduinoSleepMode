/*99999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999

This sketch includes MOST (though not all) commands for putting an Arduino to sleep.
It uses a watch dog timer (WDT) to wake the arduino up after a specified time interval.
Commands not mentioned include Brown-Out Disabling and mechanical interupts to wake it up.

Copy and paste each section in the appropriate place in your sketch

Summary: These code snippets are the core requirements to put an Arduino into and out of sleep mode.
The internal WDT wakes up the Arduino every 8 seconds and goes back to sleep. When
enough sleep cycles have passed, the Arduino completely wakes up and executes parts of the 
loop function that were previously skipped. The system then goes back to sleep when all the commands 
are executed. It is suggested that the Serial Monitor be used to monitor the activity of the Arduino 
from the inside-out.

Written By: Tony Kauffmann
Sponsored By: Voltaic Systems Inc.
Guidance From: Adafruit.com Tutorials
Updated: 12/2014

99999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999*/

/*_________ALL THE GLOBAL SLEEP MODE REQUIREMENTS__________*/

// Sleep Mode Libraries
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// Number of times to sleep (for 8 seconds) before a sensor reading is taken and sent to the server.
// 10 cycles x 8 seconds = 80 seconds before the main loop function is awake
#define MAX_SLEEP_ITERATIONS   10

// Internal state used by the sketch.
int sleepIterations = 0;
volatile bool watchdogActivated = false;

/*___________END SLEEP MODE REQUIREMENTS___________*/


// Debugging LED to test Sleep Mode
// Insert an external LED to pin 13, or use the small built-in LED on the Arduino
int test_LED = 13; 


void setup()
{
  Serial.println();
  Serial.println(F("Starting to Setup... \n")); 
  
  // initialize serial communications and wait for port to open:
  Serial.begin(9600); 

  pinMode(test_LED, OUTPUT);
  
  /*__________SLEEP SETUP REQUIREMENTS____________*/
  
  // Setup the watchdog timer to run an interrupt which wakes the Arduino from sleep every 8 seconds.
  // Note that the default behavior of resetting the Arduino with the watchdog will be disabled.
  
  // This next section of code is timing critical, so interrupts are disabled.
  // See more details of how to change the watchdog in the ATmega328P datasheet around page 50, Watchdog Timer.
  noInterrupts();
  // Set the watchdog reset bit in the MCU status register to 0.
  MCUSR &= ~(1<<WDRF);
  // Set WDCE and WDE bits in the watchdog control register.
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set watchdog clock prescaler bits to a value of 8 seconds.
  WDTCSR = (1<<WDP0) | (1<<WDP3);
  // Enable watchdog as interrupt only (no reset).
  WDTCSR |= (1<<WDIE);
  // Enable interrupts again.
  interrupts();
  /*__________END SLEEP SETUP REQUIREMENTS____________*/
  
  Serial.println(F("Setup complete \n")); 
}

void loop()
{ 
  // Don't do anything unless the watchdog timer interrupt has fired.
  // Watchdog timer should fire every 8 seconds
  if (watchdogActivated){
    
    // Light blinks 3 times when Watch Dog Timer bites
    for (int x = 0; x < 3; x++){
    digitalWrite(test_LED, HIGH);    
    delay(100);
    digitalWrite(test_LED, LOW);
    delay(100);
    }
    
    // Prints number of times the WDT bites before the arduino is fully awake
    Serial.print("watch dog bite #"); Serial.println(sleepIterations + 1);  
    
    // Reset Watch Dog Timer (WDT)
    watchdogActivated = false;
    
    // Increase the count of sleep iterations and take a sensor
    // reading once the max number of iterations has been hit.
    sleepIterations += 1;

    if (sleepIterations >= MAX_SLEEP_ITERATIONS) { // If the 8 second WDT has bitten 10 times
      
      // Reset the number of sleep iterations.
      sleepIterations = 0; 
      
      // Arduino is Awake when the test LED stays on
      digitalWrite(test_LED, HIGH);
      
      // Do whatever the code needs to do HERE!!
      // Take a sensor reading, check for WiFi connection, send a sensor reading, etc...
      **** INSERT MAIN COMMANDS HERE ****
    }     
  }
  
  // Arduino going back to sleep
  Serial.println(F("going to sleep \n"));
  digitalWrite(test_LED, LOW);
  sleep();
}


/*____________SLEEP MODE FUNCTIONS____________*/ 

// Define watchdog timer interrupt.
ISR(WDT_vect)
{
  // Set the watchdog activated flag.
  // Note that you shouldn't do much work inside an interrupt handler.
  watchdogActivated = true;
}

// Put the Arduino to sleep.
void sleep()
{
  // Set sleep to full power down.  Only external interrupts or 
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  // Turn off the ADC while asleep.
  power_adc_disable();

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point.
  
  // When awake, disable sleep mode and turn on all devices.
  sleep_disable();
  power_all_enable();
}








