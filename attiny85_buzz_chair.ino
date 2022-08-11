#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

const uint8_t WDT_SLEEP_SECONDS = 8 + 1;  // extra 1 second because of 1 second delay in is_sitting()
const uint32_t MAX_SIT_SECONDS = 1800;          // 30 minutes
const uint32_t MAX_AWAY_TIEMOUT_SECONDS = 60; // 1 minute
const uint32_t MAX_HITS = MAX_SIT_SECONDS / WDT_SLEEP_SECONDS;
const uint32_t MAX_MISSES = MAX_AWAY_TIEMOUT_SECONDS / WDT_SLEEP_SECONDS;

void blink_led(uint8_t number_of_times);
void wake();
void wdt_sleep();
void resetWatchdog();
long readVcc();

uint32_t hits = 0;
uint32_t misses = 0;

void setup()
{

     wdt_reset();

    // set Pin 2 as input
    DDRB &= ~(1 << DDB2);
    PORTB |= 1 << PB2; // enable pull up resistor at PB2



   // Set pin 4 as output;
   // Will be used for LED
    DDRB |= (1 << DDB4);


    // Set Pin 1 as output to buzzer transistor
    DDRB |= (1 << DDB1);

    // Set Pin 3 as output for chair sensing circuit transitor
    DDRB |= (1 << DDB3);    

    blink_led(2);
    wdt_sleep();
}

void blink_led(uint8_t number_of_times)
{
    unsigned long last_blink = millis();
    uint8_t blinked = 0;

    // multiply by two because a blink is
    // on and off
    while (blinked < number_of_times * 2)
    {
        if ((millis() - last_blink) > 500)
        {
            PORTB ^= (1 << PB4);
            last_blink = millis();
            blinked++;
        }
    }
}

void resetWatchdog()
{
    cli();
    // clear various "reset" flags
    MCUSR = 0;
    // allow changes, disable reset, clear existing interrupt
    WDTCR = bit(WDCE) | bit(WDE) | bit(WDIF);
    // sleep for 8 seconds
    WDTCR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0);
    // pat the dog
    wdt_reset();
    sei();
} // end of resetWatchdog


void wdt_sleep(){
    resetWatchdog();
    // Save previous ADC Register value
    byte old_ADCSRA = ADCSRA;

      // Disable ADC
      ADCSRA = 0;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sleep_bod_disable();                    // disable brown-out detection
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep
    cli();                                  // Disable interrupts
    sleep_disable();                        // Clear SE bit
      // Restore ADC bit
  ADCSRA = old_ADCSRA;
    sei();                                  // Enable interrupts
    //blink_led(3);                         // Debugging


    // do a bettery check upon wakup
    // If voltage is less than 3v, send alert
    // this will alter the WDT just a tiny bit
    long vcc = readVcc();
    if(vcc < 2800L){
        buzz_alarm(1);
        blink_led(1);
    }
    
  
}

ISR(WDT_vect)
{
    wdt_disable(); // disable watchdog
}

int is_sitting()
{
    PORTB |= (1 << PB3); // Turn on transistor
    delay(1000); // give time for comparator to get output
    
    // when sitting, going to read low
    uint8_t pinValue = PINB & (1 << PB2);
    PORTB &= ~(1 << PB3);    // Turn off transistor
    if (pinValue == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }


}

void buzz_alarm(uint8_t number_of_buzzes)
{

    uint8_t toggle = 1;
    uint8_t buzzes = 0;
    unsigned long last_buzz_time;
   
    while (buzzes < number_of_buzzes * 2)
    {
        if ((millis() - last_buzz_time) > 200)
        {
            PORTB ^= (1 << PB1);
            last_buzz_time = millis();
            buzzes++;
        }
    }


}

long readVcc() {
  // Read 1.1V reference against AVcc
  // source: https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1126400L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1024*1000
  return result; // Vcc in millivolts
}

void loop()
{
    if (is_sitting())
    {
        misses = 0;
        if (hits < MAX_HITS)
        {
            hits++;
            wdt_sleep();
        }
        else
        {
            while (is_sitting())
            {
                buzz_alarm(2);
            }
        }
    }
    else
    {
        if (misses < MAX_MISSES)
        {
            misses++;
             wdt_sleep();
        }
        else
        {
            misses = 0;
            hits = 0;
            wdt_sleep();
        }
    }
}
