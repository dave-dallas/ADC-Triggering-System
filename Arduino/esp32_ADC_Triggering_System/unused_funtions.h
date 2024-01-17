// DEFINES AND INCLUDES-------------------------------------------------------|
// Includes:
  #include <Arduino.h>
  #include <SPI.h> // Arduino SPI library

// Serial (baud 115200 usually common, but 250000 seems stable )
  #define SERIAL_BAUD 250000
  #define SERIAL_BAUD 250000
// SPI (baud theoretically > 20MHz on ADC, and 80MHz periph clock on ESP32-S3)
  #define SPI_BAUD 20000000
// ADC pin names & Arduino pin numbers:
  #define ADC_BUSY 4     // adc_pin=14,   arduino_pin=4
  #define ADC_CONVST 2   // adc_pin=9&10, arduino_pin=9,   CONVSTA/CONVSTB tied

// GLOBAL VARIABLES ----------------------------------------------------------|
  volatile bool timer_flag = false;  // TODO: REMOVE [used in old adc_test_loop()]

  // ADC Variables
  volatile bool adc_data_ready = false;  // adc conversion data waiting
  // Struct to hold adc data
  volatile struct dataBlock
  {
    int16_t ch1 = 0;  // -32768
    int16_t ch2 = 0;  // -32768
    int16_t ch3 = 0;  // -32768
    int16_t ch4 = 0;  // -32768
    float ch1_volts = 0;  // 
    float ch2_volts = 0;  // 
    float ch3_volts = 0;  // 
    float ch4_volts = 0;  //    
  } adc;  


// Hacks and Workarounds
  static bool paused_printed = false;  // TODO: For debug; Move or eliminate
  static bool paused_flag = false;  // TODO: For debug; Move or eliminate


// FUNCTION DECLARATIONS -----------------------------------------------------|
  void my_function(void);  // dummy function
  void adc_test_loop(void);  // TODO: omit in final
  void adc_capture(void);  // TODO: probably no longer necessary?
  void trigger_timer_setup(void);  // TODO: omit in final
  void printed_delay(void);                   // somewhat unnecessary in final version
  void serial_setup(bool driver_reset_delay);  // somewhat unnecessary in final version
  void adc_reset_values(void);
  void adc_print_values(void);  // ?
  void adc_write_values(void);  // ?
  void adc_framed_fetch_data(void);


  // serial_setup(true);  // Setup Serial Monitor


/* my_funtion():
 * This function does ...
 */
void my_function(void)
{
  while (0)
  {
    // do stuff
  }
} // end of my_function()


// Setup serial
void serial_setup(bool driver_reset_delay)
{
  /*  Serial Buffer:
63 Bytes fills a new line up to here:                          |
63 Bytes (Chars) fills an new/empty line up to here:           |
63 Bytes (chars) are available without blocking serial writes!  // returned
Don't forget the newline characters at the end of a line!
 */
  // end of info section
  if (driver_reset_delay == true)
  {
    delay(3000u);  // allow time for usb driver to reset
  }
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hello World!");
  // Serial.print(Serial.availableForWrite());
  // Serial.println(" Bytes (chars) are available without blocking serial writes!");
} // end serial_setup()

// Print delay to serial
void printed_delay(void)
{
  // This function runs a 5 sec delay and prints to the serial monitor
  Serial.println("Start delay");
  Serial.print("5");
  delay(1000u);
  Serial.print("4");
  delay(1000u);
  Serial.print("3");
  delay(1000u);
  Serial.print("2");
  delay(1000u);
  Serial.println("1");
  delay(1000u);
  Serial.println("End delay");
} // end printed_delay()



void adc_test_loop(void)
{
  const unsigned long iterations = 100;
  const unsigned int pulseLength = 100;
  const unsigned int channelDelay = 1000;
  const unsigned int loopDelay = 0;
  unsigned long startTime = micros();  // for benchmarking of the for loop

  for (unsigned long i = 0; i < iterations; i++)
  {
    while (!timer_flag)
    {
      // do nothing
    }
    while (timer_flag)
    {
      adc_capture();  // ADC capture values
      adc_framed_fetch_data();
      adc_print_values();
      adc_reset_values();
      timer_flag = false;
      // delay(loopDelay);
    } // end while loop

  } // end of for() loop
  // benchmark the for loop
  unsigned long endTime = micros();
  String printString = String(iterations) + " iterations took " + String(endTime - startTime) + " uSec (" + String(float(1000000) * float(iterations) / float(endTime - startTime)) + " SPS)";
  Serial.println(printString);  // sligtly faster in one step
  // Shutdown and idle
  SPI.end();  // shutdown spi
  while (1)
  {
    delay(10000u);
  } // loop infinitly without doing anything
} // end of adc_test_loop()


// ADC capture values
// TODO: Move to interrupt. This form causes lockup during while + delay loop
void adc_capture(void)
{
  // Start adc conversion
  digitalWrite(ADC_CONVST, HIGH);
  // tConv = pulseIn(ADC_BUSY, HIGH, 200);  // this seems to miss start of pulse
  // Serial.print("ADC Busy [us]: ");
  // Serial.println(tConv);
  // if adc is still busy converting, wait for new data
  // (old data in register when busy=high)
  while (digitalRead(ADC_BUSY) == HIGH)
  {
    delayMicroseconds(2u);  // conversion takes 2-191 us
  }                       // delay
  digitalWrite(ADC_CONVST, LOW);
  adc_data_ready = true;
} // end adc_capture()



// Print variable values to serial
void adc_print_values(void)
{
  // serial print causes the most slowdown;
  // removing this changes rate from 611 SPS to 13.5 kSPS (in nano every)
  Serial.println("ADC Readings: ");
  Serial.print(adc.ch1);
  Serial.print(",\tmV:");
  Serial.println(map(adc.ch1, -32768, 32768, -10000, +10000));
  Serial.print(adc.ch2);
  Serial.print(",\tmV:");
  Serial.println(map(adc.ch2, -32768, 32768, -10000, +10000));
  Serial.print(adc.ch3);
  Serial.print(",\tmV:");
  Serial.println(map(adc.ch3, -32768, 32768, -10000, +10000));
  Serial.print(adc.ch4);
  Serial.print(",\tmV:");
  Serial.println(map(adc.ch4, -32768, 32768, -10000, +10000));
  // Serial.println();
} // end adc_print_values()


// write binary values to serial
void adc_write_values(void)
{
  // serial print causes the most slowdown; using write instead
  // removing this changes rate from 611 SPS to 13.5 kSPS (in nano every)
  // Serial.write("ADC Readings: ");
  // Serial.write("\n");
  Serial.write(adc.ch1);
  // Serial.write(",\tmV:");
  // Serial.write(map(adc.ch1, -32768, 32768, -10000, +10000));
  // Serial.write("\n");
  Serial.write(adc.ch2);
  // Serial.write(",\tmV:");
  // Serial.write(map(adc.ch2, -32768, 32768, -10000, +10000));
  // Serial.write("\n");
  Serial.write(adc.ch3);
  // Serial.write(",\tmV:");
  // Serial.write(map(adc.ch3, -32768, 32768, -10000, +10000));
  // Serial.write("\n");
  Serial.write(adc.ch4);
  //Serial.write(",\tmV:");
  //Serial.write(map(adc.ch4, -32768, 32768, -10000, +10000));
  Serial.write("\n");
} // end adc_write_values()


void adc_reset_values(void)
{
  adc.ch1 = -32768;
  adc.ch2 = -32768;
  adc.ch3 = -32768;
  adc.ch4 = -32768;
  //adc.data_ready = 0;  // adc conversion completed and data waiting
  //adc.data_retrieved = 0;  TODO
  //adc.data_sent = 0;  // adc data transmitted over serial
} // end adc_reset_values()


#if 0
// Setup trigger timer
void trigger_timer_setup(void)
{
  trigger_timer = timerBegin(0, 80, true);  // divider 80 gives 1MHz (microsec)
  // Note: in future release this may change to timerBegin(uint32_t frequency)
  timerAttachInterrupt(trigger_timer, &trigger_timer_ISR, true);
  timerAlarmWrite(trigger_timer, default_trigger_interval, true); 
  timerAlarmEnable(trigger_timer);
}  // end of trigger_timer_setup
#endif



/*
// Idea courtesy Ganssle Group. Called from a 5ms timer,
// the debounced state only ever changes when the pin
// has been stable for 40ms. Initialize debounced_state
// to whatever is  "inactive" for the system (HIGH or LOW)
uint8_t DebouncePin(uint8_t pin) {
  static uint8_t debounced_state = LOW;
  static uint8_t state_history = 0;
  state_history = state_history << 1 | digitalRead(pin);
  if (state_history == 0xff) {
    debounced_state = HIGH;
  } else if (state_history == 0x00) {
    debounced_state = LOW;
  }
  return debounced_state;
} // end of DebouncePin()
*/



// Alternate styling:
#if 0
/*
//====================================================================
// EXTI2_3 EXTERNAL INTERRUPT HANDLER - EXTI2_3_IRQHandler()
//====================================================================
// DESCRIPTION: This function will be triggered by an interrupt on the
//             associated lines. Must include clearing of pending bit
//====================================================================
void EXTI2_3_IRQHandler(void){
    // Increment counter
    switch_3_count++;

    // Check value and adjust if necessary, then set on/off state accordingly
    // Doing this in main is better, but much more complicated.
    // Is probably short enough to get away with in this case
    if (switch_3_count > 3) switch_3_count = 0;
    switch (switch_3_count)
    {
        case 0:
            do_nothing = 1;
            break;
        case 1:
            do_nothing = 0;
            break;
        case 2:
            do_nothing = 1;
            break;
        case 3:
            do_nothing = 0;
            break;
        default:
            break;
    }

    // Clear pending bit by writing 1 to it
    EXTI->PR |= EXTI_PR_PR3;

}   // END OF EXTI2_3_IRQHandler()
*/
#endif

/*
// Struck Wikipedia reminders:
struct point {
   int x;
   int y;
};
struct point a_point = { .y = 2, .x = 1 };
struct point my_point = { 3, 7 };
struct point *p = &my_point;  // p is a pointer to my_point
(*p).x = 8;                   // set the first member of the struct
p->x = 8;                     // equivalent method to set the first member of the struct
*/

/*
// THE BELOW VERSION WORKS
// struct to hold adc data
typedef struct {
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  bool data_ready;  // adc conversion completed and data waiting
  bool data_retrieved;
  bool data_sent;  // adc data transmitted over serial
} dataBlock_t;
volatile dataBlock_t adc = {
  .ch1 = -32768,
  .ch2 = -32768,
  .ch3 = -32768,
  .ch4 = -32768,
  .data_ready = 0,  // adc conversion completed and data waiting
  .data_retrieved = 0,
  .data_sent = 0,  // adc data transmitted over serial
};
volatile dataBlock_t *adcPointer = &adc;
*/



// Description:
// The String reserve() function allows you to allocate a buffer in memory for manipulating Strings.
// Syntax:
// myString.reserve(size)
// Parameters:
// myString: a variable of type String.
// size: the number of bytes in memory to save for String manipulation. Allowed data types: unsigned int.


// Transfer adc data
void adc_framed_fetch_data(void)
{
  SPI.beginTransaction(SPISettings(SPI_BAUD, MSBFIRST, SPI_MODE2));  // TODO: RETURN VALUES; CHANGED FROM (20000000, MSBFIRST, SPI_MODE2)
  digitalWrite(SS, LOW);
  adc.ch1 = SPI.transfer16(0xFFFF);  // 0b1111'1111'1111'1111 or -1 (signed)
  digitalWrite(SS, HIGH);  // TODO: does SS H,L framing improve reliability?
  delayMicroseconds(1u);
  digitalWrite(SS, LOW);
  adc.ch2 = SPI.transfer16(0xFFFF);
  digitalWrite(SS, HIGH);  // TODO
  delayMicroseconds(1u);
  digitalWrite(SS, LOW);
  adc.ch3 = SPI.transfer16(0xFFFF);
  digitalWrite(SS, HIGH);  // TODO
  delayMicroseconds(1u);
  digitalWrite(SS, LOW);  
  adc.ch4 = SPI.transfer16(0xFFFF);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();  // this doesn't slow things down much
  //adc.data_ready = false; // TODO Currently lowered in loop
  //adc.data_retrieved = true;  // TODO Remove as not used
} // end adc_framed_fetch_data()
