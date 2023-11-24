/* Arduino Example Template

//Initial Author: Maxwell Vos (maxwellvosthefirst@gmail.com)
//Start date: 02/2023

NOTES:
  ADC Uses SPI Mode 2: CPOL=1 (Idle High), & CPHA=0 (Data output immediately
  on !CS/!SS activation and transition to SCLK idle voltage)
  Data is shifted out on CS activation (active low) and rising SCLK
  Data sampled on falling SCLK

  SPISettings mySettings(20000000, MSBFIRST, SPI_MODE2);  // Best for variables
  Use with:
  SPI.beginTransaction(mySettings);  // Best for variables
  or omit settings and use:
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE2));  // Best
  for constants
  The speed setting should be the maximum theoretically possible for the
  peripheral devices.
  Arduino figures out the best speed the host controller can actually achieve.

  SPI Info for the Arduino Nano ESP32:
  CS/SS = D10 (sub/slave select output)
  COPI/MOSI = D11 (data output)
  CIPO/MISO = D12 (data input)
  SCK = D13 (SPI clock output)
  In the newer pinouts CS is not defined. Any digital pin can be used, but SS
  still works.
  The new SIP pinout names are not defined on older boards. Using old names as
  they still work on most Arduino boards.

  From ESP32-S3 Technical reference manual:
  ESP32-S3 chip contains two timer groups, namely timer group 0 and timer
  group 1. Each timer group consists of two general purpose timers referred
  to as Tx (where x is 0 or 1) and one Main System Watchdog Timer. All
  general purpose timers are based on 16-bit prescalers  and 54-bit
  auto-reload-capable up-down counters. Note that programming value 0 to
  TIMG_Tx_DIVIDER [prescaler] will result in the divisor being 65536. When
  the prescaler is set to 1, the actual divisor is 2.
  The remaining prescaler values are the same as programmed values.
  [Prescaler 1 cannot be set; divider range is 2-65536; default APB tick rate
  is 80 MHz; 2^54=18*10^15.]
  [Prescaler=80 => microseconds; 2^54 => 571 years at μs resolution]
  [When setting up timers, for the first parameter of timerbegin use 0-3]
  hardware timer error is approx -10us/s (spec <= 10 ppm)

  API and Documentation Warnings:
  In summary, please be aware that the online documentation and examples may
  not work in the Arduino IDE or CLI build environments.
  It has been noted that there are some inconsistencies between the online
  documentation and the functions available in the actual build environment.
  The the Arduino Nano esp32 has many different layers of libraries and api's
  available. Besides the common Arduino funtions, there is an arduino-esp32
  library and api, which is built on top of esp32-idf api. Below that is a
  hardware abstraction layer (HAL) and register address defines.
  At the time of writing, the release packaged into the Arduino IDE seems
  to match the documentation for espressif's idf-release/v4.4 whereas the
  latest online documentation (managed/hosted by espressif) was for v5.1.
  In the board manager of the Arduino IDE the current release was 2.0.13,
  wheres the online documentation was for release 2.0.14. However the Arduino
  boards manager listed the latest espressif release as 2.0.11. Adding a
  custom url updated the espressif release to 2.0.14. However, the
  arduino-esp32 2.0.14 (&13) core is only compatable with the v4.4 releases of
  the ESP-IDF core, so that documentation needs to be used idf components.
  To get the latest arduino-esp32 core, add a custom url to the latest
  release in the Arduino IDE preferences. It will appear as "esp32 by
  Espressif Systems" in the board manager list. The Arduino published version
  will appear as a separate entry ("Arduino ESP32 Boards by Arduino").
  For Espressif ESP32, add:
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
  See wiki and instrutions at:
  https://docs.espressif.com/projects/arduino-esp32/en/latest/getting_started.html
  https://github.com/arduino/Arduino/wiki/Unofficial-list-of-3rd-party-boards-support-urls

  TLDR: Ensure the documentation versions match the api versions installed.

*/

#if 0
  // Use this to comment out code
#endif

// DEFINES AND INCLUDES-------------------------------------------------------|
// Includes:
  #include <Arduino.h>
  #include <SPI.h> // Arduino SPI library
// Serial (baud 115200 usually common, but 250000 seems stable )
  #define SERIAL_BAUD 250000
// SPI (baud theoretically > 20MHz on ADC, and 80MHz periph clock on ESP32-S3)
  #define SPI_BAUD 20000000
// DIC & Gleeble Defines:
  #define TRIGGER_PIN 9 // normally 2
  #define FPS_CHANGE_PIN 8
  #define BUTTON_PIN_RED 7   // Arduino pin connected to button's pin
  #define BUTTON_PIN_WHITE 6 // Arduino pin connected to button's pin
// ADC pin names & Arduino pin numbers:
  #define ADC_BUSY 4     // adc_pin=14,   arduino_pin=4
  #define ADC_FRSTDATA 5 // adc_pin=15,   arduino_pin=5
  #define ADC_RESET 3    // adc_pin=11,   arduino_pin=8
  #define ADC_CONVST 2   // adc_pin=9&10, arduino_pin=9,    CONVSTA/CONVSTB tied
  #define ADC_NOT_CS SS  // adc_pin=13,   arduino_pin=10,   SS/CS, ACTIVE LOW
  #define ADC_DOUTA MISO // adc_pin=24,   arduino_pin=12,   MISO/CIPO
  #define ADC_SCLK SCK   // adc_pin=12,   arduino_pin=13,   SCK, ACTIVE LOW
#if 0
  #define ADC_STBY xx    // adc_pin=7,    hardwired high,    standby if low
#endif
// GLOBAL VARIABLES ----------------------------------------------------------|
// CSV Seperator
  char csv_sep[] = ",";  // or ","; and can use string type as well as char
// Serial
  bool timingDataRecieved = false;

// Camera Trigger Timer
  hw_timer_t *trigger_timer = NULL;  // pointer to hardware timer struct
  volatile uint8_t isr_loop_count = 0;         // counter for ISR
  uint64_t default_trigger_interval = 100000;  //  delay in microsec; 10 fps
  // User set trigger timer values in milliseconds; 24 elements;
  uint64_t timer_values_millis[] = {  0, 1000, 0, 1000, 250, 100, 50, 30,
                                      0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0
                                    };  // 24 elements
   // Matches the number of elements set as defaults in timer_values_millis[]
  uint8_t DEFAULT_TIMER_ELEMENTS = 8; 
  // Trigger Timer values in microseconds; 24 elements;
  // - NEED BOUNDS CHECK OR i++ CRASHES OS!
  // - CANNOT WRITE ZERO OR TIMER CRASHES OS!
  uint64_t trigger_timer_values[] = { 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0
                                    };  // 24 elements
  uint8_t timer_elements_set = 0;
  const uint8_t TIMER_ELEMENTS_MAX = 24;
  //uint8_t timer_array_index = 0;  // TODO: COMBINE THESE
  uint8_t fps_change_count = 0;  // acts as index for trigger timing array
  volatile bool timer_flag = false;  // TODO: REMOVE [used in old adc_test_loop()]

// Run Timer
  hw_timer_t *run_timer = NULL;  // pointer to hardware timer struct
  volatile uint64_t test_run_time = 0;
  volatile uint64_t fps_change_time = 0;
  uint64_t test_end_time = 0;
  // Read with: uint64_t timerReadMilis(hw_timer_t *timer);

// Debounce timer
  hw_timer_t *debounce_timer = NULL;     // pointer to hardware timer struct
  uint64_t debounce_interval = 5u * 1000u;  // millisec * 1000 = microsec
  // will poll input debounce interval and create history of 8 values
  // ie total debounce period = 5 * 8 = 40 millisec

// ADC Variables
  volatile bool adc_data_ready = false;  // TODO: duplicated entry in struct
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
    //volatile bool data_ready = 0;              // adc conversion completed and data waiting
    //bool data_retrieved = 0;                   // adc data fetched using spi
    //bool data_sent = 0;                        // adc data transmitted over serial
  } adc;                                         // struct to hold adc data
  volatile struct dataBlock *adcPointer = &adc;  // NOTE BEHAVIOR OF SCOPE

// Debouncing buttons and inputs
  struct Debounce
  {
    const uint8_t PIN_MODE;  // not used; a reminder; pads struct to 32bit
    const uint8_t TRIGGER;
    volatile uint8_t history;
    volatile bool activated;  // bool = 1 byte or 8 bits
    //volatile bool state;
    // bool lastState;
    //const uint8_t PIN;  // not used; a reminder; pads struct to 32bit
    // const bool IS_PULLUP;
    // uint32_t activations;
  };

  Debounce fps_change_pin = {INPUT, 0x7fu, 0x00u, false};
  Debounce red_button = {INPUT, 0x7fu, 0x00u, false};  // INPUT_PULLUP, 0x80u, 0xffu, false
  Debounce white_button = {INPUT, 0x7fu, 0x00u, false};  // INPUT_PULLUP, 0x80u, 0xffu, false

// Test run states and variables
  enum state_t
  {
    Config,
    Calibration,    
    Predicting_States,
    Prestart,
    Prestart_Pause,  // do nothing, needed as cant stay in Prestart
    Starting,
    Running,
    Changing_FPS,
    Pausing,
    Paused,
    Resuming,
    Stopping,
    Stopped,
    Sleeping,
    Shut_Down,
    Error,
    Aborted

    //Updating_State,  // will result in one-loop delay
    //Unchanged,  // Remove for running    
  };
  enum state_t Run_State = Config;  // starting state of test

  state_t Run_State_Array[TIMER_ELEMENTS_MAX];  // array of state enum values


// RGB colours
  enum colour_enum
  {
    Black,
    Red,
    Green,
    Yellow,
    Blue,
    Magenta,
    Cyan,
    White
  };
  // enum colour_enum RGB_Colour = Black;

// Various counters for camera and adc data
  uint32_t trigger_frame_count = 0;
  uint32_t adc_frame_count = 0;
  int32_t data_delta = 0;  // difference between data triggered and sent

// Calibration mode
  const uint32_t number_of_calibration_images = 10;
  const uint32_t calibration_frame_delay = 2000;  // in millis

// Strings
  String data_string;

// Workarounds
  uint32_t missed_interrupts = 0;
  static bool paused_printed = false;  // TODO: Move or eliminate
  static bool paused_flag = false;  // TODO: Move or eliminate


// FUNCTION DECLARATIONS -----------------------------------------------------|
  void ARDUINO_ISR_ATTR trigger_timer_ISR(void);
  void ARDUINO_ISR_ATTR adc_data_ready_ISR(void);
  void ARDUINO_ISR_ATTR debounce_timer_ISR(void);

  void printed_delay(void);                   // somewhat unnecessary in final version
  void serial_setup(bool driver_reset_delay);  // somewhat unnecessary in final version
  void adc_setup(void);
  void trigger_timer_setup(void);  // TODO: omit in final
  void adc_capture(void);  // TODO: probably no longer necessary?
  void adc_fetch_data(void);
  void adc_reset_values(void);
  void adc_test_loop(void);  // TODO: omit in final
  void rgb_setup(void);
  void rgb_test(int loops);
  void rgb_write(colour_enum Colour);
  void run_calibration(void);
  void adc_print_values(void);  // ?
  void adc_write_values(void);  // ?
  void print_csv_header(void);
  void print_csv_data(void);
  void state_prediction(void);
  void adc_map_volts(void);

  void wait_for_serial_config(void);

  void my_function(void);  // dummy function

// ARDUINO SETUP FUNCTION ----------------------------------------------------|
void setup()
{
  rgb_setup();
  rgb_test(1);
  digitalWrite(LED_BLUE, LOW);  // turn on blue led during setup
  // serial_setup(true);  // Setup Serial Monitor
  Serial.begin(SERIAL_BAUD);  
  delay(3000u);  // allow time for usb driver to reset
  Serial.println("Hello World!");
  Serial.print("Note that the serial baud rate is currntly set to ");
  Serial.print(SERIAL_BAUD);  
  Serial.print("bps.\nRunning Setup...\n");
  
  // Setup trigger timer
  // Note: in future release this may change to timerBegin(uint32_t frequency)
  trigger_timer = timerBegin(0, 80, true);  // divider 80 gives 1MHz (microsec)
  timerAttachInterrupt(trigger_timer, &trigger_timer_ISR, true);
  timerAlarmWrite(trigger_timer, default_trigger_interval, true);
  timerAlarmEnable(trigger_timer);
  // Pause and reset trigger timer to zero (interrupt still ready)
  timerStop(trigger_timer);
  timerWrite(trigger_timer, 0);  // reset counter value to zero

  // Setup Run timer
  run_timer = timerBegin(1, 80, true);  // divider 80 gives 1MHz (microsec)
  // Note: in future release this may change to timerBegin(uint32_t frequency)
  timerStop(run_timer);     // pause run timer
  timerWrite(run_timer, 0);  // reset run value to zero

  // Setup debounce timer
  // Note: in future release this may change to timerBegin(uint32_t frequency)
  debounce_timer = timerBegin(2, 80, true);  // divider 80 gives 1MHz (microsec)
  timerAttachInterrupt(debounce_timer, &debounce_timer_ISR, true);
  timerAlarmWrite(debounce_timer, debounce_interval, true);
  timerAlarmEnable(debounce_timer);

  // Setup inputs
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  // ADC_BUSY pin high during conversion. Data ready to fetch on falling edge
  attachInterrupt(ADC_BUSY, adc_data_ready_ISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(ADC_BUSY), adc_data_ready_ISR, FALLING);
  // Connected to quench signal in Gleeble
  pinMode(FPS_CHANGE_PIN, INPUT);
  // TODO: check if BUTTONs are INPUT or INPUT_PULLUP
  pinMode(BUTTON_PIN_RED, INPUT_PULLUP);
  pinMode(BUTTON_PIN_WHITE, INPUT_PULLUP);

  adc_setup();
  //adc_print_values();          // print variable initialisation values TODO: remove


  digitalWrite(LED_BLUE, HIGH);  // turn off blue led after setup

} // end of setup()

// ARDUINO LOOP FUNCTION ----------------------------------------------------|
void loop()
{
  //static enum state_t Run_State = Config;
  //static int state_int = 0;


  // Test flags and set run state 
  // Starts in state assigned in globals (or blank will trigger default case) 
  // TODO: if fps pin and red button only used here, change to 1 interrupt flag
  // TODO: fps change time can be set here but will be slighlty later than 
  //       using the interrupt. 
  if (fps_change_pin.activated || red_button.activated)
  {
    //fps_change_time = timerReadMilis(run_timer);  // later than interrupt
    fps_change_pin.activated = false;
    red_button.activated = false;
    fps_change_count++;
    Run_State = Run_State_Array[fps_change_count];
  }


  // Control run state of test
  switch (Run_State)
  {
    case Config:
      rgb_write(Blue);  // Will flash while waiting for serial [or white?]
      // Loop and wait for config settings over serial
      wait_for_serial_config();  // see function define for details
      // Check to see if exit was forced by button press and set state if so
      if (white_button.activated == true)
      {
        white_button.activated = false;  // lower flag
        Run_State = Calibration;  // user requested calibration sequence
      }
      else if (red_button.activated)
      {
        red_button.activated = false;  // lower flag
        fps_change_time = 0u;  // reset 
        Run_State = Predicting_States;  // use default times 
      }
      else 
      {
        Run_State = Predicting_States;  // normal exit if serial entered
      }
      break;

    case Calibration:
      rgb_write(White);
      // Take a series of 10 images/triggers with 2 sec intervals
      run_calibration();
      Run_State = Stopping;  // can go back to Config but reset probably better
      break;

    case Predicting_States:
      rgb_write(Blue);
      state_prediction();
      Run_State = Prestart;
      break;        

    case Prestart:
      // This state added as should start with fps pin or button, regardless
      // of what user enters into timing array 
      rgb_write(Cyan);
      // trigger_timer and run_timer should still be set to zero from setup
      timerWrite(trigger_timer, 0);
      timerWrite(run_timer, 0);
      // Reset fps change count just in case...
      fps_change_count = 0u;  
      // Test for the extreme case where user wants to start the test timer 
      // but not trigger the cameras or adc, then
      // Preload 1st timer alarm value (fps at index 0 always 0)
      if (trigger_timer_values[fps_change_count + 1] == 0)
      {
        timerAlarmWrite(trigger_timer, 0xFFFFFFFFFFFFu, true);  // years
      }
      else
      {
        timerAlarmWrite(trigger_timer, trigger_timer_values[fps_change_count + 1], true);
      }
      // Print messages to serial
      Serial.println("Waiting for FPS Change Pin or Red Button to start test");
      Serial.println("CSV_BEGIN");
      print_csv_header();
      print_csv_data();
      // Can't stay in or loop back to Prestart state      
      Run_State = Prestart_Pause;
      break;

    case Prestart_Pause:
      // Do nothing while waiting for fps change
      // Flash rgb leds or invert: cyan/red, green/magenta, yellow/blue
      // Don't care about millis rollover behavour    
      rgb_write(Cyan);
      static unsigned long flash_time = 0;
      if (millis() > flash_time+1000u)
      {
        rgb_write(Cyan);
      }
      else if (millis() > flash_time+2000u)
      {
        flash_time = millis();
        rgb_write(Black);
      }
      // Stay prestart paused. No change to new state.
      break;

    case Starting:
      // Zero values in alarm timer already checked for in pretest, so
      // Start all timers
      timerStart(run_timer);
      timerStart(trigger_timer);
      // TODO:Ggrab time from system timer. Needs config
      // start_time = something involving gettimeofday() etc;
      rgb_write(Green);
      Run_State = Running;
      break;
    case Running:
      // do nothing
      break;    

    case Changing_FPS:
      // Hopefully precalculating Run_State works correctly or trying
      // to write a zero to the timer alarm will crash the program
      timerAlarmWrite(trigger_timer, trigger_timer_values[fps_change_count], true);
      // Use timer restart and write if auto AlarmWrite behavour unsatisfactory
      // void timerRestart(hw_timer_t *timer);
      // void timerWrite(hw_timer_t *timer, uint64_t val);
      Run_State = Running;
      break;

    case Pausing:
      timerAlarmWrite(trigger_timer, 0xFFFFFFFFFFFFu, true);
      rgb_write(Yellow);
          #if DEBUG
          Serial.println("Pausing");
          paused_printed = false;
          #endif
      Run_State = Paused;
      break;

    case Paused:
      // do nothing
          #if DEBUG
          if (!paused_printed)
          {
            Serial.print("Paused. Run_State = "); 
            Serial.print(Run_State);
            Serial.print(". enum Paused = ");
            Serial.println(Paused);
            paused_printed = true;
          }
          paused_flag = true;  // flag to catch problems using enum in if stmnt
          #endif
      // next Run_State must be triggered by fps pin or red button
      break;

    case Resuming:
      // Hopefully precalculating Run_State works correctly or trying
      // to write a zero to the timer alarm will crash the program
      timerAlarmWrite(trigger_timer, trigger_timer_values[fps_change_count], true); 
      rgb_write(Green);
          #if DEBUG
          Serial.println("Resuming");
          paused_printed = false;
          paused_flag = false;
          #endif
      Run_State = Running;
      break;

    case Stopping:
      rgb_write(Magenta);
      timerAlarmWrite(trigger_timer, 0xFFFFFFFFFFFFu, true);
      digitalWrite(TRIGGER_PIN, LOW);
      Run_State = Stopped;
      break;

    case Stopped:
      rgb_write(Red);
      Serial.println("CSV_END");
      timerStop(trigger_timer);
      timerStop(run_timer);
      test_end_time = timerReadMilis(run_timer);
      Serial.println("Test Stopped");
      //print_test_stats();
      Serial.print("Total test run time was: ");
      Serial.print(test_end_time);
      Serial.print(" milliseconds\n");
      Serial.print(missed_interrupts);
      Serial.print(" missed interrupts from ADC_BUSY PIN.\n");
      Run_State = Shut_Down;
      break;
    case Shut_Down:
      rgb_write(Red);
      Serial.println("Shutting Down");
      // TRIGGER TIMER
      //timerStop(trigger_timer);  // already stopped
      timerAlarmDisable(trigger_timer);
      timerDetachInterrupt(trigger_timer);
      timerEnd(trigger_timer);
      // RUN TIMER
      //timerStop(run_timer);  // already stopped
      timerEnd(run_timer);
      // DEBOUNCE TIMER - will disable buttons and fps input
      timerStop(debounce_timer);
      timerAlarmDisable(debounce_timer);
      timerDetachInterrupt(debounce_timer);
      timerEnd(debounce_timer);
      // SPI
      SPI.end();
      // Set default states for SPI as ADC is still connected
      digitalWrite(SS, HIGH);
      digitalWrite(MOSI, LOW);
      digitalWrite(SCK, LOW);
      // MISO is an input, so leave it alone
      // SET OTHER OUTPUTS TO SENSIBLE STATES
      digitalWrite(TRIGGER_PIN, LOW); // inactive low
      digitalWrite(ADC_CONVST, LOW);  // inactive low
      digitalWrite(ADC_RESET, LOW);   // low for run state (DON'T LEAVE HIGH!)
      // RESET ADC - optional
      digitalWrite(ADC_RESET, HIGH);  // required reset pulse>50ns (20MHz)
      delayMicroseconds(1u);          // no delay might work but...
      digitalWrite(ADC_RESET, LOW);   // low for run state (DON'T LEAVE HIGH!)
      delay(100u);                    // min time power up from shutdown = 50ms     
      // SERIAL
      Serial.println("Closing serial connection and going to sleep");
      Serial.end();  // will disable serial communication
      // Next State
      Run_State = Sleeping;
      break;
    case Sleeping:
      // Currently not a true sleep. Processer can be put into low power mode,
      // and ADC set to standby with STBY pin low (but needs hardware change)
      // eg: esp_deep_sleep_start(); // needs library and config
      // eg: digitalWrite(ADC_STBY, LOW);
      // Below will make processer loop forever but do nothing
      while (true)
      {
        delay(100000u);  //  loop, do nothing forever
      }
      break;  // this break will not be reached      

    case Error:
      Serial.println("Warning: Error Case Run_State Triggered"); // shouldn't
      Serial.println("Warning: This was probably due to a state prediction failure");
      break;

    default:
      Serial.println("Warning: Default Case Run_State Triggered");  // shouldn't
      break;
  }


  // The general idea is that everyting after the state control sections 
  // (above) are always run in each loop (if necessary). That means data is 
  // always fetched and sent before a change to a new state takes effect.

  // TODO: Add checks to drop adc data if reaching limits or notify of misses
  //       Dropping could be done by changing the interrupt loop counters

  // If adc data is available fetch it and send over serial,
  // TODO: Currently the interrupt on ADC_BUSY is missing the falling edge,
  // so a backup check has been implemented and ensuing functions duplicated
  if (adc_data_ready)
  {
    adc_data_ready = false;
    adc_fetch_data();
    adc_map_volts();  // This added in for convenience; TODO: move
    data_delta--;  // decriment before send means value should be zero
    print_csv_data();
    // Pulling CONVST low will allow triggering of next sample
    // Can be done in isr, after spi fetch or after serial send (safest)
    digitalWrite(ADC_CONVST, LOW);
  } 
  else if (digitalRead(ADC_CONVST) && !digitalRead(ADC_BUSY))
  {
    adc_data_ready = false;
    adc_fetch_data();
    adc_map_volts();  // This added in for convenience; TODO: move
    data_delta--;  // decriment before send means value should be zero
    print_csv_data();
    // the stupid interrupt has missed the falling edge
    missed_interrupts++;
    // Pulling CONVST low will allow triggering of next sample
    // Can be done in isr, after spi fetch or after serial send (safest)
    digitalWrite(ADC_CONVST, LOW);
  } 

} // end of loop()

//============================================================================|
//============================================================================|
//============================================================================|
//============================================================================|

// INTERRUPT HANDLERS --------------------------------------------------------|

// Trigger timer interrupt service routine (ARDUINO_ISR_ATTR = IRAM_ATTR)
  void ARDUINO_ISR_ATTR trigger_timer_ISR(void)
  {
    switch (isr_loop_count)
    {
    case 0:
      digitalWrite(TRIGGER_PIN, HIGH);
      ++trigger_frame_count;
      break;
    case 5:
      digitalWrite(TRIGGER_PIN, LOW);
      break;
      // default:
      //;
    }
    digitalWrite(ADC_CONVST, HIGH);
    test_run_time = timerReadMilis(run_timer);
    ++adc_frame_count;
    data_delta++;
    isr_loop_count++;
    // reset back to 0 for total of 10 states
    if (isr_loop_count > 9)
    {
      isr_loop_count = 0;
    }
  }

// ADC_BUSY pin interrupt service routine
  // ADC_BUSY pin high during conversion. Data ready to fetch on falling edge
  void ARDUINO_ISR_ATTR adc_data_ready_ISR(void)
  {
    //adc.data_ready = true;  //  TODO
    adc_data_ready = true;
    // pulling CONVST low will allow triggering of next sample
    // Can be done in isr, after spi fetch or after serial send (safest)
  #if 0
      digitalWrite(ADC_CONVST, LOW);
  #endif
  }

// Debounce interrupt service routine
  void ARDUINO_ISR_ATTR debounce_timer_ISR(void)
  {
    fps_change_pin.history = (fps_change_pin.history << 1) | digitalRead(FPS_CHANGE_PIN);
    if (fps_change_pin.history == fps_change_pin.TRIGGER)
    {
      fps_change_pin.activated = true;
      fps_change_time = timerReadMilis(run_timer);
    }

    red_button.history = (red_button.history << 1) | digitalRead(BUTTON_PIN_RED);
    if (red_button.history == red_button.TRIGGER)
    {
      red_button.activated = true;
      fps_change_time = timerReadMilis(run_timer);
      // if red button used other than main test loop, then this must be reset
    }

    white_button.history = (white_button.history << 1) | digitalRead(BUTTON_PIN_WHITE);
    if (white_button.history == white_button.TRIGGER)
    {
      white_button.activated = true;
    }
  }

// OTHER FUNCTIONS -----------------------------------------------------------|

/* state_prediction():
 * This function works out the state transitions during the test
 * before the test starts. This approach avoid unnecessary overhead
 * while the test is running, and allows for faster transitions.
 */
void state_prediction(void)
{
  enum state_t Tested_State = Error;  // variable to hold state from test
  //enum state_t Previous_State;
  for (uint8_t fps_index = 0; fps_index < TIMER_ELEMENTS_MAX; fps_index++)
  {
    //Previous_State = Tested_State;
    Tested_State = Error;  // reset

    if (fps_index == 0)
    {
      Tested_State = Prestart;  // always
      // Test for zero in timer values and replace with large number (2^48)
      if (trigger_timer_values[fps_index] == 0)
      {
        trigger_timer_values[fps_index] = 0xFFFFFFFFFFFFu;  
      }
    }
    else if (fps_index == 1)
    {
      Tested_State = Starting;  // always
      if (trigger_timer_values[fps_index] == 0)
      {
        trigger_timer_values[fps_index] = 0xFFFFFFFFFFFFu;
      }
    }
    else if (fps_index >= TIMER_ELEMENTS_MAX)
    {
      Tested_State = Stopping;
      if (trigger_timer_values[fps_index] == 0)
      {
        trigger_timer_values[fps_index] = 0xFFFFFFFFFFFFu;
      }
    }
    else if (fps_index >= timer_elements_set)
    {
      Tested_State = Stopping;
      if (trigger_timer_values[fps_index] == 0)
      {
        trigger_timer_values[fps_index] = 0xFFFFFFFFFFFFu;
      }
    }
    else if (trigger_timer_values[fps_index] == 0)
    {
      Tested_State = Pausing;
      trigger_timer_values[fps_index] = 0xFFFFFFFFFFFFu;
    }
    else if ((trigger_timer_values[fps_index] != 0) && (trigger_timer_values[fps_index - 1] == 0xFFFFFFFFFFFFu)) 
    {
      Tested_State = Resuming;
    }    
    else if (trigger_timer_values[fps_index] != 0)
    {
      Tested_State = Changing_FPS;
    }
    else 
    {
      Tested_State = Error;
    }
    // Run sanity checks on result
    if (Tested_State == 15)  // 15 = Error
    {
      Serial.println("Warning: Predicted Run_State incorrectly resolved to \"Error\"!");
    }
    if (trigger_timer_values[fps_index] == 0)
    {
      Serial.println("Warning: Timer alarm value still zero. Program will crash!");
    }
    if (trigger_timer_values[fps_index] < 0)
    {
      Serial.println("Warning: Timer alarm value is negative. Program will crash!");
    }
    if (trigger_timer_values[fps_index] < 3000u)  // 30 ms *1000 /10
    {
      Serial.println("Warning: Timer alarm value too small. May drop frames!");
    }
    // Write result to Run_State_Array
    Run_State_Array[fps_index] = Tested_State;
  }
} // end of state_prediction()


/* print_csv_header():
 * This function prints the header (first line) into the csv-styled data set
 */
void print_csv_header(void)
{
/*
63 Bytes (Chars) fills an new/empty line up to here:           |
Build strings only that long to help minimise write blocking.
Remember to omit unneeded trailing/leading spaces in csv formatting.
*/
  Serial.print("trigger_frame_count");
  Serial.print(csv_sep);
  Serial.print("adc_frame_count");
  Serial.print(csv_sep);
  Serial.print("test_run_time");
  Serial.print(csv_sep);
  Serial.print("fps_change_count");
  Serial.print(csv_sep);
  Serial.print("fps_change_time");
  Serial.print(csv_sep);
  // Serial.print("fps_change_pin.history");
  // Serial.print(csv_sep);
  // Serial.print("adc.ch1");
  // Serial.print(csv_sep);
  Serial.print("adc.ch1_volts");
  Serial.print(csv_sep);
  // Serial.print("adc.ch2");
  // Serial.print(csv_sep);
  Serial.print("adc.ch2_volts");
  Serial.print(csv_sep);
  // Serial.print("adc.ch3");
  // Serial.print(csv_sep);
  Serial.print("adc.ch3_volts");
  Serial.print(csv_sep);
  // Serial.print("adc.ch4");
  // Serial.print(csv_sep);
  Serial.print("adc.ch4_volts");
  Serial.print(csv_sep);
  Serial.print("data_delta");  
  Serial.print("\n");
}  // end of print_csv_header()


/* print_csv_data():
 * This function prints a line of data into the csv-styled data set.
 *
 * Implemented the "easy" way first, then maybe with strings objects 
 * (more resources). If using string objects, these are to be used 
 * repeatedly so its probably best to reserve the space beforehand.
 * Fastest is likely to be using serial.write to send binary data
 * arranged into a struct and passing its pointer to be written.
 * The data can be re-assembled using an identical struct on the 
 * recieving end. [est 392 bytes & reduced computational overhead]
 */
void print_csv_data(void)
{
  Serial.print(trigger_frame_count);      // 10 chars max
  Serial.print(csv_sep);
  Serial.print(adc_frame_count);          // 10 chars max
  Serial.print(csv_sep);
  Serial.print(test_run_time);            // 20 chars max (norm >8 chars)
  Serial.print(csv_sep);
  Serial.print(fps_change_count);         // 3 chars
  Serial.print(csv_sep);
  Serial.print(fps_change_time);          // 20 chars max (norm >8 chars)
  Serial.print(csv_sep);
  // Serial.print(fps_change_pin.history);   // 3 chars
  // Serial.print(csv_sep);
  // Serial.print(adc.ch1);                  // 5 chars
  // Serial.print(csv_sep);
  Serial.print(adc.ch1_volts, 4);                  // 
  Serial.print(csv_sep);
  // Serial.print(adc.ch2);                  // 5 chars
  // Serial.print(csv_sep);
  Serial.print(adc.ch2_volts, 4);                  // 
  Serial.print(csv_sep);
  // Serial.print(adc.ch3);                  // 5 chars
  // Serial.print(csv_sep);
  Serial.print(adc.ch3_volts, 4);                  // 
  Serial.print(csv_sep);
  // Serial.print(adc.ch4);                  // 5 chars
  // Serial.print(csv_sep);
  Serial.print(adc.ch4_volts, 4);                  // 
  Serial.print(csv_sep);
  Serial.print(data_delta);               // 11 chars max
  Serial.print("\n");                     // 1 char LF
  // Total = 108 chars or 864 bits per line [+ 22(null) not sent = 130].
  // Bytes/char counts don't include each line's trailing null 
  // "\0" character to let serial print when to stop.
  // E.g. "," = 1 + 1 trailing null = 2 bytes/chars in memory, but 
  // trailing null not sent over serial link.
}  // end of print_csv_header()


// Maps 16bit intiger values to +/-10 Volts centered on zero, as per ADC docs
void adc_map_volts(void)
{
  adc.ch1_volts = (float)adc.ch1 * 5 / 16354;
  adc.ch2_volts = (float)adc.ch2 * 5 / 16354;
  adc.ch3_volts = (float)adc.ch3 * 5 / 16354;
  adc.ch4_volts = (float)adc.ch4 * 5 / 16354;
}



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

// Setup RGB LEDs
void rgb_setup(void)
{
  // LED_BUILTIN not configured as it shares GPIO48 pin with SPI SCK
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  // These pins are active-low -
  // To turn ON one of the LEDs write LOW to it
  // To turn OFF write HIGH to it
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
} // end of rgb_setup()

// flash the rgb leds for number loops given in arg
void rgb_test(int loops)
{
  for (int i = 0; i < loops; i++)
  {
    delay(250u);
    digitalWrite(LED_RED, LOW);
    delay(250u);
    digitalWrite(LED_RED, HIGH);
    // delay(250u);
    digitalWrite(LED_GREEN, LOW);
    delay(250u);
    digitalWrite(LED_GREEN, HIGH);
    // delay(250u);
    digitalWrite(LED_BLUE, LOW);
    delay(250u);
    digitalWrite(LED_BLUE, HIGH);
  }
}

// Write values to RGB LEDs to give colour in arg
void rgb_write(colour_enum Colour)
{
  digitalWrite(LED_RED, !(Colour & 0b0001));
  digitalWrite(LED_GREEN, !(Colour & 0b0010));
  digitalWrite(LED_BLUE, !(Colour & 0b0100));
}

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

// Setup ADC and SPI
void adc_setup(void)
{
// Ensure the Arduino SPI library is added to includes
#include <SPI.h> // Arduino SPI library
  /*  Info on ADC:
  For the Arduino Nano ESP32:
  CS/SS = D10 (sub/slave select output)
  COPI/MOSI = D11 (data output)  // not used by ADC
  CIPO/MISO = D12 (data input)
  SCK = D13 (SPI clock output)  //
  WARNING : DO NOT TRY USE LED_BUILTIN
  LED_BUILTIN must not be configured as it shares GPIO48 pin with SPI SCK
  In the newer pinouts CS is not defined. Any digital pin can be used, but
  SS still works. The new SIP pinout names are not defined on older boards.
  Using old names here as they still work on most Arduino boards.
  */
  // end of info section
  // Explicitly setup pin modes, but these should be handled by SPI.begin()
  pinMode(SS, OUTPUT);    // CS/SS     = D10,  adc_pin=13,  arduino_pin=13
  pinMode(MOSI, OUTPUT);  // COPI/MOSI = D11,  adc_pin=NA,  arduino_pin=11
  pinMode(MISO, INPUT);   // CIPO/MISO = D12,  adc_pin=24,  arduino_pin=12
  pinMode(SCK, OUTPUT);   // SCK       = D13,  adc_pin=12,  arduino_pin=8
  digitalWrite(SS, HIGH);  // Pin D10 high to inactivate
  // Initialize the SPI bus by setting SCK, MOSI, and SS to outputs,
  // pulling SCK and MOSI low, and SS high.
  SPI.begin();
  // SETUP ADC CONTROL PINS
  pinMode(ADC_CONVST, OUTPUT);  // adc_pin=9&10, arduino_pin=2
  // CONVSTA tied to CONVSTB
  pinMode(ADC_RESET, OUTPUT);   // adc_pin=11, arduino_pin=3
  pinMode(ADC_BUSY, INPUT);     // adc_pin=14, arduino_pin=4
  pinMode(ADC_FRSTDATA, INPUT);  // adc_pin=15, arduino_pin=5
  
  #if 0
  // POWER UP FROM SLEEP - currently hardwired to high/on
  pinMode(ADC_STBY, OUTPUT);
  digitalWrite(ADC_STBY, HIGH);
  #endif
  // RESET ADC AFTER POWER UP
  delay(100u);                    // min time power up from shutdown = 50ms
  digitalWrite(ADC_RESET, HIGH);  // required pulse>50ns (20MHz)
  delayMicroseconds(10u);          // no delay might work but...
  digitalWrite(ADC_RESET, LOW);   // low for run state (DON'T LEAVE HIGH!)
  delayMicroseconds(10u);  // wait min of 25ns before conversion start
} // end of adc_setup()

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

// Transfer adc data
void adc_fetch_data(void)
{
  SPI.beginTransaction(SPISettings(SPI_BAUD, MSBFIRST, SPI_MODE2));  // TODO: RETURN VALUES; CHANGED FROM (20000000, MSBFIRST, SPI_MODE2)
  digitalWrite(SS, LOW);
  adc.ch1 = SPI.transfer16(0xFFFF);  // 0b1111'1111'1111'1111 or -1 (signed)
  // digitalWrite(SS, HIGH);  // TODO: does SS H,L framing improve reliability?
  // delayMicroseconds(1u);
  // digitalWrite(SS, LOW);
  adc.ch2 = SPI.transfer16(0xFFFF);
  // digitalWrite(SS, HIGH);  // TODO
  // delayMicroseconds(1u);
  // digitalWrite(SS, LOW);
  adc.ch3 = SPI.transfer16(0xFFFF);
  // digitalWrite(SS, HIGH);  // TODO
  // delayMicroseconds(1u);
  // digitalWrite(SS, LOW);  
  adc.ch4 = SPI.transfer16(0xFFFF);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();  // this doesn't slow things down much
  //adc.data_ready = false; // TODO Currently lowered in loop
  //adc.data_retrieved = true;  // TODO Remove as not used
} // end adc_fetch_data()

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
      adc_fetch_data();
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


// Loop and wait for config settings over serial
void wait_for_serial_config(void)
{
  // TODO: Default list not loaded properly. Only the first value added
  // TODO: Clean up and add comments.
  // TODO: Note that index can overrun timing array bounds
  // TODO: serRead is a String classs instance of undefined size
  Serial.println("CHOOSE AN OPTION:");
  Serial.println("* Enter a list of timing values in ms.");
  Serial.println("  Comma separated; No spaces; Max 24 items; Min value 30 ms.");
  Serial.println("* Enter a blank line to use the first default value only.");
  Serial.println("* Press the red button for the full default set of values.");
  Serial.println("* Press the white button for a 10-image calibration sequence.");
  Serial.println("WAITING");
  // String and serial data variables
  char array[250];
  char *strings[250];  // an array of pointers to the pieces of the above array after strtok()
  char *ptr = NULL;
  String serRead = "";  // TODO: here empty but when assigned can be any size
  String convertTemp;
  uint64_t uintTemp;  
  bool timingDataRecieved = false;
  byte index = 0;
  bool notify = false;
  // Save LED state
  int red_state = digitalRead(LED_RED);
  int green_state = digitalRead(LED_GREEN);
  int blue_state = digitalRead(LED_BLUE);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  // set rgb led to white
  rgb_write(Blue);
  // Wait for timing values from serial
  while (timingDataRecieved == false)
  {
    // First giving a way to break out and do calibration cycle
    if (white_button.activated || red_button.activated)
    {
      timingDataRecieved = true;  // hasn't really but need to break out loop
      // lowering button flags and run state change handled in switch case
      timer_elements_set = DEFAULT_TIMER_ELEMENTS;  // or max array size
      index = DEFAULT_TIMER_ELEMENTS; // this just to print values
      // Print timing values, but not if white button was pushed
      if (!white_button.activated)
      {
        for (int n = 0; n < index; n++)
        {
          Serial.print(n);
          Serial.print("  ");
          Serial.println(timer_values_millis[n]);
          delay(200u);
        }
      }
    }
    else if (Serial.available() > 0) // look for incoming serial data
    {
      serRead = Serial.readStringUntil('\n');
      serRead.toCharArray(array, 250);
      ptr = strtok(array, ",");  // delimiter

      while (ptr != NULL)
      {
        strings[index] = ptr;
        convertTemp = String(ptr);
        uintTemp = (unsigned long)convertTemp.toInt();  // hope this works...
        // Always start in paused state, so add it if not present
        if ( (index == 0) && (uintTemp != 0) )
        {
          timer_values_millis[index] = 0u;
          index++;
          notify = true;
        }
        timer_values_millis[index] = uintTemp;
        index++;
        ptr = strtok(NULL, ",");
      }
      if (notify)
      {
        notify = false;
        Serial.println("An extra entry was added to start test in a paused state");
      }
      // local index updates global
      if (index == 0) {timer_elements_set = TIMER_ELEMENTS_MAX;}
      else {timer_elements_set = index;}
      for (int n = 0; n < index; n++)
      {
        Serial.print(n);
        Serial.print("  ");
        Serial.println(timer_values_millis[n]);
        delay(300u);
      }
      Serial.println("RECIEVED");
      timingDataRecieved = true;

    } // end of while loop if and if else
    // flash leds at 1Hz while waiting for serial data (inverts current state)
    #if 0
    digitalWrite(LED_RED, !digitalRead(LED_RED));
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    #endif   
    digitalWrite(LED_BLUE, !digitalRead(LED_BLUE));
    delay(500u);
  } // end of while loop
  // reset flag
  timingDataRecieved = false;
    // Convert trigger timer values to microseconds
    for (int i = 0; i < TIMER_ELEMENTS_MAX; i++)
    {
      trigger_timer_values[i] = timer_values_millis[i] * 100u;  // array to hold timing values in microsec; 5 high + 5 low = 1 Hz, thus /10, & 1000/10=100;
    }

  // Restore LED states
  digitalWrite(LED_RED, red_state);
  digitalWrite(LED_GREEN, green_state);
  digitalWrite(LED_BLUE, blue_state);
} // end of wait_for_config()


/* run_calibration():
 * This function runs the calibration sequence. No timers or adc values used.
 */
void run_calibration(void)
{
    Serial.println("CALIBRATION IMAGE AQUISITION STARTED");
    delay(10000u);  // LOCKUP & wait 10sec for camera to stabilize
    uint32_t period = calibration_frame_delay / 2u;    
    for (uint32_t i = 0; i < number_of_calibration_images; i++)
    {
      digitalWrite(TRIGGER_PIN, HIGH);
      delay(period);
      digitalWrite(TRIGGER_PIN, LOW);
      delay(period);
      i++;
    }
    Serial.println("CALIBRATION IMAGES CAPTURED");   
} // End of run_calibration()


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

// ---------------------------------------------------------------------------|
// ---------------------------------------------------------------------------|
// ---------------------------------------------------------------------------|
// ---------------------------------------------------------------------------|

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

/*


// Description:
// The String reserve() function allows you to allocate a buffer in memory for manipulating Strings.
// Syntax:
// myString.reserve(size)
// Parameters:
// myString: a variable of type String.
// size: the number of bytes in memory to save for String manipulation. Allowed data types: unsigned int.


// BACKUP OF MOSTLY WORKING CODE TO SELECT NEXT RUN STATE ON THE FLY
// DEPRECIATED IN FAVOUR OF PRE-CALCULATING THE STATES BEFORE STARTING TEST

    if (fps_change_count >= TIMER_ELEMENTS_MAX)
    {
      Run_State = Stopping;
    }
    else if (fps_change_count >= timer_elements_set)
    {
      Run_State = Stopping;
    }
    else if (trigger_timer_values[fps_change_count] == 0)
    {
      Run_State = Pausing;
    }
    else if ( Run_State == 6 ) // putting enum name here won't work
    {
      Run_State = Resuming;
      paused_flag = false;
    }    
    else if (paused_flag) // just putting Run_State == Paused not working,  neither does casting to int or !(Run_State ^ Paused)
    {
      Run_State = Resuming;
      paused_flag = false;
    }
    else
    {
      timerAlarmWrite(trigger_timer, trigger_timer_values[fps_change_count], true);
      Run_State = Running;
    }









*/