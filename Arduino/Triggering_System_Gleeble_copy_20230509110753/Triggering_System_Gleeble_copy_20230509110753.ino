//DISCLAIMER: This is the wild west of arduino code. There are no rules, confusion reigns supreme, and this is the first time I am seeing most of this shit.

//Author: Maxwell Vos (maxwellvosthefirst@gmail.com)
//Start date: 02/2023

//FPS : OCR1A
//0.125  3 036
//0.25	 34 286
//0.5	   49 911
//1	  57 724
//2	  61 630
//5	  63 974
//10	64 755
//15	65 015
//20	65 145
//30	65 276
//35	65 313
//40	65 341
//45	65 362
//50	65 380
//55	65 394
//60	65 406
//65	65 416
//70	65 424
//75	65 432
//80	65 438
//85	65 444
//90	65 449
//95	65 454
//100	65 458

  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

int number_of_calibration_images = 10;
int Trigger_Pin = 11;
volatile int state = LOW; // variable to hold the state of digital pin 8
int prevState = LOW;
const int fps_change_pin = 8;

bool stop_FPS = false;
volatile int32_t counter_start = 200;//34286;
const int32_t timingValues[] ={34286, 0, 65449, 0};//array of compare values for the different timing states
volatile int32_t TCTN1_Values[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//{0,57724,61630,57724,61630,57724,61630,57724,61630,57724,61630,57724,61630,57724,61630,0};
const int arrLen = sizeof(timingValues) / sizeof(timingValues[0]);
volatile int timingState = 0; // variable to hold the current timing state

#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 myADS;
unsigned long t0;
unsigned long t_last_q4;
unsigned long t_current_q4;
unsigned long t_diff_q4;
int16_t latest_adc_value;
int ard_A0;
int t_last = 0;
String str;
String str2;
String shutter_str;
String convertTemp;
const int intPin = 2;
volatile bool new_data = false;
volatile bool start_ard_adc_0 = false;
int quench_count = 0;
int quench_state = 0;
unsigned long quench_time = 0;
volatile bool stop_trigger = true;

char array[250];
char *strings[250]; // an array of pointers to the pieces of the above array after strtok()
char *ptr = NULL;
int dataRecieved = 0;
String writeString = "00";
String serRead = "";
String fps_index = "";
float a0_milivolt = 0;

//button variables
const int BUTTON_PIN_RED = 5; // Arduino pin connected to button's pin
const int BUTTON_PIN_WHITE = 4; // Arduino pin connected to button's pin
int lastButtonState_red;    
int currentButtonState_red; 
int lastButtonState_white;   
int currentButtonState_white; 
bool button_change_fps = false;
int total_quench_count = 0;
bool frame_triggered = false;
int frame_count = 0;
volatile bool edge_bool = false;
volatile int trigger_pin_past = 0;
volatile int trigger_pin_current = 0;
bool first_trigger = false;
int last_FPS_pin_state = false;
int current_FPS_pin_state = false;
int calibration_frame_count = 0;
bool calibration_mode = false;


void newDataReady() {
  new_data = true;
  start_ard_adc_0 = true;      
}

void setup() {
//  noInterrupts();
  Serial.begin(115200);
  pinMode(Trigger_Pin, OUTPUT);
  pinMode(fps_change_pin, INPUT);
  digitalWrite(Trigger_Pin, LOW); 
  attachInterrupt(digitalPinToInterrupt(intPin), newDataReady, FALLING);
  myADS.begin();
  myADS.setGain(GAIN_TWOTHIRDS); //+- 6.114V (5.5V max)
  myADS.setDataRate(RATE_ADS1115_475SPS); //860SPS is theoretically the max but seems to be writing at 550sps so setting it to 475sps is probably good enough for our aplications
  myADS.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0 , true);

  TCCR1A = 0;
  TCCR1B = 0;
  counter_start = TCTN1_Values[quench_count];
  TCNT1 = counter_start;
  TCCR1B  |= (1<<CS12) | (1<<CS10);
  TIMSK1 |= (1<<TOIE1);
  t0 = millis();
  t_last_q4 = millis();
//  interrupts();
  pinMode(BUTTON_PIN_RED, INPUT_PULLUP);
  pinMode(BUTTON_PIN_WHITE, INPUT_PULLUP);
  currentButtonState_red = digitalRead(BUTTON_PIN_RED);
  currentButtonState_white = digitalRead(BUTTON_PIN_WHITE);
  
}

ISR(TIMER1_OVF_vect){
  if (stop_trigger == false){
    TCNT1 = counter_start;
    digitalWrite(Trigger_Pin, !digitalRead(Trigger_Pin));  
  }
}


void loop() {
  //dataRecieved = 1;
  while (dataRecieved == 0)
  {
    if (Serial.available() > 0) //temp enable so can look at serial moitor
    {
      serRead = Serial.readStringUntil('\n');
      serRead.toCharArray(array, 250);

      byte index = 0;
      ptr = strtok(array, ",");  // delimiter
      while (ptr != NULL)
      {
        strings[index] = ptr;
        convertTemp = String(ptr);
        TCTN1_Values[index] = convertTemp.toInt();
        index++;
        ptr = strtok(NULL, ",");
      }
      for (int n = 0; n < index; n++)
      {
        Serial.print(n);
        Serial.print("  ");
        Serial.println(TCTN1_Values[n]);
        delay(300);
      }
      Serial.println("RECIEVED");
      dataRecieved++;
    }             
  }

  lastButtonState_white = currentButtonState_white;      
  currentButtonState_white = digitalRead(BUTTON_PIN_WHITE); 
  //When the white button is clicked, it will start taking images every 2 seconds and capture 30 images. For now the image timing data is not recorded but this could easily be adapted to a 'burst mode' when it is needed.
  if(lastButtonState_white == LOW && currentButtonState_white == HIGH) {

    Serial.println("CALIBRATION IMAGE AQUISITION STARTED");
    delay(10000); //wait 10sec for camera to stabilize
    calibration_mode = true;
    counter_start = 49911;
    stop_trigger = false; 
    
  } 

  if (calibration_mode == true){
    trigger_pin_past = trigger_pin_current;
    trigger_pin_current = digitalRead(Trigger_Pin);

    if (trigger_pin_past == 0 && trigger_pin_current == 1) {
      calibration_frame_count++;  
    }

    if (calibration_frame_count == number_of_calibration_images) {
      Serial.print("CALIBRATION IMAGES CAPTURED");
      counter_start = 0;
      stop_trigger = true;  
      calibration_mode = false;
      digitalWrite(Trigger_Pin, LOW);
      calibration_frame_count = 0;
    }
  }

  //If ADC has data ready, updates the current ADC read value. It can also be used as a slower 475Hz clock cycle if needed.
  if (new_data) {   
    new_data = false;
    latest_adc_value = myADS.getLastConversionResults();
    a0_milivolt = latest_adc_value * 0.1875;   //Voltage conversion is in the comments at the top (using the ADS1115 at 2/3 gain)

    //checks for button presses here for lower chances of double clicks with the slower clock cycle
    lastButtonState_white = currentButtonState_white;      
    currentButtonState_white = digitalRead(BUTTON_PIN_WHITE); 
    lastButtonState_red = currentButtonState_red;      
    currentButtonState_red = digitalRead(BUTTON_PIN_RED); 

    //The red button is used to simulate a quench state change so that the user doesnt have to be connected to the gleeble to run this thing
    if(lastButtonState_red == LOW && currentButtonState_red == HIGH) {
      button_change_fps = true;
      quench_count++;
      if (quench_count == 1){
        t0 = millis();
      }
      quench_time = millis() - t0;
      quench_state = current_FPS_pin_state;
    }

    //Still deciding what to do witht the white button. It makes a nice clicking sound for now.
     
  }
  
  

  last_FPS_pin_state = current_FPS_pin_state;
  current_FPS_pin_state = digitalRead(fps_change_pin);
  

  //when the quench 4 state changes, the time from the first quench is recorded in ms along with the state and count. This data is included with every frame data output which may not be the most efficient way of doing it but it keeps everything in one place.
  if(last_FPS_pin_state != current_FPS_pin_state) {
    //Serial.println(current_FPS_pin_state);
    button_change_fps = true;
    quench_state = current_FPS_pin_state;
    quench_count++;
    if (quench_count == 1){
      t0 = millis();
    }
    quench_time = millis() - t0;
  } 

  if (button_change_fps == true) { //Runs if there is a change in the FPS change pin (either from gleeble Q4 or user button click)
    button_change_fps = false; 
    TCNT1 = 65015; // Adds a 33ms delay to avoid double triggers as the max continuous shooting speed of the current cameras are around 35 fps
    if (quench_count == 1) { //if its the first trigger, there is no need for a dellay so we set TCNT1 to 1 below the overflow value
      TCNT1 = 65535; 
    }
    counter_start = TCTN1_Values[quench_count];
    stop_trigger = false;
    if (counter_start == 0){
      TCNT1 = 0;
      digitalWrite(Trigger_Pin, LOW); 
      stop_trigger = true;
    }
  }

  //Check for if a frame has been triggered and outputs the relevant data for that frame
  trigger_pin_past = trigger_pin_current;
  trigger_pin_current = digitalRead(Trigger_Pin);

  if (trigger_pin_past == 0 && trigger_pin_current == 1) {
    shutter_str = "";
    shutter_str += frame_count;
    shutter_str += "\t";
    shutter_str += millis()-t0;
    shutter_str += "\t";
    //shutter_str += a0_milivolt; //this would be used if reading a volate output form a machine such as force data
    //shutter_str += "\t";
    shutter_str += quench_state;
    shutter_str += "\t";
    shutter_str += quench_count;
    shutter_str += "\t";
    shutter_str += quench_time;
    shutter_str += "\n";
    frame_count++;
    Serial.print(shutter_str);
  }
}
