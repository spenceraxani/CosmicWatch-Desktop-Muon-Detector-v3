// Tools-> Board = Teensy 4.0
// Tools-> Port = Select the port corresponding to where the detector is plugged in.
// Libraries needed. Go to Sketch->Include Libraries-> Manage Libraries and install:
//   1. Adafruit_SSD1306 : Version 2.4.0
//   2. Adafruit_BMP280 : Version 2.1.0
//   3. SDFat : Version 1.1.4

const boolean RESET_DETECTOR_NAME = false;        // Do you want to update the name of the detector?
String desired_detector_name      = "StephenHawking";     // If so, then to what?

int SHIFT_HOUR                    = 0;           // The realtime clock  doesn't understand daylight savings, sometimes you have to shift the hour.

int SIGNAL_THRESHOLD              = 50;          // The number of HGain ADC counts above the baseline before triggering
int RESET_THRESHOLD               = 25;           // Number of ADC counts above the baseline before resetting the trigger
const boolean OLED                = true;         // Turn on/off OLED. 
const int LED_BRIGHTNESS          = 255;          // Brightness of the 5mm LED [0,255]
const int LED_BRIGHTNESS_SMALL    = 255;          // Brightness of the 3mm LED [0,255]
const boolean SAVE_ALL_EVENTS     = true;        // Do you want to save all events? Set to false if you only want to save muons.

// Set these to false to turn off noise.
const boolean PLAYSTARTMUSIC      = false;         // Play music during startup
const boolean PLAYEVENTNOISE      = false;         // Make a noise during an event.

// You don't need this.
const boolean CALIBRATE           = false;        // Do you want to calibrate the detector? 
const boolean VERBOSE             = true;         // print out detector information through the serial port.

//############################################################################################################################



// A boolean to store whether or not another detector is plugged in.
boolean COINCIDENCE = false; // Is there another detector plugged into the RJ45 connector?

// Setup the real-time clock
#include <TimeLib.h>

// Setup the reset button.
#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
const int reset_pin = 23;

// Setup ADC for measuring the SiPM Pulses
#include <ADC.h>
#include <ADC_util.h>
ADC *adc = new ADC();
float sampling_period = 0; // sampling_period defines how many us it takes to make 1 ADC sample.

// Setup the OLED screen. It would be good to use a different library. This is the dominant source of deadtime.
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1
IntervalTimer myTimer;
Adafruit_SSD1306 display(OLED_RESET);
boolean trigger_OLED_readout = true;

// Setup the temperature/pressure sensor, BMP280
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SPI.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 temp_pressure_sensor; // I2C
float event_temperature = 0;          // C
float event_pressure    = 0;          // Pa

      
// Setup the microSD Card writer. The SensePin tells you if a physical SD card is ins
#include "SdFat.h"
SdFat SD;
SdFile myFile;
const uint8_t chipSelect = SS;
char filename[] = "File_000.txt"; // The default file name on the SDCard.
boolean trigger_microSD_readout = true;
IntervalTimer myTimer_microSD;
const int SD_sense_pin = 4;
boolean SDAvailable = true;

// Setup the speaker/buzzer to play sounds.
#include "pitches.h"
const int speaker_pin = 7;
int Event_noteDuration = 8;
int Event_noteKey = _C4;


// Setup the LED pin to flash when there is an event.
const int led_pin  = 14; // LED for all events
const int led_pin_small = 15; // LED for muon-only events.

// Setup the code for the detector name.
#include <EEPROM.h>
const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 511;
const int BUFSIZE = 40;
char detector_name[BUFSIZE]; // The detector name 
int chars_in_detector_name = 0;

// Event buffer information. Each entry in the array will correspond to an event.
float         event_SiPM_peak_voltage = 0;
byte          event_coincident = 0;
unsigned long long event_timestamp = 0;
float         event_measured_ADC_HGAIN = 0;
float         event_measured_ADC_LGAIN = 0;
unsigned long long event_deadtime = 0;
unsigned long long  total_deadtime = 0;
unsigned long long event_number = 0;
unsigned long long coincidence_event_number = 0L;   // A tally of the number of coincidence triggers counts observed
unsigned int event_hour;
unsigned int event_minute;
unsigned int event_second;
unsigned long long event_millis;
unsigned int event_year;
unsigned int event_month;
unsigned int event_day;




// Setup the array to store the ADC values. These are the samples that determine the pulse amplitude.
const int number_of_ADC_samples = 3; 
int adc_values_LGain[number_of_ADC_samples] = {};
int adc_values_HGain[number_of_ADC_samples] = {};

// Initiate the event counter. 
//unsigned long int counter = 0L;               // A tally of the number of triggers counts observed
//unsigned long int coincidence_counter = 0L;   // A tally of the number of coincidence triggers counts observed

// Setup the RJ45 communication. This sends digital pulses to other detectors connected with the ethernet cable.
const int coincident_pin_1 = 2;
const int coincident_pin_2 = 3;
int coincident_signal_input_pin = coincident_pin_2;
int coincident_signal_output_pin = coincident_pin_1;

//boolean readout_data = 0;  // Readout data when 1.

unsigned long long start_time                    = 0L;      // Detector start time reference variable


// The following are used to convert a measured ADC value to a SiPM pulse amplitude.
// The numbers here represent the coeffiecients to a polynomial.
const long double calibration_LGAIN[] = {7.770331963379388, 0.8551744828904521, -0.022610435459974862, 
        0.0003373183522713416, -2.7337802203549252e-06, 1.3226510403668739e-08, -4.0439095029897887e-11, 
        8.051241933191732e-14, -1.0542207510231822e-16, 9.042246953474205e-20, -5.069161049407975e-23, 
        1.9481994863843597e-26, -5.8210764000045555e-30, 1.1493759151290367e-33};
const long double calibration_HGAIN[] = {8.50709099942261, -0.11778994093618891, 0.0017674792660310806,
            -1.0974310076889382e-05, 3.860681705887885e-08, -8.15063422576611e-11, 1.0149341982138817e-13,
            -6.819510515623369e-17, 1.8973203130950888e-20};
float turn_over_value = 15.;
float sigmoid_width = 1.5; 
    
// The slopes of the above calibration polynomial fits are used to determine 
// which channel is a better measure of the SiPM pulse amplitude.
long double calibration_HGAIN_slope[sizeof(calibration_HGAIN)/sizeof(long double)]; 
long double calibration_LGAIN_slope[sizeof(calibration_HGAIN)/sizeof(long double)];

// Setup external calibration. This is to determine how to convert from ADC to SiPM peak voltage
unsigned int  n_samples = 200;         // How many samples to take per voltage level
int           step_size_low = 1;       // Detector start time reference variable
int           start_value_low = 5;     // Detector start time reference variable
int           step_size_mid = 2;      // Detector start time reference variable
int           start_value_mid = 20;    // Detector start time reference variable
int           step_size_high = 10;     // Detector start time reference variable
int           start_value_high = 50;    // Detector start time reference variable
int           desired_input_voltage = start_value_low;
int           delay_between_measurements = 5000;

//baseline properties
int n_baseline_samples = 100.;
int HGain_baseline = 0;
int LGain_baseline = 0;
float baseline_average = 0;
float baseline_std = 0;

//############################################################################################################################
void setup() {
  // Set the reset button to softRestart the detector.   
  pinMode(reset_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(reset_pin), soft_restart, FALLING);

  // Start the USB serial communication 
  Serial.begin(115200);

  // Setup the LED to output a flash when an event triggers.
  pinMode(led_pin, OUTPUT);
  pinMode(led_pin_small, OUTPUT);
  
  
  // if RESET_DETECTOR_NAME, this will rename the detector. You have to then change RESET_DETECTOR_NAME to false.
  if (RESET_DETECTOR_NAME){
    set_detector_name();
    if (VERBOSE){
      Serial.println("# Setting device ID: "+(String)desired_detector_name); 
      }
    }
  get_detector_name(detector_name);

  for (int i = 0; i<BUFSIZE; i++){
    if (detector_name[i]){
        chars_in_detector_name += 1;
        }
      }
      
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(2);         // Upside down screen (0 is right-side-up)
  OpeningScreen();                // Run the splash screen on start-up
  check_for_coincident_detector(); 

  

  // The SD sense pin checks to see if an SD card is inserted.
  pinMode(SD_sense_pin, INPUT_PULLUP);

  // Start the temp and pressure sensor
  temp_pressure_sensor.begin(0x76);
  temp_pressure_sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  

  // Get the detector name from the EEPROM
  if (VERBOSE){
    if (chars_in_detector_name > 20){
      Serial.println("# Det Name too large. Keep <= 20 characters.");
    }
    Serial.println("# Found detector ID: "+(String)detector_name); 
  }
    // Start up the OLED screen
  
  if (VERBOSE){
    look_for_devices();
    Serial.println("# The default address for OLED: 0x3C"); 
    Serial.println("# The default address for BMP280 temperature/pressure sensor: 0x76");
  }

  if (!SD.begin(chipSelect, SD_SCK_MHZ(50))) {
    Serial.println(F("# SD initialization failed!"));
    Serial.println(F("# Is there an SD card inserted?"));
    //SD.initErrorHalt();
    SDAvailable = false;
  }
  
  /*
  if (!SD.begin()) {
    Serial.println(F("# SD initialization failed!"));
    Serial.println(F("# Is there an SD card inserted?"));
  }
  */
  setup_sd_card_files();
  OpeningInfo();
  
  if (digitalRead(SD_sense_pin) == LOW) {
    if (VERBOSE){
    Serial.println("# MicroSD card inserted.");
    }
  }
  
  if (digitalRead(SD_sense_pin)==HIGH) {
    if (VERBOSE){
    Serial.println("# No MicroSD card inserted.");
    }
  }
  
  if (Serial){
    if (VERBOSE){
    Serial.println("# I see serial communication.");
    }
   }

  // Setup the ADC to make some high-frequency measurements.
  adc->adc0->setAveraging(1);
  adc->adc0->setResolution(10);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);

  // Measure the baseline mode, average, and std. Also measure the sample rate of the ADC.
  measure_baseline();

  
  // This gets the time from the computer to set the real-time clock
  
  setSyncProvider(getTeensy3Time);
  if (Serial){
    
    if (VERBOSE){
      if (timeStatus()!= timeSet) {
        Serial.println("# Unable to sync with the RTC");
      } else {
        Serial.println("# RTC has set the system time");
      }
    }
  }
  
  if (CALIBRATE){
    Serial.println("# Calibration is active. ");
    }

  display.clearDisplay();  
  delay(1500);                    // Delay some time to show the logo, and keep the Pin6 HIGH for coincidence




  // Calculate the slopes of the calibration curves. 
  //calculate_calibration_slopes();
  

  // Update the OLED every second
  myTimer.begin(trigger_OLED, 1000000);
  //myTimer.priority(255);

  // write to the sd card every 10seconds
  myTimer_microSD.begin(trigger_microSD, 10000001);


  // Print the header information to the file.
  print_header();


  // Syncronize the realtime clock with the millis function.
  int start_second = second();
  while(start_second==second()){
    continue;
    }
  start_time = millis();
  
}

//############################################################################################################################
//############################################################################################################################

void loop() {
  while(1){
  if (trigger_OLED_readout){
    update_OLED();
    }

  if (trigger_microSD_readout){
    update_microSD();
    }

  // If we see that the ADC measurement is above the threshold, trigger.
  //float adc_measurement = adc->adc0->analogRead(A8);
/*
  unsigned long int t1a = micros();
  while(adc->adc0->analogRead(A8) > RESET_THRESHOLD){
    //Serial.println(" ");
    }
  unsigned long int t2a = micros();
   */
  float adc_measurement = adc->adc0->analogRead(A8);
  //Serial.println(adc_measurement);
  if (adc_measurement > SIGNAL_THRESHOLD){
    // Turn off interupts while we measure the waveform.
    noInterrupts();
    unsigned long int t1 = micros();
    event_timestamp = millis() - start_time; 

    // Send out a coincidence signal
    digitalWrite(coincident_signal_output_pin, HIGH);
    
    // Check a few times for the coincidence signal another detector. This actually defines how long the 
    // coincidence window is open. 30 loops = 1.5us.
    for (int i = 0; i < 100; i++) {
      if (digitalRead(coincident_signal_input_pin) == HIGH){ 
        event_coincident = 1;
      }
    }
    
    //Close coincidence window
    digitalWrite(coincident_signal_output_pin, LOW);
    
    if (event_coincident){
      analogWrite(led_pin, LED_BRIGHTNESS);
      analogWrite(led_pin_small, LED_BRIGHTNESS_SMALL);
      coincidence_event_number+=1;
    }
    else{
      analogWrite(led_pin, LED_BRIGHTNESS);
    }
    
    event_hour = hour();
    event_minute = minute();
    event_second = second();
    event_millis = (millis()-start_time)%1000;
    event_year = year();
    event_month = month();
    event_day = day();

    
    // Measure the waveforms for the HGain and LGain Channel.
    for (int i = 0; i < number_of_ADC_samples; i++) {
      adc_values_LGain[i] = adc->adc0->analogRead(A6);// - LGain_baseline;
      adc_values_HGain[i] = adc->adc0->analogRead(A8);// - HGain_baseline;
    }
    //Serial.println(HGain_baseline);
    //Serial.println(LGain_baseline);
    //fit_exp fits an exponential to the measured data. It takes the ADC values (y coordinates), and the trigger time.
        // If the other detector also sees and event, consider it coincidence. 
    //if (COINCIDENCE){
    
    //event_measured_ADC_LGAIN = fit_exp(adc_values_LGain,0);
    //event_measured_ADC_HGAIN = fit_exp(adc_values_HGain, 0);

    event_measured_ADC_LGAIN = (adc_values_LGain[0]);//+adc_values_LGain[1]+adc_values_LGain[2])/3.;
    event_measured_ADC_HGAIN = (adc_values_HGain[0]);//+adc_values_HGain[1]+adc_values_HGain[2])/3.;

    /*
     * 
     // Make sure the fitting worked
    if (event_measured_ADC_LGAIN < 1.){
      break;
    }
    if (event_measured_ADC_HGAIN < 1.){
      break;
    }
     */
    
    // If the other detector also sees and event, consider it coincidence. 

    event_number +=1;
    if (PLAYEVENTNOISE){
         tone(speaker_pin, Event_noteKey,  1000/Event_noteDuration);
         delay(20);
        }
        
    // You can ignore the calibration part below. This is for internal stuff.
    if (CALIBRATE){
      if (event_number > n_samples){
        if (desired_input_voltage < start_value_mid) {
          desired_input_voltage += step_size_low;
          }
       
        if (desired_input_voltage < start_value_high) {
          if (desired_input_voltage >= start_value_mid){
               desired_input_voltage += step_size_mid;
           }
        }
        if (desired_input_voltage >= start_value_high) {
          desired_input_voltage += step_size_high;
        }
     
     
    
     event_number = 0;
     Serial.println("# --- NEXT --- #");
     Serial.println("#     " +(String)desired_input_voltage + "     #");
     delay(delay_between_measurements);
     }
     Serial.println((String)desired_input_voltage + " " +(String)event_measured_ADC_HGAIN +" "  +(String)event_measured_ADC_LGAIN);
     analogWrite(led_pin, 0);
    }
   else{
      event_temperature       = temp_pressure_sensor.readTemperature();    // C
      event_pressure          = temp_pressure_sensor.readPressure();       // Pa
      event_SiPM_peak_voltage = get_SiPM_peak_voltage(event_measured_ADC_HGAIN, event_measured_ADC_LGAIN);
      total_deadtime += event_deadtime;
      
      print_data_to_serial();
      analogWrite(led_pin, 0);
      
      print_data_to_microSD();
      event_deadtime   = 0;
      event_coincident = 0;
    }

    while(adc->adc0->analogRead(A8) > RESET_THRESHOLD){continue;}

    unsigned long int t2 = micros();
    event_deadtime += t2 - t1;
   // event_deadtime += t2a - t1a;
    analogWrite(led_pin_small, 0);
    //noTone(speaker_pin);
    interrupts();
  }
}
}

//############################################################################################################################
//############################################################################################################################
//############################################################################################################################
float sigmoid(float SiPM_value){
    return 1 / (1 + exp(-(SiPM_value-turn_over_value)/sigmoid_width));
  }
  
float get_SiPM_peak_voltage(float HGain_adc_value, float LGain_adc_value){
  // This function takes the measured ADC values and returns the expected SiPM peak voltage required to generate
  // the measured ADC values.

  float HGAIN_SiPM_peak_voltage = 0;
  for (unsigned int i = 0; i < (sizeof(calibration_HGAIN)/sizeof(long double)); i++) {
    HGAIN_SiPM_peak_voltage += calibration_HGAIN[i] * pow(HGain_adc_value,i);
    }
    
  float LGAIN_SiPM_peak_voltage = 0;
  for (unsigned int i = 0; i < (sizeof(calibration_LGAIN)/sizeof(long double)); i++) {
    LGAIN_SiPM_peak_voltage += calibration_LGAIN[i] * pow(LGain_adc_value,i);
    }

  /*
  float HGAIN_slope = 0;
  for (unsigned int i = 1; i < (sizeof(calibration_HGAIN)/sizeof(long double)); i++) {
    HGAIN_slope += calibration_HGAIN_slope[i] * pow(HGain_adc_value,i-1);
    }
  
  float LGAIN_slope = 0;
  for (unsigned int i = 1; i < (sizeof(calibration_LGAIN_slope)/sizeof(long double)); i++) {
    LGAIN_slope += calibration_LGAIN_slope[i] * pow(LGain_adc_value,i-1);
    }
  */
  //float ratio_of_slopes = HGAIN_slope/LGAIN_slope;
  /*
  Serial.println(ratio_of_slopes);
  if (ratio_of_slopes > 1.){
    return (1-1/ratio_of_slopes)*HGAIN_slope + (1/ratio_of_slopes)*LGAIN_slope;
  }
  else {
    return (1-ratio_of_slopes)*HGAIN_slope + (ratio_of_slopes)*LGAIN_slope;
  }
  */

  
  
  
  //Serial.println(sigmoid(HGAIN_SiPM_peak_voltage));
  //Serial.println(1-sigmoid(HGAIN_SiPM_peak_voltage));
  
  return (1-sigmoid(HGAIN_SiPM_peak_voltage))*HGAIN_SiPM_peak_voltage+\
            sigmoid(HGAIN_SiPM_peak_voltage)*LGAIN_SiPM_peak_voltage;
  
  /*
  if (LGain_adc_value > 50){
    return LGAIN_SiPM_peak_voltage;
  }
  else{
    return HGAIN_SiPM_peak_voltage;
  }
  */
  
}




void trigger_OLED() {
  trigger_OLED_readout = true;
}

void trigger_microSD() {
  trigger_microSD_readout = true;
}

void update_microSD() {
  // This function tells the detector to write the data to the SD card. 
  // We only call this rarely (period defined by myTimer_microSD). This 
  // is because writing to the sd card creates a large amount of deadtime.
  myFile.flush();
  trigger_microSD_readout = false;
  }



void calculate_calibration_slopes(){ 
  // The calibration curves convert the measured ADC  value to a peak SiPM voltage.
  // When the slope is low, that is, a large change in ADC counts corresponds to a small change in SiPM voltage
  // the measurement is more precise. We will use the slopes to determine which channel (HGain or LGain) is more 
  // precise.
  for (unsigned int i = 0; i < (sizeof(calibration_HGAIN)/sizeof(long double)); i++) {
    calibration_HGAIN_slope[i] = i * calibration_HGAIN[i];
    }
  for (unsigned int i = 0; i < (sizeof(calibration_LGAIN)/sizeof(long double)); i++) {
    calibration_LGAIN_slope[i] = i * calibration_LGAIN[i];
    }
  return;
}

float fit_exp(int y[], long int t_trigger){  
  // This is a least squares fit to the ADC data. After we fit an exponential to it, 
  // we then calculate the expected amplitude at the trigger time. y[] is the ADC data.
  // t_trigger is the trigger time. This function returns the ADC amplitude at time t_trigger.
  float sum_ln_y   = 0;
  float sum_x_sq   = 0;
  float sum_x      = 0;
  float sum_x_ln_y = 0;
  int n = number_of_ADC_samples;
  for (int i = 0; i < n; i++) {
    float x = i*sampling_period;
    sum_ln_y +=  log(y[i]);
    sum_x_sq += x*x;
    sum_x    += x;
    sum_x_ln_y += x*log(y[i]);
  }
  float A = (sum_ln_y*sum_x_sq-sum_x*sum_x_ln_y)/(n*sum_x_sq  - sum_x*sum_x);
  float B = (n*sum_x_ln_y-sum_x*sum_ln_y)/(n*sum_x_sq - sum_x*sum_x);
  return exp(A)*exp(-B*t_trigger);
}


void print_data_to_serial()
{
  unsigned long int t1 = micros();
  Serial.print((String)event_number+ "\t");
  if (event_hour + SHIFT_HOUR > 23){
    Serial.print(event_hour + SHIFT_HOUR - 24);
  }
  else{
    Serial.print(event_hour + SHIFT_HOUR);
    }
  Serial.print(":");
  if(event_minute < 10)
    Serial.print('0');
  Serial.print(event_minute);
  Serial.print(":");
  if(event_second < 10)
    Serial.print('0');
  Serial.print(event_second);
  Serial.print(".");
  if(event_millis < 100)
    Serial.print('0');
  if(event_millis < 10)
    Serial.print('0');
  Serial.print(event_millis);
  Serial.print("\t");
  Serial.print(event_day);
  Serial.print("/");
  Serial.print(event_month);
  Serial.print("/");
  Serial.print(event_year); 
  Serial.println( "\t"+(String)event_timestamp+ "\t"\
                    + (String)event_measured_ADC_HGAIN+ "\t"\
                    + (String)event_measured_ADC_LGAIN+ "\t"\
                    + (String)event_SiPM_peak_voltage+ "\t"\
                    + (String)event_temperature+ "\t"\
                    + (String)event_pressure+"\t"\
                    + (String)event_deadtime+"\t"\
                    + (String)event_coincident+"\t"\
                    + (String)detector_name+"\t"\
                    );
  unsigned long int t2 = micros();
  event_deadtime += t2 - t1;
}

void print_data_to_microSD()
{
  unsigned long int t1 = micros();
  myFile.print((String)event_number+ "\t");
  if (hour() + SHIFT_HOUR > 23){
    myFile.print(hour() + SHIFT_HOUR - 24);
  }
  else{
    myFile.print(hour() + SHIFT_HOUR);
    }
  myFile.print(":");
  if(minute() < 10)
    myFile.print('0');
  myFile.print(minute());
  myFile.print(":");
  if(second() < 10)
    myFile.print('0');
  myFile.print(second());
  myFile.print("\t");
  myFile.print(day());
  myFile.print("/");
  myFile.print(month());
  myFile.print("/");
  myFile.print(year()); 
  
  myFile.println( " \t"+(String)event_timestamp+ "\t"\
                    + (String)event_measured_ADC_HGAIN+ "\t"\
                    + (String)event_measured_ADC_LGAIN+ "\t"\
                    + (String)event_SiPM_peak_voltage+ "\t"\
                    + (String)event_temperature+ "\t"\
                    + (String)event_pressure+"\t"\
                    + (String)event_deadtime+"\t"\
                    + (String)event_coincident+"\t"\
                    );

  //measure_baseline();
  
  unsigned long int t2 = micros();
  event_deadtime += t2 - t1;
}


void update_OLED()
{
  unsigned long int t1 = micros();
  if (OLED){
    int NCounts;
    
    
    if (COINCIDENCE){
      NCounts = coincidence_event_number;
    }
    else{
      NCounts = event_number;
    }
  
    char counter_char[40];
    unsigned long int OLED_t1             = millis();
    float count_average                   = 0;
    float count_std                       = 0;
    float muon_count_average                   = 0;
    float muon_count_std                       = 0;
    
    //count_average   = NCounts / ((OLED_t1 - start_time - total_deadtime/1000.) / 1000.);
    muon_count_average   = coincidence_event_number / ((OLED_t1 - start_time - total_deadtime/1000.) / 1000.);
    muon_count_std       = sqrt(coincidence_event_number) / ((OLED_t1 - start_time  - total_deadtime/1000.) / 1000.);
    count_average   = event_number / ((OLED_t1 - start_time - total_deadtime/1000.) / 1000.);
    count_std       = sqrt(event_number) / ((OLED_t1 - start_time  - total_deadtime/1000.) / 1000.);


    display.setCursor(0, 0);
    display.clearDisplay();
    display.print(F("Total Count: "));
    sprintf(counter_char, "%01d", NCounts);
    display.println(counter_char);
    display.print(F("Uptime: "));
  
    int hours                   = ((OLED_t1 - start_time) / 1000 / 3600);
    int minutes                 = ((OLED_t1 - start_time) / 1000 / 60) % 60;
    int seconds                 = ((OLED_t1 - start_time) / 1000) % 60;
    
    char hour_char[6];
    char min_char[6];
    char sec_char[6];
    
    
    sprintf(min_char, "%02d", minutes);
    sprintf(sec_char, "%02d", seconds);
    sprintf(hour_char, "%02d", hours);
  
    display.print(hour_char);
    display.print(":" );
    display.print(min_char);
    display.print(":" );
    display.println(sec_char);

    char tmp_average[10];
    char tmp_std[10];
    char tmp_average_muon[10];
    char tmp_std_muon[10];
    
  
    int decimals = 2;
    if (count_average < 10) {decimals = 3;}

    int muon_decimals = 2;
    if (muon_count_average < 10) {decimals = 3;}

    
    dtostrf(count_average, 1, decimals, tmp_average);
    dtostrf(count_std, 1, decimals, tmp_std);
    
    dtostrf(muon_count_average, 1, decimals, tmp_average_muon);
    dtostrf(muon_count_std, 1, decimals, tmp_std_muon);
    
    
    if (COINCIDENCE){
      display.print(F("Rate: "));
      display.print((String)tmp_average);
      display.print(F("+/-"));
      if (decimals == 3){
        display.print((String)tmp_std);
        display.print("Hz");
      }
      else{
        display.println((String)tmp_std);
        }
      
      display.print(F("Rate: "));
      display.print((String)tmp_average_muon);
      display.print(F("+/-"));
      if (muon_decimals == 3){
        display.print((String)tmp_std_muon);
        display.println("Hz    ");
        }
      else{
        display.println((String)tmp_std_muon);
        }
      }
      else{
        if (event_SiPM_peak_voltage > 200){
            display.print(F("===---- WOW! ----==="));}
        else{
              if (COINCIDENCE) {display.print(F("C"));}
              else{display.print(F("M"));}
              for (int i = 1; i <=  (event_SiPM_peak_voltage + 10) / 10; i++) {display.print(F("-"));}}
       
      display.println(F(""));
      
      display.print(F("Rate: "));
      display.print((String)tmp_average);
      display.print(F("+/-"));
      if (decimals == 3){
        display.print((String)tmp_std);
        display.println("Hz");
      }
      else{
        display.println((String)tmp_std);
        }
      }

      
      
   
  
    display.display();
  }
  
  unsigned long int t2 = micros();
  event_deadtime += t2 - t1;
  trigger_OLED_readout = false;
  
}


void setup_sd_card_files() {
  // This function looks through the files on the SD card and creates one with a unique name.
  for (uint8_t i = 1; i <= 1000; i++) {
    int hundreds = (i - i / 1000 * 1000) / 100;
    int tens = (i - i / 100 * 100) / 10;
    int ones = i % 10;
    filename[5] = hundreds + '0';
    filename[6] = tens + '0';
    filename[7] = ones + '0';

    if (!SD.exists(filename)) {
      if (VERBOSE){
      Serial.println("# Creating MicroSD card file: " + (String)filename);
      }
      delay(500);
      myFile.open(filename, O_WRONLY | O_CREAT | O_EXCL);
      //myFile = SD.open(filename, FILE_WRITE);
      break;

    
    }
    /*
    if (!SD.exists(filename)) {
      if (VERBOSE){
      Serial.println("# Creating MicroSD card file: " + (String)filename);
      }
      delay(500);
      myFile.open(fileName, O_WRONLY | O_CREAT | O_EXCL)
      //myFile = SD.open(filename, FILE_WRITE);
      break;
    }
    */
  }
}


void check_for_coincident_detector(){
  // This function sends a signal through the RJ45 connnector (coincident_signal_output_pin). 
  // If another detector is plugged in, it sees the signal and then sends one of its own signals.
  // The first detector looks for a signal on the coincident_signal_input_pin, if seen, it also 
  // knows that another detector is plugged in. Reset both detectors within 2seconds of each other
  // for them to see each other.
  
    pinMode(coincident_pin_1, INPUT);
    if (digitalRead(coincident_pin_1) == HIGH){
      pinMode(coincident_pin_2, OUTPUT);
      digitalWrite(coincident_pin_2,HIGH);
      coincident_signal_input_pin  = coincident_pin_1;
      coincident_signal_output_pin = coincident_pin_2;
      
      digitalWrite(led_pin, HIGH);
      digitalWrite(led_pin_small, HIGH);
      delay(1000);
      digitalWrite(led_pin, LOW);
      digitalWrite(led_pin_small, LOW);
      digitalWrite(coincident_pin_2,LOW);
      
      COINCIDENCE = true; // Another detector observed.
      filename[4] = 'C'; 
      Serial.println("# Coincidence detector found.");
      delay(300);
    }
    
    else {
      pinMode(coincident_pin_1, OUTPUT);
      digitalWrite(coincident_pin_1,HIGH);
      pinMode(coincident_pin_2, INPUT);
      filename[4] = 'M'; // Label the filename as "M"aster
      //delay(200);
      for (int i = 0; i < 10; i++) {
        delay(100);
        if (digitalRead(coincident_pin_2) == HIGH){
          coincident_signal_input_pin  = coincident_pin_2;
          coincident_signal_output_pin = coincident_pin_1;
          digitalWrite(led_pin, HIGH);
          digitalWrite(led_pin_small, HIGH);
          delay(1000);
          digitalWrite(led_pin, LOW);
          digitalWrite(led_pin_small, LOW);
          COINCIDENCE = true; // Another detector observed.
          filename[4] = 'C';  // Label the filename as "C"oincidence
          Serial.println("# Coincidence detector found.");
          break;
        }
      }
  
    }
    
}


void play_start_music()
  // Here, we simply play some startup music.
{
  if (PLAYSTARTMUSIC){
  int melody[] = {
    //_C4, _G3, _G3, _A3, _G3, 0, _B3, _C4
    _C4, _D4, _E4, _F4, _G4, _A4, _B4, _C3
  };
  
  // note durations: 4 = quarter note, 8 = eighth note, etc.:
  int noteDurations[] = {
    4, 8, 8, 4, 4, 4, 4, 4
  };

  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(speaker_pin, melody[thisNote], noteDuration);
    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(speaker_pin);
  }
  }
  else {
    delay(1500);
  }
}

boolean print_header()
  // The header is printed to the Serial port and the SD Card.
  {
    Serial.println(F("################################################################################################"));
    Serial.println(F("### CosmicWatch: The Desktop Muon Detector"));
    Serial.println("### Device ID: " + (String)detector_name);
    Serial.print("### Launch time: ");     
    Serial.print(hour());
    Serial.print(":");
    if(minute() < 10)
      Serial.print('0');
    Serial.print(minute());
    Serial.print(":");
    if(second() < 10)
      Serial.print('0');
    Serial.print(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print("/");
    Serial.print(month());
    Serial.print("/");
    Serial.print(year()); 
    Serial.println(); 
    Serial.println(F("### Questions? Email Spencer N. Axani (saxani@mit.edu)"));
    Serial.println(F("### Event Time Date TimeStamp[ms] ADC1 ADC2 SiPM[mV] Temp[C] Pressure[Pa] DeadTime[us] Coincident"));
    Serial.println(F("################################################################################################"));
    
    myFile.println(F("################################################################################################"));
    myFile.println(F("### CosmicWatch: The Desktop Muon Detector"));
    myFile.println("### Device ID: " + (String)detector_name);
    myFile.print("### Launch time: ");     
    myFile.print(hour());
    
    myFile.print(":");
    if(minute() < 10)
      myFile.print('0');
    myFile.print(minute());
    myFile.print(":");
    if(second() < 10)
      myFile.print('0');
    myFile.print(second());
    myFile.print(" ");
    myFile.print(day());
    myFile.print("/");
    myFile.print(month());
    myFile.print("/");
    myFile.print(year()); 
    myFile.println(); 
    myFile.println(F("### Questions? Email Spencer N. Axani (saxani@mit.edu)"));
    myFile.println(F("### Event Time Date TimeStamp[ms] ADC1 ADC2 SiPM[mV] Temp[C] Pressure[Pa] DeadTime[us] Coincident"));
    myFile.println(F("################################################################################################"));
    myFile.flush();
    return true;
  }


void soft_restart()
  // This is to restart the detector.
  {
    WRITE_RESTART(0x5FA0004);
  }

void measure_baseline()
{
  while(true){
  // This function measures properties with the baseline

  long int HGain_baseline_adc_samples[n_baseline_samples] = {};
  long int LGain_baseline_adc_samples[n_baseline_samples] = {};
  long int HGain_hist_baseline_samples[1024]    = {};
  long int LGain_hist_baseline_samples[1024]    = {};
  
  memset(HGain_hist_baseline_samples, 0, sizeof(HGain_hist_baseline_samples));
  memset(LGain_hist_baseline_samples, 0, sizeof(LGain_hist_baseline_samples));

  //float t1_adc_cal =  micros();
  for (int i = 0; i < n_baseline_samples; i++) {
    HGain_baseline_adc_samples[i] = adc->adc0->analogRead(A8);
    delayMicroseconds(50);
    LGain_baseline_adc_samples[i] = adc->adc0->analogRead(A6);
    delayMicroseconds(50);
  }

  float t1_adc_cal =  micros();
  for (int i = 0; i < n_baseline_samples; i++) {
    adc->adc0->analogRead(A8);
  }
  float t2_adc_cal =  micros();
  sampling_period = (t2_adc_cal-t1_adc_cal)/n_baseline_samples;

  //float sum_baseline_adc_counts=0;
  float HGain_max_baseline_sample = 0;
  float LGain_max_baseline_sample = 0;
  
  float sum_baseline_adc_counts = 0;
  for (int i = 0; i < n_baseline_samples; i++) {
    HGain_hist_baseline_samples[HGain_baseline_adc_samples[i]] += 1;
    LGain_hist_baseline_samples[LGain_baseline_adc_samples[i]] += 1;
    sum_baseline_adc_counts += HGain_baseline_adc_samples[i];
    
    if (HGain_baseline_adc_samples[i] > HGain_max_baseline_sample){
      HGain_max_baseline_sample =  HGain_baseline_adc_samples[i];
    }
    if (LGain_baseline_adc_samples[i] > LGain_max_baseline_sample){
      LGain_max_baseline_sample =  LGain_baseline_adc_samples[i];
    }
  }
  
  
  int max_element = 0;
  for (int i= 0; i < 1023; i++){
      if (HGain_hist_baseline_samples[i] > max_element){
        HGain_baseline = i;
      }
  }
  
  max_element = 0;
  for (int i= 0; i < 1023; i++){
      if (LGain_hist_baseline_samples[i] > max_element){
        LGain_baseline = i;
      }
  }

  baseline_average = sum_baseline_adc_counts/n_baseline_samples;
  float xmu2 = 0;
  for (int i = 0; i < n_baseline_samples; i++) {
    xmu2 += pow(HGain_baseline_adc_samples[i] - baseline_average,2);
  }
  
  baseline_std = 1/sqrt(n_baseline_samples)*sqrt(xmu2);

  if (HGain_baseline > 30){
    Serial.println("# Baseline appears to be higher than expected");
  }
  else{
    break;
  }

   //Serial.println((String)new_HGain_baseline + " " + (String)new_LGain_baseline);
   //if baseline_std
  /*
  if (max_baseline_sample > (baseline_average + 20)){
    if (VERBOSE){
      Serial.println("# Noise or pulse likely in baseline. Remeasuring...");
      Serial.println(baseline_average);
      Serial.println(max_baseline_sample);
      Serial.println(baseline_std);
    } 
  }
  */
  }
  //else{
  SIGNAL_THRESHOLD = HGain_baseline + SIGNAL_THRESHOLD;
  RESET_THRESHOLD  = HGain_baseline + RESET_THRESHOLD;
  
  if (VERBOSE){
    Serial.println("# Sampling period of ADC: "+(String)sampling_period+"us"); 
    Serial.println("# HGain Baseline Mode: "+(String)HGain_baseline+"ADC counts"); 
    Serial.println("# LGain Baseline Mode: "+(String)HGain_baseline+"ADC counts"); 
    Serial.println("# Baseline Average: "+(String)baseline_average+"ADC counts");
    Serial.println("# Baseline STD: " +(String)baseline_std+"ADC counts"); 
    Serial.println("# Trigger threshold: "+(String)SIGNAL_THRESHOLD+"ADC counts, ");//+(String)(SIGNAL_THRESHOLD * 3.3/1024.)+"mV");
    Serial.println("# Reset treshold " +(String)RESET_THRESHOLD+"ADC counts, ");//+(String)(RESET_THRESHOLD * 3.3/1024.)+"mV");
  }
    
    //}
    //break;
  //}
}





void OpeningScreen(void)
  // This is the splash screen when the detector turns on.
  {

    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(8, 0);
    display.clearDisplay();
    display.print(F("Cosmic \n     Watch"));
    display.display();
    display.setTextSize(1);
    display.clearDisplay();
  }

void OpeningInfo(void)
  // This is the splash screen when the detector turns on.
  {
    display.setCursor(8, 0);
    display.clearDisplay();
    display.println(" Hi, my name is");

    display.setTextSize(1);

    //int16_t x1, y1;
    //uint16_t w, h;
    //display.getTextBounds(detector_name, 13, 12, &x1, &y1, &w, &h); //calc width of new string
    //Name character length is w/12. Each charachter is 4
    //display.setCursor(64 - 6*w/12, 10);

    //chars_in_detector_name
    
    display.setCursor(64 - 7*chars_in_detector_name/2,10);
    display.println(detector_name);
    display.setCursor(0, 24);
    if (SDAvailable ==false){
      display.print("No SD card available. ");
      }
    else {
      display.print("File: ");
      display.println(filename);
      }
    
    display.display();
    display.setTextSize(1);
    play_start_music();
  }

void look_for_devices()
  // This function will look through the attached devices and print out their addresses.
  // Typically, this should just be the addresses for the OLED and Temp/Pressure sensor.
  {
    byte error, address;
    int nDevices;
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
   
      if (error == 0)
      {
        if(VERBOSE){
        Serial.print("# I2C device found at address 0x");
        }
        if (address<16)
          Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
   
        nDevices++;
      }
      else if (error==4)
      {
        Serial.print("Unknown error at address 0x");
        if (address<16)
          Serial.print("0");
        Serial.println(address,HEX);
      }    
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
  }


// The remaining functions here are used to write the detector name to the EEPROM.
// The EEPROM is memory that doesn't get lost when the detector is reset. Non-volatile.
boolean eeprom_is_addr_ok(int addr) {
    return ((addr >= EEPROM_MIN_ADDR) && (addr <= EEPROM_MAX_ADDR));
  }
  
boolean eeprom_write_bytes(int startAddr, const byte* array, int numBytes) {
  int i;
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }
  for (i = 0; i < numBytes; i++) {
    EEPROM.write(startAddr + i, array[i]);
  }
  return true;
}

boolean eeprom_write_string(int addr, const char* string) {
  int numBytes; 
  numBytes = strlen(string) + 1;
  return eeprom_write_bytes(addr, (const byte*)string, numBytes);
}

boolean set_detector_name()
{
  char buf[BUFSIZE];
  for (int i = 0; i < BUFSIZE; i++) {
    buf[i] = ";";
    }
  desired_detector_name.toCharArray(buf,BUFSIZE); // convert string into array of chars.
  eeprom_write_string(0, buf);
  //strcpy(buf, det_name_Char);
  //eeprom_write_string(0, buf);
  return true;
}

  
boolean get_detector_name(char* det_name)
{
  byte ch;                              // byte read from eeprom
  int bytesRead = 0;                    // number of bytes read so far
  ch = EEPROM.read(bytesRead);          // read next byte from eeprom
  det_name[bytesRead] = ch;               // store it into the user buffer
  bytesRead++;                          // increment byte counter

  while ( (ch != 0x00) && (bytesRead < BUFSIZE) && ((bytesRead) <= 511) )
  {
    ch = EEPROM.read(bytesRead);
    if (ch != ";");
      det_name[bytesRead] = ch;           // store it into the user buffer
    bytesRead++;                        // increment byte counter
  }
  if ((ch != 0x00) && (bytesRead >= 1)) {
    det_name[bytesRead - 1] = 0;
  }
  return true;
}


time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}
