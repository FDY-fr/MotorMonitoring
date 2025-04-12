// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.
//int FDYinput;

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include <sensesp/transforms/lambda_transform.h>

//Screen 128 x 64
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Include the libraries for the DS28B20
#include <OneWire.h>
#include <DallasTemperature.h>

// flash memory R/W for engine run time counter
#include <Preferences.h>

// for RPM measurement
#include <sensesp/transforms/frequency.h>
#include <sensesp/transforms/moving_average.h>

// ePaper 128x296 includes
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h> // including both doesn't   use more code or ram
#include <U8g2_for_Adafruit_GFX.h>
// font gfx include
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBoldOblique24pt7b.h>
#include <Fonts/FreeSerifBold12pt7b.h>
#include <Fonts/FreeSerifBold9pt7b.h>
#include <Fonts/FreeSerifBold24pt7b.h>

//definition interface ecran e-paper
GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display1(GxEPD2_290_T94_V2(/*CS=5*/   SS, /*DC=*/ 17, /*RST=*/ 3, /*BUSY=*/ 4)); // GDEM029T94, Waveshare 2.9" V2 variant


/* 
FreeMono12pt7b.h		FreeSansBoldOblique12pt7b.h
FreeMono18pt7b.h		FreeSansBoldOblique18pt7b.h
FreeMono24pt7b.h		FreeSansBoldOblique24pt7b.h
FreeMono9pt7b.h			FreeSansBoldOblique9pt7b.h
FreeMonoBold12pt7b.h		FreeSansOblique12pt7b.h
FreeMonoBold18pt7b.h		FreeSansOblique18pt7b.h
FreeMonoBold24pt7b.h		FreeSansOblique24pt7b.h
FreeMonoBold9pt7b.h		FreeSansOblique9pt7b.h
FreeMonoBoldOblique12pt7b.h	FreeSerif12pt7b.h
FreeMonoBoldOblique18pt7b.h	FreeSerif18pt7b.h
FreeMonoBoldOblique24pt7b.h	FreeSerif24pt7b.h
FreeMonoBoldOblique9pt7b.h	FreeSerif9pt7b.h
FreeMonoOblique12pt7b.h		FreeSerifBold12pt7b.h
FreeMonoOblique18pt7b.h		FreeSerifBold18pt7b.h
FreeMonoOblique24pt7b.h		FreeSerifBold24pt7b.h
FreeMonoOblique9pt7b.h		FreeSerifBold9pt7b.h
FreeSans12pt7b.h		FreeSerifBoldItalic12pt7b.h
FreeSans18pt7b.h		FreeSerifBoldItalic18pt7b.h
FreeSans24pt7b.h		FreeSerifBoldItalic24pt7b.h
FreeSans9pt7b.h			FreeSerifBoldItalic9pt7b.h
FreeSansBold12pt7b.h		FreeSerifItalic12pt7b.h
FreeSansBold18pt7b.h		FreeSerifItalic18pt7b.h
FreeSansBold24pt7b.h		FreeSerifItalic24pt7b.h
FreeSansBold9pt7b.h		FreeSerifItalic9pt7b.h*/

// DS18B20 Data wire is plugged into port 17 of the ESP32
#define ONE_WIRE_BUS 27

// Flame detector on GPIO 15
#define  flamePin 15
#define  AnalogFlamePin 39
// Buzzer for smoke/flame detection on GPIO 4
#define  Buzzer 12

// ESP32's pin GPIO36 connected to analog input for the MQ2 smoke sensor
#define MQ2_PIN 36  
  

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//SensESP 
using namespace sensesp;
reactesp::ReactESP app;



// (Specific to the BMP280, and I2C. Replace this with similar code to create an instance
// of whatever sensor you may be using in your project.)
// Create an instance of the sensor using its I2C interface.
Adafruit_BME280 bme280;

// Init Display 128 x 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// flash memory R/W for engine run time counter: Name of the performance object is  =  EngiineRunTime
Preferences EngineRunTime;   
// counter in minute to limit the number of writing in the flash memory (i.e. 1000H = 60 000 ecritures)
//unsigned int CounterMinuteDisplay = 0;
//unsigned int CounterHeureDisplay = 0;
unsigned int CounterSecond = 0;
unsigned int CounterMinute = 0;
//unsigned int CounterSecondTrip =0;
unsigned int CounterMinuteTrip =0;
unsigned int CounterHeureTrip = 0;

// MQ2 Smoke and gaz value
int gasValue;
int flameValue;

// Setup a DS18B20 oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our DS18B20 oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
// DS18B20 sur circuit imprimé DeviceAddress sensor0 = { 0x28, 0x8F, 0x47, 0x21, 0xC, 0x32, 0x20, 0xB3 };
DeviceAddress sensor1 = { 0x28, 0xFF, 0x64, 0x1F, 0x5F, 0x9C, 0x42, 0xB4 };
DeviceAddress sensor2 = { 0x28, 0xFF, 0x64, 0x1F, 0x5F, 0x9D, 0xB5, 0x87 };
DeviceAddress sensor3 = { 0x28, 0xFF, 0x64, 0x1F, 0x5F, 0x9D, 0x13, 0xF5 };
DeviceAddress sensor4 = { 0x28, 0xFF, 0x64, 0x1F, 0x5F, 0x80, 0xBC, 0x7E };

// Variables associated to the temperature sensor
float MotorTemperature;
float MotorCoolant;
float MotorExhaust;
float MotorSailDrive;


// (Replace this with whatever function you need to read whatever value you want
// to read from any other sensor you're using in your project.)
// Define the function that will be called every time we want
// an updated temperature value from the sensor. The sensor reads degrees
// Celsius, but all temps in Signal K are in Kelvin, so add 273.15.
// float read_temp_callback() {return (bme280.readTemperature() + 273.15); }
float read_temp_callbackT() {
  Serial.print("Tempetature = ");
  Serial.println(bme280.readTemperature()); 
  return (bme280.readTemperature() + 273.15); 
}


float read_temp_callbackP() {
  Serial.print("Pressure    = ");
  Serial.println(bme280.readPressure()); 
  return (bme280.readPressure()); 
}

float read_temp_callbackH() {
  Serial.print("Humidity    = ");
  Serial.println(bme280.readHumidity()); 
  return (bme280.readHumidity()); 
}

// call sensors.requestTemperatures() to issue a global temperature request to all DS18B20 devices on the bus (function called only for the first sensor used for all sensors)
  // After we got the all  temperatures, we can get the temperature from the each sensor owith sensors.getTempCByIndex(?????);
float read_temp_callbackPMT() {
  sensors.requestTemperatures(); 
  MotorTemperature = sensors.getTempC(sensor1);
  MotorCoolant = sensors.getTempC(sensor2);
  MotorExhaust = sensors.getTempC(sensor3);
  MotorSailDrive = sensors.getTempC(sensor4);

    // Check if reading senseur 1 was successful
  if(MotorTemperature != DEVICE_DISCONNECTED_C) 
  {
    Serial.print("Motor Temperature: ");
    Serial.println(MotorTemperature);
    // temperature in Kelvin
    return (MotorTemperature + 273.15);
  } 
 else
  {
    Serial.println("Moteur Temperature Error");
    return (273.15);
  }
}


float read_temp_callbackPMCT() {
      // Check if reading senseur 2 was successful
  if(MotorCoolant != DEVICE_DISCONNECTED_C) 
  {
    Serial.print("Coolant Temperature: ");
    Serial.println(MotorCoolant);
    // temperature in Kelvin
    return (MotorCoolant + 273.15);
  } 
 else
  {
    Serial.println("Coolant Temperature Error");
    // temperature of -273 in Kelvin to indicate an error
    return (273.15);
  }
}  
  
  
float read_temp_callbackPMET() {
  // Check if reading senseur 3 was successful
  if(MotorExhaust != DEVICE_DISCONNECTED_C) 
  {
    Serial.print("Exhaust Temperature: ");
    Serial.println(MotorExhaust);
    // temperature in Kelvin
    return (MotorExhaust + 273.15);
  } 
 else
  {
    Serial.println("Exhaust Temperature Error");
    //display.display();
    // temperature of -273 in Kelvin to indicate an error
    return (273.15);
  }
  
}
  

float read_temp_callbackPMTT() {
  
    // Check if reading senseur 4 was successful
  if(MotorSailDrive != DEVICE_DISCONNECTED_C) 
  {
    Serial.print("Saildrive Temperature: ");
    Serial.println(MotorSailDrive);
    // temperature in Kelvin
    return (MotorSailDrive + 273.15);
  } 
 else
  {
    Serial.println("Saildrive Temperature Error");
    // temperature of -273 in Kelvin to indicate an error
    return (273.15);
  }
}  


// processing of motor total run time 
unsigned int read_temp_callbackMTotRT() {
  // Send to signal K the counter in second????

  return (CounterMinute/60);
}  


// processing of motor run time and fire/smoke tetection
unsigned int read_temp_callbackMTripRTF() {

  // Increased by +5 as this routine is called every 5 seconds
  CounterSecond+=5;

  // si une minute est réalisée
  if(CounterSecond >= 60) 
  {
    // reset counter to 0
    CounterSecond = 0;
      // flash memory Rear for engine run time counter: reading of the counter from the flash
    CounterMinute = EngineRunTime.getUInt("CounterFlash", 0);
    // Increase minute counter
    CounterMinute++;
        // Increase minute counter Trip
    CounterMinuteTrip++;

    if(CounterMinuteTrip >= 60) {
      // Increase hour counter
      CounterHeureTrip++;
      //reset minute counter
      CounterMinuteTrip=0;
    }
    // Ecrire en flash le nouveau compteur de minute
    EngineRunTime.putUInt("CounterFlash", CounterMinute);

  } 
 else
  {


  }

    // Clear display
    display.clearDisplay();

    // Display Motor temperature
    display.setCursor(0,0);
    display.print("Motor  : ");
    display.setCursor(48,0);
    display.println(static_cast<unsigned int>(MotorTemperature));  

    // Motor Exhaust
    display.setCursor(0,13);
    display.print("Exhaust: ");
    display.setCursor(48,13);
    display.println(static_cast<unsigned int>(MotorExhaust));  

    // Display total run time
    Serial.print("Total time : ");
    Serial.println(CounterMinute*60+CounterSecond);
    display.setCursor(0,26);
    display.print("Trip: ");
    display.setCursor(31,26);
    display.print(CounterHeureTrip);  
    display.print(" :");
    display.print(CounterMinuteTrip);  
    
    // read Smoke concenttration value
    gasValue = analogRead(MQ2_PIN);
    // Display Smoke and Flame values
    Serial.print("Smoke - High number = Lot of smoke : ");
    Serial.println(gasValue);
    display.setCursor(0,39);
    display.print("Smok: ");
    display.setCursor(31,39);
    display.print(gasValue);  

    // e_paper
    display1.setFont(&FreeSerifBold9pt7b);
    // effacement zone "Moteur, Exhaust, trip fumée"
    display1.setPartialWindow(74, 0, 72, 96);
    display1.fillScreen(GxEPD_WHITE);
    display1.firstPage();
    do{
      display1.setCursor(85,20);
      display1.print(static_cast<unsigned int>(MotorTemperature));
      display1.setCursor(85,40);
      display1.print(static_cast<unsigned int>(MotorExhaust));
      display1.setCursor(85,60);
      display1.print(CounterHeureTrip);  
      display1.print(" h ");
      display1.print(CounterMinuteTrip);  
      display1.setCursor(85,80);      
      display1.print(gasValue);  
      }while(display1.nextPage());




    // Display motor coolant
    display.setCursor(66,0);
    display.print("Coolant: ");
    display.setCursor(115,0);
    display.println(static_cast<unsigned int>(MotorCoolant));  


    // Motor Saildrive
    display.setCursor(66,13);
    display.print("SailDrv: ");
    display.setCursor(115,13);
    display.println(static_cast<unsigned int>(MotorSailDrive));  


    // total hours
    display.setCursor(66,26);
    display.print("Tot: ");
    display.setCursor(90,26); 
    display.print(CounterMinute/60); 
    display.print(" h");


  // read Flame analog value
  flameValue = analogRead(AnalogFlamePin);
    Serial.print("Flame - High number = No flame : ");
    Serial.println(flameValue);
    display.setCursor(66,39);
    display.print("Flam: ");
    display.setCursor(100,39);
    display.print(flameValue);

    // e_paper
    display1.setFont(&FreeSerifBold9pt7b);
    // effacement zone "Moteur, Exhaust, trip fumée"
    display1.setPartialWindow(224, 0, 72, 96);
    display1.fillScreen(GxEPD_WHITE);
    display1.firstPage();
    do{
      display1.setCursor(225,20);
      display1.print(static_cast<unsigned int>(MotorCoolant));
      display1.setCursor(225,40);
      display1.print(static_cast<unsigned int>(MotorSailDrive));
      display1.setCursor(225,60);
      display1.print(CounterMinute/60);  
      display1.setCursor(225,80);      
      display1.print(flameValue);  
      }while(display1.nextPage());


    // Efface RPM value
    display1.setFont(&FreeSerifBold24pt7b);
    display1.setPartialWindow(140, 90, 154, 38);
    display1.fillScreen(GxEPD_WHITE);
    display1.firstPage();
    do{
       display1.setCursor(150,126);
       display1.print(millis());
       //display1.print(FDYinput);
    }while(display1.nextPage());


    // Si une flame ou des fumées sont detectés
    if ((flameValue < 400) || (gasValue > 450)) {
      // inverser la couleur de l'écran
      display.invertDisplay(true);
      // Activer le buzzer
      digitalWrite(Buzzer,HIGH);
    }
    else {
      //  couleur de l'écran normale
      display.invertDisplay(false);
      // Desctiver le buzzer
      digitalWrite(Buzzer,LOW);  
    }

    display.display();


  // Send to signal K the counter in second
  //return (CounterMinute*60+CounterSecond);
  return (CounterMinuteTrip);

}  

 /* float RMP_Recup;
auto RPM_Recup_function = [](float input) ->float {
     
      RPM_Recup = input;
       return (input);
     
};*/


// The setup function performs one-time application initialization.
void setup() {


  SetupLogging();

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app. Password: thisisfine
                    ->set_hostname("Cirrus-temperature")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("wifi-macha", "ElM@Ch@Wi2014!")
                    //->set_wifi("Livebox-9540", "RthXtYu3WCavVUnFyK")
                    //->set_wifi("Freebox-A61F8A", "Exton1Exton")
                    ->set_wifi("Cirrus_Wifi", "C1rrus_W")
                    //->set_sk_server("192.168.1.41", 3000)
                    //->set_sk_server("cirruspi.local", 3000)
                    ->set_sk_server("10.42.0.1", 3000)
                    ->get_app();

  // Set up monitor
  Serial.begin(115200);

  // demarrage des senseur DS18B20
  sensors.begin();

  // Set GPIO 15 in input mode and GPIO 4 in output
  pinMode(flamePin, INPUT); // may not be used
  pinMode (Buzzer, OUTPUT) ;


  Serial.println(F("test BNE280"));
  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  // bme.begin(); pour communiquer avec votre BME280 via l’adresse 0x76 (valeur par défaut)
  // bme.begin(0x76); pour faire la même chose que begin()
  // bme.begin(0x77); pour communiquer via cette seconde adresse i2c possible
  status = bme280.begin(0x76);  
  //§§§§if (!status) {
  //§§§§  Serial.println("Could not find a valid BME280 sensor, check wiring!");
  //§§§§  while (1);
  //§§§§}
  Serial.println("BME280 = OK");

  // flash memory R/W for engine run time counter: 
  // Name space within the "EngineRunTime" performance is: Counter  
  EngineRunTime.begin("CounterFlash",false);
  // flash memory R/W for engine run time counter: reading of the counter from the flash
  CounterMinute = EngineRunTime.getUInt("CounterFlash", 0); //§§§ to be made as a comment
  //§§§§CounterMinute = 1160*60; //§§§ to be made as code
  //§§§§EngineRunTime.putUInt("CounterFlash", CounterMinute); //§§§ to be made as code



  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.println(F("SSD1306 allocation OK ---------------------------------------------------------"));
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.display();
  delay(100);
  // Clear the buffer
  display.clearDisplay();
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...
  display.display();
  //display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextSize(1.7);
  display.setTextColor(WHITE);

///////////////////////////////////////////////////////////////////


display1.init();
display1.setTextColor(GxEPD_BLACK);
display1.setRotation(1);

display1.firstPage();


  do{
    display1.fillScreen(GxEPD_WHITE); // set the background to white (fill the buffer with value for white)
    display1.setFont(&FreeSerifBold9pt7b);

    // Display Motor temperature
    display1.setCursor(0,20);
    display1.print("Moteur  :");

    // Display motor coolant
    display1.setCursor(150,20);
    display1.print("Coolant : ");

    // Motor Exhaust
    display1.setCursor(0,40);
    display1.print("Exhaust : ");

    // Motor Saildrive
    display1.setCursor(150,40);
    display1.print("SailDrv : ");

    
    // Display total run time
    display1.setCursor(0,60);
    display1.print("Trip min:");

    // total hours
    display1.setCursor(150,60);
    display1.print("Total (h):");


    display1.setCursor(0,80);
    display1.print("Fumee   :");


    display1.setCursor(150,80);
    display1.print("Flamme: ");


    display1.setFont(&FreeSerifBold24pt7b);
    display1.setCursor(8,127);
    display1.print("RPM : ");
  }while (display1.nextPage());

////////////////////////////////////////////////////////////////////

  // Engine Run Time & Read the flame sensor every 5 second
  unsigned int read_inter_runtime_flame = 5000;
  // Engine Total time every 61 seconds
   unsigned int read_inter_total_runtime = 61000;
  // Read the BME sensor every 6,3 seconds
  unsigned int read_interval_BME = 6300;
    // Read the DS18B20 sensor every 6,7 seconds
  unsigned int read_interval_DS18B20 = 6800;

  // Create a RepeatSensor with float output that reads the temperature/pressure/humidity
  // using the function defined above.
  auto* BME_Inside_Temp =
      new RepeatSensor<float>(read_interval_BME, read_temp_callbackT);
  auto* BME_Inside_Press =
      new RepeatSensor<float>(read_interval_BME, read_temp_callbackP);
  auto* BME_Inside_Humid =
      new RepeatSensor<float>(read_interval_BME, read_temp_callbackH);
    auto* DS18B20_Propultion_Temp =
      new RepeatSensor<float>(read_interval_DS18B20, read_temp_callbackPMT);
    auto* DS18B20_Propultion_Coolant_Temp =
      new RepeatSensor<float>(read_interval_DS18B20, read_temp_callbackPMCT);  
    auto* DS18B20_Propultion_Exhaust_Temp =
      new RepeatSensor<float>(read_interval_DS18B20, read_temp_callbackPMET);
    auto* DS18B20_Propultion_Trans_Temp =
      new RepeatSensor<float>(read_interval_DS18B20, read_temp_callbackPMTT);
    auto* Engine_Trip_Runtime_Flame =
      new RepeatSensor<unsigned int>(read_inter_runtime_flame, read_temp_callbackMTripRTF);
     auto* Engine_Total_Runtime =
      new RepeatSensor<unsigned int>(read_inter_total_runtime, read_temp_callbackMTotRT);     

  // Set the Signal K Path for the output
  //const char* sk_path = "propulsion.engineRoom.temperature";
  const char* sk_pathT = "environment.inside.temperature";
  const char* sk_pathP = "environment.inside.pressure";
  const char* sk_pathH = "environment.inside.relativeHumidity";
  const char* sk_pathMotTemp = "propulsion.motor.temperature";
  const char* sk_pathMotCoolant = "propulsion.motor.coolantTemperature";
  const char* sk_pathMotExhaust = "propulsion.motor.exhaustTemperature";
  const char* sk_pathMotTransTemp = "propulsion.motor.transmission.oilTemperature";
  const char* sk_pathMotTotRunTime = "propulsion.motorTotal.runtime";
  const char* sk_pathMotTripRunTime = "propulsion.motorTrip.runtime";




  // ==>/vessels/<RegExp>/propulsion/<RegExp>/temperature
  // /vessels/<RegExp>/propulsion/<RegExp>/oilTemperature
  // ==>/vessels/<RegExp>/propulsion/<RegExp>/coolantTemperature
  // /vessels/<RegExp>/propulsion/<RegExp>/intakeManifoldTemperature
  // /vessels/<RegExp>/propulsion/<RegExp>/exhaustTemperature
  // /vessels/<RegExp>/propulsion/<RegExp>/runTime ==> used also for fire or smoke detection
  // /vessels/<RegExp>/propulsion/<RegExp>/transmission/oilTemperature
  // /vessels/<RegExp>/electrical/alternators/<RegExp>/temperature


  // Send the temperature to the Signal K server as a Float
  BME_Inside_Temp->connect_to(new SKOutputFloat(sk_pathT));
  BME_Inside_Press->connect_to(new SKOutputFloat(sk_pathP));
  BME_Inside_Humid->connect_to(new SKOutputFloat(sk_pathH));
  DS18B20_Propultion_Temp->connect_to(new SKOutputFloat(sk_pathMotTemp));
  DS18B20_Propultion_Coolant_Temp->connect_to(new SKOutputFloat(sk_pathMotCoolant));
  DS18B20_Propultion_Exhaust_Temp->connect_to(new SKOutputFloat(sk_pathMotExhaust));
  DS18B20_Propultion_Trans_Temp->connect_to(new SKOutputFloat(sk_pathMotTransTemp));
  Engine_Trip_Runtime_Flame->connect_to(new SKOutputFloat(sk_pathMotTripRunTime)); 
  Engine_Total_Runtime->connect_to(new SKOutputFloat(sk_pathMotTotRunTime));


//RPM Application/////

  const char* config_path_calibrate = "/Engine RPM/calibrate";
  const char* config_path_skpath = "/Engine RPM/sk_path";
  const float multiplier = 1.0;

 // gitalInputCounter counts interrupts and reports the count every read_delay ms.
 // @param pin The GPIO pin to which the device is connected
 // @param pin_mode Will be INPUT or INPUT_PULLUP
 // @param interrupt_type Will be RISING, FALLING, or CHANGE
 // @param read_delay How often you want to read the pin, in ms
  //auto* sensorRPM = new DigitalInputCounter(3, INPUT_PULLUP, RISING, 1000);
  // Entrée normal sans pull up
  auto* sensorRPM = new DigitalInputCounter(16, INPUT, RISING, 2000);



  sensorRPM->connect_to(new Frequency(multiplier, config_path_calibrate))  
  // connect the output of sensor to the input of Frequency()
        // Le 1.0 de MovingAverage est le coeff multiplicateur our diviseur de la frequence
         ->connect_to(new MovingAverage(2, 1.0,"/Engine RPM/movingAVG"))
         ->connect_to(new SKOutputFloat("propulsion.engine.revolutions", config_path_skpath));  
          // connect the output of Frequency() to a Signal K Output as a number


 /* auto RPM_Recup_transform = new LambdaTransform<float, float>(RPM_Recup_function);
         ->connect_to(RPM_Recup_transform)*/

}

void loop() { app.tick(); }
