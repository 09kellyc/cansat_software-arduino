#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>  
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <FreqCounter.h>
#define HMC5883_WriteAddress 0x1E //  i.e 0x3C >> 1
#define HMC5883_ModeRegisterAddress 0x02
#define HMC5883_ContinuousModeCommand 0x00
#define HMC5883_DataOutputXMSBAddress  0x03
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
float flat, flon;
TinyGPS gps;
SoftwareSerial ss(9, 8);
const int chipSelect = 10;
int regb=0x01;
int regbdata=0x40;
int outputData[6];
long int frq;
const int pingPin = 7;
void setup(void) 
{
  Serial.begin(9600);
  ss.begin(4800);
  Wire.begin ();

  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   Serial.print("Initializing SD card...");
 //  make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
   //see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Time,Timer,Callsign,Pressure,Temperature,Altitude,Degrees,Humidity,Latitude,Longitude,DebrisRange,"); // so data can be directly imported into excel
}
void loop(void) 
{
  
  
  //bmp.... adafruit bmp085 connected to pins A4,A5, GND, 5V.
  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);
  float temperature; // Make a variable to hold temperature value
  bmp.getTemperature(&temperature); // Get the current temperature from BMP sensor and store it in temperature variable
  float seaLevelPressure = 1020; // Vaiable to hold current barometric pressure setting,
  
  
  //magnetometer soarkfun hmc5883 i2c sharing same pins as the Bmp085, make sure to include pull  up resistors.
    int i,x,y,z;
    double angle;
    Wire.beginTransmission(HMC5883_WriteAddress);
    Wire.write(regb);
    Wire.write(regbdata);
    Wire.endTransmission();
    Wire.beginTransmission(HMC5883_WriteAddress); //Initiate a transmission with HMC5883 (Write address).
    Wire.write(HMC5883_ModeRegisterAddress);       //Place the Mode Register Address in send-buffer.
    Wire.write(HMC5883_ContinuousModeCommand);     //Place the command for Continuous operation Mode in send-buffer.
    Wire.endTransmission();                       //Send the send-buffer to HMC5883 and end the I2C transmission.
    Wire.beginTransmission(HMC5883_WriteAddress);  //Initiate a transmission with HMC5883 (Write address).
    Wire.requestFrom(HMC5883_WriteAddress,6);      //Request 6 bytes of data from the address specified.
    //Read the value of magnetic components X,Y and Z
    if(6 <= Wire.available()) // If the number of bytes available for reading be <=6.
    {
     for(i=0; i<6;i++)
        {
     outputData[i]=Wire.read();  //Store the data in outputData buffer
        }
    }
    x=outputData[0] << 8 | outputData[1]; //Combine MSB and LSB of X Data output register
    z=outputData[2] << 8 | outputData[3]; //Combine MSB and LSB of Z Data output register
    y=outputData[4] << 8 | outputData[5]; //Combine MSB and LSB of Y Data output register
    angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
        
    //humidity hh10d sensor using the frequency counter library
    FreqCounter::f_comp= 8;             // Set compensation to 8
    FreqCounter::start(100);            // Start counting with gatetime of 100ms
    while (FreqCounter::f_ready == 0)         // wait until counter ready
      frq=FreqCounter::f_freq;        // read result  
    double humidity = ((7790-(frq*10))*393/4096)+16;
    
    //gps cooking hacks gps baud rate 4800 using the tiny gps library and the software serial to allow it to be used in conjunction with the apcc220 module
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (ss.available())
      {
        char c = ss.read();
        // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
        if (gps.encode(c)) // Did a new valid sentence come in?
          newData = true;
      }
    }

    if (newData)
    {
      gps.f_get_position(&flat, &flon);
    }

    //ultrasonic sensor maxbotix. using the arduino ping example
    // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  
    
    
    //PRINTING TO THE SERIAL MONITOR. so that it can be imported to excel over the wireless apcc220 modules
    Serial.print("DATA,TIME,TIMER,");
    Serial.print ("Candroid"); 
    Serial.print (",");
    Serial.print(event.pressure); // Write current pressure from BMP sensor to serial port
    Serial.print(",");  // Write comma to serial port for .csv file    
    Serial.print(temperature); // Write the current temperature to the serial port
    Serial.print(",");  // Write comma to serial port for .csv file
    Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature)); // Write the current pressure above sea level to serial port
    Serial.print(","); 
    Serial.print(angle,2);
    Serial.print (","); // Write comma to serial port for .csv file
    Serial.print (humidity);
    Serial.print (",");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6); 
    Serial.print(","); 
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6); 
    Serial.print(",");
    Serial.print (cm);
    Serial.println(",");
 // make a string for assembling the data to log to the sd card using the s.p.i library:
    String dataString = "";
    int alt = (bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature));
    int temp = (temperature);
    int pressure = (event.pressure);
    int deg = (angle,2); 
    int rh = (humidity);
    int dist = cm; 
    dataString += String(alt);
    dataString += ",";
    dataString += String (temp);
    dataString += ",";
    dataString += String (pressure);
    dataString += ",";
    dataString += String (rh);
    dataString += ",";
    dataString += String (dist);
    dataString += ",";
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}

  long microsecondsToInches(long microseconds)
{
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}




