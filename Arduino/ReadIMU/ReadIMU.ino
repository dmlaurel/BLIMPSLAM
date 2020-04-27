
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <SoftwareSerial.h>

#define SCL_PORT PORTC
#define SDA_PORT PORTC
#define SCL_PIN 5        //std SCL pin
#define SDA_PIN 4        //std SDA pin
#include <I2C.h>

SoftwareSerial mySerial(2, 3); // RX, TX

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example SPI Setup //
///////////////////////
// Define the pins used for our SPI chip selects. We're
// using hardware SPI, so other signal pins are set in stone.
#define LSM9DS1_M_CS  10 // Can be any digital pin
#define LSM9DS1_AG_CS 9  // Can be any other digital pin

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 50 // 250 ms between prints

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

int inPin = 7;

void setup()
{

  pinMode(inPin, INPUT);
  Serial.begin(115200);
 
  // imu.beginSPI(), which verifies communication with the IMU
  // and turns it on.
  if (imu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) == false) // note, we need to sent this our CS pins (defined above)
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }

   mySerial.begin(9600);

   I2c.begin(); 
}

char c = 0;
String msg = "";
int msg_index = 0;
int msg_element_index = 0;
bool correct_msg = false;
bool finished_gps_read = false;
bool printed_gps = false;
uint16_t i;

void loop()
{
  if (digitalRead(inPin) == 1) {
    Serial.print("T: ");
    Serial.println(millis());
    //Serial.print("DIN: ");
    //Serial.println(digitalRead(inPin));
  
      I2c.write(0x10,'D');  //take single measurement
      I2c.read(0x10,2);     // request 2 bytes from tinyLiDAR
      i = I2c.receive();        // receive MSB byte
      i = i<<8 | I2c.receive(); // receive LSB byte and put them together
      Serial.print("D: ");
      Serial.println(i);        // print distance in mm 
      //delay(10);  
    printed_gps = false;
    finished_gps_read = false;
    //while(finished_gps_read == false) {
      if (mySerial.available()) {
        c = mySerial.read();
        Serial.println(c);
        if (c == '$') {
          //Print stuff
          //Serial.print("read: ");
          Serial.println(msg);
          if (correct_msg) {
            Serial.print("L: ");
            Serial.println(msg);
            printed_gps = true;
            finished_gps_read = true;
          }
          msg = "";
          msg_index = 0;
          msg_element_index = 0;
          correct_msg = false;
        } else {
          msg += c;
          
          if (msg_index == 4) {
            //Serial.println(msg);
            if (msg == "GPGGA") {
              correct_msg = true;
            }
          }
    
          msg_index++;
        }
      }
    if (!printed_gps)
      Serial.println("L: , ");
    //}
    //delay(10);
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's magnetometer x and y
    // axes are opposite to the accelerometer, so my and mx are
    // substituted for each other.
    printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
    Serial.println();
    
    
  } else {
    Serial.println("OFF");
  }
  delay(PRINT_SPEED);
}

void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();
  
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.println(imu.calcGyro(imu.gz), 2);
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();
  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.println(imu.calcAccel(imu.az), 2);
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag()
{
  // To read from the magnetometer, you must first call the
  // readMag() function. When this exits, it'll update the
  // mx, my, and mz variables with the most current data.
  imu.readMag();
  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.println(imu.calcMag(imu.mz), 2);
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 360.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("P: ");
  Serial.println(roll, 2);
  //Serial.print(", ");
  Serial.print("R: ");
  Serial.println(pitch, 2);
  Serial.print("H: ");
  Serial.println(heading, 2);
}
