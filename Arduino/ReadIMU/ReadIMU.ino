
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <SoftwareSerial.h>
#include <I2C.h>

#define SCL_PORT PORTC
#define SDA_PORT PORTC
#define SCL_PIN 5        //std SCL pin
#define SDA_PIN 4        //std SDA pin


SoftwareSerial mySerial(2, 3); // RX, TX

LSM9DS1 imu;
#define LSM9DS1_M_CS  10 // Can be any digital pin
#define LSM9DS1_AG_CS 9  // Can be any other digital pin

#define PRINT_CALCULATED
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
 
  if (imu.beginSPI(LSM9DS1_AG_CS, LSM9DS1_M_CS) == false) // note, we need to sent this our CS pins (defined above)
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1);
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
  
    I2c.write(0x10,'D');  //take single measurement
    I2c.read(0x10,2);     // request 2 bytes from tinyLiDAR
    i = I2c.receive();        // receive MSB byte
    i = i<<8 | I2c.receive(); // receive LSB byte and put them together
    Serial.print("D: ");
    Serial.println(i);        // print distance in mm 
    
    printed_gps = false;
    finished_gps_read = false;
    
    if (mySerial.available()) {
      c = mySerial.read();
      Serial.println(c);
      if (c == '$') {
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
          if (msg == "GPGGA") {
            correct_msg = true;
          }
        }
        msg_index++;
      }
    }
    if (!printed_gps)
      Serial.println("L: , ");

    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    
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
  imu.readGyro();
  
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
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
  imu.readAccel();

  Serial.print("A: ");
#ifdef PRINT_CALCULATED
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
  imu.readMag();
  
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
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
  Serial.print("R: ");
  Serial.println(pitch, 2);
  Serial.print("H: ");
  Serial.println(heading, 2);
}
