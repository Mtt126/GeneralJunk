
#include "Ultrasonic.h"
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>


// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // 250 ms between prints

#define DECLINATION -12 // Declination (degrees) 

#define ULT_X_ECHO 8
#define ULT_X_TRIG 9
#define ULT_Y_ECHO 10
#define ULT_Y_TRIG 11
#define ULT_Z_ECHO 12
#define ULT_Z_TRIG 13


LSM9DS1 imu;
Ultrasonic ultrasonicX(ULT_X_TRIG, ULT_X_ECHO);
Ultrasonic ultrasonicY(ULT_Y_TRIG, ULT_Y_ECHO);
Ultrasonic ultrasonicZ(ULT_Z_TRIG, ULT_Z_ECHO);

int quad, color, balance;

void setup() 
{
  
  Serial.begin(9600);
  
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  balance = 0;

  if (!imu.begin())
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
}

void loop()
{
  printGyro();  // Print "G: gx, gy, gz"
  printAccel(); // Print "A: ax, ay, az"
  printMag();   // Print "M: mx, my, mz"
  
  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
  Serial.println();
  int c;
  int q;
  q = checkQuad();
  c = checkBelow();
  checkBalanced();
  Serial.println("Quad: ");
  Serial.print(q);
  Serial.println("Color: ");
  Serial.print(c);
  Serial.println("Balanced:");
  Serial.print(balance);
  if (c >0 && q >0 && balance ==1){
    
  transmit(q,c);
  }
  delay(PRINT_SPEED);
}

void printGyro()
{
  // To read from the gyroscope, you must first call the
  // readGyro() function. When this exits, it'll update the
  // gx, gy, and gz variables with the most current data.
  imu.readGyro();
  
  Serial.print("G: ");
#ifdef PRINT_CALCULATED

  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
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
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
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
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

void printAttitude(
float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}

void checkBalanced(){

  
  imu.readGyro();
  
  
  if (imu.calcGyro(imu.gx) < 10 || imu.calcGyro(imu.gx) > -10){
    
      Serial.println(" Not Tipped along the X");
      balance =1;
    
    }
  else if (imu.calcGyro(imu.gy) < 10 || imu.calcGyro(imu.gx) > -10){
    
      Serial.println("Not Tipped along the Y");
      balance = 1;
    
    }
  else if (imu.calcGyro(imu.gz) < 10 || imu.calcGyro(imu.gz) >-10){
    
      Serial.println("Not Tipped along the Z");
      balance = 1;
    
    }
    else {
      balance = 0;
      
      }
  
  }

int checkQuad(){
  int x, y;
  
  Serial.print(" X  ");
  Serial.print(ultrasonicX.Ranging(INC));
  Serial.println();
  Serial.print(" Y  ");
  Serial.print(ultrasonicY.Ranging(INC));
  Serial.println();
  x = ultrasonicX.Ranging(INC);
  y = ultrasonicY.Ranging(INC);

  //Need to compute the ranges for comparison and correct quads
  if (x < 60){
    
    if (y < 60){
      quad = 2;
      
      }else {
        quad = 3;
        
        }
    }else if (x > 60){
      if (y < 60){
      quad = 1;
      
      }else {
        quad = 4;
        
        }
      
      }
  
  
  return quad;
  }



int checkBelow(){

  int objectHeight;
  int color;
  
  Serial.print(" Z  ");
  Serial.print(ultrasonicZ.Ranging(INC));
  Serial.println();
  objectHeight = ultrasonicZ.Ranging(INC);

//Need to adjust for the height of the copter minus the object to determine what values to compare and read.


  

  if (+objectHeight < 96 && objectHeight > 78){
    // 8 to 6.5  None/WHITE
    color = 1;
  }else if (objectHeight > 60 && objectHeight < 78){
    // 6.5 to 5 RED
    color = 2;
  }else if (objectHeight > 42 && objectHeight < 60){
    //5 to 3.5  GREEN
    color = 3;
  }else if (objectHeight > 24 && objectHeight < 42){
    // 3.5 to 2 BLUE
    color = 4;
  }else {
    //sends color 0 just to send something for now
    color = 0;
    }
  return color;
}

void transmit(int q, int c){
  
  
  /* ***** The following 6 lines of code are IMPORTANT and must be executed in this EXACT order ***** */ 
  char a = 'E'; // Define character E to be used as indicator - see next line
  Serial.write(a); //FIRST: Send 'E' which indicates a new set of data is being transmitted
  // SECOND: Send the quadrant number as two bytes
  Serial.write(q/256); // returns INTEGER of division (e.g. 7/3 = 2)
  Serial.write(q%256); // returns the REMAINDER of quadrant number divided by 256 (e.g. 7%3 = 1)
  // THIRD: Send the color number as two bytes
  Serial.write(c/256);
  Serial.write(c%256);
  delay(750);
  
  
  }







