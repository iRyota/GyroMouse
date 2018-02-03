#include <Mouse.h>
#include <Keyboard.h>

#include <MPU9250.h>
#include <quaternionFilters.h>

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging
#define BAUDRATE 38400

// Pin definitions
const int mouseClickPin = 7; // digital
const int xButtonPin = 6; // digital
const int neutralButtonPin = 12; // digital
const int VERT = 0; // analog
const int HORIZ = 1; // analog
const int SEL = 8; // digital
//int myLed  = 13;  // Set up pin 13 led for toggling

const int threshold = 300;

int mouseClickState = 0;
int xButtonState = 0;
int zButtonState = 0;
int leftButtonState = 0;
int RightButtonState = 0;
int UpButtonState = 0;
int DownButtonState = 0;

MPU9250 myIMU;

//SoftwareSerial btSerial(BT_RX, BT_TX);

float mouse_delta_x;
float mouse_delta_y;

void setup()
{
  Wire.begin();
  Serial.begin(BAUDRATE);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
//    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    // initialize MPU9250
    myIMU.initMPU9250();

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  Mouse.begin();
  Keyboard.begin();

  pinMode(mouseClickPin, INPUT);
  pinMode(xButtonPin, INPUT);
  pinMode(neutralButtonPin, INPUT);

  // make the SEL line an input
  pinMode(SEL,INPUT);
  // turn on the pull-up resistor for the SEL line (see http://arduino.cc/en/Tutorial/DigitalPins)
  digitalWrite(SEL,HIGH);

}

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes;
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes;
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes;

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();


  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  int vertical, horizontal, select;

  myIMU.delt_t = millis() - myIMU.count;
  if (myIMU.delt_t > 16) {
    vertical = analogRead(VERT)-337;
    horizontal = analogRead(HORIZ)-306;
    select = digitalRead(SEL); // will be HIGH (1) if not pressed, and LOW (0) if pressed

    myIMU.count = millis();

    mouse_delta_y = (myIMU.gz)/3;
    mouse_delta_x = -(myIMU.gy)/3;

    if (!digitalRead(neutralButtonPin)) {
      Mouse.move(mouse_delta_x, mouse_delta_y);
    }
    if (abs(vertical) < 20) {
      Keyboard.release(KEY_UP_ARROW);
      Keyboard.release(KEY_DOWN_ARROW);
      Keyboard.release(KEY_LEFT_SHIFT);
    } else if (abs(vertical) < threshold) {
      Keyboard.release(KEY_LEFT_SHIFT);
      if (vertical < 0) {
        Keyboard.press(KEY_DOWN_ARROW);
        Keyboard.release(KEY_UP_ARROW);
      } else {
        Keyboard.press(KEY_UP_ARROW);
        Keyboard.release(KEY_DOWN_ARROW);
      }
    } else {
      Keyboard.press(KEY_LEFT_SHIFT);
      if (vertical < 0) {
        Keyboard.press(KEY_DOWN_ARROW);
        Keyboard.release(KEY_UP_ARROW);
      } else {
        Keyboard.press(KEY_UP_ARROW);
        Keyboard.release(KEY_DOWN_ARROW);
      }
    }
    if (abs(horizontal) < 20) {
      Keyboard.release(KEY_LEFT_ARROW);
      Keyboard.release(KEY_RIGHT_ARROW);
      Keyboard.release(KEY_LEFT_SHIFT);
    } else if (abs(horizontal) < threshold) {
      Keyboard.release(KEY_LEFT_SHIFT);
      if (horizontal > 0) {
        Keyboard.press(KEY_LEFT_ARROW);
        Keyboard.release(KEY_RIGHT_ARROW);
      } else {
        Keyboard.press(KEY_RIGHT_ARROW);
        Keyboard.release(KEY_LEFT_ARROW);
      }
    } else {
      Keyboard.press(KEY_LEFT_SHIFT);
      if (horizontal > 0) {
        Keyboard.press(KEY_LEFT_ARROW);
        Keyboard.release(KEY_RIGHT_ARROW);
      } else {
        Keyboard.press(KEY_RIGHT_ARROW);
        Keyboard.release(KEY_LEFT_ARROW);
      }
    }
  }

  if (digitalRead(mouseClickPin)) {
    Mouse.press();
    mouseClickState = 1;
  } else {
    if (mouseClickState == 1) {
      Mouse.release();
      mouseClickState = 0;
    }
  }
  if (digitalRead(xButtonPin)) {
    Keyboard.press('x');
    xButtonState = 1;
  } else {
    if (xButtonState == 1) {
      xButtonState = 0;
      Keyboard.release('x');
    }
  }
  if (!digitalRead(SEL)) {
    Keyboard.press('z');
    zButtonState = 1;
  } else {
    if (zButtonState == 1) {
      zButtonState = 0;
      Keyboard.release('z');
    }
  }

}
