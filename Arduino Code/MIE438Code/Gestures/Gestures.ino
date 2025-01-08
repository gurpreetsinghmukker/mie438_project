 // Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int FlexSensorPin = A2;
int FlexSensor;

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // Serial.print("Accelerometer range set to: ");
  // switch (mpu.getAccelerometerRange()) {
  // case MPU6050_RANGE_2_G:
  //   Serial.println("+-2G");
  //   break;
  // case MPU6050_RANGE_4_G:
  //   Serial.println("+-4G");
  //   break;
  // case MPU6050_RANGE_8_G:
  //   Serial.println("+-8G");
  //   break;
  // case MPU6050_RANGE_16_G:
  //   Serial.println("+-16G");
  //   break;
  // }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // Serial.print("Gyro range set to: ");
  // switch (mpu.getGyroRange()) {
  // case MPU6050_RANGE_250_DEG:
  //   Serial.println("+- 250 deg/s");
  //   break;
  // case MPU6050_RANGE_500_DEG:
  //   Serial.println("+- 500 deg/s");
  //   break;
  // case MPU6050_RANGE_1000_DEG:
  //   Serial.println("+- 1000 deg/s");
  //   break;
  // case MPU6050_RANGE_2000_DEG:
  //   Serial.println("+- 2000 deg/s");
  //   break;
  // }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // Serial.print("Filter bandwidth set to: ");
  // switch (mpu.getFilterBandwidth()) {
  // case MPU6050_BAND_260_HZ:
  //   Serial.println("260 Hz");
  //   break;
  // case MPU6050_BAND_184_HZ:
  //   Serial.println("184 Hz");
  //   break;
  // case MPU6050_BAND_94_HZ:
  //   Serial.println("94 Hz");
  //   break;
  // case MPU6050_BAND_44_HZ:
  //   Serial.println("44 Hz");
  //   break;
  // case MPU6050_BAND_21_HZ:
  //   Serial.println("21 Hz");
  //   break;
  // case MPU6050_BAND_10_HZ:
  //   Serial.println("10 Hz");
  //   break;
  // case MPU6050_BAND_5_HZ:
  //   Serial.println("5 Hz");
  //   break;
  // }

  //Serial.println("");
  //delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  FlexSensor = analogRead(FlexSensorPin);

  /* Print out the values */
  /*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
  */

  Serial.println("X,Y,Z:");
  Serial.println(a.acceleration.x);
  Serial.println(a.acceleration.y);
  Serial.println(a.acceleration.z);
  Serial.print("FlexSensor:");
  Serial.println(FlexSensor);


  if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=7) && (a.acceleration.y<=11)) && ((a.acceleration.z>=-3) && (a.acceleration.z<=3)) && FlexSensor<3500)
  {
    Serial.println("+x");
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-11) && (a.acceleration.y<=-7)) && ((a.acceleration.z>=-3) && (a.acceleration.z<=3)) && FlexSensor<3500)
  {
    Serial.println("-x");
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=-11) && (a.acceleration.z<=-6)) && FlexSensor<3500)
  {
    Serial.println("+y");
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=7) && (a.acceleration.z<=11)) && FlexSensor<3500)
  {
    Serial.println("-y");
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=-11) && (a.acceleration.z<=-6)) && FlexSensor>3700)
  {
    Serial.println("+z");
  }
  else if (((a.acceleration.x>=-3) && (a.acceleration.x<=3)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=7) && (a.acceleration.z<=11))&& FlexSensor>3700)
  {
    Serial.println("-z");
  }
  else if (((a.acceleration.x>=7) && (a.acceleration.x<=11)) && ((a.acceleration.y>=-3) && (a.acceleration.y<=3)) && ((a.acceleration.z>=-3) && (a.acceleration.z<=3))&& FlexSensor>3700)
  {
    Serial.println("pick");
  }
  else
  {
    Serial.println("N");
  }
  delay(50);

/*
  if (a.acceleration.x>=0 && a.acceleration.y>=0)
  {
    Serial.println("+x");
  }
  else
  {
    Serial.println("-x");
  }
*/



}