
//Gyro - Arduino UNO R3
//VCC  -  5V
//GND  -  GND
//SDA  -  A4
//SCL  -  A5
//INT - port-2

#include "PID.h"
#include <Wire.h>
float sp=1.5;

#define speed_c1 6
#define mot1_a A3
#define mot1_b A2

#define speed_c2 5
#define mot2_a 7
#define mot2_b 4

PIDController pid_mpu;



//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

long loop_timer;
int temp;

void setup() {
  /*PID IMPLEMENTAtION*/
    
    pid_mpu.limMax = 110.0;
    pid_mpu.limMin = 0.0;
    pid_mpu.Kp = 11.5;
    pid_mpu.Kd = 0;
    pid_mpu.Ki =0.5;
    pid_mpu.tau = 0.02;
    pid_mpu.T = 0.01;

    pid_mpu.limMaxInt = 0.0;
    pid_mpu.limMinInt = 0.0;

    PIDController_Init(&pid_mpu);
    
  pinMode(speed_c1, OUTPUT);
  pinMode(mot1_a, OUTPUT);
  pinMode(mot1_b, OUTPUT);
  
  pinMode(speed_c2, OUTPUT);
  pinMode(mot2_a, OUTPUT);
  pinMode(mot2_b, OUTPUT);

  Wire.begin();                                                        //Start I2C as master
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {                 //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to have 250Hz for-loop
  }

  // divide by 1000 to get avarage offset
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

   
  Serial.begin(115200);
  loop_timer = micros();                                               //Reset the loop timer
}

void loop() {

  read_mpu_6050_data();
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += gyro_x * 0.0000611 ;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                   //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066 );               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066 );               //If the IMU has yawed transfer the pitch angle to the roll angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if (set_gyro_angles) {                                               //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else {                                                               //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  Serial.print(" pitch Angle  = "); Serial.print(angle_pitch_output);
  Serial.print("   roll Angle = "); Serial.print(angle_roll_output);
  
  PIDController_Update(&pid_mpu,sp,angle_roll_output);

  Serial.print("pid out : ");Serial.println(pid_mpu.out);
  
  control_motor();

/*
  while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();//Reset the loop timer*/

}


void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();
}


void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}

void control_motor() {

  if (angle_roll_output < sp) {

    analogWrite(speed_c1, pid_mpu.out);
    digitalWrite(mot1_a, LOW);
    digitalWrite(mot1_b, HIGH);

    analogWrite(speed_c2, pid_mpu.out);
    digitalWrite(mot2_a, HIGH);
    digitalWrite(mot2_b, LOW);
  }
  else if (angle_roll_output >sp) {
    
    analogWrite(speed_c1, pid_mpu.out);
    digitalWrite(mot1_a, HIGH);
    digitalWrite(mot1_b, LOW);

    analogWrite(speed_c2, pid_mpu.out);
    digitalWrite(mot2_a, LOW);
    digitalWrite(mot2_b, HIGH);
    
  } else {
    
    digitalWrite(mot1_a, LOW);
    digitalWrite(mot1_b, LOW);
    digitalWrite(mot2_b,LOW);
    digitalWrite(mot2_a,LOW);

  }
}
