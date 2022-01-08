#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>


MPU6050 accelgyro;

#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))


int16_t ax, ay, az;
float accBiasX, accBiasY, accBiasZ;
float accAngleX, accAngleY;
double accPitch, accRoll;

int16_t gx, gy, gz;
float gyroBiasX, gyroBiasY, gyroBiasZ;
float gyroRateX, gyroRateY, gyroRateZ;
float gyroBias_oldX, gyroBias_oldY, gyroBias_oldZ;
float gyroPitch = 180;
float gyroRoll = -180;
float gyroYaw = 0;

uint32_t timer;

// input
double InputPitch, InputRoll;

// initial values
double InitialRoll;


//////////////////////////////////////////           ////////////////////////////////////

#include <Servo.h>
Servo SERVO;
int pos=0 ;



// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__HARDSERIAL

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_SERIAL Serial
#define REMOTEXY_SERIAL_SPEED 9600


/////////////////////////////////////////////////////////////

const int IN1 = 8;   
const int IN2 = 7;   
const int IN3 = 6;   
const int IN4 = 5;    
const int ENA  = 10;   
const int ENB  = 9;   



//////////////////////////////////////////////////////////////


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,5,0,0,0,51,0,8,25,0,
  1,0,67,14,12,12,6,31,94,0,
  1,0,78,25,12,12,6,31,62,0,
  1,0,55,25,12,12,6,31,60,0,
  1,0,67,38,12,12,6,31,86,0,
  3,3,13,19,8,22,6,24 };
  
// this structure defines all the variables of your control interface 
struct {

    // input variable
  uint8_t FORWARD; // =1 if button pressed, else =0 
  uint8_t RIGHT; // =1 if button pressed, else =0 
  uint8_t LEFT; // =1 if button pressed, else =0 
  uint8_t BACK; // =1 if button pressed, else =0 
  uint8_t ARM; // =0 if select position A, =1 if position B, =2 if position C, ... 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////



void setup() 
{
  RemoteXY_Init (); 
  
  
  SERVO.attach(11);
   

  /////////////////////////////////////////////////////////////

   Wire.begin();

  Serial.begin(9600);

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  delay(1500);

 
  
 //TODO: Better calibration 
  accelgyro.setXAccelOffset(2544);
  accelgyro.setYAccelOffset(-3904);
  accelgyro.setZAccelOffset(1628);
  accelgyro.setXGyroOffset(-106);
  accelgyro.setYGyroOffset(70);
  accelgyro.setZGyroOffset(26);

  gyroBiasX = 0;
  gyroBiasY = 0;
  gyroBiasZ = 0;

  accBiasX = 0;//4
  accBiasY = 0;//-4
  accBiasZ = 0;//16378

  //Get Starting Pitch and Roll
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accPitch = (atan2(-ax, -az) + PI) * RAD_TO_DEG;
  accRoll = (atan2(ay, -az) + PI) * RAD_TO_DEG;

  if (accPitch <= 360 & accPitch >= 180) {
    accPitch = accPitch - 360;
  }

  if (accRoll <= 360 & accRoll >= 180) {
    accRoll = accRoll - 360;
  }

  gyroPitch = accPitch;
  gyroRoll = accRoll;

  timer = micros();
  delay(1000);
  initializeValues ();


 
  /////////////////////////////////////////    /////////////////////////////////////////////////
//Initialise the Motor outpu pins
    pinMode (IN4, OUTPUT);
    pinMode (IN3, OUTPUT);
    pinMode (IN2, OUTPUT);
    pinMode (IN1, OUTPUT);
    pinMode (ENA, OUTPUT);
    pinMode (ENB, OUTPUT); 

       digitalWrite(IN1, HIGH);

  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);

}

//////////////////////////////////////////////////////////////////////////////


double Setpoint;


void initializeValues() {

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //////////////////////
  //  Accelerometer   //
  //////////////////////
  accPitch = (atan2(-ax/182.0, -az/182.0) + PI) * RAD_TO_DEG;
  accRoll = (atan2(ay/182.0, -az/182.0) + PI) * RAD_TO_DEG;

  if (accRoll <= 360 & accRoll >= 180) {
    accRoll = accRoll - 360;
  }

  //////////////////////
  //      GYRO        //
  //////////////////////

  gyroRateX = ((int)gx - gyroBiasX) * 131; 

  gyroPitch += gyroRateY * ((double)(micros() - timer) / 1000000);

  
  timer = micros();
  InitialRoll = accRoll;

  Setpoint = InitialRoll;
}

double filtered = 0;


//////////////////////////////////////////////////////////////////////////////////

void loop() 
{ 

  runEvery(10) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //////////////////////
    //  Accelerometer   //
    //////////////////////

    accRoll = (atan2(ay/182.0, -az/182.0) + PI) * RAD_TO_DEG;

    if (accRoll <= 360 & accRoll >= 180) {
      accRoll = accRoll - 360;
    }


    //////////////////////
    //      GYRO        //
    //////////////////////

    gyroRateX = -((int)gx - gyroBiasX) / 131; 


    double gyroVal = gyroRateX * ((double)(micros() - timer) / 1000000);

    timer = micros();

    //Complementary filter
    filtered = 0.98 * (filtered + gyroVal) + 0.02 * (accRoll);

    MotorControl(Compute(filtered - InitialRoll));

  

  ///////////////////////////////////////////////////////////////////////////////////
  RemoteXY_Handler ();
  
  if(RemoteXY.FORWARD==HIGH) {
    Forward();
    Serial.println("F");
  }
  else if(RemoteXY.BACK==1) {
    Reverse();
  }
  else if(RemoteXY.LEFT==1) {
    Left();
  }
  else if(RemoteXY.RIGHT==1) {
    Right();
  }
  else {
    Stop();
  }

delay(10);
  if(RemoteXY.ARM==0) {
    SERVO.write(0);
  }
  else if(RemoteXY.ARM==1) {
    
    for (pos = 0; pos <= 180; pos += 1) {
    SERVO.write(pos);             
    delay(15);                      
  }
  for (pos = 180; pos >= 0; pos -= 1) { 
    SERVO.write(pos);            
    delay(15);                     
  }
  
  }
  else if(RemoteXY.ARM==2) {
    
    for (pos = 0; pos <= 180; pos += 1) { 
    SERVO.write(pos);              
    delay(5);                       
  }
  for (pos = 180; pos >= 0; pos -= 1) { 
    SERVO.write(pos);             
    delay(5);                       
  }
  
  }
  
  
  ////////////////////////////////////////////////////////////////////////////////////

 
}
}

void Forward() //Code to rotate the wheel forward 
{
    digitalWrite(IN4,LOW);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN1,LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    Serial.println("F"); //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{
    digitalWrite(IN4,HIGH);
    digitalWrite(IN3,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN1,HIGH); 
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    Serial.println("R");
}

void Left() //Code to rotate the wheel Backward  
{
    digitalWrite(IN4,HIGH);
    digitalWrite(IN3,LOW);
    digitalWrite(IN2,HIGH);
    digitalWrite(IN1,LOW); 
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    Serial.println("L");
}


void Right() //Code to rotate the wheel Backward  
{
    digitalWrite(IN4,LOW);
    digitalWrite(IN3,HIGH);
    digitalWrite(IN2,LOW);
    digitalWrite(IN1,HIGH); 
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    Serial.println("R");
}

void Stop() //Code to stop both the wheels
{
    MotorControl(Compute(filtered - InitialRoll));
}
//////////////////////////////////////////////////////////////////////////////


void MotorControl(double out) {
  if (out < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.print(out);Serial.println("  balancing 1");
    
  } if (out > 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.print(out);Serial.println("  balancing 2");
   
  

  byte vel = abs(out);
  if (vel < 0)
    vel = 0;
  if (vel > 255)
    vel = 255;

  analogWrite( ENA, 255);
  analogWrite( ENB, 255);
  //Serial.println(vel);
}}

///////////////////////////////////////////////////////////////////////////////
int outMax = 255;
int outMin = -255;
float lastInput = 0;
double ITerm = 0;
double kp =14;
double ki =0.7;
double kd =6;

double Compute(double input)
{

  double error = Setpoint - input;

  ITerm += (ki * error);

  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
  double dInput = (input - lastInput);


  /*Compute PID Output*/
  double output = kp * error + ITerm + kd * dInput;

  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;

  /*Remember some variables for next time*/
  lastInput = input;
  return output;
}
