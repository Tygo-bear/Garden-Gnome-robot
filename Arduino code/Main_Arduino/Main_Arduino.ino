/*  
 *   Tygo-bear
 *   Garden Gnome
 *   Motor control
 *   
 *   for communication over I2C or serial 
 *   Send    "255255" to set motors in idle
 *            \ /\ /
 *   Motor:    A  B  
 *   
 *   Write "000000" for full reverse (both motors).
 *   Write "510510" for full forwards (both motors).
 *   Write "255510" for motor A in neutral and motor B forward.
*/
#include <Wire.h> //I2C library on port A4 and A5 

#define SLAVE_ADDRESS 0x08 //Arduino slave addres

#define enA 6 //PWM motor A
#define in1 8 //control motor A
#define in2 9 //control motor A
#define enB 5 //PWM motor B
#define in3 7 //control motor B
#define in4 4 //control motor B

#define haA 2 //A motor feedback
#define haB 3 //B motor feedback

#define BatVoltage A3 //battery voltage

// Define Trig and Echo pin:
#define trigPin1 10
#define echoPin1 11
#define trigPin2 12
#define echoPin2 13

const float soundOffset = 331.3;
const float tempetureSoundRatio = 0.606;

int temperature = 20;
int afstand1 = -1;
int afstand2 = -1;

int Delta_T = 1;

String str_recieved_from_RPi = ""; //store Received data of I2C

int MotorAMulti = -1; //Motor A multiplier
int MotorBMulti = 1; //Motor B multiplier

//Motor speed
int motorSpeedA = 0;
int motorSpeedB = 0;

//target speed of motors
int targetSpeedA = 0;
int targetSpeedB = 0;

//Motor hall sensor feedback
int ticksA = 0;
int ticksB = 0;

int batterijSpanning = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(trigPin1, OUTPUT); 
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); 
  pinMode(echoPin2, INPUT),

  pinMode(BatVoltage, INPUT);

  attachInterrupt(digitalPinToInterrupt(haA),Apuls,RISING); 
  attachInterrupt(digitalPinToInterrupt(haB),Bpuls,RISING); 
  
  Serial.begin(115200);
  Serial.setTimeout(2);

  // I2C begin on addres
  Wire.begin(SLAVE_ADDRESS);
  // I2C events
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}

int berekenAfstand(int tijd)
{
  // Calculate speed of sound in m/s:
  float speedofsound = 331.3+(0.606*temperature);
  // Calculate the distance in cm:
  int distance = tijd*(speedofsound/10000)/2;

  return distance;
}

void updateUltrasoon()
{
  afstand1 = readUltrasoon(trigPin1, echoPin1); 
  afstand2 = readUltrasoon(trigPin2, echoPin2); 
}

int readUltrasoon(int trigerPin, int echoPin)
{
  digitalWrite(trigerPin, LOW);
  delayMicroseconds(5);// Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigerPin, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(trigerPin, LOW);// Read the echoPin. This returns the duration (length of the pulse) in microseconds:
  int duration = pulseIn(echoPin, HIGH);
  
  return berekenAfstand(duration);
}

String getLog()
{
  String dataToSend = "";
  dataToSend += "{";
  dataToSend += targetSpeedA;
  dataToSend += "/";
  dataToSend += targetSpeedB;
  dataToSend += "/";
  dataToSend += motorSpeedA;
  dataToSend += "/";
  dataToSend += motorSpeedB;
  dataToSend += "/";
  dataToSend += afstand1;
  dataToSend += "/";
  dataToSend += afstand2;
  dataToSend += "/";
  dataToSend += batterijSpanning;
  dataToSend += "}";

  return dataToSend;
}

void sendData()
{
  Serial.println("datarequest");
  String dataToSend = getLog();

  char byteString[30];
  dataToSend.toCharArray(byteString, 30);
  Serial.println("sending");
  Serial.println(dataToSend);
  
  Wire.write(byteString);
}

void receiveData(int byteCount) 
{
  str_recieved_from_RPi = ""; //flush cach

  //read full string received over I2C
  while ( Wire.available()) 
  {
    char tempC = (char)Wire.read();
    if(tempC != NULL)//filter NULL chars
    {
      str_recieved_from_RPi += tempC;
    }
  }

  if(str_recieved_from_RPi != "")
  {
    proccesCommand(str_recieved_from_RPi);
    Serial.println("execute data");
  }
  
  Serial.print("received data:");
  Serial.print(str_recieved_from_RPi);
  Serial.println(":");
 
}

void loop() 
{
  motorFeedback();
  updateUltrasoon();
  batVoltageReadOut();

  Serial.println(getLog());

  delay(100);
}

void batVoltageReadOut()
{
  batterijSpanning = analogRead(BatVoltage);
}

void motorFeedback()
{
  motorSpeedA = ticksA / Delta_T;
  motorSpeedB = ticksB / Delta_T;

  ticksA = 0;
  ticksB = 0;
}

void Apuls()
{
  ticksA += 1;
}

void Bpuls()
{
  ticksB += 1;
}

void serialEvent() 
{ 
  String s = Serial.readString();
  proccesCommand(s);
}

void proccesCommand(String s)
{
  Serial.print("proccesing: ");
  Serial.println(s);
    
  String s1 = "";
  String s2 = "";

  s1 = s.substring(0,3);
  s2 = s.substring(3,6);

  int a = s1.toInt();
  int b = s2.toInt();
  
  motorControl(a-255,b-255);
}

void motorControl(int a, int b)
{
  targetSpeedA = a;
  targetSpeedB = b;
  
  a = a * MotorAMulti;
  b = b * MotorBMulti;
  
  writeMotor(a,enA,in1,in2);
  writeMotor(b,enB,in3,in4); 

}

void writeMotor(int pwm, int pwmPin, int in1M, int in2M)
{
  int r = pwm;

  if(pwm < 0)
  {
    r = r * -1;

   // Set Motor backwards
   digitalWrite(in1M, HIGH);
   digitalWrite(in2M, LOW);
  }
  else
  {
   // Set Motor forwards
   digitalWrite(in1M, LOW);
   digitalWrite(in2M, HIGH);
  }

  if(r < 70) //motor underpower protection
  {
    r = 0;
  }

  analogWrite(pwmPin, r); //write pwn signal
}
