#include "SFE_BMP180.h" // Basınç Sensör kütüphanesi
#include <Wire.h> // I2C kütüphanesi
#include "MPU9250.h"
#include <Servo.h>
#include "MS5837.h"

MPU9250 mpu;
SFE_BMP180 pressure;
MS5837 bar30;

Servo motorLeft;
Servo motorRight;
Servo motorFront;
Servo motorBack;

unsigned long timeCommunication;

int Roll;
int Pitch;
int Yaw;
int Humidity;
int Pressure;
int Temperature;
int ExternalPressure;
int ExternalTemperature;
int Depth;
int Altitude;

int A;
int B;
int C;
int D;

void setup() {
  pinMode(A0,INPUT);

  motorLeft.attach(11,1000,2000);
  motorRight.attach(10,1000,2000);
  
  motorFront.attach(13,1000,2000);
  motorBack.attach(12,1000,2000);
  
  Serial.begin(9600);
  Serial.println("Çalışıyor");
    
  Wire.begin();
  delay(500);
  
  if(pressure.begin())
    Serial.println("BMP Bağlandı");
  delay(500);

  if (mpu.setup(0x68)) 
    Serial.println("MPU Bağlandı");

 
  delay(500);
  bar30.setModel(MS5837::MS5837_30BA);
  bar30.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater) 
  bar30.init();
  timeCommunication = millis();
  armMotors();
}

void loop() {
 
  
  if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
          Roll = mpu.getRoll();
          Pitch = mpu.getPitch();
          Yaw = mpu.getYaw();
          prev_ms = millis();
      }
  }

    if(millis() - timeCommunication > 1000)
  {
    getBMP();
    getHumidity();
    getbar30();
    postData();
    timeCommunication = millis();
  }
  
  writeMotors();
}

void writeMotors(){
  
  if(Serial.available()){
    String s = Serial.readStringUntil('#'); 

    int pos[3];
    int curPos = 0;
    for(int i=0; i< s.length(); i++){
      if(s[i] == ','){
        pos[curPos] = i;
        curPos++;
      }
    }


    A = s.substring(0,pos[0]).toInt();
    B = s.substring(pos[0]+1,pos[1]).toInt();
    C = s.substring(pos[1]+1,pos[2]).toInt();
    D = s.substring(pos[2]+1,pos[3]).toInt();
    
    motorLeft.writeMicroseconds(1500+A*50);
    motorRight.writeMicroseconds(1500+B*50);
    motorBack.writeMicroseconds(1500+C*50);
    motorFront.writeMicroseconds(1450+D*50);
  }
}

void armMotors(){

  for(int i=1450; i<=1550; i++){
    motorLeft.writeMicroseconds(i);
    motorRight.writeMicroseconds(i);
    motorBack.writeMicroseconds(i);
  }
  for(int i=1400; i<1500;i++){
     motorFront.writeMicroseconds(i);
    }
  
}

void getBMP(){
  char status;
  double T,P,p0,a;
  status = pressure.startTemperature();//Bir sıcaklık ölçümü başlatılır
  if (status != 0) //
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) 
    {
      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P,T); //Tamamlanan basınç ölçümü Al :
        if (status != 0) //sıfıra eşit değilse
        {
          Pressure = P;
          Temperature = T;
        }
      else Serial.println("Basınç ölçümünde hata alındı\n");
      }
    else Serial.println("Basınç Ölçümü başlatılamadı\n");
    }
    else Serial.println("Sıcaklık değeri alınamadı\n");
  }
  else Serial.println("Sıcaklık ölçümü başlatılamadı\n");
}
void getbar30()
{
  bar30.read();
  ExternalPressure=bar30.pressure();
  ExternalTemperature=bar30.temperature();
  Depth=bar30.depth(); 
  
}

void getHumidity(){
  Humidity = analogRead(A0);
}

void postData(){
  char buffer[256];
  int n = sprintf(buffer,"$START{\"Roll\":%d,\"Pitch\":%d,\"Yaw\":%d,\"Temperature\":%d,\"Pressure\":%d,\"Humidity\":%d,\"ExternalPressure\":%d,\"ExternalTemperature\":%d,\"Depth\":%d}$END\n",Roll, Pitch, Yaw, Temperature, Pressure, Humidity,ExternalPressure,ExternalTemperature,Depth);
  Serial.write(buffer,n);
}
