#include <Wire.h>
#include <Kalman.h>

int32_t timer;

Kalman KalmanX;
Kalman KalmanY;
Kalman KalmanZ;

double KalAngleX;
double KalAngleY;
double KalAngleZ;

double gyroXangle;
double gyroYangle;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
int V = 1;     
const long interval = 1000;          
unsigned long TempoAtual = 0; 
unsigned long deltaTime = 0;
unsigned long TempoVelho = 0;

float V_0 = 5.0; 
float rho =1.1255; 
int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;
double Pitch=0;
double Roll=0;

double A=0;
double B=0;
bool avisoEstolLigado = false;
bool estolLigado=false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();

  digitalWrite(5,OUTPUT);
  digitalWrite(6,OUTPUT);
  digitalWrite(7,OUTPUT);
//---------------------------------------------
/* 1 - Leitura dos dados de Acc XYZ */
/* 2 - Organizar os dados de Acc XYZ */
recordAccelRegisters();
 /* 3 - Calculo de Pitch e Roll */ 
Pitch=returnAnguloPich();
Roll=returnAnguloRoll();
/* 4 - Inicialização do Filtro de Kalman XY */
KalmanY.setAngle(Pitch);
gyroYangle = Pitch;

KalmanX.setAngle(Roll);
gyroXangle = Roll;
timer = micros();
//---------------------------------------------------------------
  for (int ii=0;ii<offset_size;ii++){
      offset += analogRead(A0)-(1023/2);
  }
  offset /= offset_size; 
}

void loop() { 
//Manipulaçao com o MPU6050-----------------------------------
recordAccelRegisters();
recordGyroRegisters();

//Manipulaçao com o Airspeed-----------------------------------
  float adc_avg = 0; float veloc = 0.0;
  for (int ii=0;ii<veloc_mean_size;ii++){
    adc_avg+= analogRead(A0)-offset;
  }
  adc_avg/=veloc_mean_size;
  if (adc_avg>512-zero_span and adc_avg<512+zero_span){
  } else{
    if (adc_avg<512){
      veloc = -sqrt((-10000.0*((adc_avg/1023.0)-0.5))/rho);
    } else{
      veloc = sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
    }
  }


//Filtro de Kalman--------------------------------------------
  double dt = (double)(micros() - timer)/1000000;
  timer = micros();
  Pitch=returnAnguloPich();
  Roll=returnAnguloRoll();
  gyroYangle = funcaoRotY();
  gyroXangle = funcaoRotX();
  KalAngleY = KalmanY.getAngle(Pitch, gyroYangle, dt);
  KalAngleX = KalmanX.getAngle(Roll, gyroXangle, dt);

//-----------------------------------amostragem de dados-------
TempoAtual=millis();
deltaTime=TempoAtual- TempoVelho;
if(deltaTime>=interval){
  Serial.println("----------------------------------------");

  Serial.print("Angulo pitch: ");
  Serial.println(KalAngleY);
  Serial.print("Angulo roll: ");
  Serial.println(KalAngleX);
  Serial.print("Velocidade (m/s): ");
  Serial.println(veloc);
  Serial.println("  ");
  Serial.print("Velocidade B: ");
  Serial.println(returnB());
  Serial.print("Velocidade A: ");
  Serial.println(returnA());
  TempoVelho = millis();
  
//-----------------------------SITUAÇÃO DE ESTOL-----------------------------
            if ( veloc<returnA()){     
              Serial.print("SITUAÇÃO DE ESTOL ");
              if(estolLigado == false ){
                digitalWrite(5,HIGH);
                delay(500);
                digitalWrite(6,HIGH);
                delay(500);
                digitalWrite(6,LOW);
                estolLigado=true;
              }
              else{
                estolLigado=true;
                }
}


//------------------------------SITUAÇÃO DE PRE-ESTOL-----------------------------
            else if( KalAngleY>20 &&  veloc<returnB()){ 
                    Serial.print("SITUAÇÃO DE PRE ESTOL ");
                    
                    if(avisoEstolLigado ==false ){
                    digitalWrite(5,HIGH);
                    delay(500);
                    digitalWrite(7,HIGH);
                    delay(500);
                    digitalWrite(7,LOW);
                    avisoEstolLigado=true;
                  }
                  else{
                    avisoEstolLigado=true;
                    }
            } 

//-----------------------------SITUAÇÃO NORMAL-----------------------------
            else{
            digitalWrite(5,LOW);
            digitalWrite(6,LOW);
            digitalWrite(7,LOW);
            avisoEstolLigado = false;
            estolLigado = false;
            Serial.print("SITUAÇÃO NORMAL");
            }
} 
   
}//*//

//setupMPU-----------------------------------------
   
void setupMPU(){
  
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x6B); 
  Wire.write(0b00000000); 
  Wire.endTransmission(); 
   
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x1B); 
  Wire.write(0x00000000); 
  Wire.endTransmission(); 
  
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x1C); 
  Wire.write(0b00000000); 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x37); 
  Wire.write(0b0001001); 
  Wire.endTransmission(); 
  
  delay(100);
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x3B); 
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); 
                                
  while(Wire.available() < 6); 
  accelX = Wire.read()<<8|Wire.read(); 
  accelY = Wire.read()<<8|Wire.read(); 
  accelZ = Wire.read()<<8|Wire.read(); 
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); 
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); 
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); 
  gyroY = Wire.read()<<8|Wire.read(); 
  gyroZ = Wire.read()<<8|Wire.read(); 
  processGyroData();
}
double funcaoRotY() {
  recordGyroRegisters();
  return rotY;
}

double funcaoRotX() {
  recordGyroRegisters();
  return rotX;
}


void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
  
  Serial.print(" Aceleraçoes (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);  
}

  
double returnAnguloPich(){
  double pitch = atan(gForceZ/gForceX) * RAD_TO_DEG;
  if((90-pitch)<90){
    return(90-pitch);
    }
  else{
    return(-(180-(90-pitch)));
    }
}

double returnAnguloRoll(){
  double roll = atan(gForceZ/gForceY) * RAD_TO_DEG;
   if((90-roll)<90){
    return(90-roll);
    }
  else{
    return(-(180-(90-roll)));
    }
}
//-------------Return A e B
//-------------A=20 e B=22, D=20. obs: A=Vstall
//-------------Em curvas: A e B aumenta 40%

double returnA(){
   if(   abs(KalAngleX)>5){
    return (20*1.4);
    }
  else{
    return (20);
    }
}

double returnB(){
   if(   abs(KalAngleX)>5){
    return (22*1.4);
    }
  else{
    return (22);
    }
}
