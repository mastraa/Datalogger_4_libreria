/*
##########################################################
DATALOGGER_3_LIBRERIA.INO - ProjectR3 - Università di Padova
##########################################################
       Andrea     Mastrangelo
Ing.   Cristiano  Battisti
Prof.  Carlo      Bettanini
       Stefano    Pieretti
##########################################################
Thesis of Andrea Mastrangelo
Supervisors:
Prof. Carlo Bettanini (Università di Padova)
Ing.  Cristiano Battisti
Other employees;
Stefano Pieretti
##########################################################
Sketch of a prototype datalogger, Arduino based, for small
sailing race boats (SKIFF)
##########################################################
INTRUMENTS
EM-406a   --> GPS (G.top013)
DS18B20   --> Sensore Digitale Temperatura
TCN75A    --> Sensore Digitale Temperatura
MPU6050   --> Accelerometro 3, Giroscopio 3, Temperatura (non usato)
HCM5883L  --> Magnetometer
INA125P   --> Tensione Sartie (2x)
SD Shield -->
QC1602    --> LCD


CONNECTION
EM-406a   --> Serial1(Rx->19, Tx->18) [Rx su Rx e Tx su Tx]   5V
DS18B20   --> Digital pin 37                                  5V
TCN75A    --> SDL-21 SCA-20                                   5V
MPU6050   --> SDL-21 SCA-20                                 3.3V
HCM5883L  --> SDL-21 SCA-20                                 3.3V
INA12P    --> A0 (sinistra) - A1 (destra)                     5V
SD-Shield -->
LCD       --> SDL-21 SCA-20 or pin 41, 43, 45, 47, 49, 48
##########################################################

INSTRUCTION:
set the correct GPS, set baudrate for Serial_PC and Serial GPS if it isn't standard
Set frequence of interrupt, and the parameters of getFreeName function for SD
Set the level of AD0 MPU pin
Set numbers of gyro offset iteration for calibrating sensor

You can activate SD Saving, lcd and serial printing with button or serial (1-on, 0-off)

To set gps functionality use Serial1.println("instructions")

*/

//INSERT LIBRARIES
#include <Wire.h>
#include <OneWire.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS.h>
#include "nRF24L01.h"
#include "RF24.h"

//includere sempre per ultimi e in questo ordine
//#include <mastraSerial.h>
#include <Mille_MEGA.h>
#include "Const.h"


float acc[3];             //X, Y, Z accelerometer value (calibrated in G)
float offAcc[3];          //accelerometer offset parameters
float gainAcc[3][3];      //accelerometer calibration matrix

float gyr[3];             //X, Y, Z gyroscope value (calibrated in rad/s)
float offGyr[3];          //gyroscope offset parameters
float gainGyr[3][3];      //gyroscope calibration matrix

float mag[3];             //X, Y, Z gyroscope value (calibrated in rad/s)
float offMag[3];          //gyroscope offset parameters
float gainMag[3][3];      //gyroscope calibration matrix

float wind[2];            //speed, direction

unsigned int timer_counter;   //valore di partenza della variabile 'overflow'

boolean save = FALSE;
boolean fix = 0;
boolean nLoop = 1; //flag per il salvataggio dell'intestazione del file
boolean changeNameFlag = 0;

float tempDS = 0.0;
byte Wspeed, vale_1, vale_2;
float left, right;

byte cyclecounter = 0;

long delayTime;

//lascio qui la definizione per definire solo quella necessaria
//typedef struct Mvupc_t mvupc_t;
//mvupc_t mvupc;
typedef struct Mvup_t mvup_t;
mvup_t mvup;



//INITIALISATION
MPU6050 MPU(MPU6050_A0);
HMC5883L HMC;
//ATTINY WIND (TINY_ADD);
AHRSFILTER AHRS;
//LCD_I2C lcd(LCD_ADD, 16, 2); //indirizzo, colonne, righe
LCD_CLASSIC lcd(41, 43, 45, 47, 49, 48, 16, 2); //rs, en, d4-7, colonne, righe
SDCARD sd (SD_SELECT);
LED saveled (SAVELED);
GPS gps(1); //1: EM406a, 2: G.top013 //3: Ublox
//GPS gps(3, 1);//second value is frequence in hz
TEMP ds(DS_PIN);




void setup() {
//Starting libraries
  Wire.begin();
  Serial.begin(BAUD);
  Serial.println();
  lcd.start(); //initialize lcd (classic and i2c)
  ds.check();
  sd.init();
  MPU.setMPU(0x07, 0x04, 0x00, 0x00, 0x08, 0x00); //smprt_div, dlpf_conf, gyro_conf, acc_conf, pwr_mgmt_1, pwr_mgmt_2
  HMC.setHMC(0x00, 0x38, 0x20); //mode, conf_a, conf_b
  timer_counter = gps.begin(); //default 4800 baud Em406a, 9600 for G.top, write baudrate to modify
  gps.autoset();
  sd.setName(sd.getFreeName (NOMEFILE, 4, 1)); //set name with the first free index (1 for three numbers, 0 for two numbers)
  
  pinMode(SAVELED, OUTPUT);
  pinMode(SAVEBUTTON,INPUT);

  if (TELEMETRY){
    Serial3.begin(FPVBAUD);
    Serial3.println("calibration");
    Serial3.println(sd.publicName);
  }
  else{
    Serial.println("calibration");
    Serial.println(sd.publicName);
  }
  lcd.print("WAIT Keep MotLes");
  lcd.setCursor(0,1);
  lcd.print(sd.publicName);
  
  offValuesSetting(); //local function, will set offset and gain matrix, it calls MPU function

  //SETUP Timer1
  noInterrupts();    //Disabilita Interrupt
  //Calcolo il valore iniziale considerando un prescaler di 256
  if (SKIP_GPS){
      timer_counter=65536-16000000/256/2;
    }
  //Azzero i registri
  TCCR1A = 0;
  TCCR1B = 0;
  //Impostazioni dei registri
  TCNT1 = timer_counter;
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << TOIE1);
  interrupts(); //Enable Interrupt

  //Last thing to do in setup!
  AHRS.start(); //initialize time variable
  Serial.println("Everything OK, press button or send '1' to start save data and '0' to stop saving");// 0 to stop
  if (TELEMETRY) Serial3.println("Everything OK, now you can send command");// 0 to stop
  lcd.setCursor(0,0);
  lcd.print("OK ");
  lcd.print("Press Save Bt");
  mvup.tempDS = ds.getTemp();
}


ISR(TIMER1_OVF_vect){ //Interrupt function
  TCNT1 = timer_counter; //'azzera' il contatore
  if (digitalRead(SAVEBUTTON)) {
    if (millis()-delayTime>750){//evita i 'doppi-click'
      save= !save; //activate with button
      delayTime=millis();
      if(save) changeNameFlag = 1;
      }
    }
  if(Serial.available()){
    save = Serial.read()-48; //activate with serial (1-on, 2-off)
    }
  if(TELEMETRY && Serial3.available()){
    //save = Serial3.read()-48; //activate with serial (1-on, 2-off)
    leggiComando();//funzione locale
    }
  
  digitalWrite(SAVELED,save); //led-saveState
  if (save){

    //fix = gps.readGPS(&mvupc.vel, &mvupc.gradi, &mvupc.date, &mvupc.times, &mvupc.lat, &mvupc.lon); //gps reading function
    fix = gps.readGPS(&mvup.vel, &mvup.gradi, &mvup.date, &mvup.times, &mvup.lat, &mvup.lon); //gps reading function
    if (SKIP_GPS){
      fix = 1;
      }
  }  
}//end_ISR


void loop() {
  if (changeNameFlag){//cambio nome
    sd.setName(sd.getFreeName (NOMEFILE, 4, 1)); //set name with the first free index (1 for three numbers, 0 for two numbers)
    changeNameFlag = 0;
    if (TELEMETRY) Serial3.println(sd.publicName);
    Serial.println(sd.publicName);
    
    nLoop = 1;
  }
  
  //IMU request
  MPU.readMPU(offAcc, gainAcc, offGyr, gainGyr, acc, gyr); //get calibrated values
  //MPU.readMPU(acc, gyr); //get raw values
  HMC.readHMC(offMag, gainMag, mag);//get calibrated values values
  //HMC.readHMC(mag); //get raw values
  //AHRS.Filter(gyr, acc, mag, BETA, mvupc.attitude); //Filter
  AHRS.Filter(gyr, acc, mag, BETA, mvup.attitude); //Filter
  
  
  if (fix && !changeNameFlag){
    //serialAttitude(mvupc.attitude, nLoop); //Yaw Pitch Roll
    //serialAttitude(mvup.attitude, nLoop); //Yaw Pitch Roll
    //serialGPS(mvup.vel, mvup.gradi, mvup.date, mvup.times, mvup.lat, mvup.lon, nLoop);
    //Serial.println(mvup.tempDS);
    //serialGPS(mvupc.vel, mvupc.gradi, mvupc.date, mvupc.times, mvupc.lat, mvupc.lon, nLoop);
    if (TELEMETRY) printFPVMVUP(mvup, 5);
    else printMVUP(mvup);
    
  
    sd.openFile('w');//opening file in write mode
    if (nLoop){
      sd.printFile(sd.getName());sd.printFile(" - ");
      sd.printFile(mvup.date);sd.printFile(" - ");sd.printFile(mvup.times);
      sd.newLineFile();
      
      nLoop = 0;
    }
  
  
    //lettura temperatura
    if (cyclecounter==60){//1 aggiornamento di T al minuto circa
      mvup.tempDS = ds.getTemp();
      cyclecounter = 0;
    }

    //sd.printMVUPC(mvupc); //frase senza condizioni ambientali ed estensimetro
    sd.printMVUP(mvup); //stringa completa
    sd.closeFile(); //close file
  
    //lcd.Attitude(mvupc.attitude);
    //lcd.Attitude(mvup.attitude);
    lcd.AttitudeShort(mvup.attitude, sd.publicName);
  
    fix = 0;
    ++cyclecounter;
  
  }
  
  //WIND.readTiny(5, wind); //Read AtTiny85 wind station

}

void offValuesSetting(){
  
  /* calibrazione pre 26-6-2015
  offAcc[0] =  157.860561;
  offAcc[1] =  264.857188;
  offAcc[2] =  458.620122;
  
  gainAcc[0][0] =  0.992987 / 16384.;
  gainAcc[0][1] = -0.011437 / 16384.;
  gainAcc[0][2] =  0.066142 / 16384.;
  gainAcc[1][0] = -0.011437 / 16384.;
  gainAcc[1][1] =  1.000824 / 16384.;
  gainAcc[1][2] =  0.003092 / 16384.;
  gainAcc[2][0] =  0.066142 / 16384.;
  gainAcc[2][1] =  0.003092 / 16384.;
  gainAcc[2][2] =  0.984898 / 16384.;
  */
  
  //calibrazione 26-6-2015
  offAcc[0] =  365.218156;
  offAcc[1] =  267.962312;
  offAcc[2] =  409.114178;
  
  gainAcc[0][0] =  1.002878;
  gainAcc[0][1] =0.000546;
  gainAcc[0][2] =  -0.000873;
  gainAcc[1][0] = 0.000546;
  gainAcc[1][1] =  0.995752;
  gainAcc[1][2] =  0.000329;
  gainAcc[2][0] =  -0.000873;
  gainAcc[2][1] =  0.000329;
  gainAcc[2][2] =  0.974197;
  
  

  gainGyr[0][0] = DEGTORAD/131.0f;
  gainGyr[0][1] = 0.;
  gainGyr[0][2] = 0.;
  gainGyr[1][0] = 0.;
  gainGyr[1][1] = DEGTORAD/131.0f;
  gainGyr[1][2] = 0.;
  gainGyr[2][0] = 0.;
  gainGyr[2][1] = 0.;
  gainGyr[2][2] = DEGTORAD/131.0f;
  
 /* Calibrazione vecchia (pre-26/6/2015)
 offMag[0] =    52.369866;
  offMag[1] =  -112.225169;
  offMag[2] =   135.192094;
  gainMag[0][0] =  0.928968;
  gainMag[0][1] =  0.004310;
  gainMag[0][2] =  0.015526;
  gainMag[1][0] =  0.004310;
  gainMag[1][1] =  0.867471;
  gainMag[1][2] =  0.022721;
  gainMag[2][0] =  0.015526;
  gainMag[2][1] =  0.022721;
  gainMag[2][2] =  0.993729;
 */

//calibrazione 26-6-2015
offMag[0] =    68.380833;
  offMag[1] =  -102.902140;
  offMag[2] =   -29.077363;
  gainMag[0][0] =  1.128904;
  gainMag[0][1] =  -0.011211;
  gainMag[0][2] =  0.008574;
  gainMag[1][0] =  -0.011211;
  gainMag[1][1] =  1.072952;
  gainMag[1][2] =  -0.030651;
  gainMag[2][0] =  0.008574;
  gainMag[2][1] =  -0.030651;
  gainMag[2][2] =  1.208459;
 
  //Setting gyro offset matrix
  MPU.offsetGyr(GYRO_SAMPLES, offGyr);
}

void leggiComando(){
  byte command[LENGTH+1];
  byte error = 100;
  if(Serial3.available()){
    error = readCommand(command, LENGTH+1);
    if (!error){
      if(!command[0]){
        if(command[1]==5) save = 1;
        if(command[1]==6) save = 0;
      }
    }
    sendCommand(0, &error, sizeof(error)/sizeof(byte), '$', '\n');
  }
}


