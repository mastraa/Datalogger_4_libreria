//
//  Const.h
//  
//
//  Created by Andrea Mastrangelo on 02/04/15.
//
//

#ifndef _Const_h
#define _Const_h

#define DEGTORAD        0.017453293f
#define RADTODEG       57.295780f
#define BETA            0.05

#define MPU6050_A0          LOW   //MPU6050 device AD0 level
#define GYRO_SAMPLES        1000  //number of samples for gyro offset calibration
#define TINY_ADD            0x26  //AtTiny address
#define LCD_ADD             0x27
#define SD_SELECT              4
#define SAVEPIN               14//BLU
//#define SAVELED               15//VIOLA
#define BAUD              115200
#define FPVBAUD            57600
#define TELEMETRY              0
#define LENGTH                 1
#define CONNECTION_TIMEOUT    60000//millis
#define NOMEFILE            "vela000.txt"

#define SAVEBUTTON           12
#define SAVELED              13

#define DS_PIN               11

#define SKIP_GPS              0

#endif
