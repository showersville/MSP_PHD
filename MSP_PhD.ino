/*
 Name:		Multiwiii_rasperry_pico.ino
 Created:	3/10/2023 9:52:08 PM
 Author:	Inyeni
*/
#include <Arduino.h>
#include <SoftwareSerial.h>

#define MSP_ATTITUDE 108
#define MSP_STATUS 110
#define MSP_SET_MOTOR 214
float droneData[4] = {1, 1, 1, 1}; //roll, pitch, yaw, battery
byte databuf[28];
char sz0[28] = { ' ' };

float  val0 = 0.0; 
int val0_int = 0; 
float val0_float = 0.0;
int val0_fra = 0;

float  val1 = 0.0; 
int val1_int = 0; 
float val1_float = 0.0;
int val1_fra = 0;

float  val2 = 0.0;  // test value
int val2_int = 0;   // compute the integer part of the float
float val2_float = 0.0;
int val2_fra = 0;

float  val3 = 0.0;  // test value
int val3_int = 0;   // compute the integer part of the float
float val3_float = 0.0;
int val3_fra = 0;
SoftwareSerial mspSerial(1, 0); // RX TX
SoftwareSerial dataSerial(9, 8); // RX TX, sends MSP data to esp32
void setup() {
    mspSerial.begin(115200);
    Serial.begin(115200);
    dataSerial.begin(115200);
}

void loop() {
    uint8_t datad = 0;
    uint8_t* data = &datad;

    sendMSP(MSP_ATTITUDE, data, 0);
    getAttitudeData();
    delay(20);
    //Serial.println("here");
    sendMSP(MSP_STATUS, data, 0);
    getStatusData();
    delay(20);
}

void sendMSP(uint8_t cmd, uint8_t* data, uint8_t n_bytes) {

    uint8_t checksum = 0;

    mspSerial.write((byte*)"$M<", 3);
    mspSerial.write(n_bytes);
    checksum ^= n_bytes;

    mspSerial.write(cmd);
    checksum ^= cmd;

    mspSerial.write(checksum);
}
void getAttitudeData() {
    delay(40);

    byte count = 0;

    int16_t roll;
    int16_t pitch;
    int16_t yaw;

    while (mspSerial.available()) {
        count += 1;
        byte c = mspSerial.read();
        switch (count) {
        case 6:
            roll = c;
            break;
        case 7:
            roll <<= 8;
            roll += c;
            roll = (roll & 0xFF00) >> 8 | (roll & 0x00FF) << 8; // Reverse the order of bytes
            break;
        case 8:
            pitch += c;
            break;
        case 9:
            pitch <<= 8;
            pitch += c;
            pitch = (pitch & 0xFF00) >> 8 | (pitch & 0x00FF) << 8; // Reverse the order of bytes
            break;
        case 10:
            yaw += c;
            break;
        case 11:
            yaw <<= 8;
            yaw += c;
            yaw = (yaw & 0xFF00) >> 8 | (yaw & 0x00FF) << 8; // Reverse the order of bytes
            break;
        }
    }

    //Serial.print("Roll: " + String(roll / 10.0));
    //Serial.print(" Pitch: " + String(pitch / 10.0));
    //Serial.println(" Yaw: " + String(yaw));
    droneData[0] = roll / 10.0;
    droneData[1] = pitch / 10.0;
    droneData[2] = yaw;
}
void getStatusData() {
     delay(40);

    byte count = 0;

    int16_t batteryVoltage;
    int16_t pitch1;
    int16_t yaw1;

    while (mspSerial.available()) {
        count += 1;
        byte c = mspSerial.read();
        switch (count) {
        case 6:
            batteryVoltage = c;
            break;
        case 7:
            batteryVoltage <<= 8;
            //roll += c;
            batteryVoltage = (batteryVoltage & 0xFF00) >> 8 | (batteryVoltage & 0x00FF) << 8; // Reverse the order of bytes
            break;
        }
    }

    //Serial.println("Voltage: " + String(batteryVoltage / 10.0));
    droneData[3] = batteryVoltage / 10.0;


    //Serial.println("droneData " + droneDataString );
    //Serial.print("Roll: " + String(droneData[0]));
    //Serial.print(" Pitch: " + String(droneData[1]));
    //Serial.print(" Yaw: " + String(droneData[2]));
   // Serial.println(" Voltage: " + String(droneData[3]));
   
    val0 = droneData[0];  // test value
    int val0_int = (int)val0;   // compute the integer part of the float
    float val0_float = (abs(val0) - abs(val0_int)) * 10;
    int val0_fra = (int)val0_float;

    val1 = droneData[1];  // test value
    val1_int = (int)val1;   // compute the integer part of the float
    val1_float = (abs(val1) - abs(val1_int)) * 10;
    val1_fra = (int)val1_float;

    val2 = droneData[2];  // test value
    val2_int = (int)val2;   // compute the integer part of the float
    val2_float = (abs(val2) - abs(val2_int)) * 10;
    val2_fra = (int)val2_float;

    val3 = droneData[3];  // test value
    val3_int = (int)val3;   // compute the integer part of the float
    val3_float = (abs(val3) - abs(val3_int)) * 10;
    val3_fra = (int)val3_float;
    
    sprintf(sz0, "%d.%d,%d.%d,%d.%d,%d.%d,i",val0_int, val0_fra,val1_int, val1_fra, val2_int, val2_fra, val3_int, val3_fra);
    Serial.println(sz0);
    //Serial.println("here");


    if (dataSerial.available() > 0){
      String mssgs = dataSerial.readStringUntil('&');
     // Serial.println(mssgs);
    }
    dataSerial.write(sz0);
   
   // memset(buffr, 0, 24);

}
