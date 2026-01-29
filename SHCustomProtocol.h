#ifndef __SHCUSTOMPROTOCOL_H__
#define __SHCUSTOMPROTOCOL_H__

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include "CRC8.h"

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

int speedL;
int shift;
String gear;
uint32_t time2 = 0;
int airbag_seatbelt_counter = 0;
int FuelRaw = 100;
int oiltemp = 0;
int CCcount = 0, checkengcount = 0, indicatorcount = 0;
int c_rpm = 1000, c_speed = 100;
int c_sport = 0;
int id = 0;
int c_sportPlus = 0;
int c_comfort = 0;
int c_ecopro = 0;
int c_ESCTC = 0;
int c_Drift = 0;
uint32_t blinkerstamp = 0, lastBrakeSend = 0;
unsigned long last2 = 0;
unsigned long last = 0;
uint32_t time = 0;
uint16_t lastSpeed = 0;

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const long interval = 1;
static uint8_t count = 0x00;
static uint8_t count1 = 0x00;


enum Headlights {
  FULLBEAM,
  SHORTBEAM,
  OFF
};
enum Gear {
  DRIVE,
  PARK,
  REVERSE,
  NEUTRAL,
  SEQUENTIAL,
  MANUAL
};
enum DriveMode {
  COMFORT,
  COMFORTPLUS,
  SPORT,
  SPORTPLUS,
  ECOPRO,
  ECOPROPLUS,
  COMFORTADAPTIVE,
  ESCTC,
  Drift
};

CRC8 crc8Calculator;

int Cluster = 3;
int VolUp = 4;
int VolDwn = 5;
int FrontPanel = 6;

int gearNumberCan;

int igncunt;
int counter4Bit;
uint8_t rpmCounter;
uint8_t gearSelectorMessageCounter;



class SHCustomProtocol {
private:
  int rpm = 0;
  int speed = 0;
  float fuel = 0.0f;
  bool indLeft = false;
  bool indRight = false;
  bool lightsOn = false;
  bool highBeam = false;
  bool handbrake = false;
  bool lowBeam = false;
  bool FL = false;
  bool FR = false;
  bool RR = false;
  bool RL = false;
  bool HD = false;
  bool BN = false;
  bool FTL = false;
  bool FTR = false;
  bool RTR = false;
  bool RTL = false;
  int ignitionStatus = 0;
  String timeStr = "";
  String ChkEngine;
  int eng = 0;
public:
  void setup() {

    crc8Calculator.begin();

    CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);


    CAN.setMode(MCP_ANY);

    //Buttons
    pinMode(Cluster, INPUT_PULLUP);
    pinMode(VolUp, INPUT_PULLUP);
    pinMode(VolDwn, INPUT_PULLUP);
    pinMode(FrontPanel, INPUT_PULLUP);
  }
  void CanSend2b(short address, byte a, byte b) {
    unsigned char DataToSend[2] = { a, b };
    CAN.sendMsgBuf(address, 0, 2, DataToSend);
  }

  void CanSend3B(short address, byte a, byte b, byte c) {
    unsigned char DataToSend[3] = { a, b, c };
    CAN.sendMsgBuf(address, 0, 3, DataToSend);
  }

  void CanSend4B(short address, byte a, byte b, byte c, byte d) {
    unsigned char DataToSend[4] = { a, b, c, d };
    CAN.sendMsgBuf(address, 0, 4, DataToSend);
  }

  void CanSend5B(short address, byte a, byte b, byte c, byte d, byte e) {
    unsigned char DataToSend[5] = { a, b, c, d, e };
    CAN.sendMsgBuf(address, 0, 5, DataToSend);
  }

  void CanSend6B(short address, byte a, byte b, byte c, byte d, byte e, byte f) {
    unsigned char DataToSend[6] = { a, b, c, d, e, f };
    CAN.sendMsgBuf(address, 0, 6, DataToSend);
  }

  void CanSend7B(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g) {
    unsigned char DataToSend[7] = { a, b, c, d, e, f, g };
    CAN.sendMsgBuf(address, 0, 7, DataToSend);
  }

  void CanSend(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
    unsigned char DataToSend[8] = { a, b, c, d, e, f, g, h };
    CAN.sendMsgBuf(address, 0, 8, DataToSend);
  }

  // here is where the shit starts:


  //Button Events

  void sendButtons() {
    if (digitalRead(Cluster) == LOW) {  //4
      CanSend2b(0x1EE, 0x40, 0xFF);     //64= odo,mpg,aveg,compass,blank
    } else {
      CanSend2b(0x1EE, 0x00, 0xFF);
    }
  }

  void sendNeccesaryCanId() {
    //CanSend(0x7c3, random(0,255), random(0,255), random(0,255), random(0,255), random(0,255), random(0,255), random(0,255), random(0,255));
    CanSend5B(0x349, 0xCA, 0x03, 0xF4, 0x01, 0xFF);

    //CanSend(0x287, 0x05, random(0,255), random(0,255), 0, 0x5c, 0x00,3, random(0,255));

    //airbag
    CanSend(0xD7, count, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    //SOS
    CanSend(0x2C3, random(0x00, 0x0C), 0x15, 0x0F, 0x00, 0x00, 0x70, 0xFF, 0xFF);
    //Byte 3: error

    //Vehicle Status
    CanSend(0x3A0, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFF, 0xF0, random(0xFC, 0xFD));
    CanSend(0x30b, count, 0x50, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x51, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x52, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x53, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x54, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x55, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x56, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x57, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x58, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x59, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x5A, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x5B, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x5C, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x5D, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x5E, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    CanSend(0x30b, count, 0x5F, count, 0xC8, 0xFF, 0xFF, 0xFF, 0xFF);
    //ABS 36e
    unsigned char abs1WithoutCRC[] = { 0xF0 | counter4Bit, 0xFE, 0xFF, 0x14 };
    unsigned char abs1WithCRC[] = { crc8Calculator.get_crc8(abs1WithoutCRC, 4, 0xD8), abs1WithoutCRC[0], abs1WithoutCRC[1], abs1WithoutCRC[2], abs1WithoutCRC[3] };
    CAN.sendMsgBuf(0x36E, 0, 5, abs1WithCRC);

    //SeatBelt
    CanSend(0x581, 0x40, 0x4D, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    //Byte 4: 0x29 seatbelt on, 0x28 off

    //Restraint system (airbag?)
    unsigned char restraintWithoutCRC[] = { 0x40 | counter4Bit, 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF };
    unsigned char restraintWithCRC[] = { crc8Calculator.get_crc8(restraintWithoutCRC, 7, 0xFF), restraintWithoutCRC[0], restraintWithoutCRC[1], restraintWithoutCRC[2], restraintWithoutCRC[3], restraintWithoutCRC[4], restraintWithoutCRC[5], restraintWithoutCRC[6] };
    CAN.sendMsgBuf(0xAB, 0, 8, restraintWithCRC);

    //Restraint system (seatbelt?)
    unsigned char restraint2WithoutCRC[] = { 0xE0 | counter4Bit, 0xF1, 0xF0, 0xF2, 0xF2, 0xFE };
    unsigned char restraint2WithCRC[] = { crc8Calculator.get_crc8(restraint2WithoutCRC, 6, 0x28), restraint2WithoutCRC[0], restraint2WithoutCRC[1], restraint2WithoutCRC[2], restraint2WithoutCRC[3], restraint2WithoutCRC[4], restraint2WithoutCRC[5] };
    CAN.sendMsgBuf(0x297, 0, 7, restraint2WithCRC);

    //Stability DSC
    unsigned char steeringColumnWithoutCRC[] = { 0xF0 | counter4Bit, 0xFE, 0x00, 0x14 };
    unsigned char steeringColumnWithCRC[] = { crc8Calculator.get_crc8(steeringColumnWithoutCRC, 4, 0x9E), steeringColumnWithoutCRC[0], steeringColumnWithoutCRC[1], steeringColumnWithoutCRC[2], steeringColumnWithoutCRC[3] };
    CAN.sendMsgBuf(0x2a7, 0, 5, steeringColumnWithCRC);

    //Steering column
    CanSend5B(0x294, 0x73, 0xC7, 0xFE, 0xFF, 0x14);
    CanSend5B(0x294, 0x3C, 0xCA, 0xFE, 0xFF, 0x14);
    CanSend5B(0x294, 0x55, 0xCC, 0xFE, 0xFF, 0x14);
    //Oil for IDrive
    CanSend2b(0x381, 0x79, 0xFF);
    //Byte 1: 0x19=low, 0x20=Ok, 0x79=Intilising, 0x00=?,
    //Rpm/gears/water/coolant
    unsigned char oilWithoutCRC[] = { 0x10 | counter4Bit, 0x82, 0x4E, 138, 153, 0x05, 0x89 };
    //Byte 4: coolant temp 138=90c
    //Byte 5: oil Temp 153=105c
    unsigned char oilWithCRC[] = { crc8Calculator.get_crc8(oilWithoutCRC, 7, 0xF1), oilWithoutCRC[0], oilWithoutCRC[1], oilWithoutCRC[2], oilWithoutCRC[3], oilWithoutCRC[4], oilWithoutCRC[5], oilWithoutCRC[6] };
    CAN.sendMsgBuf(0x3F9, 0, 8, oilWithCRC);

    //TPMS
    unsigned char TPMSWithoutCRC[] = { 0xF0 | counter4Bit, 0xA0, 0xA0, 0xA0 };
    unsigned char TPMSWithCRC[] = { crc8Calculator.get_crc8(TPMSWithoutCRC, 4, 0xC5), TPMSWithoutCRC[0], TPMSWithoutCRC[1], TPMSWithoutCRC[2], TPMSWithoutCRC[3] };
    CAN.sendMsgBuf(0x369, 0, 5, TPMSWithCRC);



    //Odo
    unsigned char mpg2WithoutCRC[] = { 0xF0 | counter4Bit, 0x00, 0x00, 0xF2 };
    unsigned char mpg2WithCRC[] = { crc8Calculator.get_crc8(mpg2WithoutCRC, 4, 0xde), mpg2WithoutCRC[0], mpg2WithoutCRC[1], mpg2WithoutCRC[2], mpg2WithoutCRC[3], mpg2WithoutCRC[4] };
    CAN.sendMsgBuf(0x2BB, 0, 5, mpg2WithCRC);
    //time,date,and language
    CanSend(0x291, 2, 18, 57, 0x00, 0x00, 0x00, 0x00, 0x00);


    //TPMS
    CanSend(0xB68, 0x00, count, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    //check control
  }

  void sendPark(){
  if(handbrake){
    unsigned char abs3WithoutCRC[] = { 0xF0|counter4Bit, 0x38, 0, 0x15 };
  unsigned char abs3WithCRC[] = { crc8Calculator.get_crc8(abs3WithoutCRC, 4, 0x17), abs3WithoutCRC[0], abs3WithoutCRC[1], abs3WithoutCRC[2], abs3WithoutCRC[3] };
  CAN.sendMsgBuf(0x36F, 0, 5, abs3WithCRC);
  //Byte 4: 0x15 P Brake on, 0x14 off
  }
  else{
    unsigned char abs3WithoutCRC[] = { 0xF0|counter4Bit, 0x38, 0, 0x14 };
    unsigned char abs3WithCRC[] = { crc8Calculator.get_crc8(abs3WithoutCRC, 4, 0x17), abs3WithoutCRC[0], abs3WithoutCRC[1], abs3WithoutCRC[2], abs3WithoutCRC[3] };
    CAN.sendMsgBuf(0x36F, 0, 5, abs3WithCRC);
    //Byte 4: 0x15 P Brake on, 0x14 off
  }
  //P Brake----F20 is 34F
  
  }

  void sendIgnition(int status) {
    if (status == 1) {
      CanSend(0x5c0, 0x40, 40, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
      //Ignition ON showing OFF on rpm
      CanSend(0x03c, 0xDA, 0x74, 0x03, 0x97, 0x11, 0x00, 0xE7, 0xFF);  // keep with 12F
      CanSend(0x03c, 0x87, 0x75, 0x03, 0x97, 0x11, 0x00, 0xE7, 0xFF);  //e7   */
        //wake up
      CanSend(0x510, 0x40, 0x10, 0x00, 0x02, 0x02, 0x12, 0x11, 0x00);

      //Wake up 2
      CanSend(0x12F, count, random(0x70, 0x7f), 0xFA, 0xDD, 0xFF, 0xFF, 0xFF, 0x01);
      //Byte 3: 0xfa=ON,
      //Byte 8: 0x01=Waked,
    } else if (status == 2) {
      CanSend(0x5c0, 0x40, 40, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
      //Ignition ON Engine running
      CanSend(0x03C, 0x92, 0x74, 0x97, 0x00, 0x11, 0x97, 0x0A, 0xFF);
      CanSend(0x03C, 0xCF, 0x75, 0x97, 0x00, 0x11, 0x97, 0x0A, 0xFF);  //0x0A
                                                                       //wake up
      CanSend(0x510, 0x40, 0x10, 0x00, 0x02, 0x02, 0x12, 0x11, 0x00);

      //Wake up 2
      CanSend(0x12F, count, random(0x70, 0x7f), 0xFA, 0xDD, 0xFF, 0xFF, 0xFF, 0x01);
      //Byte 3: 0xfa=ON,
      //Byte 8: 0x01=Waked,
    } else if (status == 0) {
      CanSend(0x03C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
      CanSend(0x12F, count, random(0x70, 0x7f), 0x01, 0xDD, 0xFF, 0xFF, 0xFF, 0x01);
    }
  }

  void sendHeadlights(Headlights status) {
    if (status == FULLBEAM) {
      CanSend3B(0x21A, 0x02, 0xC0, 0xF7);
    } else if (status == SHORTBEAM) {
      CanSend3B(0x21A, 0x04, 0xC0, 0xF7);
    } else {
      CanSend3B(0x21A, 0x00, 0xC0, 0xF7);
    }
    //HeadLights
    //CanSend3B(0x21A, 0x00, 0xC0, 0xF7);
    //Byte 1: High beam=2, 4 = main lights,
    //32 = front fog light, 64 = rear fog light,
  }

  void sendIndicators(boolean left, boolean right) {
    if (left && !right) {
      CanSend2b(0x1f6, 17, 0xF0);
    } else if (right && !left) {
      CanSend2b(0x1f6, 33, 0xF0);
    } else if (right && left) {
      CanSend2b(0x1f6, 49, 0xF0);
    } else {
      CanSend2b(0x1f6, 0x80, 0xF0);
    }
  }
  uint8_t fracStepper = 0;  // rolls for smooth needle

  // Smooth, boundary-safe RPM sender for 0x0F3
  // Scale derived from your 4000 rpm log: raw = rpm * 6424 / 4000


  uint8_t rpmCounter = 0;
  uint8_t phase = 0;  // triangle wave phase

  void sendRPM(int rpm) {
    // 1) scale rpm to raw
    uint16_t raw = (uint32_t)rpm * 6424UL / 4000UL;
    uint8_t coarse = (raw >> 8) & 0xFF;
    uint8_t frac = raw & 0xFF;

    // 2) make a tiny triangle wave (0,1,2,1 → repeat)
    // instead of 0,1,2,1 pattern:
    int8_t tri = (phase & 1) ? 1 : -1;
    phase++;
    if (phase >= 2) phase = 0;




    // 3) apply dither safely
    uint8_t byte1;
    if (frac > 2 && frac < 0xFD) {
      byte1 = frac + tri;  // wiggle ±2 safely
    } else {
      byte1 = frac;  // near edges, freeze (no bounce)
    }

    uint8_t byte2 = coarse;

    // 4) build payload
    uint8_t rpmBuff[7] = {
      byte1,
      byte2,
      rpmCounter++,
      0xC0, 0xF0, 0xC4, 0xFF
    };

    // 5) CRC
    uint8_t crc = crc8Calculator.get_crc8(rpmBuff, 7, 0x7A);

    // 6) send
    CanSend(0x0F3, crc,
            rpmBuff[0], rpmBuff[1], rpmBuff[2],
            rpmBuff[3], rpmBuff[4], rpmBuff[5], rpmBuff[6]);
  }












  void sendGear(Gear gear, int gearNumber) {
    if (gearNumber == 1) {
      gearNumberCan = random(17, 27);
    }
    if (gearNumber == 2) {
      gearNumberCan = random(32, 37);
    }
    if (gearNumber == 3) {
      gearNumberCan = random(49, 59);
    }
    if (gearNumber == 4) {
      gearNumberCan = random(65, 75);
    }
    if (gearNumber == 5) {
      gearNumberCan = random(81, 95);
    }
    if (gearNumber == 6) {
      gearNumberCan = random(97, 111);
    }
    if (gearNumber == 7) {
      gearNumberCan = random(113, 127);
    }
    if (gearNumber == 8) {
      gearNumberCan = random(129, 143);
    }
    if (gearNumber == 9) {
      gearNumberCan = random(145, 159);
    }
    if (gear == MANUAL) {
      unsigned char transmissionWithoutCRC[] = { gearNumberCan, 0x02, 0, 0, 0 };
      unsigned char transmissionWithCRC[] = { crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6), transmissionWithoutCRC[0], transmissionWithoutCRC[1], transmissionWithoutCRC[2], transmissionWithoutCRC[3] };
      CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
    }
    if (gear == SEQUENTIAL) {
      unsigned char transmissionWithoutCRC[] = { gearNumberCan, 0x01, 0, 0, 0 };
      unsigned char transmissionWithCRC[] = { crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6), transmissionWithoutCRC[0], transmissionWithoutCRC[1], transmissionWithoutCRC[2], transmissionWithoutCRC[3] };
      CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
    }
    if (gear == DRIVE) {
      unsigned char transmissionWithoutCRC[] = { gearNumberCan, 0x80, 0, 0, 0 };
      unsigned char transmissionWithCRC[] = { crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6), transmissionWithoutCRC[0], transmissionWithoutCRC[1], transmissionWithoutCRC[2], transmissionWithoutCRC[3] };
      CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
    }
    if (gear == PARK) {
      unsigned char transmissionWithoutCRC[] = { random(1, 10), 0x20, 0, 0, 0 };
      unsigned char transmissionWithCRC[] = { crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6), transmissionWithoutCRC[0], transmissionWithoutCRC[1], transmissionWithoutCRC[2], transmissionWithoutCRC[3] };
      CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
    }
    if (gear == REVERSE) {
      unsigned char transmissionWithoutCRC[] = { random(1, 10), 0x40, 0, 0, 0 };
      unsigned char transmissionWithCRC[] = { crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6), transmissionWithoutCRC[0], transmissionWithoutCRC[1], transmissionWithoutCRC[2], transmissionWithoutCRC[3] };
      CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
    }
    if (gear == NEUTRAL) {
      unsigned char transmissionWithoutCRC[] = { random(1, 10), 0x60, 0, 0, 0 };
      unsigned char transmissionWithCRC[] = { crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6), transmissionWithoutCRC[0], transmissionWithoutCRC[1], transmissionWithoutCRC[2], transmissionWithoutCRC[3] };
      CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
    }
  }
  void sendShiftLight(int level) {
    //Shift Light
    CanSend(0xDF, level, 0, 0, 0, 0, 0, 0, 0);
    //Byte 1: Bar Increments 0-8 max
  }



  uint8_t speedToByte(uint16_t speedKmh) {
    const float scale = 3.875;
    uint8_t bestByte = 0;
    uint16_t minError = 65535;

    // Search for best byte that minimizes error
    for (uint8_t b = 0; b <= 255; b++) {
      uint16_t displayedSpeed = (uint16_t)((b * scale) + 0.5);  // cluster rounds
      uint16_t error = (displayedSpeed > speedKmh) ? displayedSpeed - speedKmh : speedKmh - displayedSpeed;
      if (error < minError) {
        minError = error;
        bestByte = b;
      }
    }
    return bestByte;
  }
  void sendSpeed(uint8_t count, uint16_t speedKmh) {
    uint16_t raw = speedKmh * 64.0;  // fixed-point (1/64 km/h)

    uint8_t data[6];
    data[0] = count;
    data[1] = count;
    data[2] = raw & 0xFF;  // fractional (LSB)
    data[3] = raw >> 8;    // integer / coarse
    data[4] = 0xAA;
    data[5] = 0x00;

    CAN.sendMsgBuf(0x1A1, 0, 6, data);
  }
  void sendSL(){
      CanSend(0x287, 0x05, 140, random(0,255), 0, 0x5c, 0x00,0, random(0,255));
  }
  void sendDoors() {
    if (FR) {
      //check control
      CanSend(0x5c0, 0x40, 14, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 14, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }

    if (FL) {
      //check control
      CanSend(0x5c0, 0x40, 15, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 15, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }

    if (RR) {
      //check control
      CanSend(0x5c0, 0x40, 17, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 17, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }

    if (RL) {
      //check control
      CanSend(0x5c0, 0x40, 16, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 16, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }

    if (HD) {
      //check control
      CanSend(0x5c0, 0x40, 530, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 530, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }

    if (BN) {
      //check control
      CanSend(0x5c0, 0x40, 19, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 19, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }
  }

  void sentTPMS() {
    if (FTL) {
      CanSend(0x5c0, 0x40, 139, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 139, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }
    if (FTR) {
      CanSend(0x5c0, 0x40, 143, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 143, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }
    if (RTL) {
      CanSend(0x5c0, 0x40, 141, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 141, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }
    if (RTR) {
      CanSend(0x5c0, 0x40, 140, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF);
    } else {
      CanSend(0x5c0, 0x40, 140, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF);
    }
  }

  void sendTemp(int temp) {
    //Temp
    unsigned char engineTempWithoutCRC[] = { 0x3e, temp, 0x64, 0x64, 0x64, 0x01, 0xF1 };
    //Byte 2: TOG Temp
    unsigned char engineTempWithCRC[] = { crc8Calculator.get_crc8(engineTempWithoutCRC, 7, 0xB2), engineTempWithoutCRC[0], engineTempWithoutCRC[1], engineTempWithoutCRC[2], engineTempWithoutCRC[3], engineTempWithoutCRC[4], engineTempWithoutCRC[5], engineTempWithoutCRC[6] };
    CAN.sendMsgBuf(0x2C4, 0, 8, engineTempWithCRC);
  }
  byte meow = 0x00;
  void sendDriveMode(DriveMode mode) {
    if (mode == COMFORT) {
      meow = 0xC4;
    }
    if (mode == COMFORTPLUS) {
      meow = 0xC5;
    }
    if (mode == SPORT) {
      meow = 0xC1;
    }
    if (mode == SPORTPLUS) {
      meow = 0xC2;
    }
    if (mode == ECOPRO) {
      meow = 0xC7;
    }
    if (mode == COMFORTADAPTIVE) {
      meow = 0xCa;
    }
    if (mode == ESCTC) {
      meow = 0xC2;
      //Traction modes       byte 3.
      CanSend(0x31b, 0x00, 0x00, 1, 0x00, 0x00, 0x00, 0x00, 0x00);
      //1= DSC off, 4= Traction,
    } else if (mode == Drift) {
      meow = 0xC3;
      //Traction modes       byte 3.
      CanSend(0x31b, 0x00, 0x00, 4, 0x00, 0x00, 0x00, 0x00, 0x00);
      //1= DSC off, 4= Traction,
    } else {
      //Traction modes       byte 3.
      CanSend(0x31b, 0x00, 0x00, 0, 0x00, 0x00, 0x00, 0x00, 0x00);
      //1= DSC off, 4= Traction,
    }
    //Drivemode
    CanSend(0x3d8, 0xCF, meow, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
    //Byte 2: 0xC1 =sport, C2 =sport+, C3 =sport individual, C4 =comfort, C5 =comfort +, C7 =eco pro, C8 eco pro 2, C9 eco pro indiv, 0xCa adaptive
  }

  // here is where the shit ends:


  void read() {
    String hour = FlowSerialReadStringUntil(';');
    String minute = FlowSerialReadStringUntil(';');
    String rpmStr = FlowSerialReadStringUntil(';');
    String shiftStr = FlowSerialReadStringUntil(';');
    String speedStr = FlowSerialReadStringUntil(';');
    String lightsStr = FlowSerialReadStringUntil(';');
    String indLeftStr = FlowSerialReadStringUntil(';');
    String indRightStr = FlowSerialReadStringUntil(';');
    String displayStr = FlowSerialReadStringUntil(';');
    String displayStr2 = FlowSerialReadStringUntil(';');
    String speedlimit = FlowSerialReadStringUntil(';');
    String gearStr = FlowSerialReadStringUntil(';');
    String idStr = FlowSerialReadStringUntil(';');
    String plidStr = FlowSerialReadStringUntil('\n');
    CanSend(0x39e, hour.toInt(), minute.toInt(), 0x00, 0x0D, 0x1F, 0xDF, 0x07, 0xF2);
    id = idStr.toInt();
    speedL = speedlimit.toInt();
    rpm = rpmStr.toInt();
    shift = shiftStr.toInt();
    speed = speedStr.toInt();
    indLeft = indLeftStr.toInt() > 0;
    indRight = indRightStr.toInt() > 0;
    highBeam = lightsStr.indexOf("FULLBEAM") != -1;
    handbrake = lightsStr.indexOf("HANDBRAKE") != -1;
    lowBeam = displayStr.indexOf("LB") != -1;
    FR = displayStr.indexOf("Fr") != -1;
    FL = displayStr.indexOf("Fl") != -1;
    RR = displayStr.indexOf("Rr") != -1;
    RL = displayStr.indexOf("Rl") != -1;
    BN = displayStr.indexOf("Bn") != -1;
    HD = displayStr.indexOf("Hd") != -1;
    FTR = displayStr.indexOf("FTR") != -1;
    FTL = displayStr.indexOf("FTL") != -1;
    RTR = displayStr.indexOf("RTR") != -1;
    RTL = displayStr.indexOf("RTL") != -1;

    ignitionStatus = plidStr.toInt();
    eng = ChkEngine.toInt();
    gear = gearStr;
    c_rpm = rpm;
    c_speed = speed;
    c_sport = displayStr2.indexOf("ttSport") != -1 || displayStr2.indexOf("Sport") != -1;
    c_sportPlus = displayStr2.indexOf("ttSport+") != -1 || displayStr2.indexOf("Sport+") != -1;
    c_comfort = displayStr2.indexOf("Comfort") != -1;
    c_ecopro = displayStr2.indexOf("ecopro") != -1;
    c_ESCTC = displayStr2.indexOf("ESC & TC Off") != -1;
    c_Drift = displayStr2.indexOf("Drift") != -1;
  }

  // Called once per arduino loop, timing can't be predicted,
  // but it's called between each command sent to the arduino
  void loop() {
    
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 0) {
      previousMillis = currentMillis;
      sentTPMS();
      sendNeccesaryCanId();
      sendDoors();
      sendIgnition(ignitionStatus);
      if (lowBeam == 1) {
        sendHeadlights(SHORTBEAM);
      } else if (highBeam == 1) {
        sendHeadlights(FULLBEAM);
      } else {
        sendHeadlights(OFF);
      }
      sendIndicators(indLeft, indRight);
      sendRPM(c_rpm);
      if (id == 1) {
        sendGear(PARK, 0);
      } else if (gear == "N") {
        sendGear(NEUTRAL, 0);
      } else if (gear == "R") {
        sendGear(REVERSE, 0);
      }
      sendSL();
      sendPark();
      sendShiftLight(shift / 1.75);
      sendSpeed(count, c_speed);
      sendTemp(90);
      if (c_sport == 1 && c_sportPlus == 0) {
        sendDriveMode(SPORT);
        if (gear != "N" && gear != "R" && id != 1) {
          sendGear(SEQUENTIAL, gear.toInt());
        }
      } else if (c_sportPlus == 1) {
        sendDriveMode(SPORTPLUS);
        if (gear != "N" && gear != "R" && id != 1) {
          sendGear(SEQUENTIAL, gear.toInt());
        }
      } else if (c_comfort == 1) {
        sendDriveMode(COMFORT);
        if (gear != "N" && gear != "R" && id != 1) {
          sendGear(DRIVE, gear.toInt());
        }
      } else if (c_ecopro == 1) {
        sendDriveMode(ECOPRO);
        if (gear != "N" && gear != "R" && id != 1) {
          sendGear(DRIVE, gear.toInt());
        }
      } else if (c_ESCTC == 1) {
        sendDriveMode(ESCTC);
        if (gear != "N" && gear != "R" && id != 1) {
          sendGear(MANUAL, gear.toInt());
        }
      } else if (c_Drift == 1) {
        sendDriveMode(Drift);
        if (gear != "N" && gear != "R" && id != 1) {
          sendGear(MANUAL, gear.toInt());
        }
      }
      rpmCounter++;
      if (rpmCounter >= 0x15) { rpmCounter = 0x00; }
      counter4Bit++;
      if (counter4Bit == 0x00) {
        counter4Bit = 14;
        counter4Bit++;
      }
      gearSelectorMessageCounter++;
      if (gearSelectorMessageCounter >= 15) { gearSelectorMessageCounter = 0; }
      count++;
      if (count == 254) {
        count = 0;
        count++;
      }
      count1++;
      if (count1 == 1) {
        count1 = 0;

        count1++;
      }
      igncunt++;
      if (igncunt == 34) {
        igncunt = 244;
        igncunt++;
      }
    }
  }

  // Called once between each byte read on arduino,
  // THIS IS A CRITICAL PATH :
  // AVOID ANY TIME CONSUMING ROUTINES !!!
  // PREFER READ OR LOOP METHOS AS MUCH AS POSSIBLE
  // AVOID ANY INTERRUPTS DISABLE (serial data would be lost!!!)
  void idle() {
  }
};

#endif