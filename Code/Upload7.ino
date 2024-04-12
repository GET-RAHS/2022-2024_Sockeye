// Include Libraries

#include <config.h>

// Interfaces
#include <DataFrame.h>
#include <status.h>

// Log
#include <logger.h>

// Ethernet
#include <Ethernet.h> // https://github.com/arduino-libraries/Ethernet // native ethernet really
//#include <NativeEthernet.h> // https://github.com/vjmuzik/NativeEthernet
#include <MySQL_Connection.h> // https://github.com/ChuckBell/MySQL_Connector_Arduino
#include <MySQL_Cursor.h> // https://github.com/ChuckBell/MySQL_Connector_Arduino

// I2C interface
#include <Wire.h>

// SD card libraries
#include <SD.h>

// Serial interface
#include <SPI.h>

// I2C display
#include <LiquidCrystal_I2C.h>

// Teensy time
#include <TimeLib.h>

// EEPROM
#include <EEPROM.h>

uint32_t flag = 0x0;
// INA variables
//                             addr, a,  ohm,    bvct, svct,  avg,VF,              AF,             label
// INA* SBat = new INA(&Wire1, 0x45, 24, 0.03,   7,    7,     2,  1.0,             1.0/3.0,        "Suppl. Batt. INA");
// INA* Mot1 = new INA(&Wire1, 0x40, 80, 0.001,  7,    7,     2,  1.0,             5.02/4.99*-1.0, "Mot1 INA");
// INA* Sol1 = new INA(&Wire1, 0x41, 80, 0.01,   7,    7,     2,  1.0,             5.0/50.71,      "Sol1 INA");
// INA* Sol2 = new INA(&Wire1, 0x44, 80, 0.01,   7,    7,     2,  1.0,             5.0/50.78,      "Sol2 INA");

INA* SBat = new INA(0x45, 24, 0.03,   7,    7,     2,  1.0,             1.0,        "Suppl. Batt. INA");
INA* Mot1 = new INA(0x40, 80, 0.001,  7,    7,     2,  1.0,             -1.0, "Mot1 INA");
INA* Sol1 = new INA(0x41, 80, 0.01,   7,    7,     2,  1.0,             5.0/24.77,      "Sol1 INA");
INA* Sol2 = new INA(0x44, 80, 0.01,   7,    7,     2,  1.0,             5.0/24.94,      "Sol2 INA");

// INA functions
void INAInit()   
{
    
    flag |= (!_init_ina(SBat))*INA_SBAT_FLAG;
    flag |= (!_init_ina(Mot1))*INA_MOT1_FLAG;
    flag |= (!_init_ina(Sol1))*INA_SOL1_FLAG;
    flag |= (!_init_ina(Sol2))*INA_SOL2_FLAG;
}

void INARead()
{
    _read_ina_V(SBat, false);
    _read_ina_A(SBat, false);
    _read_ina_A(Mot1, false);
    _read_ina_V(Mot1, false);
    _read_ina_A(Sol1, false);
    _read_ina_A(Sol2, false);
}

// GPS variables
GPS* gps = new GPS(&Serial3);

void gps_flags(gps_fix::valid_t valid)
{
    flag |= !valid.status*GPS_FIX_FLAG;
    flag |= !valid.speed*GPS_KPH_FLAG;
    flag |= !valid.altitude*GPS_ALT_FLAG;                           
    flag |= !valid.satellites*GPS_SAT_FLAG;
    flag |= !valid.time*GPS_TIM_FLAG;
}

// GPS functions
void GPSInit()
{
    _init_gps(gps);
    delay(500);
    _gps_getfix(gps);
}

void GPSRead()
{
    gps_fix::valid_t valid = _gps_getfix(gps);
    gps_flags(valid);
    
    _read_gps_unix(gps);
    _read_gps_loc(gps);
    _read_gps_kph(gps);
    if (valid.status)
    {
        Log(&Serial,0,"GPS fix","not valid");
    }
}


// Ethernet connection
bool EthernetSuccess = false;
EthernetClient client;

bool DatabaseSuccess = false;
MySQL_Connection mysql_conn((Client *)&client);

MySQL_Cursor* exc_cursor = NULL;
uint32_t exc_flag = 0x0;

// DPS variables
int altimeter_reset_pin = 29;                           

DPS* dps = new DPS("Press. DPS", altimeter_reset_pin);

// DPS functions
void DPSInit()
{
    flag |= !_init_dps(dps)*DPS_FLAG;
}

void DPSRead_P()
{
    _read_dps_P(dps);
}

void DPSRead_T()
{
    _read_dps_T(dps);
}

// BMS variables
BMS* bms = new BMS(&Serial2, 30);
byte bms_request[] = {0xDB,0xDB,0x00,0x00,0x00,0x00};

// BMS functions
void BMSInit()
{
    _init_bms(bms);
}

bool BMSRead()
{
    _request_bms_data(bms, bms_request,sizeof(bms_request));
    delay(100);
    Serial.print("BMS Bytes Available: ");
    Serial.println(Serial2.available());
    bool success = _raw_read_bms(bms);
    flag|=!success*BMS_FLAG;
    if (success)
    {
        _format_bms_data(bms);
    }

    return success;
}

// const uint8_t MPPT_CS = 10;

// MCP2515* mcp2515 = new MCP2515(MPPT_CS);
// can_frame* MPPT_receive_frame = new can_frame;
// can_frame* MPPT_broadcast_frame = new can_frame;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2; // not pointer
CAN_message_t* MPPT_receive_frame = new CAN_message_t;
CAN_message_t* MPPT_broadcast_frame = new CAN_message_t;

void MPPTInit()
{
    can2.begin();
    can2.setBaudRate(500000);
}

unsigned long MPPT_read_tries=0;

// bool MPPTRead()
// {
//     bool success = _mppt_read_message(&can2, MPPT_receive_frame);
//     if (!success)
//     {
//         if (MPPT_read_tries<5)
//         {
//             MPPT_read_tries++;
//             Log(&Serial,0,"MPPT failed. Attempt",MPPT_read_tries);
//             MPPTRead();
//         }
//     }
//     MPPT_read_tries=0;
//     return success;
// }

// unsigned long MPPT_broadcast_tries=0;
// bool MPPTBroadcast()
// {
//     bool success = _mppt_broadcast_message(&can2,MPPT_broadcast_frame);
//     if (!success)
//     {
//         if (MPPT_broadcast_tries<5)
//         {
//             MPPT_broadcast_tries++;
//             Log(&Serial,0,"MPPT failed. Attempt",MPPT_broadcast_tries);
//             MPPTBroadcast();
//         }
//     }
//     MPPT_broadcast_tries=0;
//     return success;
// }

// Teensy software reset
void reset()
{
    Log(&Serial,0,"Stopping","");
    SCB_AIRCR = 0x05FA0004;
}

void DatabaseInit()
{
    DatabaseSuccess = database_connect(&mysql_conn,&cloud_addr,cloud_user,cloud_password);
    if (DatabaseSuccess)
    {
        memsafe_mysql_execute(&mysql_conn, "DELETE FROM GET23.get31_data;");
    }
}

void WirelessInit()
{
    EthernetSuccess = network_connect(mac_addr);

    if (EthernetSuccess)
    {
        Log(&Serial,0,"Network connect","success");
        DatabaseInit();
        Log(&Serial,0,"DatabaseInit","success");
    } else {
        Log(&Serial,0,"Network connect","fail");
    }

    Log(&Serial,0,"Database connect",DatabaseSuccess?"success":"fail");
}

// SD Card variables
File sd_file;

// SD Card functions

void SDCardInit()
{

    bool sd_success = _init_sdcard(SDCARD_CS);
    flag |= !sd_success*SD_FLAG;
    if (sd_success)
    {
        _open_sdcard(&sd_file, name, gps->fix.dateTime.full_year() - 2000, gps->fix.dateTime.month, gps->fix.dateTime.date);
        for (int i = 0; i < 8; i++) { file[i]=name[i];}
        Log(&Serial,0,"SD card init","success");
    } else {
        // digitalWrite(2, HIGH); // Red
        Log(&Serial,0,"SD card init", "fail");
    }
}

void SDCard_Save(DataFrame* dataFrame, File* sdf)
{
    if (*sdf)
    {
        if (iteration==0)
        {
            sdf->println(dataFrame->labels);
            Log(&Serial,0,"SD card header:",dataFrame->labels);
        }
        Serial.println(dataFrame->valueFormat);
        sdf->println(dataFrame->valueFormat);
        Log(&Serial,0,"SD card record data attempted",dataFrame->valueFormat);
    } else {
        Log(&Serial,0,"SD card","not available");
    }
}

int mtr_temp_pin = A14;
int mtr_temp_val = 0;
int mtr_fan_temp_pin = A15;
int mtr_fan_temp_val = 0;
int PedalV = 0;


unsigned long lastHallTime=0;
unsigned long hallPulses=0;

// Speed and Distance
float WheelCircMM = 1735.0;
unsigned long timeout = 60000;

byte total_distance_arr[4] = {};
unsigned long total_distance = 0;
unsigned long total_distance_read = 0;
unsigned long trip_distance = 0;
unsigned long kph = 0;

float Pulses_PerMS = 0.0;
float HallKPH = 0.0;
float HallKPH_onepulse = 0.0;

unsigned long odo_run = millis();
unsigned long net_run = millis();
unsigned long dat_run = millis();

unsigned int BrakeOn=false,
CCOn=false;
unsigned long TripDistance=0;
unsigned long hall_pulse_pin=33;

unsigned long LTime = 0,
HTime = 0,
TTime = 0;

void pulse()
{
 unsigned long dt=micros()-lastHallTime;
 HallKPH=(WheelCircMM/dt)*3600.0/16.0;
 lastHallTime=micros();
 hallPulses++;
}

void Odometer() {
//   LTime = pulseIn(hall_pulse_pin, LOW, timeout);
//   HTime = pulseIn(hall_pulse_pin, HIGH, timeout);
//   TTime = LTime+HTime;
//   float WFreq = 1.0/TTime;
  //Log(&Serial,0,"TTime",TTime);
  //Log(&Serial,0,"WFreq",WFreq);
//   HallKPH = (float)TTime>0.0 ? WFreq*WheelCircMM*225.0:0.0;
  trip_distance += hallPulses*WheelCircMM/16.0;
  total_distance = total_distance_read + trip_distance;
  hallPulses=0;

  if (micros()-lastHallTime>1000*100)
  {
    HallKPH=0.0;
  }
}  

// :/
void odometer_read()
{
    total_distance_arr[0] = EEPROM.read(0);
    total_distance_arr[1] = EEPROM.read(1);
    total_distance_arr[2] = EEPROM.read(2);
    total_distance_arr[3] = EEPROM.read(3);
    total_distance_read = ((unsigned long)total_distance_arr[3] << 24) + (((unsigned long)total_distance_arr[2] << 16)) + ((unsigned long)total_distance_arr[1] << 8) + ((unsigned long)total_distance_arr[0]);
    Log(&Serial,0,"Total distance read:",total_distance_read);
}

void odometer_write()
{
    EEPROM.write(0, (byte)total_distance & 0xFF);
    EEPROM.write(1, (byte)(total_distance >> 8) & 0xFF);
    EEPROM.write(2, (byte)(total_distance >> 16) & 0xFF);
    EEPROM.write(3, (byte)(total_distance >> 24) & 0xFF);
    Log(&Serial,0,"Total distance write:",total_distance);
}

unsigned long run_every(unsigned long d, unsigned long p, void(*f)())
{
    unsigned long n = millis();
    if (n-p>=d)
    {
        (*f)();
        return n;
    }
    return p;
}

unsigned long start = millis();

unsigned long how_long(const char* s)
{
    unsigned long n = millis();
    unsigned long d = n-start;
    Log(&Serial,0,s,d);
    start = n;
    return d;
}

// float roll_avg(float*s, float*v, float*a, int n)
// {
//     *s = (*s) - (*(a+n-1))+(*v);
//     for (int i=1;i<=n;i++)
//     {
//         *(a+(n-i))=*(a+(n-i-1));
//     }
//     *(a)=*v;

//     return (*s)/(float)n;
// }

void setup()
{
    // attachInterrupt(digitalPinToInterrupt(TEENSY_RESET_PIN), reset, LOW);
    attachInterrupt(digitalPinToInterrupt(hall_pulse_pin),pulse,FALLING);

    setDebug(true); // printing
    setPriority(0); // printing
    
    // pinMode(4, OUTPUT); // Blue
    // pinMode(3, OUTPUT); // Green
    // pinMode(2, OUTPUT); // Red

    Serial.begin(115200);
    Serial2.begin(19200);
    Serial4.begin(19200);
    // Serial5.begin(9600);
    // Serial5.setTimeout(50);
    Serial.println("  _______ ______ _      ______ __  __ ______ _______ _______     __   _____ ______ _______ _    _ _____  \n |__   __|  ____| |    |  ____|  \\/  |  ____|__   __|  __ \\ \\   / /  / ____|  ____|__   __| |  | |  __ \\ \n    | |  | |__  | |    | |__  | \\  / | |__     | |  | |__) \\ \\_/ /  | (___ | |__     | |  | |  | | |__) |\n    | |  |  __| | |    |  __| | |\\/| |  __|    | |  |  _  / \\   /    \\___ \\|  __|    | |  | |  | |  ___/ \n    | |  | |____| |____| |____| |  | | |____   | |  | | \\ \\  | |     ____) | |____   | |  | |__| | |     \n    |_|  |______|______|______|_|  |_|______|  |_|  |_|  \\_\\ |_|    |_____/|______|  |_|   \\____/|_|     \n                                                                                                         \n                                                                                                         ");
    WirelessInit();
    Log(&Serial,0,"Ethernet","starting");
    pinMode(hall_pulse_pin,INPUT);

    pinMode(mtr_temp_pin,INPUT);
    pinMode(mtr_fan_temp_pin,INPUT);

    // GPS
    GPSInit();
    Serial << gps->fix.dateTime; Serial.println();

    // SD Card
    SDCardInit();

    odometer_read();

    // I2C
    Wire.begin();
    Wire1.begin();

    // DPS
    // pinMode(25, INPUT_PULLUP); // for DPS310 sensor / pressure sensor
    DPSInit();

    // INA226
    INAInit();

    // BMS
    BMSInit();

    // MPPT
    MPPTInit();

    //Data Sending
    Wire.setSCL(19);
    Wire.setSDA(18);

    Serial.println("\n\n\n");
    Serial.println("Starting...");
    Serial.println("\n\n\n");
    delay(1000);
}

unsigned long Now = millis();
unsigned long Last = Now;

//DATA RECEIVING--------------------------------------------------------------------------------------
unsigned int numbytes = 0;

//expected size (in byes) of values recieved
const unsigned int PedVSize=4,
CCVSize=4,
StatusesSize=1,
BrakeInSize=4,
KPHSize=4,
TripDistanceSize=4,
tokenSize = 1,
expectedTransmitSize = PedVSize + CCVSize + BrakeInSize + KPHSize + TripDistanceSize + StatusesSize + tokenSize;

unsigned char SecondaryTeensyData[expectedTransmitSize]={0};

//default set values recieved
float PedV=0.0,
CCV=0.0,
BrakeIn=0.0,
KPH=0.0;

uint8_t Statuses = 0b00000000;
uint8_t token = 0b00000000;

float float32min=1.17549435082e-38;
float float32max=3.40282346639e+38;

unsigned long LARSEN___________a___frbinexpwithbias(float x, int* exp)
{
    unsigned long bin=0;
    unsigned int i=0;
    float normalized=x;

    if (x==0.0 || float32min>x) { // 0.0 or less than min
        return 0;
    } else if (float32max<x) { // larger than max (inf)
        *exp=0b11111111;
        return 0;
    }
    if (x>=2.0)
    {
        while (normalized>=2.0) // normalize
        {
            normalized/=2;
            (*exp)++;
        }
    } else if (x<1.0) {
        while (normalized<1.0) // normalize
        {
            normalized*=2;
            (*exp)--;
        }
    } else if (x==1.0) { // weird
        *exp=127;
        return 0;
    }

    float temp=normalized;
    while (true)
    {
        temp*=2;
        if (temp>1.0)
        {
            bin|=1<<(22-i);
            temp-=1.0;
        }
        i++;
        if (temp==1.0 || i==23)
        {
            break;
        }
    }
    (*exp)+=127;
    return bin;
}


// SEND CODE

float frabs(float x)
{
    return x>0?x:-x;
}

void sendFloat32(float x)
{
    unsigned int signBIT=0;
    if (0.0>x)
    {
        signBIT=1;
    }
    int exp=0;
    unsigned long frbin=LARSEN___________a___frbinexpwithbias(frabs(x),&exp);
    unsigned long bits=signBIT<<31 | (exp&0xFF) << 23 | frbin&0b11111111111111111111111;
    Wire.write((unsigned char)bits&0xFF);
    Wire.write((unsigned char)bits>>8&0xFF);
    Wire.write((unsigned char)bits>>16&0xFF);
    Wire.write((unsigned char)bits>>24&0xFF);
}
void sendUInt8(unsigned long x)
{
    Wire.write((unsigned char)(x&0xFF));
}

void sendUInt16(unsigned long x)
{
    Wire.write((unsigned char)x&0xFF);
    Wire.write((unsigned char)x>>8&0xFF);
}

void sendUInt32(unsigned long x)
{
    Wire.write((unsigned char)x&0xFF);
    Wire.write((unsigned char)x>>8&0xFF);
    Wire.write((unsigned char)x>>16&0xFF);
    Wire.write((unsigned char)x>>24&0xFF);
}
void sendInt8(int x)
{
    unsigned int mag=abs(x)&0x7F;
    unsigned int int8bin=mag;
    unsigned int sign=0;
    if (0>x){
        sign=1;
        int8bin=~mag+1|sign<<7; // check precedence
    }
    Wire.write((unsigned char)int8bin&0xFF);
}

void sendInt16(int x)
{
    unsigned int mag=abs(x)&0x7FFF;
    unsigned int int16bin=mag;
    unsigned int sign=0;
    if (0>x){
        sign=1;
        int16bin=~mag+1|sign<<15;
    }
    Wire.write((unsigned char)int16bin&0xFF);
    Wire.write((unsigned char)int16bin>>8&0xFF);
}

void sendInt32(long x)
{
    unsigned long mag=abs(x)&0x7FFFFFFF;
    unsigned long int32bin=mag;
    unsigned int sign=0;
    if (0>x){
        sign=1;
        int32bin=~mag+1|sign<<31;
    }
    Wire.write((unsigned char)int32bin&0xFF);
    Wire.write((unsigned char)int32bin>>8&0xFF);
    Wire.write((unsigned char)int32bin>>16&0xFF);
    Wire.write((unsigned char)int32bin>>24&0xFF);
}
// SEND CODE 

unsigned int readUInt8(unsigned char** pData)
{
    unsigned char* data=*pData;
    *pData+=1;
    return (unsigned int)data[0];
}

unsigned int readUInt16(unsigned char** pData)
{
    unsigned char* data=*pData;
    *pData+=2;
    return (unsigned int)data[0]+((unsigned int)data[1]<<8);
}

unsigned long readUInt32(unsigned char** pData)
{
    unsigned char* data=*pData;
    *pData+=4;
    return (unsigned long)data[0]+((unsigned long)data[1]<<8)+((unsigned long)data[2]<<16)+((unsigned long)data[3]<<24);
}

int readInt8(unsigned char** pData)
{
    unsigned char* data=*pData;

    unsigned long bin=(unsigned long)data[0];
    int sign=(bin>>7)&1;
    unsigned int mag=bin;
    *pData+=1;
    if (sign==1)
    {
        mag=~((bin&0x7F)-1);
        return -(int)mag; // check precedent
    }
    return (int)mag;
}

int readInt16(unsigned char** pData)
{
    unsigned char* data=*pData;
    unsigned long bin=(unsigned long)data[0]+((unsigned long)data[1]<<8);
    int sign=(bin>>15)&1;
    unsigned int mag=bin;
    *pData+=2;
    if (sign==1)
    {
        mag=~((bin&0x7FFF)-1);
        return -(int)mag; // check precedence
    }
    return (int)mag;
}

long readInt32(unsigned char** pData)
{
    unsigned char* data=*pData;
    unsigned long bin =(unsigned long)data[0]+((unsigned long)data[1]<<8)+((unsigned long)data[2]<<16)+((unsigned long)data[3]<<24);
    int sign=(bin>>31)&1;
    unsigned long mag=bin;
    *pData+=4;
    if (sign==1)
    {
        mag=~((bin&0x7FFFFFFF)-1);
        return -(long)mag; // check precedence
    }
    return (long)mag;
}

unsigned long readBitFlag8(unsigned char** pData)
{
    unsigned char* data=*pData;
    *pData+=1;
    return (unsigned char)data[0];
}

float decodeMantissa23(unsigned long bits)
{
    float sum=0.0;
    for (int i=0;i<23;i++)
    {
        bool flipped=bits>>((23-1)-i)&1;
        if (flipped)
        {
            sum+=1/pow(2,i+1);
        }
    }
    return sum+1.0; // Magical 1
}

float readFloat32(unsigned char** pData)
{
    unsigned char* data=*pData;
    int sign=1;
    float n=0.00;
    unsigned long float32bin=(unsigned long)data[0]+((unsigned long)data[1]<<8)+((unsigned long)data[2]<<16)+((unsigned long)data[3]<<24);
    Serial.printf("\nbits: %lx\n",float32bin);
    bool IsNegative = float32bin >> 31 & 1;
    if (IsNegative)
    {
        sign=-1;
    }
    int exp = (float32bin>>23 & 0xFF) - 127;
    unsigned long mantissaBits = (float32bin & 0b11111111111111111111111);
    float mantissa=decodeMantissa23(mantissaBits);
    Serial.printf("pow: %f\n",pow(2, exp));
    n = sign*mantissa*(pow(2, exp));
    *pData+=4;
    Serial.printf("Float: %f",n);
    return n;
}

float raw_readFloat32(unsigned long float32bin)
{
    int sign=1;
    float n=0.00;
    Serial.printf("\nbits: %lx\n",float32bin);
    bool IsNegative = float32bin >> 31 & 1;
    if (IsNegative)
    {
        sign=-1;
    }
    int exp = (float32bin>>23 & 0xFF) - 127;
    unsigned long mantissaBits = (float32bin & 0b11111111111111111111111);
    float mantissa=decodeMantissa23(mantissaBits);
    Serial.printf("pow: %f\n",pow(2, exp));
    n = sign*mantissa*(pow(2, exp));
    Serial.printf("Float: %f",n);
    return n;
}
    
void DecipherData(unsigned char* data)
{
    //recv order: pedv -> ccv -> statuses (byte) -> anBrakeIn -> kph -> tripDistance
    
    unsigned char* pData=data;

    // Serial.print("Real token");Serial.println(data[21]);
    
    PedV = readFloat32(&pData);
    Serial.printf("PedV: %f", PedV);
    // CCV = readFloat32(&pData);
    
    // BrakeIn = readFloat32(&pData);    
    // KPH = readFloat32(&pData);
    
    // Statuses = readBitFlag8(&pData); //check this
    // BrakeOn=Statuses>>7&1,CCOn=Statuses>>6&1; //statuses being sent through; add more as more statuses are added
    // pData+=StatusesSize;
    
    // TripDistance = readUInt32(&pData);

    // token = readUInt8(&pData);
    // Serial.print("Token");Serial.println(token);
}


void RequestData(unsigned char* data) {
    if (Wire1.available())
    {
        Serial.println("I2C buffer not empty");
        while (Wire1.available())
        {
            Wire1.read();
        }
    }
    Wire1.requestFrom(0x01, 4);
    numbytes=0;
    memset(data,0,sizeof data);
    // if (Wire1.available()) {
    //     Serial.println("data recieved");
    // } else {
    //     Serial.println("recieve error");
    // }

    while (Wire1.available()) {
      data[numbytes] = Wire1.read();
        Serial.printf("HELLO%x\n",data[numbytes]);
        numbytes++;
    }
    Serial.printf("\nnumbytes:%d\n",numbytes);

    if (numbytes != expectedTransmitSize) {
        Serial.println("Transmission error; expected bytes does not match recieved bytes.");
        Serial.print(numbytes);Serial.println(" recieved bytes.");
        Serial.print(expectedTransmitSize);Serial.println(" expected bytes.");
    }
}

// void RequestData(unsigned char* data) {
//     if (Serial5.available())
//     {
//         while (Serial5.available())
//         {
//             Serial5.readBytes(data,Serial5.available());
//         }
//     }
//     // Wire.requestFrom(0x01, 4);
//     numbytes=0;
//     memset(data,0,sizeof data);
//     // if (Wire.available()) {
//     //     Serial.println("data recieved");
//     // } else {
//     //     Serial.println("recieve error");
//     // }

//     Serial.printf("Available: %d",Serial5.available());
//     if (Serial5.available()) {
//         Serial5.readBytes(data,expectedTransmitSize);
//         Serial.printf("HELLO%x\n",data[numbytes]);
//         numbytes++;
//     }
//     Serial.printf("\nnumbytes:%d\n",numbytes);

//     if (numbytes != expectedTransmitSize) {
//         Serial.println("Transmission error; expected bytes does not match recieved bytes.");
//         Serial.print(numbytes);Serial.println(" recieved bytes.");
//         Serial.print(expectedTransmitSize);Serial.println(" expected bytes.");
//     }
// }


float motorW_float = 0, solarW1_float = 0, solarW2_float = 0, BattV_float, PedalV_float;
uint32_t motorW_int = 0, solarW1_int = 0, solarW2_int = 0, BattA_int = 0, BattV_int, PedalV_int, CCV_int;

#define fourthByte(w) ((uint8_t)((w) >> 24))
#define thirdByte(w) ((uint8_t)((w) >> 16))
#define secondByte(w) ((uint8_t)((w) >> 8))
#define firstByte(w) ((uint8_t)((w)&0xFF))
byte contrl[8];
byte tele[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


void loop()                                                            
{
    //delay(3000);
    how_long("Loop end time (0)");
    Now = millis();
    DataFrame CurrentDataFrame; // Current data frame
    Wire.requestFrom(0x01,292929);
    RequestData(SecondaryTeensyData);
    DecipherData(SecondaryTeensyData);
    // bool suc =MPPTRead();
    if (can2.read(*MPPT_receive_frame))
    {
        CAN_message_t msg=*MPPT_receive_frame;
        Serial.print("CAN2 ");
        Serial.print("MB: "); Serial.print(msg.mb);
        Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
        if (msg.id==0x614 || msg.id==0x6F4)
        {
            unsigned long bits=(unsigned long)msg.buf[0] + ((unsigned long)msg.buf[1]<<8) + ((unsigned long)msg.buf[2]<<16) + ((unsigned long)msg.buf[3]<<24);
            Serial.printf("Limit: %f",raw_readFloat32(bits));
        }
        Serial.print("  EXT: "); Serial.print(msg.flags.extended);
        Serial.print("  LEN: "); Serial.print(msg.len);
        Serial.print(" DATA: ");
        for ( uint8_t i = 0; i < 8; i++ ) {
            Serial.print(msg.buf[i]); Serial.print(" ");
        }
    }
    Odometer();

    InsertV(&CurrentDataFrame, "N", &iteration);

    InsertV(&CurrentDataFrame, "HallKPH_onepulse", &HallKPH, 1);
    how_long("Hall KPH time");

    InsertV(&CurrentDataFrame, "file", file);

    InsertV(&CurrentDataFrame, "CCV", &CCV,2);

    InsertV(&CurrentDataFrame, "PedV", &PedV,2);

    InsertV(&CurrentDataFrame, "Brake", &BrakeOn);

    InsertV(&CurrentDataFrame, "AnalogBrake", &BrakeIn,2);

    InsertV(&CurrentDataFrame, "CCOn", &CCOn);

    mtr_temp_val = analogRead(mtr_temp_pin);
    InsertV(&CurrentDataFrame, "MotorTemp", &mtr_temp_val);

    mtr_fan_temp_val = (((float)analogRead(mtr_fan_temp_pin)/1023.0)-(498.0/1600.0))*100.0/(502.0/1600.0);                 
    InsertV(&CurrentDataFrame, "MotorFanTemp", &mtr_fan_temp_val);

    InsertV(&CurrentDataFrame, "TripDistance", &trip_distance);
    InsertV(&CurrentDataFrame, "TotalDistance", &total_distance);       

    how_long("Misc (cc, brake, trip) time");

    // GPS
    GPSRead();
    
    InsertV(&CurrentDataFrame, "GPSTime", gps->unix_str);
    InsertV(&CurrentDataFrame, "Lat", &gps->lat, 6);
    InsertV(&CurrentDataFrame, "Lon", &gps->lon, 6);
    InsertV(&CurrentDataFrame, "GPSKph", &gps->kph, 1);

    how_long("GPS time");
    
    // INA
    INARead();

    InsertV(&CurrentDataFrame, "SBatA", &SBat->A, 2); // 12.50
    InsertV(&CurrentDataFrame, "SBatV", &SBat->V, 2);
    InsertV(&CurrentDataFrame, "Mot1A", &Mot1->A, 1); // 8.0
    InsertV(&CurrentDataFrame, "Sol1A", &Sol1->A, 1);
    InsertV(&CurrentDataFrame, "Sol2A", &Sol2->A, 1);
    InsertV(&CurrentDataFrame, "BattV", &Mot1->V, 1); // INA
    float BattA=Sol1->A+Sol2->A-Mot1->A;
    InsertV(&CurrentDataFrame, "BattA", &BattA, 1);
    
    how_long("INA time");

    // DPS
    DPSRead_P();
    DPSRead_T();

    InsertV(&CurrentDataFrame, "Pressure", &dps->Press, 1);
    InsertV(&CurrentDataFrame, "Temperature", &dps->Temp, 1);

    how_long("DPS Time");

    // BMS
    BMSRead();

    InsertV(&CurrentDataFrame, "BMSTime", &bms->Time);
    InsertV(&CurrentDataFrame, "BattCharge", &bms->BattCharge);
    InsertV(&CurrentDataFrame, "BMSBattV", &bms->BattV);
    InsertV(&CurrentDataFrame, "BMSBattA", &bms->BattA);
    InsertV(&CurrentDataFrame, "BMSBattW", &bms->BattW);
    InsertV(&CurrentDataFrame, "HighVNum", &bms->HighVNum);
    InsertV(&CurrentDataFrame, "HighV", &bms->HighV);
    InsertV(&CurrentDataFrame, "AvgV", &bms->AvgV);
    InsertV(&CurrentDataFrame, "LowVNum", &bms->LowVNum);
    InsertV(&CurrentDataFrame, "LowV", &bms->LowV);

    int spread = bms->HighV-bms->LowV;
    InsertV(&CurrentDataFrame, "DCellV", &spread); // spread
    InsertV(&CurrentDataFrame, "MOS_Temp", &bms->MOS_Temp);
    InsertV(&CurrentDataFrame, "TBal", &bms->TBal);
    InsertV(&CurrentDataFrame, "Temp1Neg", &bms->Temp1Neg);
    InsertV(&CurrentDataFrame, "Temp2", &bms->Temp2);
    InsertV(&CurrentDataFrame, "Temp3", &bms->Temp3);
    InsertV(&CurrentDataFrame, "Temp4Pos", &bms->Temp4Pos);
    InsertV(&CurrentDataFrame, "CharMOS", &bms->CharMOS);
    InsertV(&CurrentDataFrame, "DisMOS", &bms->DisMOS);
    InsertV(&CurrentDataFrame, "BalState", &bms->BalState);

    for (unsigned int i=0;i<bms->n_cells;i++)
    {
        char name[7]="";
        sprintf(name, "V%d", i);

        InsertV(&CurrentDataFrame, name, &bms->CellV[i]);
    }

    InsertV(&CurrentDataFrame, "NumCells", &bms->NumCells);
    InsertV(&CurrentDataFrame, "PhysCap", &bms->PhysCap);
    InsertV(&CurrentDataFrame, "RemCap", &bms->RemCap);
    InsertV(&CurrentDataFrame, "CyclCap", &bms->CyclCap);

    how_long("BMS Time");

    int BattV = bms->BattV/10.0;
    
    int BattP = BattA*BattV;

    InsertV(&CurrentDataFrame, "BattP", &BattP);

    int Sol1P = Sol1->A*BattV;
    int Sol2P = Sol2->A*BattV;

    int SolrP = Sol1P+Sol2P;
    InsertV(&CurrentDataFrame, "SolrP", &SolrP);

    float SolrA = Sol1->A+Sol2->A;
    InsertV(&CurrentDataFrame, "SolrA", &SolrA, 1);

    int Mot1P = Mot1->A*BattV;
    InsertV(&CurrentDataFrame, "Mot1P", &Mot1P);

    how_long("INA calc time");

    InsertV(&CurrentDataFrame, "flag", &flag);
    InsertV(&CurrentDataFrame, "t", &Now);

    how_long("Misc (flag, t) time");

    if (EthernetSuccess && Ethernet.linkStatus() == LinkON)
    {
        if (DatabaseSuccess && mysql_conn.connected())
        {

            ConstructQuery(&CurrentDataFrame, schema, table); // Build a string of all the data, separated by comma.
            // digitalWrite(4, HIGH); // Blue
            memsafe_mysql_execute(&mysql_conn, CurrentDataFrame.query); // Send to the 200 row database

            Log(&Serial,0,"Regular Query: ",CurrentDataFrame.query); 

            ConstructQuery(&CurrentDataFrame, schema, cumulative_table); // Build a string of all the data, separated by comma.
            memsafe_mysql_execute(&mysql_conn, CurrentDataFrame.query);  // Send to the big database

            Log(&Serial,0,"Cumulative Query: ",CurrentDataFrame.query);

            strcpy(CurrentDataFrame.query, ""); // Remove contents of the query string
            sprintf(CurrentDataFrame.query, "DELETE FROM GET23.get31_data WHERE N < %d;", (int)iteration-200);     //Likely will need to change where the data goes                    // Making the string that tells the database to delete rows past N=200
            memsafe_mysql_execute(&mysql_conn, CurrentDataFrame.query); // Send command to database to delete rows past N=200
            // digitalWrite(4, LOW); // Blue

            Log(&Serial,0,"Delete Query: ",CurrentDataFrame.query);
            dat_run = millis();
        } else {
            // dat_run = run_every(30000,dat_run,DatabaseInit);
            Log(&Serial,0,"Labels:",CurrentDataFrame.labels);
            Log(&Serial,0,"Values:",CurrentDataFrame.valueFormat);
            Log(&Serial,0,"Database","not connected.");
        }
        net_run = millis();
    } else {
        // net_run = run_every(30000,net_run,WirelessInit);
        Log(&Serial,0,"Labels:",CurrentDataFrame.labels);
        Log(&Serial,0,"Values:",CurrentDataFrame.valueFormat);
        Log(&Serial,0,"Ethernet","not connected to network.");
    }

    how_long("Database time");

    SDCard_Save(&CurrentDataFrame, &sd_file);
    sd_file.flush();

    odo_run = run_every(30000,odo_run,odometer_write);
    
    Log(&Serial,0,"Loop time: ",millis()-CurrentDataFrame.start);
    Log(&Serial,0,"Iteration:",iteration++);
    // LastDataFrame = CurrentDataFrame;

    how_long("SD time");

  
  
  //output to control teensy
  if(Serial4.availableForWrite() >= 20){
    Serial.println("writing to Control");

    motorW_float = ((Mot1->A)*(Mot1->V));
    solarW1_float = ((Sol1->A)*(Mot1->V)); 
    solarW2_float = ((Sol2->A)*(Mot1->V));
    BattV_float = (bms->BattV/10.0);


    //turns a float into an integer, while keeping all the data
    memcpy(&motorW_int, &motorW_float, 4);
    tele[0] = fourthByte(motorW_int);
    tele[1] = thirdByte(motorW_int);
    tele[2] = secondByte(motorW_int);
    tele[3] = firstByte(motorW_int);

    //solarW1
    memcpy(&solarW1_int, &solarW1_float, 4);
    tele[4] = fourthByte(solarW1_int);
    tele[5] = thirdByte(solarW1_int);
    tele[6] = secondByte(solarW1_int);
    tele[7] = firstByte(solarW1_int);

    //solarW2
    memcpy(&solarW2_int, &solarW2_float, 4);
    tele[8] = fourthByte(solarW2_int);
    tele[9] = thirdByte(solarW2_int);
    tele[10] = secondByte(solarW2_int);
    tele[11] = firstByte(solarW2_int);

    //BattV
    memcpy(&BattV_int, &BattV_float, 4);
    tele[12] = fourthByte(BattV_int);
    tele[13] = thirdByte(BattV_int);
    tele[14] = secondByte(BattV_int);
    tele[15] = firstByte(BattV_int);

    //BattA
    memcpy(&BattA_int, &BattA, 4);
    tele[16] = fourthByte(BattA_int);
    tele[17] = thirdByte(BattA_int);
    tele[18] = secondByte(BattA_int);
    tele[19] = firstByte(BattA_int);

    //writes array to control teensy
    Serial4.write(tele, 20);
    delay(10);
    Serial4.flush();
    for(int i = 0; i < 20;){
      Serial.print(tele[i]);
      Serial.print("  ");
      i++;
    }
    
    if(Serial4.available() >= 8){
      Serial4.readBytes(contrl, 8);

      PedalV_int = (contrl[0] << 24) + (contrl[1] << 16) + (contrl[2] << 8) + contrl[3];
      memcpy(&PedalV_float, &PedalV_int, 4);

      CCV_int = (contrl[4] << 24) + (contrl[5] << 16) + (contrl[6] << 8) + contrl[7];
      memcpy(&CCV, &CCV_int, 4);

      Serial4.end();
      Serial4.begin(19200);
    }
  }
  
  Serial.println();
  for(int i = 0; i < 8;){
    Serial.print(contrl[i]);
    Serial.print("  ");
    i++;
  }
  Serial.println("\n\n\n");
  Last = Now;
}