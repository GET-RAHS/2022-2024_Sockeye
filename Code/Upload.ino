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
// //                  addr, a,  ohm,    bvct, svct,   avg,VF,         AF,         label
// INA* SBat = new INA(0x47, 24, 0.03,   7,    7,      0,  1.0,        1.0/3.0,    "Suppl. Batt. INA");
// INA* Mot1 = new INA(0x41, 80, 0.001,  7,    7,      0,  1.0,        13.4/13.10, "Mot1 INA");
// INA* Mot2 = new INA(0x42, 80, 0.001,  7,    7,      0,  1.0,        13.19/13.4, "Mot2 INA");
// INA* Sol1 = new INA(0x43, 80, 0.01,   7,    7,      0,  1.0,        2.6/26.92,  "Sol1 INA");
// INA* Sol2 = new INA(0x44, 80, 0.01,   7,    7,      0,  1.0,        2.8/28.91,  "Sol2 INA");
// INA* Sol3 = new INA(0x45, 80, 0.01,   7,    7,      0,  1.0,        3.2/32.66,  "Sol3 INA");
// INA* Sol4 = new INA(0x46, 80, 0.01,   7,    7,      0,  1.0,        3.0/30.93,  "Sol4 INA");
// INA* Batt = new INA(0x40, 80, 0.001,  7,    7,      0,  23.92/127.3,13.4/13.41, "Main Batt. INA"); // previous vf was 127.4/24.14, changed to calc// maybe this current doesnt need calibration

// INA variables
//                  addr, a,  ohm,    bvct, svct,  avg,VF,         AF,         label
INA* SBat = new INA(0x47, 24, 0.03,   7,    7,     2,  1.0,        1.0/3.0,    "Suppl. Batt. INA");
INA* Mot1 = new INA(0x41, 80, 0.001,  7,    7,     2,  1.0,        5.02/4.99*-1.0, "Mot1 INA"); // calibrated with power supply not bms. shouldnt be so bad though. for bms do 9.3/9.19. for power supply do 9.25/9.19. i think the bms rounded up anyways
INA* Mot2 = new INA(0x42, 80, 0.001,  7,    7,     2,  1.0,        1.0*-1.0, "Mot2 INA"); // 9.3/9.29
INA* Sol1 = new INA(0x43, 80, 0.01,   7,    7,     2,  1.0,        5.0/50.71,  "Sol1 INA");
INA* Sol2 = new INA(0x44, 80, 0.01,   7,    7,     2,  1.0,        5.0/50.78,  "Sol2 INA");
INA* Sol3 = new INA(0x45, 80, 0.01,   7,    7,     2,  1.0,        5.0/50.99,  "Sol3 INA");
INA* Sol4 = new INA(0x46, 80, 0.01,   7,    7,     2,  1.0,        5.0/51.44,  "Sol4 INA");
INA* Batt = new INA(0x40, 80, 0.001,  7,    7,     2,  100.6/19.03,9.3/9.42, "Main Batt. INA"); // previous vf was 127.4/24.14, changed to calc// maybe this current doesnt need calibration


// INA functions
void INAInit() // [todo] experiment with inlining. not just this function
{
    
    flag |= (!_init_ina(SBat))*INA_SBAT_FLAG;
    flag |= (!_init_ina(Mot1))*INA_MOT1_FLAG;
    flag |= (!_init_ina(Mot2))*INA_MOT2_FLAG;
    flag |= (!_init_ina(Sol1))*INA_SOL1_FLAG;
    flag |= (!_init_ina(Sol2))*INA_SOL2_FLAG;
    flag |= (!_init_ina(Sol3))*INA_SOL3_FLAG;
    flag |= (!_init_ina(Sol4))*INA_SOL4_FLAG;
    flag |= (!_init_ina(Batt))*INA_BATT_FLAG;
}

void INARead()
{
    _read_ina_V(SBat, false);
    _read_ina_A(SBat, false);
    _read_ina_A(Mot1, false);
    _read_ina_A(Mot2, false);
    _read_ina_A(Sol1, false);
    _read_ina_A(Sol2, false);
    _read_ina_A(Sol3, false);
    _read_ina_A(Sol4, false);
    _read_ina_V(Batt, false);
    _read_ina_A(Batt, false);
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

MySQL_Cursor* display_cursor = NULL;
char display_message[12+10] = "";
int display_CCV = 0;

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
BMS* bms = new BMS(&Serial1, 31);

// BMS functions
void BMSInit()
{
    _init_bms(bms);
}

bool BMSRead()
{
    bool success = _raw_read_bms(bms);
    flag|=!success*BMS_FLAG;
    if (success)
    {
        _format_bms_data(bms);
    }

    return success;
}

const uint8_t MPPT_CS = 10;

MCP2515* mcp2515 = new MCP2515(MPPT_CS);
can_frame* frame = new can_frame;

void MPPTInit()
{
    mcp2515->reset();
    mcp2515->setBitrate(CAN_500KBPS);
    mcp2515->setNormalMode();
}

bool MPPTRead()
{
    unsigned long tries = 0;

    hell:
    bool success = _mppt_read_message(mcp2515, frame)==MCP2515::ERROR_OK;
    if (!success)
    {
        if (tries<5)
        {
            tries++;
            Log(&Serial,0,"MPPT failed. Attempt",tries);
            goto hell;
        }
    }

    return success;
}


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
        memsafe_mysql_execute(&mysql_conn, "DELETE FROM green_car_database.get31_data;");
    }
}

void WirelessInit()
{
    EthernetSuccess = network_connect(mac_addr);

    if (EthernetSuccess)
    {
        Log(&Serial,0,"Network connect","success");
        DatabaseInit();
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
    // GPSRead();

    //bool gps_tim_success = !((flag & GPS_TIM_FLAG) == GPS_TIM_FLAG);

    bool sd_success = _init_sdcard(SDCARD_CS);
    flag |= !sd_success*SD_FLAG;
    if (sd_success)
    {
        _open_sdcard(&sd_file, name, gps->fix.dateTime.full_year() - 2000, gps->fix.dateTime.month, gps->fix.dateTime.date);
        for (int i = 0; i < 8; i++) { file[i]=name[i];}
        Log(&Serial,0,"SD card init","success");
    } else {
        digitalWrite(2, HIGH); // Red
        Log(&Serial,0,"SD card init", "fail");
    }
    
    //set_sd_card(sd_file);
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
        Log(&Serial,0,"SD card data (supposedly) recorded",dataFrame->valueFormat);
    } else {
        Log(&Serial,0,"SD card","not available");
    }
}

// I2C display
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display

void LCDInit(LiquidCrystal_I2C* lcd)
{
    lcd->init();
    lcd->clear();         
    lcd->backlight();

    lcd->setCursor(0,0);
    lcd->print("Dont text");

    lcd->setCursor(0,1);
    lcd->print("while driving!");
}

int CTemp_pin = A15;

const int R1 = 10000;
float ctemp = 0.0;
const int ctemp_n = 20;
float ctemp_sum = 0.0;
float ctemp_arr[ctemp_n] = {0.0};

float ReadCTemp()
{
    int rawData = analogRead(CTemp_pin);
    float voltageOut = rawData * 3.3/1023;
    float thermistorResistance = (voltageOut * R1)/(3.3 - voltageOut);
    float temperatureInCelsius = 1 / ((1.131519670 * pow(10, -3))+ (2.232341523 * pow(10, -4))* log(thermistorResistance) + (1.975828128 * pow(10, -7)) * pow(log(thermistorResistance), 3)) - 273.15;
    return temperatureInCelsius;
}

// PedV and CCV
int ped_v_pin = A17;
int ped_v_val = 0;
int cc_v_pin  = A16;
int cc_v_val = 0;
int brake_pin = 31;
int brake_val = 0;
int cc_arm_pin = 30;
int cc_arm_val = 0;
int cc_status_pin = 32;
int cc_status_val = 0;

int mtr_temp_pin = A14;
int mtr_temp_val = 0;

// Speed and Distance
int WheelCircMM = 1586;
int hall_pulse_pin = 27;
int timeout = 60000;

unsigned long hall_now = 0;
unsigned long hall_last = 0;

unsigned long LTime = 0;
unsigned long HTime = 0;
unsigned long TTime = 0;

unsigned long hallpulses = 0;

byte total_distance_arr[4] = {};
unsigned long total_distance = 0;
unsigned long total_distance_read = 0;
unsigned long trip_distance = 0;
unsigned long kph = 0;

unsigned long hall_factor = 35685; // 1586/16 * 3.6

float Pulses_PerMS = 0.0;
float HallKPH = 0.0;
float HallKPH_onepulse = 0.0;

void GetHallKPH()
{
    hall_now = millis();
    LTime = pulseIn(hall_pulse_pin, LOW, timeout);
    HTime = pulseIn(hall_pulse_pin, HIGH, timeout);
    TTime = LTime+HTime;
    hallpulses = ((hall_now-hall_last)*1000)/TTime;
    unsigned long pulses = toMM(hallpulses);
    trip_distance += pulses;
    total_distance = total_distance_read + trip_distance;

    float WFreq = 1.0/TTime;

    Log(&Serial,0,"TTime",TTime);
    Log(&Serial,0,"WFreq",WFreq);
    
    hall_last = hall_now;
    HallKPH_onepulse = (float)TTime>0.0 ? WFreq*1586.0*225.0:0.0;
}

unsigned long toMM(unsigned long pulses)
{
    return (pulses*625*WheelCircMM)/10000;
}

// void GetHallInfo()
// {
//     Log(&Serial,0,"Hall pulses",hallpulses);
//     hall_now = millis();
    
//     Pulses_PerMS = (float)hallpulses/(hall_now-hall_last);
//     kph = Pulses_PerMS * hall_factor;

//     trip_distance += toMM(hallpulses);
//     hall_last = hall_now;
//     hallpulses = 0;
// }

void distance() {
    hallpulses++;
}

void rc_exc(MySQL_Connection* mysql_conn)
{
    exc_cursor = mysql_execute(mysql_conn, "SELECT CCV, Message FROM green_car_database.driver WHERE now = 1");
    
    exc_cursor->get_columns();
    row_values *row = NULL;
    do {
        row = exc_cursor->get_next_row();
        if (row != NULL) {
            exc_flag = atol(row->values[0]);
        }
    } while (row != NULL);

    delete exc_cursor;
    delete row;

    if ((exc_flag & TNSY_RESET) == TNSY_RESET)
    {
        reset();
    }
    if ((exc_flag & ALT_RESET) == ALT_RESET)
    {
        // ?
    }
}

float roll_avg(float*s, float*v, float*a, int n)
{
    *s = (*s) - (*(a+n-1))+(*v);
    for (int i=1;i<=n;i++)
    {
        *(a+(n-i))=*(a+(n-i-1));
    }
    *(a)=*v;

    return (*s)/(float)n;
}

unsigned long odo_run = millis();
unsigned long net_run = millis();
unsigned long dat_run = millis();

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

void setup()
{
    // attachInterrupt(digitalPinToInterrupt(TEENSY_RESET_PIN), reset, LOW);
    // attachInterrupt(digitalPinToInterrupt(hall_pulse_pin), distance, FALLING);

    setDebug(true); // printing
    setPriority(0); // printing

    pinMode(4, OUTPUT); // Blue
    pinMode(3, OUTPUT); // Green
    pinMode(2, OUTPUT); // Red

    delay(500);

    Serial.begin(115200);
    Serial.println("  _______ ______ _      ______ __  __ ______ _______ _______     __   _____ ______ _______ _    _ _____  \n |__   __|  ____| |    |  ____|  \\/  |  ____|__   __|  __ \\ \\   / /  / ____|  ____|__   __| |  | |  __ \\ \n    | |  | |__  | |    | |__  | \\  / | |__     | |  | |__) \\ \\_/ /  | (___ | |__     | |  | |  | | |__) |\n    | |  |  __| | |    |  __| | |\\/| |  __|    | |  |  _  / \\   /    \\___ \\|  __|    | |  | |  | |  ___/ \n    | |  | |____| |____| |____| |  | | |____   | |  | | \\ \\  | |     ____) | |____   | |  | |__| | |     \n    |_|  |______|______|______|_|  |_|______|  |_|  |_|  \\_\\ |_|    |_____/|______|  |_|   \\____/|_|     \n                                                                                                         \n                                                                                                         ");

/*
  _______ ______ _      ______ __  __ ______ _______ _______     __   _____ ______ _______ _    _ _____  
 |__   __|  ____| |    |  ____|  \/  |  ____|__   __|  __ \ \   / /  / ____|  ____|__   __| |  | |  __ \ 
    | |  | |__  | |    | |__  | \  / | |__     | |  | |__) \ \_/ /  | (___ | |__     | |  | |  | | |__) |
    | |  |  __| | |    |  __| | |\/| |  __|    | |  |  _  / \   /    \___ \|  __|    | |  | |  | |  ___/ 
    | |  | |____| |____| |____| |  | | |____   | |  | | \ \  | |     ____) | |____   | |  | |__| | |     
    |_|  |______|______|______|_|  |_|______|  |_|  |_|  \_\ |_|    |_____/|______|  |_|   \____/|_|     
                                                                                                        
*/

    // Serial.println(digitalRead(TEENSY_RESET_PIN)); // debugging for if teensy reset pin is pressed at the start

    WirelessInit();
    Log(&Serial,0,"Ethernet","starting");

    // GPS
    GPSInit();
    Serial << gps->fix.dateTime; Serial.println();

    // SD Card
    SDCardInit();

    odometer_read();

    for (int i=0;i<ctemp_n;i++) // obselete
    {
        ctemp = ReadCTemp();
        roll_avg(&ctemp_sum, &ctemp, ctemp_arr, ctemp_n);
    }

    // I2C
    Wire.begin();

    // DPS
    pinMode(25, INPUT_PULLUP); // for DPS310 sensor
    DPSInit();

    // INA226
    INAInit();

    // BMS
    BMSInit();

    // MPPT
    MPPTInit();

    // LCD
    LCDInit(&lcd);
    
    Serial.println("\n\n\n");
    Serial.println("Starting...");
    Serial.println("\n\n\n");
    delay(1000);
}

unsigned long Now = millis();
unsigned long Last = Now;

// DataFrame LastDataFrame; // Last data frame

void loop()
{
    how_long("Loop end time (0)");
    Now = millis();
    DataFrame CurrentDataFrame; // Current data frame

    InsertV(&CurrentDataFrame, "N", &iteration);

    GetHallKPH();
    InsertV(&CurrentDataFrame, "HallKPH_onepulse", &HallKPH_onepulse, 1);
    how_long("Hall KPH time");

    InsertV(&CurrentDataFrame, "file", file);
    
    cc_v_val = analogRead(cc_v_pin)>10?analogRead(cc_v_pin)*74+1828:0;
    ped_v_val = analogRead(ped_v_pin)>10?analogRead(ped_v_pin)*74+1828:0;

    cc_arm_val = digitalRead(cc_arm_pin);
    InsertV(&CurrentDataFrame, "CCArm", &cc_arm_val);

    InsertV(&CurrentDataFrame, "CCV", &cc_v_val);
    InsertV(&CurrentDataFrame, "PedV", &ped_v_val);

    brake_val = digitalRead(brake_pin);
    InsertV(&CurrentDataFrame, "Brake", &brake_val);

    cc_status_val = digitalRead(cc_status_pin);
    InsertV(&CurrentDataFrame, "CCOn", &cc_status_val);

    mtr_temp_val = analogRead(mtr_temp_pin)/507.0*20.26;
    InsertV(&CurrentDataFrame, "MotorTemp", &mtr_temp_val);

    // GetHallInfo();
    InsertV(&CurrentDataFrame, "TripDistance", &trip_distance);
    InsertV(&CurrentDataFrame, "TotalDistance", &total_distance);
    // InsertV(&CurrentDataFrame, "HallKPH", &kph);

    how_long("Misc (cc, brake, trip) time");

    // MPPTRead();
    // Log(&Serial,0,"MPPT can id",frame.can_id);

    // how_long("MPPT time");

    // GPS
    GPSRead();
    
    InsertV(&CurrentDataFrame, "GPSTime", gps->unix_str);
    InsertV(&CurrentDataFrame, "Lat", &gps->lat, 6);
    InsertV(&CurrentDataFrame, "Lon", &gps->lon, 6);
    InsertV(&CurrentDataFrame, "GPSKph", &gps->kph, 1);

    how_long("GPS time");
    
    // INA
    INARead();

    InsertV(&CurrentDataFrame, "SBatA", &SBat->A, 2);
    InsertV(&CurrentDataFrame, "SBatV", &SBat->V, 2);
    InsertV(&CurrentDataFrame, "Mot1A", &Mot1->A, 1);
    InsertV(&CurrentDataFrame, "Mot2A", &Mot2->A, 1);
    InsertV(&CurrentDataFrame, "Sol1A", &Sol1->A, 1);
    InsertV(&CurrentDataFrame, "Sol2A", &Sol2->A, 1);
    InsertV(&CurrentDataFrame, "Sol3A", &Sol3->A, 1);
    InsertV(&CurrentDataFrame, "Sol4A", &Sol4->A, 1);
    InsertV(&CurrentDataFrame, "BattV", &Batt->V, 1);
    InsertV(&CurrentDataFrame, "BattA", &Batt->A, 1);
    
    how_long("INA time");

    // DPS
    DPSRead_P();
    DPSRead_T();

    InsertV(&CurrentDataFrame, "Pressure", &dps->Press, 1);
    InsertV(&CurrentDataFrame, "Temperature", &dps->Temp, 1);

    how_long("DPS Time");

    ctemp = ReadCTemp();

    InsertV(&CurrentDataFrame, "CTemp", &ctemp, 2);

    ctemp = roll_avg(&ctemp_sum, &ctemp, ctemp_arr, ctemp_n);

    InsertV(&CurrentDataFrame, "CTemp_avg", &ctemp, 2);

    how_long("CTemp time");

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

    for (unsigned int i=0;i<bms->n_cells;i++) // maybe bms->ncells
    {
        char name[7];
        sprintf(name, "V%d", i);

        InsertV(&CurrentDataFrame, name, &bms->CellV[i]);
    }

    // InsertV(&CurrentDataFrame, "WheelCircmm", &bms->WheelCircmm);
    // InsertV(&CurrentDataFrame, "MagPairs", &bms->MagPairs);
    // InsertV(&CurrentDataFrame, "RelSwitch", &bms->RelSwitch);
    InsertV(&CurrentDataFrame, "NumCells", &bms->NumCells);
    InsertV(&CurrentDataFrame, "PhysCap", &bms->PhysCap);
    InsertV(&CurrentDataFrame, "RemCap", &bms->RemCap);
    InsertV(&CurrentDataFrame, "CyclCap", &bms->CyclCap);

    how_long("BMS Time");

    int BattV = Batt->V;
    int BattP = Batt->A*BattV;

    InsertV(&CurrentDataFrame, "BattP", &BattP);

    int Sol1P = Sol1->A*BattV;
    int Sol2P = Sol2->A*BattV;
    int Sol3P = Sol3->A*BattV;
    int Sol4P = Sol4->A*BattV;

    int SolrP = Sol1P+Sol2P+Sol3P+Sol4P;
    InsertV(&CurrentDataFrame, "SolrP", &SolrP);

    float SolrA = Sol1->A+Sol2->A+Sol3->A+Sol4->A;
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
            display_cursor = mysql_execute(&mysql_conn, "SELECT CCV, Message FROM green_car_database.driver WHERE now = 1");
            
            display_cursor->get_columns();
            row_values *row = NULL;
            do {
                row = display_cursor->get_next_row();
                if (row != NULL) {
                    display_CCV = atol(row->values[0]);
                    strcpy(display_message, "");
                    strcpy(display_message, row->values[1]);
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print(display_message);
                    lcd.setCursor(0,1);
                    lcd.print("CC: "); lcd.print((float)display_CCV/10.0);
                    strcpy(display_message, "");
                    strcat(display_message, "\"");
                    strcat(display_message, row->values[1]);
                    strcat(display_message, "\"");
                }
            } while (row != NULL);

            delete display_cursor;
            delete row;

            InsertV(&CurrentDataFrame, "display_message", display_message);
            InsertV(&CurrentDataFrame, "display_CCV", &display_CCV);

            ConstructQuery(&CurrentDataFrame, schema, table);
            digitalWrite(4, HIGH); // Blue
            memsafe_mysql_execute(&mysql_conn, CurrentDataFrame.query); // [todo] maybe do something with schema and tabl

            Log(&Serial,0,"Regular Query: ",CurrentDataFrame.query);

            ConstructQuery(&CurrentDataFrame, schema, cumulative_table);
            memsafe_mysql_execute(&mysql_conn, CurrentDataFrame.query);

            Log(&Serial,0,"Cumulative Query: ",CurrentDataFrame.query);

            strcpy(CurrentDataFrame.query, "");
            sprintf(CurrentDataFrame.query, "DELETE FROM green_car_database.get31_data WHERE N < %d;", (int)iteration-200);
            memsafe_mysql_execute(&mysql_conn, CurrentDataFrame.query);
            digitalWrite(4, LOW); // Blue

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

    Serial.println("\n\n\n");
    Last = Now;
}