
//these macros define the splitting of a 4 byte integer
#define fourthByte(w) ((unsigned long)((w) >> 24))
#define thirdByte(w) ((unsigned long)((w) >> 16))
#define secondByte(w) ((unsigned long)((w) >> 8))
#define firstByte(w) ((unsigned long)((w)&0xFF))

byte contrl[4];
byte tele[16];

uint16_t PedV = 750, CCV = 600;

float motorW_float, solarW1_float, solarW2_float, batV_float;
uint32_t motorW_int, solarW1_int, solarW2_int, batV_int;

void setup() {

Serial.begin(9600);
Serial4.begin(19200);

}

void loop() {
  delay(100);
  //updates integers to simulate a fluctuating value
  PedV += 1;
  CCV += 2;


  //splits 2 byte integer into 2 independent bytes
  contrl[0] = highByte(PedV);
  contrl[1] = lowByte(PedV);

  //cruise control voltage
  contrl[2] = highByte(CCV);
  contrl[3] = lowByte(CCV);


  
  





  //intake from telemetry teensy
  if(Serial4.available() > 16){

    //reads the bytes sent from the telemetry teansy
    Serial4.readBytes(tele, 16);
    Serial4.flush();



    //sets the int to the sum of the bytes
    motorW_int = (tele[0] << 24) + (tele[1] << 16) + (tele[2] << 8) + tele[3];

    //solarW1
    solarW1_int = (tele[4] << 24) + (tele[5] << 16) + (tele[6] << 8) + tele[7];

    //solarW2
    solarW2_int = (tele[8] << 24) + (tele[9] << 16) + (tele[10] << 8) + tele[11];

    //batV
    batV_int =(tele[12] << 24) + (tele[13] << 16) + (tele[14] << 8) + tele[15];
    


    //turns the integer back into a float, while keeping all the data
    //motorW
    memcpy(&motorW_float, &motorW_int, 4);

    //solarW1
    memcpy(&solarW1_float, &solarW1_int, 4);

    //solarW2
    memcpy(&solarW2_float, &solarW2_int, 4);

    //batV
    memcpy(&batV_float, &batV_int, 4);



    //prints arrays, and variables that were input
    Serial.print("motorW_float: ");
    Serial.println(motorW_float);

    //solarW1
    Serial.print("solarW1_float: ");
    Serial.println(solarW1_float);

    //solarW2
    Serial.print("solarW2_float: ");
    Serial.println(solarW2_float);

    //batV
    Serial.print("batV_float: ");
    Serial.println(batV_float);

    //array
    for(int i = 0; i < 16; i++){
      Serial.print(tele[i]);
      Serial.print("    ");
    } 




    //output to telemetry teensy
    
    //writes arry to telemetry teensy
    Serial4.write(contrl, 4);
    Serial4.flush();

    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();
    //prints the variables, and arrays that are output
    Serial.print("PedV: ");
    Serial.println(PedV);

    //cruise control voltage
    Serial.print("CCV: ");
    Serial.println(CCV);

    //array
    for(int i = 0; i < 4; i++){
      Serial.print(contrl[i]);
      Serial.print("    ");
      Serial.flush();
    } 
    Serial.println();
    Serial.println();
    
  }
  Serial4.flush();
}