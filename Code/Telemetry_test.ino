
//these macros define the splitting of a 4 byte integer
#define fourthByte(w) ((uint8_t)((w) >> 24))
#define thirdByte(w) ((uint8_t)((w) >> 16))
#define secondByte(w) ((uint8_t)((w) >> 8))
#define firstByte(w) ((uint8_t)((w)&0xFF))

byte contrl[4];
byte tele[16];

uint16_t PedV = 0, CCV = 0;

float motorW_float = 0, solarW1_float = 0, solarW2_float = 0, batV_float = 0;
uint32_t motorW_int, solarW1_int, solarW2_int, batV_int;

void setup() {

Serial.begin(9600);
Serial4.begin(19200);

}


void loop() {
  Serial.println();
  Serial.println();

  //update floats to simulate a fluctuating value
  motorW_float += .01;
  solarW1_float += .02;
  solarW2_float += .04;
  batV_float += .08;



  //turns a float into an integer, while keeping all the data
  memcpy(&motorW_int, &motorW_float, 4);

  //solarW1
  memcpy(&solarW1_int, &solarW1_float, 4);

  //solarW2
  memcpy(&solarW2_int, &solarW2_float, 4);

  //batV
  memcpy(&batV_int, &batV_float, 4);



  //splits integers into individual bytes, inside an array
  tele[0] = fourthByte(motorW_int);
  tele[1] = thirdByte(motorW_int);
  tele[2] = secondByte(motorW_int);
  tele[3] = firstByte(motorW_int);

  //splits solarW1
  tele[4] = fourthByte(solarW1_int);
  tele[5] = thirdByte(solarW1_int);
  tele[6] = secondByte(solarW1_int);
  tele[7] = firstByte(solarW1_int);

  //splits solarW2
  tele[8] = fourthByte(solarW2_int);
  tele[9] = thirdByte(solarW2_int);
  tele[10] = secondByte(solarW2_int);
  tele[11] = firstByte(solarW2_int);

  //splits batV
  tele[12] = fourthByte(batV_int);
  tele[13] = thirdByte(batV_int);
  tele[14] = secondByte(batV_int);
  tele[15] = firstByte(batV_int);



  //intake from control teensy
  if(Serial4.available() > 4){

    //reads the bytes sent from the control teensy
    Serial4.readBytes(contrl, 4);
    Serial4.flush();


    //sets intake variable to the sum of two bytes, one of which is shifted left 8 bits
    PedV = (contrl[0] << 8) + contrl[1];
    CCV = (contrl[2] << 8) + contrl[3];


    //prints variables found (should move, or delete when implemented into main code)
    Serial.print("PedV: ");
    Serial.println(PedV);

    //cruise control voltage
    Serial.print("CCV: ");
    Serial.println(CCV);

    //prints entire array
    for(int i = 0; i < 4; i++){
      Serial.print(contrl[i]);
      Serial.print("    "); 
    }
    Serial.println();
  }





  //output to control teensy
  if(Serial4.availableForWrite() > 16){

    //writes array to control teensy
    Serial4.write(tele, 16);
    Serial4.flush();


    //prints array, and variables
    Serial.println();
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
    Serial.flush();
    for(int i = 0; i < 16; i++){
      Serial.print(tele[i]);
      Serial.print("    ");
      Serial.flush();
    }
  }
  delay(100);
}