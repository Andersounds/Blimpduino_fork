// Functionality for i2c communication with a rpi configured as i2c slave
// Read/Write from/to rpi

#include <Wire.h>
/*
Legend over the wire-library, when used as master:
--Function call---                    ---Explanation---
Wire.begin(); (no argument)           To enter the i2c bus as master. (with argument bus is joined as slave
Wire.read();                          Read one byte that was recieved from a slave after a requestFrom()- call
Wire.write();                         Write the data to be sent to the buffer. Is framed by Wire.beginTransmission() and Wire.endTransmission().
                                      When used as slave to write to master it is done after requestFrom()- call
  Wire.write(value);                  -write one byte 
  Wire.write(string);                 -write a string as a series of bytes 
  Wire.write(data,length);            -write an array of data of size length-bytes
Wire.beginTransmission(address);      Start a transmission over i2c to the given address. Next, fill the buffer using Wire.write as above.
Wire.endTransmission();               End transmission. When this is called, the data contained in the buffer is physically transmitted over the bus
Wire.onRequest();                     -NOT USED BY MASTER- This gets called from a slave if a master request data
Wire.onReceive();                     -NOT USED BY MASTER- This gets called from a slave if a master sends data
Wire.requestFrom(address,quantity);   This is used by master to request data from a slave
*/


// I2C Addres of RPI slave
byte RPI_I2C_ADDRESS = 0x03;
//Encode, decode scales
float scales_i2c[4] = {1,10,100,1000};
int rpi_encode_scale = 2; //The scale used when encoding floats that are sent to rpi




int writeOneByteToRpi(byte data){
    size_t dataLength = 1;
    int status;
    Wire.beginTransmission(RPI_I2C_ADDRESS);// Take control of bus and prepare to sent to rpi
    status = Wire.write(&data, dataLength);          // Write to buffer.
    status = Wire.endTransmission(true);             // Send the buffer content to rpi and release bus
    return status; 
}



int writeToRpi(char info,int16_t roll,int16_t pitch,int16_t height_s){//dim(roll, pitch): [rad*100], dim(height):[cm]
    // Bit shifting of signed integers have undefined behaviour.
    // Send an additional byte called info where the sign of the variables are encoded, possibly together with additional info
  
    //the three lowest bits of the info byte encode the sign of the variables  if(x>0):1, if(x<0):0
    //Mask to keep the upper 5 bits and set the lower 3 bits according to the sign of the values
    uint8_t info_sign =(info&0xF8)|((roll>0)<<2 | (pitch>0)<<1 | (height>0));
    //Construct a byte-array of length 7 with info and unsigned high byte, low byte of roll, pitch, height
    char data[7] = {(char)info_sign,
                    (char)(abs(roll)>>8), ((char)roll&0xFF),
                    (char)(abs(pitch)>>8), ((char)pitch&0xFF),
                    (char)(abs(height)>>8), ((char)height&0xFF)};
 
 uint8_t data_uint8 = 114;

 uint8_t data_arr[3] = {114,56,200};
SerialUSB.print("Laser height: "); SerialUSB.print(height,BIN);SerialUSB.print('\n');
//SerialUSB.print("HB: "); SerialUSB.print(data[4],BIN);SerialUSB.print('\n');
//SerialUSB.print("LB: "); SerialUSB.print(data[5],BIN);SerialUSB.print('\n');

    //  uint16_t hb = data[4]<<8;
     // uint16_t lb = data[5];
      //uint16_t rebuild = hb | lb;
//SerialUSB.print("Rebuild: ");     SerialUSB.print(rebuild,BIN);

      
      //SerialUSB.println("\n");  
    size_t dataLength = 6;//Hardcoded length
    int status;
    Wire.beginTransmission(RPI_I2C_ADDRESS);// Take control of bus and prepare to sent to rpi
    for(int i=0;i<7;i++){
        //status = Wire.write(data[i]);        // Write one byte to buffer
        status = Wire.write(&data[i],1);//
    }
    //status = Wire.write(charArray);
    //status = Wire.write(h);
    //status = Wire.write(&data_uint8,1);//DEnna funkar
    /*status = Wire.write(&data_arr[0],1);//Dessa också
    status = Wire.write(&data_arr[1],1);//
    status = Wire.write(&data_arr[2],1);//
    */
 
    status = Wire.endTransmission(true);             // Send the buffer content to rpi and release bus
    return status; 
}


/*
//Info byte
07 06 05 04 03 02 01 00
-  -  A  A  A  S  S  ID
//Sign byte
07 06 05 04 03 02 01 00
            S3 S2 S1 ID
//Data bytes
07 - 01              00
<data>               ID
//Legend
ID:     bit identifying the info-byte. 1: info byte, 0: data or sign byte
S:      bits encoding the scale of floats. 00: 1, 01:10, 10:100, 11:1000
A:      bits encoding the message length. 1-7 bytes.
SX:     Bit identifying the sign of float X. 1: neg, 0: pos

Each float is encoded as 7 high bits and 7 low bits in that order.
*/



//int encodeData()

//int decodeData()

int writeToRpi(float* msg, int sizeOf){//does not have to be float!
    if(sizeOf>16){return -1;}//Too large value
    uint8_t txbuffer[16];
    uint8_t infoByte = (uint8_t (rpi_encode_scale<<1))|0b1; //Info bit with encoded scale code and info byte identification bit
    infoByte|= ((uint8_t)sizeOf<<3); //Encode message size in info bit (0-7 bytes)
    uint8_t signByte = 0;
    int bufferIndex = 2;//start value of data fields in tx buffer
    int size_of_tx_msg = sizeOf*2+2;//Size of complete message in bytes
    for(int i=0;i<sizeOf;i++){
        uint16_t unsignedScaleValue = (uint16_t)abs( (int)(msg[i]*scales_i2c[rpi_encode_scale]) );//Scale, remove sign, and cast the float
        uint8_t HB = (unsignedScaleValue>>7)&0xFE;
        uint8_t LB = (unsignedScaleValue<<1)&0xFE;
        signByte|=(uint8_t)((msg[i]<0)<<i);//Set sign bit in sign byte
        txbuffer[bufferIndex] = HB;
        txbuffer[bufferIndex+1] = LB;
        bufferIndex+=2;
    }
    txbuffer[0] = infoByte;
    txbuffer[1] = signByte;
//Write to buffer
    Wire.beginTransmission(RPI_I2C_ADDRESS);// Take control of bus and prepare to sent to rpi
    int status = Wire.write(txbuffer,size_of_tx_msg);//Or do not have &? or give txbuffer[0] ? should be same
    status = Wire.endTransmission(true);             // Send the buffer content to rpi and release bus
    return status;
}

//int readFromRpi()
