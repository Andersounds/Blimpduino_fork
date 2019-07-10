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
//Rpi slave register map
#define RPI_I2C_PITCH    0x01   // W
#define RPI_I2C_ROLL     0x02   // W
#define RPI_I2C_YAW      0x02   // R
#define RPI_I2C_X        0x02   // R
#define RPI_I2C_Y        0x02   // R
#define RPI_I2C_Z        0x02   // R

#define RPI_I2C_LOW_BAT  0x02   // 
#define RPI_I2C_HDNG     0x02   // 


//Maybe send a struct like this?
  struct
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;


int writeOneByteToRpi(byte data){
    size_t dataLength = 1;
    int status;
    Wire.beginTransmission(RPI_I2C_ADDRESS);// Take control of bus and prepare to sent to rpi
    status = Wire.write(&data, dataLength);          // Write to buffer.
    status = Wire.endTransmission(true);             // Send the buffer content to rpi and release bus
    return status; 
}



int writeToRpi(int16_t roll,int16_t pitch,int16_t height_s){//dim(roll, pitch): [rad*100], dim(height):[cm]
    // Bit shifting of signed integers have undefined behaviour.
    // Send an additional byte called info where the sign of the variables are encoded, possibly together with additional info
  
    //the three lowest bits of the info byte encode the sign of the variables  if(x>0):1, if(x<0):0
    uint8_t info =(roll>0)<<2 | (pitch>0)<<1 | (height>0);
    //Construct a byte-array of length 7 with info and unsigned high byte, low byte of roll, pitch, height
    char data[7] = {info,
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
