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
byte RPI_I2C_ADDRESS = 0x04;
//Encode, decode scales
float scales_i2c[4] = {1,10,100,1000};
uint8_t rpi_encode_scale = 3; //The scale used when encoding floats that are sent to rpi




int writeOneByteToRpi(byte data){
    size_t dataLength = 1;
    int status;
    Wire.beginTransmission(RPI_I2C_ADDRESS);// Take control of bus and prepare to sent to rpi
    status = Wire.write(&data, dataLength);          // Write to buffer.
    status = Wire.endTransmission(true);             // Send the buffer content to rpi and release bus
    return status; 
}

// Encodes an array of floats to a series of bytes and sends to rpi over i2c. 
// TODO: Specify the message identifier so that different messages can be sent and decoded properly
int writeToRpi(float* msg, int sizeOf){//does not have to be float!
    if(sizeOf>7){return -1;}//Too large value
    uint8_t txbuffer[16];
    uint8_t infoByte = 0b1; //Info byte with info byte identification bit
    infoByte|=((uint8_t)(rpi_encode_scale<<1));//Encode scale
    infoByte|= ((uint8_t)sizeOf<<3); //Encode message size in info bit (0-7 bytes)
    uint8_t signByte = 0;
    int bufferIndex = 2;//start value of data fields in tx buffer
    int size_of_tx_msg = sizeOf*2+2;//Size of complete message in bytes
    for(int i=0;i<sizeOf;i++){
        uint16_t unsignedScaleValue = (uint16_t)abs( (int)(msg[i]*scales_i2c[rpi_encode_scale]) );//Scale, remove sign, and cast the float
        uint8_t HB = (uint8_t)(unsignedScaleValue>>6)&0xFE;
        uint8_t LB = (uint8_t)(unsignedScaleValue<<1)&0xFE;
        signByte|=(uint8_t)((msg[i]<0)<<i+1);//Set sign bit in sign byte (+1 because of info ID bit at bit 0)
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


// Reads i2c message from rpi and decodes it into a series of floats. Returns the number of successfully decoded floats
// TODO: Read what message has been recieved from info byte and decode accordingly.
int readFromRpi(float* msg){
    //float recieved_values[3];
    int bytes = Wire.requestFrom(RPI_I2C_ADDRESS, 16);//Request everything that the rpi has. If buffer is not full then last byte will repeat
    if(bytes<=0){SerialUSB.println("No answer from rpi. releasing bus and moving on\n");return 0;}
    uint8_t rx_buffer[16];
    int info_byte_index = -1; //If this value is kept then info byte has not been found
    int sgn_byte_index;
    int msgSize = 0;
    int i = 0;
    while(Wire.available()){//Empty the whole buffer
        rx_buffer[i] = Wire.read();       //Read one byte to buffer
        uint8_t rx_byte = rx_buffer[i];
        if(rx_byte&0b1){                  //Check if it is the info byte
            //Here there should be a check of what message type it is and a switch-case that either reads at uint8:s or floats. not necessary yet
            int msgSize_temp = (int)((rx_byte>>3)&0b111); //Decode message size (In no of floats)
            if((i+1+2*msgSize_temp)<16){                  //Check if whole message can be decoded. i.e. if the complete message of the this info byte can fit in the rest of the buffer.
                info_byte_index = i;          //Set info byte index
                sgn_byte_index = i+1;         //Set info byte index
                msgSize = msgSize_temp;
            }//Can do an additional check here. If above is false, and info_byte_index == -1 i.e there are no other messages, then return another error code.
        }
        i++;
    }
    if(info_byte_index==-1){return -3;}
    uint8_t info_byte = rx_buffer[info_byte_index];
    uint8_t sgn_byte = rx_buffer[sgn_byte_index];
    int scale_id = (int)((info_byte>>1)&0b11);//Get scale code
    float scale_decode = scales_i2c[scale_id];//Get corresponding actual scale
    int sign_index = 0;//
    for(int floatindex=2;floatindex<=(msgSize*2);floatindex+=2){
        int i = info_byte_index + floatindex;//Index of HB of the float. LB index is i+1
        int scaledValue = (int)((rx_buffer[i]<<6)|(rx_buffer[i+1]>>1));//build HB and LB to a int
        float value = ((float)scaledValue)/scale_decode;
        if((sgn_byte>>sign_index)&0b1){//Flip sign to neg if it is stated
            value*=-1;
        }
        msg[sign_index] = value;
        sign_index++;
    }
    return (sign_index);//Amount of decoded floats. One more than sign index of last. it is incremented
}


// Ready function to send message stucture 00 (0) to rpi. Argument is the execution period in milliseconds
int send_msg_rpi_00(int refreshRate){
    static int count_print = 0;
    static long timer = 0; //Setting timer
    count_print ++;
    int stat = 100; //Return status 100 if it is not yet time to send msg
    if((millis() - timer) > refreshRate){
        //Calculate dist and approcimated height, scaled to meters
        float lidar_dist_m =(float)height/1000.0; // [m]
        float rpi_height_m = ((float)rpi_height )/1000.0;
        // height: distance measurement from lidar [mm] (filtered)
        // lidar_dist_m : distance measurement from lidar [m] (filtered and scaled)
        // rpi_height: height estimation. (normalizes away tilt from lidar dist and filters value)
        float messageRPI[5] = {pitch_rpi,roll_rpi,lidar_dist_m,rpi_height_m,(float)BatteryValue};
        int stat = writeToRpi(messageRPI,5);
        if(count_print > 1000){  
            switch(stat){
                case 0:{SerialUSB.println("Success wrote to pi!");break;}
                case 1:{SerialUSB.println("Data too long for tx buffer");break;}
                case 2:{SerialUSB.println("Recieved NACK on transmit of address");break;}
                case 3:{SerialUSB.println("Recieved NACK on transmit of data");break;}
                case 4:{SerialUSB.println("Some error in i2c transmit");break;}
            }   
            SerialUSB.print(roll_rpi*RAD2GRAD);
            SerialUSB.print("\t");
            SerialUSB.println(pitch_rpi*RAD2GRAD);
            count_print = 0;
        }
    }
    timer = millis();//Reset timer
    return stat;
}





/*     ====================== MESSAGE DEFINITIONS =======================
//Info byte
07 06 05 04 03 02 01 00
D  D  A  A  A  S  S  ID
//Sign byte
07 06 05 04 03 02 01 00
S7 S6 S5 S4 S3 S2 S1 ID
//Data bytes
07 06 05 04 03 02 01 00
        <data>       ID
//Legend
ID:     bit identifying the info-byte. 1: info byte, 0: data or sign byte
S:      bits encoding the scale of floats. 00: 1, 01:10, 10:100, 11:1000
A:      bits encoding the message length. 1-7 floats (7 floats = 14 unsigned bytes + sgn byte + info byte = 16 bytes.
D:      bits identifying which message it is (0-3). Can specify before that 0: 3 floats in certain order, 1: 2 ints etc.
SX:     Bit identifying the sign of float X. 1: neg, 0: pos

Each float is encoded as 7 high bits and 7 low bits in that order.


DD message number specification:
bin     dec     description
00       0
                byte 0:  info byte
                byte 1:  sign byte
    byte 2:  pitch (rad) HB
                byte 3:  pitch (rad) LB
                byte 4:  roll (rad) HB
                byte 5:  roll (rad) LB
                byte 6:  lidar dist HB
                byte 7:  lidar dist LB
                byte 8:  height est HB
                byte 9:  height est LB
    byte 10: atsam batt HB
                byte 11: atsam batt LB
01       1
10       2
11       3
*/
