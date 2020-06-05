#include "mbed.h"
#include "canIds.h"
#include "CAN.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

//Pin definitions
AnalogIn RTD0(A0);      //Analog lines from RTD sensors
AnalogIn RTD1(A1);
AnalogIn RTD2(A2);
AnalogIn RTD3(A3);
AnalogIn RTD4(A4);
AnalogIn RTD5(A5);
AnalogIn RTD6(A6);
AnalogIn RTD7(A7); 

CAN can(D10, D2); //D10 is Rx, D2 is Tx
Serial pc(USBTX, USBRX);
int maxTemp; 

int convertADC(AnalogIn RTD){
    float temperature;
    float floatRTD = (float)RTD;
    float maxTemp = 200;          //Temporary for now, Matthew will ask Hallock about later
    temperature = (floatRTD*maxTemp)/4096;
    
    return temperature;
}

int main() {
    
    const int RTD_ID = 0x620; 
    
    float RTD0temp;
    float RTD1temp;
    float RTD2temp;
    float RTD3temp;
    float RTD4temp;
    float RTD5temp;
    float RTD6temp;
    float RTD7temp;

        while(1){
            
            RTD0temp = convertADC(RTD0);
            RTD1temp = convertADC(RTD1);
            RTD2temp = convertADC(RTD2);
            RTD3temp = convertADC(RTD3);
            RTD4temp = convertADC(RTD4);
            RTD5temp = convertADC(RTD5);
            RTD6temp = convertADC(RTD6);
            RTD7temp = convertADC(RTD7);
        
            can.write(CANMessage(RTD_ID, (char*)(&RTD0temp),12));
            can.write(CANMessage(RTD_ID, (char*)(&RTD1temp),12));
            can.write(CANMessage(RTD_ID, (char*)(&RTD2temp),12));
            can.write(CANMessage(RTD_ID, (char*)(&RTD3temp),12));
            can.write(CANMessage(RTD_ID, (char*)(&RTD4temp),12));
            can.write(CANMessage(RTD_ID, (char*)(&RTD5temp),12));
            can.write(CANMessage(RTD_ID, (char*)(&RTD6temp),12));
            can.write(CANMessage(RTD_ID, (char*)(&RTD7temp),12));
            
            pc.printf("RTD0: %f\n", RTD0temp);
            pc.printf("RTD1: %f\n", RTD0temp);
            pc.printf("RTD2: %f\n", RTD0temp);
            pc.printf("RTD3: %f\n", RTD0temp);
            pc.printf("RTD4: %f\n", RTD0temp);
            pc.printf("RTD5: %f\n", RTD0temp);
            pc.printf("RTD6: %f\n", RTD0temp);
            pc.printf("RTD7: %f\n", RTD0temp);
            
            wait(.5);
            
        //A7-A0 = RTD
        //Send message of 7 every half second
        //can.write should send message w/o having to worry
        //Message ID, data (char buffer), length of char buffer
        //ID for Can Bus, (8 byte index of RTD)(rest of data floating point), length of char buffer
        
        //can.write(CANMessage(msgId, data, len))
        //Max 5 V tolerance, 0-3.3 V read
        }
}

