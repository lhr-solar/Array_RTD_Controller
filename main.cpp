// #include <iostream>
#include "mbed.h"
#include "MAX31865.h"
#include <cstdio>

//Serial pc(SERIAL_TX, SERIAL_RX)
MAX31865_RTD rtd0(MAX31865_RTD::RTD_PT100, D11, D12, D13, A1); //miso = master input slave output (SDI on nucleo), 
//RTD parameters: (ptd_type type,PinName mosi, PinName miso, PinName sclk, PinName nss); nss is chip select, negative logic
// main() runs in its own thread in the OS


MAX31865_RTD rtd1(MAX31865_RTD::RTD_PT100, D11, D12, D13, A0); //miso = master input slave output (SDI on nucleo), 
MAX31865_RTD rtd2(MAX31865_RTD::RTD_PT100, D11, D12, D13, D8); //miso = master input slave output (SDI on nucleo),
MAX31865_RTD rtd3(MAX31865_RTD::RTD_PT100, D11, D12, D13, D9); //miso = master input slave output (SDI on nucleo),
MAX31865_RTD rtd4(MAX31865_RTD::RTD_PT100, D11, D12, D13, A3); //miso = master input slave output (SDI on nucleo),
MAX31865_RTD rtd5(MAX31865_RTD::RTD_PT100, D11, D12, D13, D6); //miso = master input slave output (SDI on nucleo),
MAX31865_RTD rtd6(MAX31865_RTD::RTD_PT100, D11, D12, D13, A6); //miso = master input slave output (SDI on nucleo),
MAX31865_RTD rtd7(MAX31865_RTD::RTD_PT100, D11, D12, D13, D3); //miso = master input slave output (SDI on nucleo),

int main()
{
    //when configuring RTDs to test, make sure that ALL RTDs are configured! If not, the default setting of the chip select pins is sitting at 0,
    //which is normally the chip enable signal. When the RTD is configured, the pin idles at high and prevents unwanted data being sent by
    //chips over the SPI bus, as all chips are tied to the same bus.
    
    //configure( bool v_bias, bool conversion_mode, bool one_shot, bool three_wire, uint8_t fault_cycle, bool fault_clear,
                            //   bool filter_50hz, uint16_t low_threshold, uint16_t high_threshold )

    rtd0.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
                   true, true, 0x0000, 0x7fff );
    rtd1.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
                   true, true, 0x0000, 0x7fff );
    // rtd2.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
    //                true, true, 0x0000, 0x7fff );
    rtd3.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
                   true, true, 0x0000, 0x7fff );
    rtd4.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
                   true, true, 0x0000, 0x7fff );
   rtd5.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
                   true, true, 0x0000, 0x7fff );
    rtd6.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
                   true, true, 0x0000, 0x7fff );
    rtd7.configure( true, true, false, false, MAX31865_FAULT_DETECTION_NONE,
                   true, true, 0x0000, 0x7fff );

    double temp0_buffer = 0; // set buffers to maintain values in case 0 or 400 ohms is read (error)
    double temperature0 = 0;
    double temp1_buffer = 0;
    double temperature1 = 0;
    double temp2_buffer = 0;
    double temperature2 = 0;
    double temp3_buffer = 0;
    double temperature3 = 0;
    double temp4_buffer = 0;
    double temperature4 = 0;
    double temp5_buffer = 0;
    double temperature5 = 0;
    double temp6_buffer = 0;
    double temperature6 = 0;
    double temp7_buffer = 0;
    double temperature7 = 0;

    int epochtime = 0;
    char epoch_buffer[11];
    int index = 0;
    BufferedSerial pc(USBTX, USBRX,19200);
    //printf("Enter a 10-digit decimal value for epoch time. Press j to enter.");
    bool waiting = true;
    do {
        char myChar;
        pc.read(&myChar, 1);
        pc.write(&myChar, 1);
        if (myChar == 'j') waiting = false;
        else {
            /* Put your character into epoch_buffer... assuming your data is sanitized, punk */
            epoch_buffer[index] = myChar;
            index++;
        }
    } while (waiting);
    epochtime = atoi(epoch_buffer);
    //char hello[] = "hello world\n\r";
    //pc.write(hello, 12);

    set_time(epochtime);
    osDelay(500);

     time_t seconds = time(NULL);
     printf("Time as seconds since January 1, 1970 = %d\n", seconds);
     printf("Time as a basic string = %s", ctime(&seconds));
     char buffer[32];
     strftime(buffer, 32, "%I:%M %p\n", localtime(&seconds));
     printf(" %s", buffer);
    osDelay(500);
    
    // static BufferedSerial pc(USBTX, USBRX, 19200);
    // char epochtime[] = "1647099000";
    // int epoch;
    // pc.read(epochtime,11);
    // sscanf(epochtime, "%d", &epoch);
    // printf("%i\n\r",epoch);
    // set_time(epoch);




    

    while (1) {
    
        time_t seconds = time(NULL);
        printf("%s\n\r", ctime(&seconds));
        //if( rtd.status( ) == 0 ) {

            //rtd0 is good

            rtd0.read_all( );
            temp0_buffer = rtd0.temperature( );
            if(temp0_buffer > -10.0 && temp0_buffer < 150.0){
                temperature0 = temp0_buffer;
            }
            printf( " T0 = %f deg C \n\r",temperature0);
            double resistance0 = rtd0.resistance();
            printf("Resistance0 is %f \n", resistance0);

            // osDelay(1000);
            
 
        // } else
        // {
        //     printf( "RTD fault register: %d :\r\n",rtd.status( ));
        //     if( rtd.status( ) & MAX31865_FAULT_HIGH_THRESHOLD ) {
        //         printf( "RTD high threshold exceeded\r\n" );
        //     } else if( rtd.status( ) & MAX31865_FAULT_LOW_THRESHOLD ) {
        //         printf( "RTD low threshold exceeded\r\n" );
        //     } else if( rtd.status( ) & MAX31865_FAULT_REFIN ) {
        //         printf( "REFIN- > 0.85 x V_BIAS\r\n" );
        //     } else if( rtd.status( ) & MAX31865_FAULT_REFIN_FORCE ) {
        //         printf( "REFIN- < 0.85 x V_BIAS, FORCE- open\r\n" );
        //     } else if( rtd.status( ) & MAX31865_FAULT_RTDIN_FORCE ) {
        //         printf( "RTDIN- < 0.85 x V_BIAS, FORCE- open\r\n" );
        //     } else if( rtd.status( ) & MAX31865_FAULT_VOLTAGE ) {
        //         printf( "Overvoltage/undervoltage fault\r\n");
        //     } else {
        //         printf( "Unknown fault; check connection\r\n" );
        //     }
        // }
        
        //rtd 1 good
        rtd1.read_all( );
        temp1_buffer = rtd1.temperature( );
        if(temp1_buffer > -10.0 && temp1_buffer < 150.0){
            temperature1 = temp1_buffer;
        }
        printf( " T1 = %f deg C \n\r",temperature1);
        double resistance1 = rtd1.resistance();
        printf("Resistance1 is %f \n", resistance1);

        //need new board
        // rtd2.read_all( );
        // temp2_buffer = rtd2.temperature( );
        // if(temp2_buffer > -10.0 && temp2_buffer < 150.0){
        //   temperature2 = temp2_buffer;
        //}
        // printf( " T2 = %f deg C \n\r",temperature2);
        // double resistance2 = rtd2.resistance();
        // printf("Resistance2 is %f \n", resistance2);

        //rtd3 is good
        rtd3.read_all( );
        temp3_buffer = rtd3.temperature( );
        if(temp3_buffer > -10.0 && temp3_buffer < 150.0){
            temperature3 = temp3_buffer;
        }
        printf( " T3 = %f deg C \n\r",temperature3);
        double resistance3 = rtd3.resistance();
        printf("Resistance3 is %f \n", resistance3);        
        
        
        // rtd4.read_all( );   //rtd4 is good
        // temp4_buffer = rtd4.temperature( );
        // if(temp4_buffer > -10.0 && temp4_buffer < 150.0){
        //     temperature4 = temp4_buffer;
        // }
        // printf( " T4 = %f deg C \n\r",temperature4);
        // double resistance4 = rtd4.resistance();
        // printf("Resistance4 is %f \n", resistance4);

        
        // // rtd5 is good
        // rtd5.read_all( );
        // temp5_buffer = rtd5.temperature( );
        // if(temp5_buffer > -10.0 && temp5_buffer < 150.0){
        //     temperature5 = temp5_buffer;
        // }
        // printf( " T5 = %f deg C \n\r",temperature5);
        // double resistance5 = rtd5.resistance();
        // printf("Resistance5 is %f \n", resistance5);

        // // rtd6 is good
        // rtd6.read_all( );
        // temp6_buffer = rtd6.temperature( );
        // if(temp6_buffer > -10.0 && temp6_buffer < 150.0){
        //     temperature6 = temp6_buffer;
        // }
        // printf( " T6 = %f deg C \n\r",temperature6);
        // double resistance6 = rtd6.resistance();
        // printf("Resistance6 is %f \n", resistance6);

        // // rtd7 is good
        // rtd7.read_all( );
        // temp7_buffer = rtd7.temperature( );
        // if(temp7_buffer > -10.0 && temp7_buffer < 150.0){
        //     temperature7 = temp7_buffer;
        // }
        // printf( " T7 = %f deg C \n\r",temperature7);
        // double resistance7 = rtd7.resistance();
        // printf("Resistance7 is %f \n", resistance7);
        

        printf("\n");
        osDelay(1000);
 
    }
}

