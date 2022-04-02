/**************************************************************************
 * Arduino driver library for the MAX31865.
 *
 * Copyright (C) 2015 Ole Wolf <wolf@blazingangles.com>
 *
 *
 * Wire the circuit as follows, assuming that level converters have been
 * added for the 3.3V signals:
 *
 *    Arduino Uno            -->  MAX31865
 *    ------------------------------------
 *    CS: any available pin  -->  CS
 *    MOSI: pin 11           -->  SDI (mandatory for hardware SPI)
 *    MISO: pin 12           -->  SDO (mandatory for hardware SPI)
 *    SCK: pin 13            -->  SCLK (mandatory for hardware SPI)
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
  Edited by Elijah Pustilnik, Spring 2022
**************************************************************************/

//Max31865 datasheet located at https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf

//#include <Arduino.h>
//#include <SPI.h>
#include <MAX31865.h>
#include "mbed.h"

/**
 * The constructor for the MAX31865_RTD class registers the CS pin and
 * configures it as an output.
 *
 * @param [in] cs_pin Arduino pin selected for the CS signal.
 */
MAX31865_RTD::MAX31865_RTD( ptd_type type,PinName mosi, PinName miso, PinName sclk, PinName nss)
    :spi( mosi, miso, sclk ),
     nss( nss )
     

{
  /* Set the type of PTD. */
      this->type = type;
      
      spi.format(8,3); //8 bit messages, CPOL/SPHA mode set to 3 (11), see SPI.h
      //SPI clock polarity/phase (CPOL & CPHA) is set to 11 in spi.format (bit 1 = polarity, bit 0 = phase, see SPI.h)
      //polarity of 1 indicates that the SPI reading idles high (default setting is 1; polarity of 0 means idle is 0)
      //phase of 1 indicates that data is read on the first edge/low-to-high leg (as opposed to phase 0,
      //which reads data on the second edge/high-to-low transition)
      //see https://deardevices.com/2020/05/31/spi-cpol-cpha/ to understand CPOL/CPHA
      spi.frequency(100000); 
    
    //nss = 1;
  /* CS pin for the SPI device. */
  //this->cs_pin = cs_pin;
  //pinMode( this->cs_pin, OUTPUT );

  /* Pull the CS pin high to avoid conflicts on SPI bus. */
//   nss = 1;
  //nss = type;
}



/**
 * Configure the MAX31865.  The parameters correspond to Table 2 in the MAX31865
 * datasheet.  The parameters are combined into a control bit-field that is stored
 * internally in the class for later reconfiguration, as are the fault threshold values.
 *
 * @param [in] v_bias Vbias enabled (@a true) or disabled (@a false).
 * @param [in] conversion_mode Conversion mode auto (@a true) or off (@a false).
 * @param [in] one_shot 1-shot measurement enabled (@a true) or disabled (@a false).
 * @param [in] three_wire 3-wire enabled (@a true) or 2-wire/4-wire (@a false).
 * @param [in] fault_detection Fault detection cycle control (see Table 3 in the MAX31865
 *             datasheet).
 * @param [in] fault_clear Fault status auto-clear (@a true) or manual clear (@a false).
 * @param [in] filter_50hz 50 Hz filter enabled (@a true) or 60 Hz filter enabled
 *             (@a false).
 * @param [in] low_threshold Low fault threshold.
 * @param [in] high_threshold High fault threshold.
*/
void MAX31865_RTD::configure( bool v_bias, bool conversion_mode, bool one_shot,
                              bool three_wire, uint8_t fault_cycle, bool fault_clear,
                              bool filter_50hz, uint16_t low_threshold,
                              uint16_t high_threshold )
{
  uint8_t control_bits = 0;
   nss = 1;
  /* Assemble the control bit mask. */
  control_bits |= ( v_bias ? 0x80 : 0 );
  control_bits |= ( conversion_mode ? 0x40 : 0 );
  control_bits |= ( one_shot ? 0x20 : 0 );
  control_bits |= ( three_wire ? 0x10 : 0 );
  control_bits |= fault_cycle & 0b00001100;
  control_bits |= ( fault_clear ? 0x02 : 0 );   //fault clear auto resets to 0 upon each new cycle
  control_bits |= ( filter_50hz ? 0x01 : 0 );

  /* Store the control bits and the fault threshold limits for reconfiguration
     purposes. */
  this->configuration_control_bits   = control_bits;
  this->configuration_low_threshold  = low_threshold;
  this->configuration_high_threshold = high_threshold;

  /* Perform an initial "reconfiguration." */
  reconfigure( );
}



/**
 * Reconfigure the MAX31865 by writing the stored control bits and the stored fault
 * threshold values back to the chip.
 */ 

 //must edit reconfigure to be 2, 3, or 4-wire mode
void MAX31865_RTD::reconfigure( )
{
  /* Write the configuration to the MAX31865. */

  spi.select();

  nss = 0;      //Chip select is negative logic;
  wait_us(100);
  spi.write( 0x80 );    //configuration write register is address 0x80
  spi.write( this->configuration_control_bits);
  nss = 1;

  /* Write the threshold values. */
  nss = 0;
 // wait_us(100);
  spi.write( 0x83 );    //threshold registers start from 0x83
  spi.write( ( this->configuration_high_threshold >> 8 ) & 0x00ff ); //MSBs of high threshold get written to MSB high register
  spi.write(   this->configuration_high_threshold        & 0x00ff );    //LSBs of high threshold get written to MSB high register
  spi.write( ( this->configuration_low_threshold >> 8 ) & 0x00ff );     //MSBs of low threshold get written to MSB high register
  spi.write(   this->configuration_low_threshold        & 0x00ff );     //lSBs of low threshold get written to MSB high register
  nss = 1;
  
  //relinquish control of SPI bus
  spi.deselect();
}



/**
 * Apply the Callendar-Van Dusen equation to convert the RTD resistance
 * to temperature:
 *
 *   \f[
 *   t=\frac{-A\pm \sqrt{A^2-4B\left(1-\frac{R_t}{R_0}\right)}}{2B}
 *   \f],
 *
 * where
 *
 * \f$A\f$ and \f$B\f$ are the RTD coefficients, \f$R_t\f$ is the current
 * resistance of the RTD, and \f$R_0\f$ is the resistance of the RTD at 0
 * degrees Celcius.
 *
 * For more information on measuring with an RTD, see:
 * <http://newton.ex.ac.uk/teaching/CDHW/Sensors/an046.pdf>.
 *
 * @param [in] resistance The measured RTD resistance.
 * @return Temperature in degrees Celcius.
 */
double MAX31865_RTD::temperature( ) const
{
  static const double a2   = 2.0 * RTD_B;
  static const double b_sq = RTD_A * RTD_A;

  const double rtd_resistance =
    ( this->type == RTD_PT100 ) ? RTD_RESISTANCE_PT100 : RTD_RESISTANCE_PT1000;

  double c = 1.0 - resistance( ) / rtd_resistance;
  double D = b_sq - 2.0 * a2 * c;
  double temperature_deg_C = ( -RTD_A + sqrt( D ) ) / a2;

  return( temperature_deg_C );
}



/**
 * Read all settings and measurements from the MAX31865 and store them
 * internally in the class.
 *
 * @return Fault status byte
 */
uint8_t MAX31865_RTD::read_all( )
{
  uint16_t combined_bytes = 0;

  //SPI clock polarity/phase (CPOL & CPHA) is set to 11 in spi.format (bit 1 = polarity, bit 0 = phase, see SPI.h)
  //polarity of 1 indicates that the SPI reading idles high (default setting is 1; polarity of 0 means idle is 0)
  //phase of 1 indicates that data is read on the first edge/low-to-high leg (as opposed to phase 0,
  //which reads data on the second edge/high-to-low transition)
  //see https://deardevices.com/2020/05/31/spi-cpol-cpha/ to understand CPOL/CPHA

  //chip select is negative logic, idles at 1
  //When chip select is set to 0, the chip is then waiting for a value to be written over SPI
  //That value represents the first register that it reads from
  //registers to read from are from addresses 00h to 07h (h = hex, so 0x01, 0x02, etc)
  //00 = configuration register, 01 = MSBs of resistance value, 02 = LSBs of
  //Registers available on datasheet at https://datasheets.maximintegrated.com/en/ds/MAX31865.pdf
  //The chip then automatically increments to read from the next register
  
  // when reading from multiple chips sequentially, the SPI connection for a chip must be specified before the spi.write function executes.
  // if spi.select() has not been executed beforehand, the SPI connection between the ADC and microcontroller will re-initialize/transfer control to the current ADC as soon as
  // it calls the first SPI write after nss=0. However, transferring SPI control causes the clock to go low to re-initialize the SPI connection/handshake (this is while nss = 0), 
  // so the microcontroller sees the SPI re-initialization as one clock cycle.
  // the re-initialization of the SPI connection to the given ADC can be done before nss = 0 by executing spi.select() or performing spi.write(0x00). (0x00 is an arbitrary value)
  // example: https://i.stack.imgur.com/o3KoR.jpg (erroneous low at start)
  // see explanation at https://stackoverflow.com/questions/71366463/spi-extra-clock-cycle-over-communication-between-stm32-nucleo-l432kc-and-max3186
  spi.select();
  
  /* Start the read operation. */
  nss = 0; //tell the MAX31865 we want to start reading, waiting for starting address to be written
  /* Tell the MAX31865 that we want to read, starting at register 0. */
  
  
  spi.write( 0x00 ); //start reading values starting at register 00h

  /* Read the MAX31865 registers in the following order:
       Configuration (00)
       RTD (01 = MSBs, 02 = LSBs)
       High Fault Threshold (03 = MSBs, 04 = LSBs)
       Low Fault Threshold (05 = MSBs, 06 = LSBs)
       Fault Status (07) */
    
    this->measured_resistance = 0;

  this->measured_configuration = spi.write( 0x00 ); //read from register 00
    //automatic increment to register 01
  combined_bytes  = spi.write( 0x00 ) << 8; //8 bit value from register 01, bit shifted 8 left
    //automatic increment to register 02, OR with previous bit shifted value to get complete 16 bit value
  combined_bytes |= spi.write( 0x00 );
  //bit 0 of LSB is a fault bit, DOES NOT REPRESENT RESISTANCE VALUE
  //bit shift 16-bit value 1 right to remove fault bit and get complete 15 bit raw resistance reading
  this->measured_resistance = combined_bytes >> 1;
    //high fault threshold
  combined_bytes  = spi.write( 0x00 ) << 8;
  combined_bytes |= spi.write( 0x00 );
  this->measured_high_threshold = combined_bytes >> 1;
    //low fault threshold
  combined_bytes  = spi.write( 0x00 ) << 8;
  combined_bytes |= spi.write( 0x00 );
  this->measured_low_threshold = combined_bytes >> 1;
    //fault status
  this->measured_status = spi.write( 0x00 );

    //set chip select to 1; chip stops incrementing registers when chip select is high; ends read cycle
  nss = 1;
  //relingquish SPI connection when chip is done reading
  spi.deselect();

  /* Reset the configuration if the measured resistance is
     zero or a fault occurred. */
  if(    ( this->measured_resistance == 0 )
      || ( this->measured_status != 0 ) )
  {
    reconfigure( );
  }

  return( status( ) );
}


