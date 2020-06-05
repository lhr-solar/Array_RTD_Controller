// BPS
#define BPS_TRIP            0x002
#define BPS_ALL_CLEAR       0x101
#define BPS_OFF             0x102
#define CURRENT_DATA        0x103
#define VOLTAGE_DATA        0x104
#define TEMPERATURE_DATA    0x105
#define SOC_DATA            0x106
#define WDOG_TRIGGERED      0x107
#define CAN_ERROR           0x108
#define BPS_COMMAND_MSG     0x109
 
// Motor
#define DC_BUS_CURRENT 0x600
#define DC_BUS_VOLTAGE 0x601
#define PHASE_B_CURRENT 0x602
#define PHASE_C_CURRENT 0x603
#define VEHICLE_VELOCITY 0x604
#define MOTOR_VELOCITY 0x605
#define VD 0x606
#define VQ 0x607
#define ID 0x608
#define IQ 0x609
#define BEMFD 0x60A
#define BEMFQ 0x60B
#define HEAT_SINK_TEMPERATURE 0x60C
#define MOTOR_TEMPERATURE 0x60D
#define DC_BUS_AMP_HOURS 0x60E
#define ODOMETER 0x60F
 
// MPPT
#define MPPT_BASE_R     0x710 // MPPT Base ID (Request)
#define MPPT_BASE_A     0x770 // MPPT Base ID (Answer)
#define MPPT_OFF        0xF //MPPT Offset ID (set by DIP switch on board)
 
// Array Temperature
// WARNING: TEMPORARY IDS 
#define ARRAY_TEMP_1    0x798
#define ARRAY_TEMP_2    0x799