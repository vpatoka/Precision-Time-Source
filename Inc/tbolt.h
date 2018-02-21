/**
  ******************************************************************************
  * @file    tbolt.h
  * @brief   Header for tbolt.c file
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TBOLT_H
#define __TBOLT_H

/* Includes ------------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/*
TSIP packet structure is the same for both commands and reports. The packet format is:
<DLE> <id> <data string bytes> <DLE> <ETX>
Where:
• <DLE> is the byte 0x10
• <ETX> is the byte 0x03
• <id> is a packet identifier byte, which can have any value excepting
<ETX> and <DLE>.
*/

// TSIP byte definitions.
#define ETX 0x03
#define DLE 0x10

#define MAX_CHAN 16
#define NSV 32
#define NS 1e9
#define DO_CHECKS 0

#define TSBUF 255	// TSIP MSG is always less than 256 bytes

/* Private macro -------------------------------------------------------------*/
#define LITTLE (*(uint8_t *)&endian == 1)
#define Y2K_ADJUST(y) ( (((y) < 80) ? 2000 : 1900) + (long)(y) )
#define PI 3.1415926535898 // ICD-GPS-200 version of pi
#define DEG(rad) (180.0 * (rad) / PI)

#define Upper32(d) ( ((int32_t *)&(d))[1] )
#define Lower32(d) ( ((int32_t *)&(d))[0] )

typedef struct {
	uint8_t TSIP[TSBUF];
	uint16_t index;
} tb_msg;

/* Private function prototypes -----------------------------------------------*/

int32_t Float32 (double d);
uint16_t little_end( uint16_t val );
int32_t long_little_end ( int32_t val );
int16_t swap_int16( int16_t val );
uint32_t swap_uint32( uint32_t val );
int32_t swap_int32( int32_t val );

void tsip_8F_AB (void);
void tsip_8F_AC (void);
void tsip_6D (void);
void tb_GetMinorAlarm(uint16_t alm);
void tb_GetMajorAlarm(uint16_t alm);


/* Private typedef -----------------------------------------------------------*/
#pragma pack(1)

//#pragma  pack(push,1) // this will  make sure your struct is on a 1 byte boundary.
// struct deceleration goes here 
//#pragma pop();

typedef struct tb_6d { 
   	uint8_t  id;
    	uint8_t  fix;
    	float  pdop;
    	float  hdop;
    	float  vdop;
    	float  tdop;
	int8_t prn[MAX_CHAN];
} mytb_6b; 


typedef struct tb_8f_ab { 
	uint8_t id;		//     : 1
	uint8_t sub; 	//0    : 1
	uint32_t tow;	//1-4  : 4
	uint16_t wn;	//5-6  : 2 
	int16_t ls;		//7-8  : 2
	uint8_t tflag;	//9    : 1
	uint8_t sec;	//10   : 1
	uint8_t min;	//11   : 1
	uint8_t hr;		//12   : 1
	uint8_t day;	//13   : 1
	uint8_t month;	//14   : 1
    	uint16_t year;	//15-16 : 2
} mytb_8f_ab; 

typedef struct tb_8f_a70 { 
    	uint8_t  id;
    	uint8_t  sub;
    	uint8_t  format;
	uint32_t tof;
	float	   bias_clk;
	float    bias_rate;
	uint8_t  satid;
	float    bias_sat;
} mytb_8f_a70;

typedef struct tb_8f_a71 { 
    	uint8_t  id;
    	uint8_t  sub;
    	uint8_t  format;
	uint32_t tof;
	int16_t  bias_clk;
	int16_t  bias_rate;
	uint8_t  satid;
	int16_t  bias_sat;
} mytb_8f_a71;

typedef struct tb_8f_ac {
    uint8_t  id;
    uint8_t  sub;
    uint8_t  receiver_mode;
    uint8_t  disciplining_mode;
    uint8_t  self_survey_progress;
    uint32_t holdover_duration;
    uint16_t critical_alarms;
    uint16_t minor_alarms;
    uint8_t  gps_decoding_status;
    uint8_t  disciplining_activity; 
    uint8_t  spare_status1;
    uint8_t  spare_status2;
    float  pps_offset_ns;
    float  ten_mhz_offset_ppb;
    uint32_t dac_value;
    float  dac_voltage;
    float  temperature;
    double latitude;
    double longitude;
    double altitude;
    double spare;
} mytb_8f_ac;



/* Exported variables --------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


#endif /* __TBOLT_H */  


/******************* (C) COPYRIGHT 2016 Patoka Consulting Inc *****END OF FILE****/
