/**
  ******************************************************************************
  * @file    tbolt.c
  * @brief   This file provide functions to manage messages received from T_Bolt
  *
  * Author: V.Patoka (Patoka Consulting Inc., 2016)
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "LiquidCrystal_I2C.h"
#include "tbolt.h"


/* TSIP MESSAGES    ---------------------------------------------------------*/

const char disciplining_mode_msg[][22] =  {"Normal Disciplining ", "Power-Up            ", "Auto Holdover      ", "Manual Holdover     ", "Recovery Discipl.   ", 
                                       "Discipl. disabled   ", "Unknown disc. mode  " }; 

const char decoding_status_msg[][22] =    {"Doing fixes         ", "Do not have GPS time", "PDOP is too high   ", "No usable sats      ", "Only 1 usable sat   ", 
				       "Only 2 usable sat   ", "Only 3 usable sat   ", "Sat is unusable    ", "TRAIM rejected fix  ", "Unknown GPS status  " };

const char minor_alarms_msg[][22] =       {"Voltage is near rail", "Antenna open        ", "Antenna shorted    ", "No satellites       ", "Not disciplining OSC",
                                       "Survey in progress  ", "No stored position  ", "Leap second pending", "In test mode        ", "Low accuracy        ",
                                       "Currupted EEPROM    ", "Old Almanach        ", "PPS is not accurate", "Unknown minor ALM   ", "No minor alarms     " };

const char critical_alarms_msg[][22] =    {"ROM checksum error  ", "RAM check failed    ", "PSU failure        ", "FPGA check failed   ", "OSC Voltage low     ",
                                       "Unknown major ALM   ", "No major alarms     " };

const char disc_activity_msg[][22] =      {"Phase locking       ", "Oscillator warm up  ", "Frequency locking  ", "Placing PPS         ", "Init. loop filter   ",
                                       "Compensating OCXO   ", "Inactive            ", "Not used           ", "Recovery mode       ", "Unknown disc. stat. ", "Unknown RAT value  " };



/* Private variables ---------------------------------------------------------*/

extern osMutexId LCDmutex;                                // Mutex ID
extern osMutexId PRNmutex; 

extern void StrPad(uint8_t l);

extern uint8_t lcdstr[MAX_LCD_STRING+3];
extern __IO uint8_t tptr[TSBUF];

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef stimestructureget;
extern RTC_DateTypeDef sdatestructureget;


uint8_t syncf = 1;			// Its time to sync clocks
uint8_t utcf = 1;		       	// Guess: GPS or UTC ?
uint8_t sat_cnt = 0;		       	// Current Number of Visible Satelites


// ====================== Protocol Description
// TSIP documentaion from: ThunderBoltBook2003.pdf

//
// A.9.34 Report Packet 0x6D Satellite Selection List
//  - the GPS receiver sends this packet in response to packet 0x24
//  - if enabled with packet 8E-A5, the receiver will send this packet
//    whenever the selection list is updated
//
void tsip_6D (void)
{
    mytb_6b s6d;
    void  *val;

    val = memcpy((void *)&s6d, (void *) tptr, sizeof(s6d));
    if(val == NULL)
	return;

    sat_cnt = (s6d.fix & 0xF0) >> 4;
    /*
    printf("# 0x%.2X: %d dim (134=023D), %d mode (0=auto), %d nsv \n\r",
            s6d.id,
            (s6d.fix & 0x07),       // fix dimenision
            (s6d.fix & 0x08) >> 3,  // fix mode
            sat_cnt                   // number of sv in fix
            );
    */
}


//
// A.10.30 Report Packet 0x8F-AB Primary Timing Packet.
//
void tsip_8F_AB (void)
{
    mytb_8f_ab s8f_ab;
    //static uint32_t otow = 99;
    //uint32_t tow;
    static uint8_t validation = 99;
    void  *val;

   // Do no waste the MCU cycles to decode Time message if its unnecessary
   if(!syncf)
	return;

    val = memcpy((void *)&s8f_ab, (void *) tptr, sizeof(s8f_ab));
    if(val == NULL) {
	return;
    }

    //tow = long_little_end(s8f_ab.tow);

    // Check the packet validity. If its consistent change - go for it. If not - double check
    if(validation != s8f_ab.tflag) {
	validation = s8f_ab.tflag;
	return;
    }

    /*
    printf("# 0x%.2X-%.2X: %lu tow, %u wn, %u ls, 0x%.2X, %.4u-%.2u-%.2u %.2u:%.2u:%.2u\n\r",
            s8f_ab.id , s8f_ab.sub, tow, little_end(s8f_ab.wn),
            little_end(s8f_ab.ls), s8f_ab.tflag, little_end(s8f_ab.year), s8f_ab.month, 
            s8f_ab.day, s8f_ab.hr, s8f_ab.min, s8f_ab.sec);
    */

/*
    // S: = Number of Satelites 
    if(s8f_ab.tflag & 0x01)
        sprintf(lcdstr, "UTC: %.2u:%.2u:%.2u S: %.2u", 
            s8f_ab.hr, s8f_ab.min, s8f_ab.sec, sat_cnt);
    else
        sprintf(lcdstr, "GPS: %.2u:%.2u:%.2u S: %.2u", 
            s8f_ab.hr, s8f_ab.min, s8f_ab.sec, sat_cnt);
*/
    if(s8f_ab.tflag & 0x01) 
	utcf = 1;
    else
	utcf = 0;

    // Do RTC sync with T-Bolt time
    // Get the RTC current Time and sync it to GPS
    HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
    // Set Time struct
    stimestructureget.Seconds = s8f_ab.sec;
    stimestructureget.Minutes = s8f_ab.min;
    stimestructureget.Hours = s8f_ab.hr;
    HAL_RTC_SetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
    sdatestructureget.Date = s8f_ab.day;
    sdatestructureget.Month = s8f_ab.month;
    sdatestructureget.Year = s8f_ab.year;
    sdatestructureget.WeekDay =  WeekDay((2000+sdatestructureget.Year), sdatestructureget.Month, sdatestructureget.Date);
    if(HAL_RTC_SetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN) == HAL_OK && HAL_RTC_SetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN) == HAL_OK)
	syncf = 0;

    // Printing of time string is from MAIN thread
}

//
// A.10.31 Report Packet 0x8F-AC Supplemental Timing Packet
//
void tsip_8F_AC (void)
{
    mytb_8f_ac s8f_ac;
    static uint16_t validation = 99;
    static uint8_t rat = 0;
    uint16_t alm;
    static uint16_t almbit1 = 1; // Minor Alarms
    static uint16_t almbit2 = 1; // Major Alarms
    void  *val;

    val = memcpy((void *)&s8f_ac, (void *) tptr, sizeof(s8f_ac));
    if(val == NULL) {
	return;
    }

    // Check the packet validity. If its consistent change - go for it. If not - double check
    if(validation != s8f_ac.minor_alarms) {
	validation = s8f_ac.minor_alarms;
	return;
    }

    /*
    printf("# 0x%.2X-%.2X:a %u rcv, %u dsc, %u pct, %lu hold, %.4X crit, %.4X min, %u gps, %u act\n\r",
            s8f_ac.id, s8f_ac.sub,
            s8f_ac.receiver_mode,
            s8f_ac.disciplining_mode,
            s8f_ac.self_survey_progress,
            long_little_end(s8f_ac.holdover_duration),
            little_end(s8f_ac.critical_alarms),
            little_end(s8f_ac.minor_alarms),
            s8f_ac.gps_decoding_status,
            s8f_ac.disciplining_activity);
    */

    LCDI2C_setCursor(0, 1);			    // First String for this packet (GPS/UTC time)
    switch (rat) {
        case 0:
            switch(s8f_ac.disciplining_mode) {
                case 0:
		    LCDI2C_write_String(disciplining_mode_msg[0]);
                    break;
                case 1:
		    LCDI2C_write_String(disciplining_mode_msg[1]);
                    break;
                case 2:
		    LCDI2C_write_String(disciplining_mode_msg[2]);
                    break;
                case 3:
		    LCDI2C_write_String(disciplining_mode_msg[3]);
                    break;
                case 4:
		    LCDI2C_write_String(disciplining_mode_msg[4]);
                    break;
                case 6:
		    LCDI2C_write_String(disciplining_mode_msg[5]);
                    break;
                default:
		    LCDI2C_write_String(disciplining_mode_msg[6]);
                    break;
            }
            rat++;
            break;
        case 1:
            switch(s8f_ac.gps_decoding_status) {
                case 0:
                    LCDI2C_write_String(decoding_status_msg[0]);
                    break;
                case 1:
                    LCDI2C_write_String(decoding_status_msg[1]);
                    break;
                case 3:
                    LCDI2C_write_String(decoding_status_msg[2]);
                    break;
                case 8:
		    sat_cnt = 0;
                    LCDI2C_write_String(decoding_status_msg[3]);
                    break;
                case 9:
		    sat_cnt = 1;
                    LCDI2C_write_String(decoding_status_msg[4]);
                    break;
                case 0x0A:
		    sat_cnt = 2;
                    LCDI2C_write_String(decoding_status_msg[5]);
                    break;
                case 0x0B:
		    sat_cnt = 3;
                    LCDI2C_write_String(decoding_status_msg[6]);
                    break;
                case 0x0C:
		    sat_cnt = 0;
                    LCDI2C_write_String(decoding_status_msg[7]);
                    break;
                case 0x10:
                    LCDI2C_write_String(decoding_status_msg[8]);
                    break;
                default:
                    LCDI2C_write_String(decoding_status_msg[9]);
                    break;
            }    
            rat++;
            break;
        case 2:
        case 3:
        case 4:
        case 5:
            if ( (alm = little_end(s8f_ac.minor_alarms)) ) {
    	      switch (almbit1) {
                case 1:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[0]);
                    break;
                case 2:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[1]);
                    break;
                case 4:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[2]);
                    break;
                case 8:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[3]);
                    break;
                case 16:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[4]);
                    break;
                case 32:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[5]);
                    break;
                case 64:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[6]);
                    break;
                case 128:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[7]);
                    break;
                case 256:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[8]);
                    break;
                case 512:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[9]);
                    break;
                case 1024:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[10]);
                    break;
                case 2048:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[11]);
                    break;
                case 4096:
                    if((alm & almbit1)) 
                        LCDI2C_write_String(minor_alarms_msg[12]);
                    break;
                default:
	            sprintf(lcdstr, "Minor Alarm: 0x%X", alm);
                    StrPad(MAX_LCD_STRING);
	            LCDI2C_write_String(lcdstr);
                    break;
                }
            } else {
                        LCDI2C_write_String(minor_alarms_msg[14]);
            }
            if( (almbit1 = (almbit1<<1)) > 8192) almbit1 = 1;    
            rat++;
            break;
        case 6:
        case 7:
            if ( (alm = little_end(s8f_ac.critical_alarms)) ) {
    	      switch (almbit2) {
                case 1:
                    if((alm & almbit2)) 
                        LCDI2C_write_String(critical_alarms_msg[0]);
                    break;
                case 2:
                    if((alm & almbit2)) 
                        LCDI2C_write_String(critical_alarms_msg[1]);
                    break;
                case 4:
                    if((alm & almbit2)) 
                        LCDI2C_write_String(critical_alarms_msg[2]);
                    break;
                case 8:
                    if((alm & almbit2)) 
                        LCDI2C_write_String(critical_alarms_msg[3]);
                    break;
                case 16:
                    if((alm & almbit2)) 
                        LCDI2C_write_String(critical_alarms_msg[4]);
                    break;
                default:
	            sprintf(lcdstr, "Critical Alarm: 0x%X" );
		    printf("# T-Bolt %s\r\n", lcdstr);
                    StrPad(MAX_LCD_STRING);
	            LCDI2C_write_String(lcdstr);
                    break;
                }
            } else {
                        LCDI2C_write_String(critical_alarms_msg[6]);
            }
            if( (almbit2 = (almbit2<<1)) > 32) almbit2 = 1;    
            rat++;
            break;
        case 8:
            switch(s8f_ac.disciplining_activity) {
                case 0:
                    LCDI2C_write_String(disc_activity_msg[0]);
                    break;
                case 1:
                    LCDI2C_write_String(disc_activity_msg[1]);
                    break;
                case 2:
                    LCDI2C_write_String(disc_activity_msg[2]);
                    break;
                case 3:
                    LCDI2C_write_String(disc_activity_msg[3]);
                    break;
                case 4:
                    LCDI2C_write_String(disc_activity_msg[4]);
                    break;
                case 5:
                    LCDI2C_write_String(disc_activity_msg[5]);
                    break;
                case 6:
                    LCDI2C_write_String(disc_activity_msg[6]);
                    break;
                case 7:
                    LCDI2C_write_String(disc_activity_msg[7]);
                    break;
                case 8:
                    LCDI2C_write_String(disc_activity_msg[8]);
                    break;
                default:
                    LCDI2C_write_String(disc_activity_msg[9]);
                    break;
            }    
            rat = 0;
            break;
        default:
            rat = 0;
                    LCDI2C_write_String(disc_activity_msg[10]);
            break;
    }    
        
    /*
    printf("0x%.2X-%.2X:b %.6lf C, %.6lf ns, %.6lf ppb, 0x%.5lx DAC, %.6lf V\n",
            s8f_ac.id, s8f_ac.sub,
            s8f_ac.temperature,
            s8f_ac.pps_offset_ns,
            s8f_ac.ten_mhz_offset_ppb,
            s8f_ac.dac_value,
            s8f_ac.dac_voltage);

    printf("0x%.2X-%.2X:c %.9lf lat, %.9lf lon, %.6lf alt\n",
            s8f_ac.pid, s8f_ac.sub,
            DEG(s8f_ac.latitude),
            DEG(s8f_ac.longitude),
            s8f_ac.altitude);
    */
    /*
        printf("# 0x%.2X-%.2X:d 0x%.8lx,%.8lx %.8lx lat, 0x%.8lx,%.8lx %.8lx lon, 0x%.8lx,%.8lx %.8lx alt\n\r",
                s8f_ac.id, s8f_ac.sub,
                Upper32(s8f_ac.latitude), Lower32(s8f_ac.latitude), Float32(s8f_ac.latitude),
                Upper32(s8f_ac.longitude), Lower32(s8f_ac.longitude), Float32(s8f_ac.longitude),
                Upper32(s8f_ac.altitude), Lower32(s8f_ac.altitude), Float32(s8f_ac.altitude));
    */

    //StrPad(MAX_LCD_STRING);
    //printf("# GPS2: %s\r\n", lcdstr);				// DEBUG
    //LCDI2C_setCursor(0, 1);			    // Second String for this packet (T-Bolt status)
    //LCDI2C_write_String(lcdstr);
}



// ====================== Supplement functions


// Return double as float as binary long.
int32_t Float32 (double d)
{
    float f = (float) d;
    int32_t i = *(int32_t *)&f;
    return i;
}


// Byte swap (Big Endian -> Little Endian)
uint16_t little_end( uint16_t val )
{
        return (val << 8) | (val >> 8 );
} 

// Byte swap (Big Endian -> Little Endian) [ int32 ]
int32_t long_little_end ( int32_t val )
{
        val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
        return (val << 16) | ((val >> 16) & 0xFFFF);
} 

//! Byte swap 16 bit
int16_t swap_int16( int16_t val )
{
        return (val << 8) | ((val >> 8) & 0xFF);
}

//! Byte swap unsigned 32 bit
uint32_t swap_uint32( uint32_t val )
{
        val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
        return (val << 16) | (val >> 16);
}

//! Byte swap signed 32 bit
int32_t swap_int32( int32_t val )
{
        val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF );
        return (val << 16) | ((val >> 16) & 0xFFFF);
} 

/******************* (C) COPYRIGHT 2016 Patoka Consulting Inc *****END OF FILE****/
