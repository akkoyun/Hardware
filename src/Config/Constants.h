// Include Arduino Library
#ifndef Arduino_h
    #include <Arduino.h>
#endif

// LED Definitions
// -----------------------------
#define LED_RED 	        1
#define LED_GREEN 	        2
#define LED_BLUE 	        3
#define LED_WHITE 	        4
#define LED_YELLOW 	        5

// Serial Name Definitions
// -----------------------------
#define SERIAL_TERMINAL		Serial
#define SERIAL_ENERGY		Serial2
#define SERIAL_GSM			Serial3

// PCINTx Definitions
// -----------------------------
#define INTERRUPT_PCINT0	0
#define INTERRUPT_PCINT1	1
#define INTERRUPT_PCINT2	2
#define INTERRUPT_PCINT3	3
#define INTERRUPT_PCINT4	4
#define INTERRUPT_PCINT5	5
#define INTERRUPT_PCINT6	6
#define INTERRUPT_PCINT7	7
#define INTERRUPT_PCINT8	8
#define INTERRUPT_PCINT9	9
#define INTERRUPT_PCINT10	10
#define INTERRUPT_PCINT11	11
#define INTERRUPT_PCINT12	12
#define INTERRUPT_PCINT13	13
#define INTERRUPT_PCINT14	14
#define INTERRUPT_PCINT15	15
#define INTERRUPT_PCINT16	16
#define INTERRUPT_PCINT17	17
#define INTERRUPT_PCINT18	18
#define INTERRUPT_PCINT19	19
#define INTERRUPT_PCINT20	20
#define INTERRUPT_PCINT21	21
#define INTERRUPT_PCINT22	22
#define INTERRUPT_PCINT23	23

// EEPROM Definitions
// -----------------------------

#ifdef _B107AA_

    // RTC EEPROM Definitions
    #ifndef __EEPROM_Online_Interval__
        #define __EEPROM_Online_Interval__					(uint8_t)0x00
    #endif
    #ifndef __EEPROM_Offline_Interval__
        #define __EEPROM_Offline_Interval__					(uint8_t)0x01
    #endif
    #ifndef __EEPROM_V_Min_MSB__
        #define __EEPROM_V_Min_MSB__						(uint8_t)0x06
    #endif
    #ifndef __EEPROM_V_Min_LSB__
        #define __EEPROM_V_Min_LSB__						(uint8_t)0x07
    #endif
    #ifndef __EEPROM_V_Max_MSB__
        #define __EEPROM_V_Max_MSB__						(uint8_t)0x08
    #endif
    #ifndef __EEPROM_V_Max_LSB__
        #define __EEPROM_V_Max_LSB__						(uint8_t)0x09
    #endif
    #ifndef __EEPROM_I_Max_MSB__
        #define __EEPROM_I_Max_MSB__						(uint8_t)0x0A
    #endif
    #ifndef __EEPROM_I_Max_LSB__
        #define __EEPROM_I_Max_LSB__						(uint8_t)0x0B
    #endif
    #ifndef __EEPROM_FQ_Min__
        #define __EEPROM_FQ_Min__							(uint8_t)0x0C
    #endif
    #ifndef __EEPROM_FQ_Max__
        #define __EEPROM_FQ_Max__							(uint8_t)0x0D
    #endif
    #ifndef __EEPROM_VIMB_Max__
        #define __EEPROM_VIMB_Max__							(uint8_t)0x10
    #endif
    #ifndef __EEPROM_IIMB_Max__
        #define __EEPROM_IIMB_Max__							(uint8_t)0x11
    #endif
    #ifndef __EEPROM_Current_Ratio__
        #define __EEPROM_Current_Ratio__					(uint8_t)0x15
    #endif
    #ifndef __EEPROM_PMIN_MSB__
        #define __EEPROM_PMIN_MSB__							(uint8_t)0x1A
    #endif
    #ifndef __EEPROM_PMIN_LSB__
        #define __EEPROM_PMIN_LSB__							(uint8_t)0x1B
    #endif
    #ifndef __EEPROM_PMAX_MSB__
        #define __EEPROM_PMAX_MSB__							(uint8_t)0x1C
    #endif
    #ifndef __EEPROM_PMAX_LSB__
        #define __EEPROM_PMAX_LSB__							(uint8_t)0x1D
    #endif
    #ifndef __EEPROM_PSLOP_EMAX__
        #define __EEPROM_PSLOP_EMAX__						(uint8_t)0x1E
    #endif
    #ifndef __EEPROM_STOP_MASK_MSB_1__
        #define __EEPROM_STOP_MASK_MSB_1__					(uint8_t)0x20
    #endif
    #ifndef __EEPROM_STOP_MASK_MSB_2__
        #define __EEPROM_STOP_MASK_MSB_2__					(uint8_t)0x21
    #endif
    #ifndef __EEPROM_STOP_MASK_LSB_1__
        #define __EEPROM_STOP_MASK_LSB_1__					(uint8_t)0x22
    #endif
    #ifndef __EEPROM_STOP_MASK_LSB_2__
        #define __EEPROM_STOP_MASK_LSB_2__					(uint8_t)0x23
    #endif
    #ifndef __EEPROM_PUBLISH_MASK_MSB_2__
        #define __EEPROM_PUBLISH_MASK_MSB_2__				(uint8_t)0x24
    #endif
    #ifndef __EEPROM_PUBLISH_MASK_MSB_1__
        #define __EEPROM_PUBLISH_MASK_MSB_1__				(uint8_t)0x25
    #endif
    #ifndef __EEPROM_PUBLISH_MASK_LSB_2__
        #define __EEPROM_PUBLISH_MASK_LSB_2__				(uint8_t)0x26
    #endif
    #ifndef __EEPROM_PUBLISH_MASK_LSB_1__
        #define __EEPROM_PUBLISH_MASK_LSB_1__				(uint8_t)0x27
    #endif

    // EEPROM Default Values
    #ifndef __PUBLISH_REGISTER_DEFAULT__
        #define __PUBLISH_REGISTER_DEFAULT__			    (uint32_t)0x87F00DEF
    #endif
    #ifndef __STOP_REGISTER_DEFAULT__
        #define __STOP_REGISTER_DEFAULT__				    (uint32_t)0x87F00DEF
    #endif

#endif // B107AA