// Include Arduino Library
#ifndef Arduino_h
    #include <Arduino.h>
#endif

// LED Definitions
// -----------------------------
#define LED_RED												(uint8_t)1
#define LED_GREEN											(uint8_t)2
#define LED_BLUE											(uint8_t)3
#define LED_WHITE											(uint8_t)4
#define LED_YELLOW											(uint8_t)5

// Relay Definitions
// -----------------------------
#define RELAY_START											(bool)true
#define RELAY_STOP											(bool)false
#define RELAY_LOCK											(bool)true
#define RELAY_UNLOCK										(bool)false

// PCINTx Definitions
// -----------------------------
#define INTERRUPT_PCINT0									(uint8_t)0
#define INTERRUPT_PCINT1									(uint8_t)1
#define INTERRUPT_PCINT2									(uint8_t)2
#define INTERRUPT_PCINT3									(uint8_t)3
#define INTERRUPT_PCINT4									(uint8_t)4
#define INTERRUPT_PCINT5									(uint8_t)5
#define INTERRUPT_PCINT6									(uint8_t)6
#define INTERRUPT_PCINT7									(uint8_t)7
#define INTERRUPT_PCINT8									(uint8_t)8
#define INTERRUPT_PCINT9									(uint8_t)9
#define INTERRUPT_PCINT10									(uint8_t)10
#define INTERRUPT_PCINT11									(uint8_t)11
#define INTERRUPT_PCINT12									(uint8_t)12
#define INTERRUPT_PCINT13									(uint8_t)13
#define INTERRUPT_PCINT14									(uint8_t)14
#define INTERRUPT_PCINT15									(uint8_t)15
#define INTERRUPT_PCINT16									(uint8_t)16
#define INTERRUPT_PCINT17									(uint8_t)17
#define INTERRUPT_PCINT18									(uint8_t)18
#define INTERRUPT_PCINT19									(uint8_t)19
#define INTERRUPT_PCINT20									(uint8_t)20
#define INTERRUPT_PCINT21									(uint8_t)21
#define INTERRUPT_PCINT22									(uint8_t)22
#define INTERRUPT_PCINT23									(uint8_t)23

// Interrupt Mask Definitions
// -----------------------------
#define INTERRUPT_MASK_PCINT4								(uint8_t)0
#define INTERRUPT_MASK_PCINT5								(uint8_t)1
#define INTERRUPT_MASK_PCINT6								(uint8_t)2
#define INTERRUPT_MASK_PCINT7								(uint8_t)3
#define INTERRUPT_TERMINAL_SENSE							(uint8_t)6
#define INTERRUPT_TERMINAL_START							(uint8_t)7

// Interrupt Definitions
// ---------------------
#define INTERRUPT_DISPLAY									(uint8_t)0
#define INTERRUPT_ENERGY									(uint8_t)1
#define INTERRUPT_ENVIRONMENT								(uint8_t)2
#define INTERRUPT_RS485										(uint8_t)3
#define INTERRUPT_RTC										(uint8_t)4

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