/* *******************************************************************************
 *  Copyright (C) 2014-2023 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun.
 *
 *	Library				: B107AA PinDefinition
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *
 *********************************************************************************/

// Include Arduino Library
#ifndef __Arduino__
    #include <Arduino.h>
#endif

// Include PinOut
#include "PinOut.h"

// Module Pin Definitions
inline void Set_PinOut(void) {

    // Set RELAY_START as Output with Pull-Down
    DDR_RELAY_START |= (1 << PIN_RELAY_START);
    PORT_RELAY_START &= ~(1 << PIN_RELAY_START);

    // Set RELAY_STOP as Output with Pull-Down
    DDR_RELAY_STOP |= (1 << PIN_RELAY_STOP);
    PORT_RELAY_STOP &= ~(1 << PIN_RELAY_STOP);

    // Set INT_ENERGY_1 as Input with Pull-Up
    DDR_INT_ENERGY_1 &= ~(1 << PIN_INT_ENERGY_1);
    PORT_INT_ENERGY_1 |= (1 << PIN_INT_ENERGY_1);

    // Set INT_ENERGY_2 as Input with Pull-Up
    DDR_INT_ENERGY_2 &= ~(1 << PIN_INT_ENERGY_2);
    PORT_INT_ENERGY_2 |= (1 << PIN_INT_ENERGY_2);

    // Set INT_ENV as Input with Pull-Up
    DDR_INT_ENV &= ~(1 << PIN_INT_ENV);
    PORT_INT_ENV |= (1 << PIN_INT_ENV);

    // Set INT_RTC as Input with Pull-Up
    DDR_INT_RTC &= ~(1 << PIN_INT_RTC);
    PORT_INT_RTC |= (1 << PIN_INT_RTC);

    // Set 3V8_EN as Output with Pull-Down
    DDR_3V8_EN |= (1 << PIN_3V8_EN);
    PORT_3V8_EN &= ~(1 << PIN_3V8_EN);

    // Set RS485_DIR as Output with Pull-Down
    DDR_RS485_DIR |= (1 << PIN_RS485_DIR);
    PORT_RS485_DIR &= ~(1 << PIN_RS485_DIR);

    // Set INT_RS485 as Input with Pull-Down
    DDR_INT_RS485 &= ~(1 << PIN_INT_RS485);
    PORT_INT_RS485 &= ~(1 << PIN_INT_RS485);

    // Set INT_CHARGER as Input with Pull-Up
    DDR_INT_CHARGER &= ~(1 << PIN_INT_CHARGER);
    PORT_INT_CHARGER |= (1 << PIN_INT_CHARGER);

    // Set INT_GAUGE as Input with Pull-Up
    DDR_INT_GAUGE &= ~(1 << PIN_INT_GAUGE);
    PORT_INT_GAUGE |= (1 << PIN_INT_GAUGE);

    // Set 3V3_BUZZER as Output with Pull-Down
    DDR_3V3_BUZZER |= (1 << PIN_3V3_BUZZER);
    PORT_3V3_BUZZER &= ~(1 << PIN_3V3_BUZZER);

    // Set LCD_SENSE as Input with Pull-Down
    DDR_LCD_SENSE &= ~(1 << PIN_LCD_SENSE);
    PORT_LCD_SENSE &= ~(1 << PIN_LCD_SENSE);

    // Set FOTA_POWER_EN as Output with Pull-Down
    DDR_FOTA_POWER_EN |= (1 << PIN_FOTA_POWER_EN);
    PORT_FOTA_POWER_EN &= ~(1 << PIN_FOTA_POWER_EN);

    // Set SD_SENSE as Input with Pull-Down
    DDR_SD_SENSE &= ~(1 << PIN_SD_SENSE);
    PORT_SD_SENSE &= ~(1 << PIN_SD_SENSE);

    // Set SD_EN as Output with Pull-Down
    DDR_SD_EN |= (1 << PIN_SD_EN);
    PORT_SD_EN &= ~(1 << PIN_SD_EN);

    // Set TERMINAL_SENSE as Input with Pull-Down
    DDR_TERMINAL_SENSE &= ~(1 << PIN_TERMINAL_SENSE);
    PORT_TERMINAL_SENSE &= ~(1 << PIN_TERMINAL_SENSE);

    // Set GSM_RING as Input with Pull-Down
    DDR_GSM_RING &= ~(1 << PIN_GSM_RING);
    PORT_GSM_RING &= ~(1 << PIN_GSM_RING);

    // Set GSM_PMON as Input with Pull-Down
    DDR_GSM_PMON &= ~(1 << PIN_GSM_PMON);
    PORT_GSM_PMON &= ~(1 << PIN_GSM_PMON);

    // Set GSM_SWREADY as Input with Pull-Down
    DDR_GSM_SWREADY &= ~(1 << PIN_GSM_SWREADY);
    PORT_GSM_SWREADY &= ~(1 << PIN_GSM_SWREADY);

    // Set GSM_COMM_EN as Output with Pull-Up
    DDR_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN);
    PORT_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN);

    // Set GSM_ONOFF as Output with Pull-Down
    DDR_GSM_ONOFF |= (1 << PIN_GSM_ONOFF);
    PORT_GSM_ONOFF &= ~(1 << PIN_GSM_ONOFF);

    // Set GSM_SDOWN as Output with Pull-Down
    DDR_GSM_SDOWN |= (1 << PIN_GSM_SDOWN);
    PORT_GSM_SDOWN &= ~(1 << PIN_GSM_SDOWN);

    // Set 3V3_Sense_1 as Input with Pull-Down
    DDR_3V3_Sense_1 &= ~(1 << PIN_3V3_Sense_1);
    PORT_3V3_Sense_1 &= ~(1 << PIN_3V3_Sense_1);

    // Set 3V3_Sense_2 as Input with Pull-Down
    DDR_3V3_Sense_2 &= ~(1 << PIN_3V3_Sense_2);
    PORT_3V3_Sense_2 &= ~(1 << PIN_3V3_Sense_2);

    // Set 3V3_Sense_3 as Input with Pull-Down
    DDR_3V3_Sense_3 &= ~(1 << PIN_3V3_Sense_3);
    PORT_3V3_Sense_3 &= ~(1 << PIN_3V3_Sense_3);

    // Set 3V3_Sense_4 as Input with Pull-Down
    DDR_3V3_Sense_4 &= ~(1 << PIN_3V3_Sense_4);
    PORT_3V3_Sense_4 &= ~(1 << PIN_3V3_Sense_4);

    // Set 3V3_Sense_5 as Input with Pull-Down
    DDR_3V3_Sense_5 &= ~(1 << PIN_3V3_Sense_5);
    PORT_3V3_Sense_5 &= ~(1 << PIN_3V3_Sense_5);

    // Set 3V3_Sense_6 as Input with Pull-Down
    DDR_3V3_Sense_6 &= ~(1 << PIN_3V3_Sense_6);
    PORT_3V3_Sense_6 &= ~(1 << PIN_3V3_Sense_6);

    // Set 3V3_Sense_7 as Input with Pull-Down
    DDR_3V3_Sense_7 &= ~(1 << PIN_3V3_Sense_7);
    PORT_3V3_Sense_7 &= ~(1 << PIN_3V3_Sense_7);

    // Set 3V3_Sense_8 as Input with Pull-Down
    DDR_3V3_Sense_8 &= ~(1 << PIN_3V3_Sense_8);
    PORT_3V3_Sense_8 &= ~(1 << PIN_3V3_Sense_8);

    // Set MCU_LED_RED as Output with Pull-Down
    DDR_MCU_LED_RED |= (1 << PIN_MCU_LED_RED);
    PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);

    // Set MCU_LED_GREEN as Output with Pull-Down
    DDR_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN);
    PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);

    // Set MCU_LED_BLUE as Output with Pull-Down
    DDR_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE);
    PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);

    // Set HEARTBEAT as Output with Pull-Down
    DDR_HEARTBEAT |= (1 << PIN_HEARTBEAT);
    PORT_HEARTBEAT &= ~(1 << PIN_HEARTBEAT);
}
