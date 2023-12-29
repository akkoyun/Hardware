/* *******************************************************************************
 *  Copyright (C) 2014-2023 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun.
 *
 *	Library				: Hardware
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *
 *********************************************************************************/

#ifndef __Hardware__
#define __Hardware__

	// Define Arduino Library
	#ifndef __Arduino__
		#include <Arduino.h>
	#endif

	// Define LED Colors
	#define RED 	1
	#define GREEN 	2
	#define BLUE 	3
	#define WHITE 	4
	#define YELLOW 	5

	// B107AA Class
	#ifdef _B107AA_

		// Get Module PinOut
		#include "B107AA.h"

		// Hardware Class
		class Hardware {

			// Private Context
			private:

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

				// Module 1 Second Timer
				inline void AVR_Timer(void) {

					// Clear Registers
					TCCR5A = 0x00;
					TCCR5B = 0x00;

					// Clear Counter
					TCNT5 = 0;

					// Set Counter Value
					OCR5A = (F_CPU / (1024)) - 1;

					// Set CTC Mod
					TCCR5B |= (1 << WGM52);

					// Set Rescale (1024)
					TCCR5B |= (1 << CS52) | (1 << CS50);

					// Start Timer
					TIMSK5 |= (1 << OCIE5A);

				}

				// Module INT0 Interrupt Function [INT0]
				inline void INT0_Interrupt(bool _State = false, bool _Trigger = false) {

					// Control for INT0
					if (_State) {

						// Set INT0
						EIMSK |= (1 << INT0);

						// Set EICRA
						if (_Trigger) {
						
							// Set INT0 Interrupt rising edge triggered Interrupt
							EICRA |= (1 << ISC01) | (1 << ISC00);

						} else {
							
							// Set INT0 Interrupt falling edge triggered Interrupt
							EICRA |= (1 << ISC01);
							EICRA &= ~(1 << ISC00);

						}

					} else {

						// Clear INT0
						EIMSK &= ~(1 << INT0);

					}

				}

				// Module INT1 Interrupt Function [INT1]
				inline void INT1_Interrupt(bool _State = false, bool _Trigger = false) {

					// Control for INT1
					if (_State) {

						// Set INT1
						EIMSK |= (1 << INT1);

						// Set EICRA
						if (_Trigger) {
						
							// Set INT1 Interrupt rising edge triggered Interrupt
							EICRA |= (1 << ISC11) | (1 << ISC10);

						} else {
							
							// Set INT1 Interrupt falling edge triggered Interrupt
							EICRA |= (1 << ISC11);
							EICRA &= ~(1 << ISC10);

						}

					} else {

						// Clear INT1
						EIMSK &= ~(1 << INT1);

					}

				}

				// Module INT2 Interrupt Function [INT2]
				inline void INT2_Interrupt(bool _State = false, bool _Trigger = false) {

					// Control for INT2
					if (_State) {

						// Set INT2
						EIMSK |= (1 << INT2);

						// Set EICRA
						if (_Trigger) {
						
							// Set INT2 Interrupt rising edge triggered Interrupt
							EICRA |= (1 << ISC21) | (1 << ISC20);

						} else {
							
							// Set INT2 Interrupt falling edge triggered Interrupt
							EICRA |= (1 << ISC21);
							EICRA &= ~(1 << ISC20);

						}

					} else {

						// Clear INT2
						EIMSK &= ~(1 << INT2);

					}

				}

				// Module INT3 Interrupt Function [INT3]
				inline void INT3_Interrupt(bool _State = false, bool _Trigger = false) {

					// Control for INT3
					if (_State) {

						// Set INT3
						EIMSK |= (1 << INT3);

						// Set EICRA
						if (_Trigger) {
						
							// Set INT3 Interrupt rising edge triggered Interrupt
							EICRA |= (1 << ISC31) | (1 << ISC30);

						} else {
							
							// Set INT3 Interrupt falling edge triggered Interrupt
							EICRA |= (1 << ISC31);
							EICRA &= ~(1 << ISC30);

						}

					} else {

						// Clear INT3
						EIMSK &= ~(1 << INT3);

					}

				}

				// Module INT4 Interrupt Function [INT4]
				inline void INT4_Interrupt(bool _State = false, bool _Trigger = false) {

					// Control for INT4
					if (_State) {

						// Set INT4
						EIMSK |= (1 << INT4);

						// Set EICRB
						if (_Trigger) {
						
							// Set INT4 Interrupt rising edge triggered Interrupt
							EICRB |= (1 << ISC41) | (1 << ISC40);

						} else {
							
							// Set INT4 Interrupt falling edge triggered Interrupt
							EICRB |= (1 << ISC41);
							EICRB &= ~(1 << ISC40);

						}

					} else {

						// Clear INT4
						EIMSK &= ~(1 << INT4);

					}

				}

				// Module INT5 Interrupt Function [INT5]
				inline void INT5_Interrupt(bool _State = false, bool _Trigger = false) {

					// Control for INT5
					if (_State) {

						// Set INT5
						EIMSK |= (1 << INT5);

						// Set EICRB
						if (_Trigger) {
						
							// Set INT5 Interrupt rising edge triggered Interrupt
							EICRB |= (1 << ISC51) | (1 << ISC50);

						} else {
							
							// Set INT5 Interrupt falling edge triggered Interrupt
							EICRB |= (1 << ISC51);
							EICRB &= ~(1 << ISC50);

						}

					} else {

						// Clear INT5
						EIMSK &= ~(1 << INT5);

					}

				}

				// Module INT6 Interrupt Function [INT6]
				inline void INT6_Interrupt(bool _State = false, bool _Trigger = false) {

					// Control for INT6
					if (_State) {

						// Set INT6
						EIMSK |= (1 << INT6);

						// Set EICRB
						if (_Trigger) {
						
							// Set INT6 Interrupt rising edge triggered Interrupt
							EICRB |= (1 << ISC61) | (1 << ISC60);

						} else {
							
							// Set INT6 Interrupt falling edge triggered Interrupt
							EICRB |= (1 << ISC61);
							EICRB &= ~(1 << ISC60);

						}

					} else {

						// Clear INT6
						EIMSK &= ~(1 << INT6);

					}

				}

				// Module INT7 Interrupt Function [INT7]
				inline void INT7_Interrupt(bool _State = false, bool _Trigger = false) {

					// Control for INT7
					if (_State) {

						// Set INT7
						EIMSK |= (1 << INT7);

						// Set EICRB
						if (_Trigger) {
						
							// Set INT7 Interrupt rising edge triggered Interrupt
							EICRB |= (1 << ISC71) | (1 << ISC70);

						} else {
							
							// Set INT7 Interrupt falling edge triggered Interrupt
							EICRB |= (1 << ISC71);
							EICRB &= ~(1 << ISC70);

						}

					} else {

						// Clear INT7
						EIMSK &= ~(1 << INT7);

					}

				}

				// Module PCINT Interrupt Function [PCIE0]
				inline void PCIE0_Interrupt(bool _PCINT0 = false, bool _PCINT1 = false, bool _PCINT2 = false, bool _PCINT3 = false, bool _PCINT4 = false, bool _PCINT5 = false, bool _PCINT6 = false, bool _PCINT7 = false) {

					// Control for PCIE0
					if (_PCINT0 || _PCINT1 || _PCINT2 || _PCINT3 || _PCINT4 || _PCINT5 || _PCINT6 || _PCINT7) {

						// Set PCIE0
						PCICR |= (1 << PCIE0);

						// Set PCINTx
						if (_PCINT0) {PCMSK0 |= (1 << PCINT0);} else {PCMSK0 &= ~(1 << PCINT0);}
						if (_PCINT1) {PCMSK0 |= (1 << PCINT1);} else {PCMSK0 &= ~(1 << PCINT1);}
						if (_PCINT2) {PCMSK0 |= (1 << PCINT2);} else {PCMSK0 &= ~(1 << PCINT2);}
						if (_PCINT3) {PCMSK0 |= (1 << PCINT3);} else {PCMSK0 &= ~(1 << PCINT3);}
						if (_PCINT4) {PCMSK0 |= (1 << PCINT4);} else {PCMSK0 &= ~(1 << PCINT4);}
						if (_PCINT5) {PCMSK0 |= (1 << PCINT5);} else {PCMSK0 &= ~(1 << PCINT5);}
						if (_PCINT6) {PCMSK0 |= (1 << PCINT6);} else {PCMSK0 &= ~(1 << PCINT6);}
						if (_PCINT7) {PCMSK0 |= (1 << PCINT7);} else {PCMSK0 &= ~(1 << PCINT7);}

					} else {

						// Clear PCIE0
						PCICR &= ~(1 << PCIE0);

						// Set PCINTx
						PCMSK0 &= ~(1 << PCINT0);
						PCMSK0 &= ~(1 << PCINT1);
						PCMSK0 &= ~(1 << PCINT2);
						PCMSK0 &= ~(1 << PCINT3);
						PCMSK0 &= ~(1 << PCINT4);
						PCMSK0 &= ~(1 << PCINT5);
						PCMSK0 &= ~(1 << PCINT6);
						PCMSK0 &= ~(1 << PCINT7);

					}

				}

				// Module PCINT Interrupt Function [PCIE1]
				inline void PCIE1_Interrupt(bool _PCINT8 = false, bool _PCINT9 = false, bool _PCINT10 = false, bool _PCINT11 = false, bool _PCINT12 = false, bool _PCINT13 = false, bool _PCINT14 = false, bool _PCINT15 = false) {

					// Control for PCIE1
					if (_PCINT8 || _PCINT9 || _PCINT10 || _PCINT11 || _PCINT12 || _PCINT13 || _PCINT14 || _PCINT15) {

						// Set PCIE1
						PCICR |= (1 << PCIE1);

						// Set PCINTx
						if (_PCINT8) {PCMSK1 |= (1 << PCINT8);} else {PCMSK1 &= ~(1 << PCINT8);}
						if (_PCINT9) {PCMSK1 |= (1 << PCINT9);} else {PCMSK1 &= ~(1 << PCINT9);}
						if (_PCINT10) {PCMSK1 |= (1 << PCINT10);} else {PCMSK1 &= ~(1 << PCINT10);}
						if (_PCINT11) {PCMSK1 |= (1 << PCINT11);} else {PCMSK1 &= ~(1 << PCINT11);}
						if (_PCINT12) {PCMSK1 |= (1 << PCINT12);} else {PCMSK1 &= ~(1 << PCINT12);}
						if (_PCINT13) {PCMSK1 |= (1 << PCINT13);} else {PCMSK1 &= ~(1 << PCINT13);}
						if (_PCINT14) {PCMSK1 |= (1 << PCINT14);} else {PCMSK1 &= ~(1 << PCINT14);}
						if (_PCINT15) {PCMSK1 |= (1 << PCINT15);} else {PCMSK1 &= ~(1 << PCINT15);}

					} else {

						// Clear PCIE1
						PCICR &= ~(1 << PCIE1);

						// Set PCINTx
						PCMSK1 &= ~(1 << PCINT8);
						PCMSK1 &= ~(1 << PCINT9);
						PCMSK1 &= ~(1 << PCINT10);
						PCMSK1 &= ~(1 << PCINT11);
						PCMSK1 &= ~(1 << PCINT12);
						PCMSK1 &= ~(1 << PCINT13);
						PCMSK1 &= ~(1 << PCINT14);
						PCMSK1 &= ~(1 << PCINT15);

					}

				}

				// Module PCINT Interrupt Function [PCIE2]
				inline void PCIE2_Interrupt(bool _PCINT16 = false, bool _PCINT17 = false, bool _PCINT18 = false, bool _PCINT19 = false, bool _PCINT20 = false, bool _PCINT21 = false, bool _PCINT22 = false, bool _PCINT23 = false) {

					// Control for PCIE2
					if (_PCINT16 || _PCINT17 || _PCINT18 || _PCINT19 || _PCINT20 || _PCINT21 || _PCINT22 || _PCINT23) {

						// Set PCIE2
						PCICR |= (1 << PCIE2);

						// Set PCINTx
						if (_PCINT16) {PCMSK2 |= (1 << PCINT16);} else {PCMSK2 &= ~(1 << PCINT16);}
						if (_PCINT17) {PCMSK2 |= (1 << PCINT17);} else {PCMSK2 &= ~(1 << PCINT17);}
						if (_PCINT18) {PCMSK2 |= (1 << PCINT18);} else {PCMSK2 &= ~(1 << PCINT18);}
						if (_PCINT19) {PCMSK2 |= (1 << PCINT19);} else {PCMSK2 &= ~(1 << PCINT19);}
						if (_PCINT20) {PCMSK2 |= (1 << PCINT20);} else {PCMSK2 &= ~(1 << PCINT20);}
						if (_PCINT21) {PCMSK2 |= (1 << PCINT21);} else {PCMSK2 &= ~(1 << PCINT21);}
						if (_PCINT22) {PCMSK2 |= (1 << PCINT22);} else {PCMSK2 &= ~(1 << PCINT22);}
						if (_PCINT23) {PCMSK2 |= (1 << PCINT23);} else {PCMSK2 &= ~(1 << PCINT23);}

					} else {

						// Clear PCIE2
						PCICR &= ~(1 << PCIE2);

						// Set PCINTx
						PCMSK2 &= ~(1 << PCINT16);
						PCMSK2 &= ~(1 << PCINT17);
						PCMSK2 &= ~(1 << PCINT18);
						PCMSK2 &= ~(1 << PCINT19);
						PCMSK2 &= ~(1 << PCINT20);
						PCMSK2 &= ~(1 << PCINT21);
						PCMSK2 &= ~(1 << PCINT22);
						PCMSK2 &= ~(1 << PCINT23);

					}

				}

				// Module Interrupt Enable Function
				inline void AVR_Interrupt(void) {

					// Disable Interrupts
					cli();

					// Set INT4 as falling edge triggered Interrupt
					this->INT4_Interrupt(true, false);
					#define INT_RS485 (((PIN_REGISTER_INT_RS485) >> (PIN_INT_RS485)) & 0x01)

					// Set INT5 as falling edge triggered Interrupt
//					this->INT5_Interrupt(true, false);
//					#define INT_CHARGER (((PIN_REGISTER_INT_CHARGER) >> (PIN_INT_CHARGER)) & 0x01)

					// Set INT6 as falling edge triggered Interrupt
//					this->INT6_Interrupt(true, false);
//					#define INT_GAUGE (((PIN_REGISTER_INT_GAUGE) >> (PIN_INT_GAUGE)) & 0x01)

					// Set PCINT4, PCINT5, PCINT6, PCINT7 Interrupt
					this->PCIE0_Interrupt(false, false, false, false, true, true, true, true);
					#define INT_ENERGY_1 (((PIN_REGISTER_INT_ENERGY_1) >> (PIN_INT_ENERGY_1)) & 0x01)	// PCINT4
					#define INT_ENERGY_2 (((PIN_REGISTER_INT_ENERGY_2) >> (PIN_INT_ENERGY_2)) & 0x01)	// PCINT5
					#define INT_ENVIRONMENT (((PIN_REGISTER_INT_ENV) >> (PIN_INT_ENV)) & 0x01)			// PCINT6
					#define INT_RTC (((PIN_REGISTER_INT_RTC) >> (PIN_INT_RTC)) & 0x01)					// PCINT7

					// Set PCINT11, PCINT12, PCINT13 Interrupt
					this->PCIE1_Interrupt(false, false, false, true, true, true, false, false);
					#define INT_GSM_RING (((PIN_REGISTER_GSM_RING) >> (PIN_GSM_RING)) & 0x01)			// PCINT11
					#define INT_GSM_PMON (((PIN_REGISTER_GSM_PMON) >> (PIN_GSM_PMON)) & 0x01)			// PCINT12
					#define INT_GSM_SWREADY (((PIN_REGISTER_GSM_SWREADY) >> (PIN_GSM_SWREADY)) & 0x01)	// PCINT13

					// Set PCINT16, PCINT17, PCINT18, PCINT19, PCINT20, PCINT21, PCINT22, PCINT23 Interrupt
					this->PCIE2_Interrupt(true, true, true, true, true, true, true, true);
					#define INT_SENSE_1 (((PIN_REGISTER_3V3_Sense_1) >> (PIN_3V3_Sense_1)) & 0x01)		// PCINT16
					#define INT_SENSE_2 (((PIN_REGISTER_3V3_Sense_2) >> (PIN_3V3_Sense_2)) & 0x01)		// PCINT17
					#define INT_SENSE_3 (((PIN_REGISTER_3V3_Sense_3) >> (PIN_3V3_Sense_3)) & 0x01)		// PCINT18
					#define INT_SENSE_4 (((PIN_REGISTER_3V3_Sense_4) >> (PIN_3V3_Sense_4)) & 0x01)		// PCINT19
					#define INT_SENSE_5 (((PIN_REGISTER_3V3_Sense_5) >> (PIN_3V3_Sense_5)) & 0x01)		// PCINT20
					#define INT_SENSE_6 (((PIN_REGISTER_3V3_Sense_6) >> (PIN_3V3_Sense_6)) & 0x01)		// PCINT21
					#define INT_SENSE_7 (((PIN_REGISTER_3V3_Sense_7) >> (PIN_3V3_Sense_7)) & 0x01)		// PCINT22
					#define INT_SENSE_8 (((PIN_REGISTER_3V3_Sense_8) >> (PIN_3V3_Sense_8)) & 0x01)		// PCINT23

					// Start Interrupts
					sei();

				}

			// Public Context
			public:

				// Bool Controller
				#define Up_Time_Control 	0
				#define Environment_Control 1
				#define Battery_Control 	2
				#define Input_Control 		3
				#define Energy_Control 		4
				#define Status_Update		5
				#define Environment_INT		6
				uint8_t Bool_Controller = 0x10;

				// Module Constructor
				Hardware(void) {

					// Set Pin Out
					this->Set_PinOut();

				}

				// Begin Function
				void Begin(void) {

					// Set 1 Second Timer
					this->AVR_Timer();

					// Set Interrupts
					this->AVR_Interrupt();

				}

				// LED Function
				void LED(const uint8_t _Color = WHITE, const uint8_t _Blink = 1, const uint16_t _Interval = 500) {

					// Switch Color
					switch (_Color)	{

						// Red Color
						case RED: {

							// Blink
							for (size_t i = 0; i < _Blink; i++) {

								// Turn ON Red LED
								PORT_MCU_LED_RED |= (1 << PIN_MCU_LED_RED);

								// Delay
								delay(_Interval);

								// Turn OFF Red LED
								PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);

								// Delay
								delay(_Interval);

							}

							// End Case
							break;

						}

						// Green Color
						case GREEN: {

							// Blink
							for (size_t i = 0; i < _Blink; i++) {

								// Turn ON Green LED
								PORT_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN);

								// Delay
								delay(_Interval);

								// Turn OFF Green LED
								PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);

								// Delay
								delay(_Interval);

							}

							// End Case
							break;

						}

						// Blue Color
						case BLUE: {

							// Blink
							for (size_t i = 0; i < _Blink; i++) {

								// Turn ON Blue LED
								PORT_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE);

								// Delay
								delay(_Interval);

								// Turn OFF Blue LED
								PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);

								// Delay
								delay(_Interval);

							}

							// End Case
							break;

						}

						// White Color
						case WHITE: {

							// Blink
							for (size_t i = 0; i < _Blink; i++) {

								// Turn ON White LED
								PORT_MCU_LED_RED |= (1 << PIN_MCU_LED_RED);
								PORT_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN);
								PORT_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE);

								// Delay
								delay(_Interval);

								// Turn OFF White LED
								PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);
								PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);
								PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);

								// Delay
								delay(_Interval);

							}

							// End Case
							break;

						}

						// Default
						default: {

							// Turn OFF all LED
							PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);
							PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);
							PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);

							// End Case
							break;

						}

					}

				}

				// Heartbeat Function
				void Heartbeat(bool _LED = false) {

					// Turn ON HeartBeat
					PORT_HEARTBEAT |= (1 << PIN_HEARTBEAT);

					// HeartBeat LED Blink
					if (_LED) this->LED(GREEN, 1, 50);

					// Turn OFF HeartBeat
					PORT_HEARTBEAT &= ~(1 << PIN_HEARTBEAT);

				}

		};

	#endif

	// B108AA Class
	#ifdef _B108AA_

		// Get Module PinOut
		#include "B108AA.h"


	#endif

	// B152BA Class
	#ifdef _B152BA_

		// Get Module PinOut
		#include "B152BA.h"


	#endif

	// B153AA Class
	#ifdef _B153AA_

		// Get Module PinOut
		#include "B153AA.h"



	#endif

#endif /* defined(AVR_Functions) */
