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

	// Include Arduino Library
	#ifndef __Arduino__
		#include <Arduino.h>
	#endif

	// Define Constants
	#include "Config/Constants.h"

	// B107AA Class
	#ifdef _B107AA_

		// Get Module PinOut
		#include "Pinout/B107AA.h"

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

					// Set TERMINAL_SENSE as Input with Pull-Up
					DDR_TERMINAL_SENSE &= ~(1 << PIN_TERMINAL_SENSE);
					PORT_TERMINAL_SENSE |= (1 << PIN_TERMINAL_SENSE);

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

				// Module PCIEx Mask Function [PCIE0]
				inline void PCIEx_Mask(bool _PCIE0 = false, bool _PCIE1 = false, bool _PCIE2 = false) {

					// Control for PCIE0
					if (_PCIE0) {

						// Set PCIE0 Bit on PCICR
						PCICR |= (1 << PCIE0);

					} else {

						// Clear PCIE0 Bit on PCICR
						PCICR &= ~(1 << PCIE0);

					}

					// Control for PCIE1
					if (_PCIE1) {

						// Set PCIE1 Bit on PCICR
						PCICR |= (1 << PCIE1);

					} else {

						// Clear PCIE1 Bit on PCICR
						PCICR &= ~(1 << PCIE1);

					}

					// Control for PCIE2
					if (_PCIE2) {

						// Set PCIE2 Bit on PCICR
						PCICR |= (1 << PCIE2);

					} else {

						// Clear PCIE2 Bit on PCICR
						PCICR &= ~(1 << PCIE2);

					}

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

				// PCINTxx Interrupt Function
				inline void PCINTxx_Interrupt(uint8_t _PCINT, bool _Status = false) {

					// Control for PCINT
					if (_PCINT >= 0 and _PCINT <= 7) {

						// Control for Status
						if (_Status) {

							// Set PCINT0 Bit on PCMSK0
							PCMSK0 |= (1 << _PCINT);

						} else {

							// Clear PCINT0 Bit on PCMSK0
							PCMSK0 &= ~(1 << _PCINT);

						}

					} else if (_PCINT >= 8 and _PCINT <= 15) {

						// Control for Status
						if (_Status) {

							// Set PCINT0 Bit on PCMSK0
							PCMSK1 |= (1 << (_PCINT - 8));

						} else {

							// Clear PCINT0 Bit on PCMSK0
							PCMSK1 &= ~(1 << (_PCINT - 8));

						}

					} else if (_PCINT >= 16 and _PCINT <= 23) {

						// Control for Status
						if (_Status) {

							// Set PCINT0 Bit on PCMSK0
							PCMSK2 |= (1 << (_PCINT - 16));

						} else {

							// Clear PCINT0 Bit on PCMSK0
							PCMSK2 &= ~(1 << (_PCINT - 16));

						}

					}

				}

			// Public Context
			public:

				// Define Interrupt Variables Structure
				struct Interrupt_Struct {

					// Define Interrupt Mask Structure
					struct Interrupt_Mask_Structure {

						// PCINT4 Buffer
						bool PCINT4_State = false;

						// PCINT5 Buffer
						bool PCINT5_State = false;

						// PCINT6 Buffer
						bool PCINT6_State = false;

						// PCINT7 Buffer
						bool PCINT7_State = false;

					} Mask;

					// Define Terminal Display Interrupt Structure
					struct Terminal_Display_Struct {

						// Status Display Interrupt
						bool Status = false;

						// Uptime Display Interrupt
						bool Uptime = false;

						// Environment Display Interrupt
						bool Environment = false;

						// Battery Display Interrupt
						bool Battery = false;

					} Display;

					// Define Interrupt Status Structure
					struct Interrupt_Status_Structure {

						// Define Input Interrupt
						bool Input = false;

						// Define Energy Interrupt
						bool Energy = false;

						// Define Environment Interrupt
						bool Environment = false;

						// Define RS485 Interrupt
						bool RS485 = false;

						// Define RTC Interrupt
						bool RTC = false;

					} Status;

				};

				// Define Terminal Variables Structure
				struct Terminal_Struct {

					// Terminal Variables
					bool Sense = false;

					// Terminal Start
					bool Start = false;

				};

				// Define Interrupt Variables
				static Interrupt_Struct Hardware_Interrupt;
				static Terminal_Struct Hardware_Terminal;

				// Module Constructor
				Hardware(void) {

					// Set Pin Out
					this->Set_PinOut();

				}

				// Begin Function
				void Begin(void) {

					// Control for Terminal Sense
					if (bitRead(PIN_REGISTER_TERMINAL_SENSE, PIN_TERMINAL_SENSE)) {

						// Set Terminal Sense Variable	
						this->Hardware_Terminal.Sense = false;
						
					} else {
						
						// Set Terminal Sense Variable
						this->Hardware_Terminal.Sense = true;
						
					}

					// Disable Interrupts
					cli();

					// Set 1 Second Timer
					this->AVR_Timer();

					// Set INT4 as falling edge triggered Interrupt
					this->INT4_Interrupt(true, false);

					// Set PCINTxx Mask Register
					this->PCIEx_Mask(true, true, true);

					// Set PCINT4 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT4, true);

					// Set PCINT5 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT5, true);

					// Set PCINT6 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT6, true);

					// Set PCINT7 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT7, true);

					// Set PCINT11 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT11, true);

					// Set PCINT12 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT12, true);

					// Set PCINT13 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT13, true);

					// Set PCINT16 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT16, true);

					// Set PCINT17 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT17, true);

					// Set PCINT18 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT18, true);

					// Set PCINT19 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT19, true);

					// Set PCINT20 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT20, true);

					// Set PCINT21 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT21, true);

					// Set PCINT22 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT22, true);

					// Set PCINT23 Interrupt
					this->PCINTxx_Interrupt(INTERRUPT_PCINT23, true);

					// Read Boot Default Interrupt Status
					this->Hardware_Interrupt.Mask.PCINT4_State = bitRead(PIN_REGISTER_INT_ENERGY_1, PIN_INT_ENERGY_1);
					this->Hardware_Interrupt.Mask.PCINT5_State = bitRead(PIN_REGISTER_INT_ENERGY_2, PIN_INT_ENERGY_2);
					this->Hardware_Interrupt.Mask.PCINT6_State = bitRead(PIN_REGISTER_INT_ENV, PIN_INT_ENV);
					this->Hardware_Interrupt.Mask.PCINT7_State = bitRead(PIN_REGISTER_INT_RTC, PIN_INT_RTC);

					// Set Interrupt Updater
					this->Hardware_Interrupt.Status.Energy = this->Hardware_Interrupt.Mask.PCINT4_State and this->Hardware_Interrupt.Mask.PCINT5_State;
					this->Hardware_Interrupt.Status.Environment = this->Hardware_Interrupt.Mask.PCINT6_State;
					this->Hardware_Interrupt.Status.RTC = this->Hardware_Interrupt.Mask.PCINT7_State;

					// Start Interrupts
					sei();

				}

				// LED Function
				void LED(const uint8_t _Color = LED_WHITE, const uint8_t _Blink = 1, const uint16_t _Interval = 500) {

					// Switch Color
					switch (_Color)	{

						// Red Color
						case LED_RED: {

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
						case LED_GREEN: {

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
						case LED_BLUE: {

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
						case LED_WHITE: {

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
					if (_LED) this->LED(LED_GREEN, 1, 50);

					// Turn OFF HeartBeat
					PORT_HEARTBEAT &= ~(1 << PIN_HEARTBEAT);

				}

				// Timer Interrupt Function
				static void TIMER5_Handler(void) {

					// Set UpTime Display Interrupt
					Hardware_Interrupt.Display.Uptime = true;

					// Set Environment Display Interrupt
					Hardware_Interrupt.Display.Environment = true;

					// Set Battery Display Interrupt
					Hardware_Interrupt.Display.Battery = true;

				}

				// INT4 Interrupt Function
				static void INT4_Handler(void) {

					// Set RS485 Interrupt
					Hardware_Interrupt.Status.RS485 = true;

				}

				// PCMSK0 Mask Handler Function
				static void PCMSK0_Handler(void) {

					// Control for Energy Interrupt
					if (Hardware_Interrupt.Mask.PCINT4_State != INT_ENERGY_1) {

						// Set Interrupt
						Hardware_Interrupt.Status.Energy = true;

						// Set Interrupt Buffer
						Hardware_Interrupt.Mask.PCINT4_State = INT_ENERGY_1;

					}
					if (Hardware_Interrupt.Mask.PCINT5_State != INT_ENERGY_2) {

						// Set Interrupt
						Hardware_Interrupt.Status.Energy = true;

						// Set Interrupt Buffer
						Hardware_Interrupt.Mask.PCINT5_State = INT_ENERGY_2;

					}

					// Control for Environment Interrupt
					if (Hardware_Interrupt.Mask.PCINT6_State != INT_ENVIRONMENT) {

						// Set Interrupt
						Hardware_Interrupt.Status.Environment = true;

						// Set Interrupt Buffer
						Hardware_Interrupt.Mask.PCINT6_State = INT_ENVIRONMENT;

					
					}

					// Control for RTC Interrupt
					if (Hardware_Interrupt.Mask.PCINT7_State != INT_RTC) {

						// Set Interrupt
						Hardware_Interrupt.Status.RTC = true;

						// Set Interrupt Buffer
						Hardware_Interrupt.Mask.PCINT7_State = INT_RTC;

					}

				}

				// PCMSK1 Mask Handler Function
				static void PCMSK1_Handler(void) {



				}

				// PCMSK2 Mask Handler Function
				static void PCMSK2_Handler(void) {

					// Set Input Interrupt
					Hardware_Interrupt.Status.Input = true;

				}

		};

		// Define Interrupt Variables Structure
		Hardware::Interrupt_Struct Hardware::Hardware_Interrupt;
		Hardware::Terminal_Struct Hardware::Hardware_Terminal;

		// Interrupt Routine TIMER5
		ISR(TIMER5_COMPA_vect) {

			// Call Timer Handler
			Hardware::TIMER5_Handler();

		}

		// RS485 Interrupt
		ISR(INT4_vect, ISR_NOBLOCK) {

			// Call INT4 Handler
			Hardware::INT4_Handler();

		}

		// Interrupt Routine PCMSK0
		ISR(PCINT0_vect, ISR_NOBLOCK) {

			// PCMSK0 Handler
			Hardware::PCMSK0_Handler();

		}

		// Interrupt Routine PCMSK1
		ISR(PCINT1_vect, ISR_NOBLOCK) {

			// PCMSK1 Handler
			Hardware::PCMSK1_Handler();

		}

		// Interrupt Routine PCMSK2
		ISR(PCINT2_vect, ISR_NOBLOCK) {	

			// PCMSK2 Handler
			Hardware::PCMSK2_Handler();

		}


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

#endif /* defined(Hardware) */
