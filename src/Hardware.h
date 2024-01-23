/* *******************************************************************************
 *  Copyright (C) 2014-2023 Mehmet Gunce Akkoyun Can not be copied and/or
 *	distributed without the express permission of Mehmet Gunce Akkoyun.
 *
 *	Library				: Hardware
 *	Code Developer		: Mehmet Gunce Akkoyun (akkoyun@me.com)
 *						: Mustafa Recep Senbaş (recepsenbas@gmail.com)
 *
 *********************************************************************************/

#ifndef __Hardware__
#define __Hardware__

	// Include Arduino Library
	#ifndef __Arduino__
		#include <Arduino.h>
	#endif

	// Include Config Files
	#include "Config/Constants.h"
	#include "Config/Macros.h"

	// B107AA Class
	#ifdef _B107AA_

		// Get Module PinOut
		#include "Pinout/B107AA.h"

		// Include Libraries
		#ifndef __RV3028__
			#include <RV3028.h>
		#endif
		#ifndef __DS28C__
			#include <DS28C.h>
		#endif
		#ifndef __Environment__
			#include <Environment.h>
		#endif
		#ifndef __MAX17055__
			#include <MAX17055.h>
		#endif
		#ifndef __BQ24298__
			#include <BQ24298.h>
		#endif

		// Hardware Class
		class Hardware : public RV3028, public DS28C, public HDC2010, public MAX17055, public BQ24298 {

			// Private Context
			private:

				// Module Pin Definitions
				inline void Set_PinOut(void) {
					
					// Set Module Pin Definitions
					SET_PIN_OUTPUT_PULLDOWN(RELAY_START);		// Set RELAY_START as Output with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(RELAY_STOP);		// Set RELAY_STOP as Output with Pull-Down
					SET_PIN_INPUT_PULLUP(INT_ENERGY_1);			// Set INT_ENERGY_1 as Input with Pull-Up
					SET_PIN_INPUT_PULLUP(INT_ENERGY_2);			// Set INT_ENERGY_2 as Input with Pull-Up
					SET_PIN_INPUT_PULLUP(INT_ENV);				// Set INT_ENV as Input with Pull-Up
					SET_PIN_INPUT_PULLUP(INT_RTC);				// Set INT_RTC as Input with Pull-Up
					SET_PIN_OUTPUT_PULLDOWN(EN_3V8);			// Set 3V8_EN as Output with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(RS485_DIR);			// Set RS485_DIR as Output with Pull-Down
					SET_PIN_INPUT_PULLUP(INT_RS485);			// Set INT_RS485 as Input with Pull-Down
					SET_PIN_INPUT_PULLUP(INT_CHARGER);			// Set INT_CHARGER as Input with Pull-Up
					SET_PIN_INPUT_PULLUP(INT_GAUGE);			// Set INT_GAUGE as Input with Pull-Up
					SET_PIN_OUTPUT_PULLDOWN(BUZZER);			// Set 3V3_BUZZER as Output with Pull-Down
					SET_PIN_INPUT_PULLUP(LCD_SENSE);			// Set LCD_SENSE as Input with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(FOTA_POWER_EN);		// Set FOTA_POWER_EN as Output with Pull-Down
					SET_PIN_INPUT_PULLUP(SD_SENSE);				// Set SD_SENSE as Input with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(SD_EN);				// Set SD_EN as Output with Pull-Down
					SET_PIN_INPUT_PULLUP(TERMINAL_SENSE);		// Set TERMINAL_SENSE as Input with Pull-Up
					SET_PIN_INPUT_PULLDOWN(GSM_RING);			// Set GSM_RING as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(GSM_PMON);			// Set GSM_PMON as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(GSM_SWREADY);		// Set GSM_SWREADY as Input with Pull-Down
					SET_PIN_OUTPUT_PULLUP(GSM_COMM_EN);			// Set GSM_COMM_EN as Output with Pull-Up
					SET_PIN_OUTPUT_PULLDOWN(GSM_ONOFF);			// Set GSM_ONOFF as Output with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(GSM_SDOWN);			// Set GSM_SDOWN as Output with Pull-Down
					SET_PIN_INPUT_PULLDOWN(SENSE_1);			// Set SENSE_1 as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(SENSE_2);			// Set SENSE_2 as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(SENSE_3);			// Set SENSE_3 as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(SENSE_4);			// Set SENSE_4 as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(SENSE_5);			// Set SENSE_5 as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(SENSE_6);			// Set SENSE_6 as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(SENSE_7);			// Set SENSE_7 as Input with Pull-Down
					SET_PIN_INPUT_PULLDOWN(SENSE_8);			// Set SENSE_8 as Input with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(MCU_LED_RED);		// Set MCU_LED_RED as Output with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(MCU_LED_GREEN);		// Set MCU_LED_GREEN as Output with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(MCU_LED_BLUE);		// Set MCU_LED_BLUE as Output with Pull-Down
					SET_PIN_OUTPUT_PULLDOWN(HEARTBEAT);			// Set HEARTBEAT as Output with Pull-Down

				}

				// Module 1 Second Timer
				inline void AVR_Timer() {

					// Set CTC Mod and Rescale (1024)
					TCCR5A = 0;
					TCCR5B = (1 << WGM52) | (1 << CS52) | (1 << CS50);

					// Set Counter Value
					OCR5A = (F_CPU / 1024) - 1;

					// Enable Timer Interrupt
					TIMSK5 |= (1 << OCIE5A);

				}

				// Module INTx Interrupt Function [INT0 or INT1]
				inline void INTx_Interrupt(uint8_t intNum, bool _State = false, bool _Trigger = false) {

					// Control for State
					if (_State) {

						// Set INTx
						EIMSK |= (1 << intNum);

						// Set EICRA or EICRB based on intNum
						if (intNum <= INT3) {
							EICRA = (_Trigger) ? (EICRA | (1 << (intNum * 2 + 1)) | (1 << (intNum * 2))) : (EICRA | ((1 << (intNum * 2 + 1)) & ~(1 << (intNum * 2))));
						} else if (intNum <= INT7) {
							EICRB = (_Trigger) ? (EICRB | (1 << ((intNum - INT4) * 2 + 1)) | (1 << ((intNum - INT4) * 2))) : (EICRB | ((1 << ((intNum - INT4) * 2 + 1)) & ~(1 << ((intNum - INT4) * 2))));
						}

					} else {

						// Clear INTx
						EIMSK &= ~(1 << intNum);

					}

				}

				// Module PCIEx Mask Function [PCIE0]
				inline void PCIEx_Mask(bool _PCIE0 = false, bool _PCIE1 = false, bool _PCIE2 = false) {

					// Define PCICR Mask
					uint8_t _PCICR_Mask = 0x00;

					// Set PCICR Mask
					if (_PCIE0) _PCICR_Mask |= (1 << PCIE0);	// Set PCIE0
					if (_PCIE1) _PCICR_Mask |= (1 << PCIE1);	// Set PCIE1
					if (_PCIE2) _PCICR_Mask |= (1 << PCIE2);	// Set PCIE2

					// Set PCICR with the calculated mask
					PCICR = _PCICR_Mask;

				}

				// PCINTxx Interrupt Function
				inline void PCINTxx_Interrupt(uint8_t _PCINT, bool _Status = false) {

					// Set PCINTxx Interrupt
					if (_PCINT >= 0 && _PCINT <= 7) {

						// Control for Status
						if (_Status) {

							// Set PCINTxx [0 - 7]
							PCMSK0 |= (1 << _PCINT);

						} else {

							// Clear PCINTxx [0 - 7]
							PCMSK0 &= ~(1 << _PCINT);

						}

					} else if (_PCINT >= 8 && _PCINT <= 15) {

						// Control for Status
						if (_Status) {

							// Set PCINTxx [8 - 15]
							PCMSK1 |= (1 << (_PCINT - 8));

						} else {

							// Clear PCINTxx [8 - 15]
							PCMSK1 &= ~(1 << (_PCINT - 8));

						}

					} else if (_PCINT >= 16 && _PCINT <= 23) {

						// Control for Status
						if (_Status) {

							// Set PCINTxx [16 - 23]
							PCMSK2 |= (1 << (_PCINT - 16));

						} else {

							// Clear PCINTxx [16 - 23]
							PCMSK2 &= ~(1 << (_PCINT - 16));

						}

					}

				}

			// Public Context
			public:

				// Define Register Structure
				struct B107AA_Register_Struct {

					// Status Register
					uint32_t Status = 0x00000000;

					// Buffer Register
					uint32_t Buffer = 0x00000000;

					// Publish Register
					uint32_t Publish = 0x00000000;

					// Stop Register
					uint32_t Stop = 0x00000000;

				};

				// Define Time Variables Structure
				struct B107AA_Time_Struct {

					// Define Loop Timer
					uint32_t Last_Loop_Time = 0;

					// Define Loop Time
					uint16_t Loop_Time = 0;

				};

				// Define Interrupt Variables Structure
				struct B107AA_Interrupt_Struct {

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

						// PCINT11 Buffer
						bool PCINT11_State = false;

						// PCINT12 Buffer
						bool PCINT12_State = false;

					} Mask;

					// Define Interrupt Status Structure
					struct Interrupt_Status_Structure {

						// Define Display Interrupt
						bool Display = false;

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

						// Define GSM Interrupt Structure
						struct B107AA_GSM_Interrupt_Structure {

							// Define GSM Ring Interrupt
							bool Ring = false;

							// Define GSM Power Monitor Interrupt
							bool Power_Monitor = false;

						} GSM;

					} Status;

				};

				// Define Terminal Variables Structure
				struct B107AA_Terminal_Struct {

					// Terminal Variables
					bool Sense = false;

					// Terminal Start
					bool Start = false;

				};

				// Define Interrupt Variables
				B107AA_Time_Struct B107AA_Time;
				B107AA_Terminal_Struct B107AA_Terminal;
				B107AA_Register_Struct B107AA_Register;
				static B107AA_Interrupt_Struct B107AA_Interrupt;

				// Module Constructor
				Hardware(void) : RV3028(), DS28C(), HDC2010(), MAX17055(), BQ24298() {

					// Set Pin Out
					this->Set_PinOut();

				}

				// Begin Function
				void Begin(void) {

					// Control for Terminal Sense
					this->B107AA_Terminal.Sense = !CONTROL_TERMINAL;

					// Disable Interrupts
					cli();

					// Set 1 Second Timer
					this->AVR_Timer();

					// Set INT4 as falling edge triggered Interrupt
					this->INTx_Interrupt(INT4, true, false);

					// Set PCINTxx Mask Register
					this->PCIEx_Mask(true, true, true);

					// Set PCINT4-23 Interrupts
					for (uint8_t i = INTERRUPT_PCINT4; i <= INTERRUPT_PCINT23; i++) this->PCINTxx_Interrupt(i, true);

					// Get Serial ID
					DS28C::Begin();

					// Start RTC
					RV3028::Begin();

					// Start TH Sensor
					HDC2010::Begin();

					// Start Battery Gauge
					MAX17055::Begin();

					// Start Charger
					BQ24298::Begin();

					// Start Serial Communication
					SERIAL_TERMINAL.begin(115200);
					SERIAL_ENERGY.begin(38400);
					SERIAL_GSM.begin(115200);

					// Read Register from EEPROM
					this->Read_Publish_Register();
					this->Read_Stop_Register();

					// Read Boot Default Interrupt Status
					this->B107AA_Interrupt.Mask.PCINT4_State = CONTROL_ENERGY_1;
					this->B107AA_Interrupt.Mask.PCINT5_State = CONTROL_ENERGY_2;
					this->B107AA_Interrupt.Mask.PCINT6_State = CONTROL_ENVIRONMENT;
					this->B107AA_Interrupt.Mask.PCINT7_State = CONTROL_RTC;
					this->B107AA_Interrupt.Mask.PCINT11_State = CONTROL_GSM_RING;
					this->B107AA_Interrupt.Mask.PCINT12_State = CONTROL_GSM_PMON;

					// Set Interrupt Updater
					this->B107AA_Interrupt.Status.Energy = this->B107AA_Interrupt.Mask.PCINT4_State && this->B107AA_Interrupt.Mask.PCINT5_State;
					this->B107AA_Interrupt.Status.Environment = this->B107AA_Interrupt.Mask.PCINT6_State;
					this->B107AA_Interrupt.Status.RTC = this->B107AA_Interrupt.Mask.PCINT7_State;
					this->B107AA_Interrupt.Status.GSM.Ring = this->B107AA_Interrupt.Mask.PCINT11_State;
					this->B107AA_Interrupt.Status.GSM.Power_Monitor = this->B107AA_Interrupt.Mask.PCINT12_State;

					// Set Last Loop Time
					this->B107AA_Time.Last_Loop_Time = millis();

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

				// Terminal Display Controller
				void Terminal_Control(void) {

					// Control for Terminal Sense
					#if defined(_DEBUG_)

						// Set Terminal Sense Variable	
						if (CONTROL_TERMINAL) {

							// Set Terminal Sense Variable
							this->B107AA_Terminal.Sense = false;

							// End Serial Stream
							if (this->B107AA_Terminal.Start) Serial.end();

							// Set Terminal Variable
							this->B107AA_Terminal.Start = false;

						} else {

							// Set Terminal Sense Variable
							this->B107AA_Terminal.Sense = true;

							// Start Serial Stream
							if (!this->B107AA_Terminal.Start) {

								// Set Terminal Start
								Serial.begin(115200);

								// Set Terminal Variable
								this->B107AA_Terminal.Start = true;

							}

						}

					#endif

				}

				// Read PUBLISH Register from EEPROM Function
				void Read_Publish_Register(void) {

					// Declare Variables
					uint8_t _Publish_Register_MSB_2;
					uint8_t _Publish_Register_MSB_1;
					uint8_t _Publish_Register_LSB_2;
					uint8_t _Publish_Register_LSB_1;

					// Read EEPROM
					RV3028::EEPROM(READ, __EEPROM_PUBLISH_MASK_MSB_2__, _Publish_Register_MSB_2);
					RV3028::EEPROM(READ, __EEPROM_PUBLISH_MASK_MSB_1__, _Publish_Register_MSB_1);
					RV3028::EEPROM(READ, __EEPROM_PUBLISH_MASK_LSB_2__, _Publish_Register_LSB_2);
					RV3028::EEPROM(READ, __EEPROM_PUBLISH_MASK_LSB_1__, _Publish_Register_LSB_1);

					// Control for EEPROM
					if (_Publish_Register_MSB_2 == 0x00 && _Publish_Register_MSB_1 == 0x00 && _Publish_Register_LSB_2 == 0x00 && _Publish_Register_LSB_1 == 0x00) {

						// Define Default Values
						uint8_t _Publish_Register_Default_MSB_2 = ((__PUBLISH_REGISTER_DEFAULT__ >> 24) & 0xFF);
						uint8_t _Publish_Register_Default_MSB_1 = ((__PUBLISH_REGISTER_DEFAULT__ >> 16) & 0xFF);
						uint8_t _Publish_Register_Default_LSB_2 = ((__PUBLISH_REGISTER_DEFAULT__ >> 8) & 0xFF);
						uint8_t _Publish_Register_Default_LSB_1 = ((__PUBLISH_REGISTER_DEFAULT__ >> 0) & 0xFF);

						// Set Default Value
						RV3028::EEPROM(WRITE, __EEPROM_PUBLISH_MASK_MSB_2__, _Publish_Register_Default_MSB_2);
						RV3028::EEPROM(WRITE, __EEPROM_PUBLISH_MASK_MSB_1__, _Publish_Register_Default_MSB_1);
						RV3028::EEPROM(WRITE, __EEPROM_PUBLISH_MASK_LSB_2__, _Publish_Register_Default_LSB_2);
						RV3028::EEPROM(WRITE, __EEPROM_PUBLISH_MASK_LSB_1__, _Publish_Register_Default_LSB_1);

						// Set Default Value
						this->B107AA_Register.Publish = __PUBLISH_REGISTER_DEFAULT__;

					} else {

						// Set Default Value
						this->B107AA_Register.Publish = (((uint32_t)_Publish_Register_MSB_2 << 24) | ((uint32_t)_Publish_Register_MSB_1 << 16) | ((uint32_t)_Publish_Register_LSB_2 << 8) | (uint32_t)_Publish_Register_LSB_1);

					}

				}

				// Read STOP Register from EEPROM Function
				void Read_Stop_Register(void) {

					// Declare Variables
					uint8_t _Stop_Register_MSB_2;
					uint8_t _Stop_Register_MSB_1;
					uint8_t _Stop_Register_LSB_2;
					uint8_t _Stop_Register_LSB_1;

					// Read EEPROM
					RV3028::EEPROM(READ, __EEPROM_STOP_MASK_MSB_2__, _Stop_Register_MSB_2);
					RV3028::EEPROM(READ, __EEPROM_STOP_MASK_MSB_1__, _Stop_Register_MSB_1);
					RV3028::EEPROM(READ, __EEPROM_STOP_MASK_LSB_2__, _Stop_Register_LSB_2);
					RV3028::EEPROM(READ, __EEPROM_STOP_MASK_LSB_1__, _Stop_Register_LSB_1);

					// Control for EEPROM
					if (_Stop_Register_MSB_2 == 0x00 && _Stop_Register_MSB_1 == 0x00 && _Stop_Register_LSB_2 == 0x00 && _Stop_Register_LSB_1 == 0x00) {

						// Define Default Values
						uint8_t _Stop_Register_Default_MSB_2 = ((__STOP_REGISTER_DEFAULT__ >> 24) & 0xFF);
						uint8_t _Stop_Register_Default_MSB_1 = ((__STOP_REGISTER_DEFAULT__ >> 16) & 0xFF);
						uint8_t _Stop_Register_Default_LSB_2 = ((__STOP_REGISTER_DEFAULT__ >> 8) & 0xFF);
						uint8_t _Stop_Register_Default_LSB_1 = ((__STOP_REGISTER_DEFAULT__ >> 0) & 0xFF);

						// Set Default Value
						RV3028::EEPROM(WRITE, __EEPROM_STOP_MASK_MSB_2__, _Stop_Register_Default_MSB_2);
						RV3028::EEPROM(WRITE, __EEPROM_STOP_MASK_MSB_1__, _Stop_Register_Default_MSB_1);
						RV3028::EEPROM(WRITE, __EEPROM_STOP_MASK_LSB_2__, _Stop_Register_Default_LSB_2);
						RV3028::EEPROM(WRITE, __EEPROM_STOP_MASK_LSB_1__, _Stop_Register_Default_LSB_1);

						// Set Default Value
						this->B107AA_Register.Stop = __STOP_REGISTER_DEFAULT__;

					} else {

						// Set Default Value
						this->B107AA_Register.Stop = (((uint32_t)_Stop_Register_MSB_2 << 24) | ((uint32_t)_Stop_Register_MSB_1 << 16) | ((uint32_t)_Stop_Register_LSB_2 << 8) | (uint32_t)_Stop_Register_LSB_1);

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

					// Calculate Loop Time
					this->B107AA_Time.Loop_Time = (millis() - this->B107AA_Time.Last_Loop_Time);
					B107AA_Time.Last_Loop_Time = millis();

					// Control Terminal
					this->Terminal_Control();

				}

				// Timer Interrupt Function
				static void TIMER5_Handler(void) {

					// Set Display Interrupts
					B107AA_Interrupt.Status.Display = true;

				}

				// INT4 Interrupt Function
				static void INT4_Handler(void) {

					// Set RS485 Interrupt
					B107AA_Interrupt.Status.RS485 = true;

				}

				// PCMSK0 Mask Handler Function
				static void PCMSK0_Handler(void) {

					// Control for ENERGY Interrupt
					if (B107AA_Interrupt.Mask.PCINT4_State != CONTROL_ENERGY_1) {
						B107AA_Interrupt.Mask.PCINT4_State = CONTROL_ENERGY_1;
						B107AA_Interrupt.Status.Energy = true;
					}

					// Control for ENERGY Interrupt
					if (B107AA_Interrupt.Mask.PCINT5_State != CONTROL_ENERGY_2) {
						B107AA_Interrupt.Mask.PCINT5_State = CONTROL_ENERGY_2;
						B107AA_Interrupt.Status.Energy = true;
					}

					// Control for ENVIRONMENT Interrupt
					if (B107AA_Interrupt.Mask.PCINT6_State != CONTROL_ENVIRONMENT) {
						B107AA_Interrupt.Mask.PCINT6_State = CONTROL_ENVIRONMENT;
						B107AA_Interrupt.Status.Environment = true;
					}

					// Control for RTC Interrupt
					if (B107AA_Interrupt.Mask.PCINT7_State != CONTROL_RTC) {
						B107AA_Interrupt.Mask.PCINT7_State = CONTROL_RTC;
						B107AA_Interrupt.Status.RTC = true;
					}

				}

				// PCMSK1 Mask Handler Function
				static void PCMSK1_Handler(void) {

					// Control for RING Interrupt
					if (B107AA_Interrupt.Mask.PCINT11_State != CONTROL_GSM_RING) {
						B107AA_Interrupt.Mask.PCINT11_State = CONTROL_GSM_RING;
						B107AA_Interrupt.Status.GSM.Ring = true;
					}

					// Control for PMON Interrupt
					if (B107AA_Interrupt.Mask.PCINT12_State != CONTROL_GSM_PMON) {
						B107AA_Interrupt.Mask.PCINT12_State = CONTROL_GSM_PMON;
						B107AA_Interrupt.Status.GSM.Power_Monitor = true;
					}

				}

				// PCMSK2 Mask Handler Function
				static void PCMSK2_Handler(void) {

					// Set Input Interrupt
					B107AA_Interrupt.Status.Input = true;

				}

		};

		// Define Interrupt Variables Structure
		Hardware::B107AA_Interrupt_Struct Hardware::B107AA_Interrupt;

		// Interrupt Routine TIMER5
		ISR(TIMER5_COMPA_vect, ISR_NOBLOCK) {

			// Call Timer Handler
			Hardware::TIMER5_Handler();

		}

		// RS485 Interrupt
		ISR(INT4_vect, ISR_NOBLOCK) {

			// Call INT4 Handler
			Hardware::INT4_Handler();

		}

		// Interrupt Routine PCMSK0 [PCINT0 - PCINT7]
		ISR(PCINT0_vect, ISR_NOBLOCK) {

			// PCMSK0 Handler
			Hardware::PCMSK0_Handler();

		}

		// Interrupt Routine PCMSK1 [PCINT8 - PCINT15]
		ISR(PCINT1_vect, ISR_NOBLOCK) {

			// PCMSK1 Handler
			Hardware::PCMSK1_Handler();

		}

		// Interrupt Routine PCMSK2 [PCINT16 - PCINT23]
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
