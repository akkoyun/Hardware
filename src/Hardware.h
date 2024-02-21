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
	#ifndef Arduino_h
		#include <Arduino.h>
	#endif

	// Include Config Files
	#include "Config/Constants.h"
	#include "Config/Macros.h"
	#include "Config/Config.h"

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
		class B107AA : public RV3028, public DS28C, public HDC2010, public MAX17055, public BQ24298 {

			// Private Context
			private:

				// Module Pin Definitions
				void Set_PinOut(void) {

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
				void AVR_Timer(void) {

					// Set CTC Mod and Rescale (1024)
					TCCR5A = 0;
					TCCR5B = (1 << WGM52) | (1 << CS52) | (1 << CS50);

					// Set Counter Value
					OCR5A = (F_CPU / 1024) - 1;

					// Enable Timer Interrupt
					TIMSK5 |= (1 << OCIE5A);

				}

				// Module INTx Interrupt Function [INT0 or INT1]
				void INTx_Interrupt(uint8_t intNum, bool _State = false, bool _Trigger = false) {

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
				void PCIEx_Mask(bool _PCIE0 = false, bool _PCIE1 = false, bool _PCIE2 = false) {

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
				void PCINTxx_Interrupt(uint8_t _PCINT, bool _Status = false) {

					// Set PCINTxx Interrupt
					if (_PCINT <= 7) {

						// Control for Status
						if (_Status) {

							// Set PCINTxx [0 - 7]
							PCMSK0 |= (1 << _PCINT);

						} else {

							// Clear PCINTxx [0 - 7]
							PCMSK0 &= ~(1 << _PCINT);

						}

					} else if (_PCINT <= 15) {

						// Control for Status
						if (_Status) {

							// Set PCINTxx [8 - 15]
							PCMSK1 |= (1 << (_PCINT - 8));

						} else {

							// Clear PCINTxx [8 - 15]
							PCMSK1 &= ~(1 << (_PCINT - 8));

						}

					} else if (_PCINT <= 23) {

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

				// Read PUBLISH Register from EEPROM Function
				void Read_Register(void) {

					// Read Publish Register
					this->Register.Publish = (((uint32_t)RV3028::Read_EEPROM(__EEPROM_PUBLISH_MASK_MSB_2__) << 24) | ((uint32_t)RV3028::Read_EEPROM(__EEPROM_PUBLISH_MASK_MSB_1__) << 16) | ((uint32_t)RV3028::Read_EEPROM(__EEPROM_PUBLISH_MASK_LSB_2__) << 8) | (uint32_t)RV3028::Read_EEPROM(__EEPROM_PUBLISH_MASK_LSB_1__));

					// Read Stop Register
					this->Register.Stop = (((uint32_t)RV3028::Read_EEPROM(__EEPROM_STOP_MASK_MSB_2__) << 24) | ((uint32_t)RV3028::Read_EEPROM(__EEPROM_STOP_MASK_MSB_1__) << 16) | ((uint32_t)RV3028::Read_EEPROM(__EEPROM_STOP_MASK_LSB_2__) << 8) | (uint32_t)RV3028::Read_EEPROM(__EEPROM_STOP_MASK_LSB_1__));

				}

			// Public Context
			public:

				// Define Register Structure
				struct Register_Struct {
					uint32_t Status = 0x00000000;
					uint32_t Buffer = 0x00000000;
					uint32_t Publish = 0x00000000;
					uint32_t Stop = 0x00000000;
				} Register;

				// Define Time Variables Structure
				struct Time_Struct {
					uint32_t Last_Loop_Time = 0;
					uint32_t Loop_Time = 0;
				} Time;

				// Control for Debug
				#if defined(_DEBUG_)

					// Define Terminal Variables Structure
					struct Terminal_Struct {
						bool Sense = false;
						bool Start = false;
					} Terminal;

				#endif

				// Define Interrupt Structure
				struct Interrupt_Mask_Structure {
					bool PCINT4_State = false;
					bool PCINT5_State = false;
					bool PCINT6_State = false;
					bool PCINT7_State = false;
				};
				struct Interrupt_Status_Structure {
					bool Display = false;
					bool Energy = false;
					bool Environment = false;
					bool RS485 = false;
					bool RTC = false;
				};

				// Define Interrupt Variables
				static Interrupt_Mask_Structure Interrupt_Mask;
				static Interrupt_Status_Structure Interrupt_Status;

				// Module Instance
				static B107AA* instance;

				// Module Constructor
				B107AA(void) : RV3028(), DS28C(), HDC2010(), MAX17055(), BQ24298() {

					// Set Pin Out
					this->Set_PinOut();

					// Set Instance
					instance = this;

				}

				// Begin Function
				void Begin(void) {

					// Start Objects
					DS28C::Begin();
					RV3028::Begin();
					HDC2010::Begin();
					MAX17055::Begin();
					BQ24298::Begin();

					// Read Register from EEPROM
					this->Read_Register();

					// Read Boot Default Interrupt Status
					this->Interrupt_Mask.PCINT4_State = CONTROL_ENERGY_1;
					this->Interrupt_Mask.PCINT5_State = CONTROL_ENERGY_2;
					this->Interrupt_Mask.PCINT6_State = CONTROL_ENVIRONMENT;
					this->Interrupt_Mask.PCINT7_State = CONTROL_RTC;

					// Set Interrupt Updater
					this->Interrupt_Status.Energy = this->Interrupt_Mask.PCINT4_State && this->Interrupt_Mask.PCINT5_State;
					this->Interrupt_Status.Environment = this->Interrupt_Mask.PCINT6_State;
					this->Interrupt_Status.RTC = this->Interrupt_Mask.PCINT7_State;

					// Control for Inputs
					this->Read_Inputs();

					// Disable Interrupts
					cli();

					// Set 1 Second Timer
					this->AVR_Timer();

					// Set INT4 as falling edge triggered Interrupt
					this->INTx_Interrupt(INT4, true, false);

					// Set PCINTxx Mask Register
					this->PCIEx_Mask(true, false, true);

					// Set PCINT4-23 Interrupts
					this->PCINTxx_Interrupt(4, true);
					this->PCINTxx_Interrupt(5, true);
					this->PCINTxx_Interrupt(6, true);
					this->PCINTxx_Interrupt(7, true);

					// Set PCINT16-23 Interrupts
					this->PCINTxx_Interrupt(16, true);
					this->PCINTxx_Interrupt(17, true);
					this->PCINTxx_Interrupt(18, true);
					this->PCINTxx_Interrupt(19, true);
					this->PCINTxx_Interrupt(20, true);
					this->PCINTxx_Interrupt(21, true);
					this->PCINTxx_Interrupt(22, true);
					this->PCINTxx_Interrupt(23, true);

					// Enable Interrupts
					sei();

					// Set Last Loop Time
					this->Time.Last_Loop_Time = millis();

				}

				// Terminal Functions
				// ------------------

				// Terminal Display Controller
				bool Terminal_Control(void) {

					// Control for Terminal Sense
					#if defined(_DEBUG_)

						// Set Terminal Sense Variable	
						if (CONTROL_TERMINAL) {

							// End Serial Stream
							if (this->Terminal.Start) {

								// End Serial Stream
								Serial.end();

								// Set Terminal Variable
								this->Terminal.Start = false;

							}

							// End Function
							return(false);

						} else {

							// Start Serial Stream
							if (!this->Terminal.Start) {

								// Set Terminal Start
								Serial.begin(115200);

								// Set Terminal Variable
								this->Terminal.Start = true;

							}

							// End Function
							return(true);

						}

					#endif

					// End Function
					return(false);

				}

				// Input Control Functions
				// -----------------------

				// Static Read Inputs Function
			    static void Read_Inputs_Static() {

					// Control for Instance
			        if (instance) {

						// Read Inputs
            			instance->Read_Inputs();

        			}

    			}

				// Read Input Signal Function
				void Read_Inputs(void) {

					// Set Input Port Variables
					uint8_t _Port_Buffer = PINK;

					// Handle Phase R Status
					if (bitRead(_Port_Buffer, __INPUT_PIN_R__)) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_PHASE_R__);

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_PHASE_R__);

					}

					// Handle Phase S Status
					if (bitRead(_Port_Buffer, __INPUT_PIN_S__)) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_PHASE_S__);

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_PHASE_S__);

					}

					// Handle Phase T Status
					if (bitRead(_Port_Buffer, __INPUT_PIN_T__)) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_PHASE_T__);

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_PHASE_T__);

					}

					// Handle Thermic Relay Status
					if (bitRead(_Port_Buffer, __INPUT_PIN_TH__)) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_FAULT_TH__);

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_FAULT_TH__);

					}

					// Handle Motor Protection Relay Status
					if (bitRead(_Port_Buffer, __INPUT_PIN_MP__)) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_FAULT_MP__);

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_FAULT_MP__);

					}

					// Handle for Pump Status
					if (bitRead(_Port_Buffer, __INPUT_PIN_M1__) && bitRead(_Port_Buffer, __INPUT_PIN_M2__) && !bitRead(_Port_Buffer, __INPUT_PIN_M3__)) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_PUMP__);

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_PUMP__);

					}

					// Handle for System Anomaly Status
					if (
						(bitRead(_Port_Buffer, __INPUT_PIN_M1__) && !bitRead(_Port_Buffer, __INPUT_PIN_M2__) && !bitRead(_Port_Buffer, __INPUT_PIN_M3__)) || 																		// Just M1 Active
						(!bitRead(_Port_Buffer, __INPUT_PIN_M1__) && bitRead(_Port_Buffer, __INPUT_PIN_M2__) && !bitRead(_Port_Buffer, __INPUT_PIN_M3__)) || 																		// Just M2 Active
						(!bitRead(_Port_Buffer, __INPUT_PIN_M1__) && !bitRead(_Port_Buffer, __INPUT_PIN_M2__) && bitRead(_Port_Buffer, __INPUT_PIN_M3__)) || 																		// Just M3 Active
						(bitRead(this->Register.Status, __STATUS_PUMP__) && bitRead(_Port_Buffer, __INPUT_PIN_TH__)) || 																											// Pump Active and TH Active
						(bitRead(this->Register.Status, __STATUS_PUMP__) && bitRead(_Port_Buffer, __INPUT_PIN_MP__)) || 																											// Pump Active and MP Active
						(bitRead(this->Register.Status, __STATUS_PHASE_R__) && bitRead(this->Register.Status, __STATUS_PHASE_S__) && bitRead(this->Register.Status, __STATUS_PHASE_T__) && bitRead(_Port_Buffer, __INPUT_PIN_MP__))	// All Phases Active and MP Active
					) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_FAULT_SA__);

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_FAULT_SA__);

					}

				}

				// SD Card Functions
				// -----------------

				// SD Multiplexer Function
				void SD_Multiplexer(const bool _State) {

					// Control for SD Sense
					if (_State) {

						// Set SD_EN
						PORT_SD_EN |= (1 << PIN_SD_EN);

					} else {

						// Clear SD_EN
						PORT_SD_EN &= ~(1 << PIN_SD_EN);

					}

					// SD Wait Delay
					delay(200);

				}

				// ISR Handler Functions
				// ---------------------

				// Timer Interrupt Function
				static void TIMER5_Handler(void) {

					// Set Display Interrupts
					Interrupt_Status.Display = true;

				}

				// INT4 Interrupt Function
				static void INT4_Handler(void) {

					// Set RS485 Interrupt
					Interrupt_Status.RS485 = true;

				}

				// PCMSK0 Mask Handler Function
				static void PCMSK0_Handler(void) {

					// Control for ENERGY Interrupt
					if (Interrupt_Mask.PCINT4_State != CONTROL_ENERGY_1) {
						Interrupt_Mask.PCINT4_State = CONTROL_ENERGY_1;
						Interrupt_Status.Energy = true;
					}

					// Control for ENERGY Interrupt
					if (Interrupt_Mask.PCINT5_State != CONTROL_ENERGY_2) {
						Interrupt_Mask.PCINT5_State = CONTROL_ENERGY_2;
						Interrupt_Status.Energy = true;
					}

					// Control for ENVIRONMENT Interrupt
					if (Interrupt_Mask.PCINT6_State != CONTROL_ENVIRONMENT) {
						Interrupt_Mask.PCINT6_State = CONTROL_ENVIRONMENT;
						Interrupt_Status.Environment = true;
					}

					// Control for RTC Interrupt
					if (Interrupt_Mask.PCINT7_State != CONTROL_RTC) {
						Interrupt_Mask.PCINT7_State = CONTROL_RTC;
						Interrupt_Status.RTC = true;
					}

				}

				// LED Functions
				// -------------

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

				// WatchDog Functions
				// ------------------

				// Heartbeat Function
				void Heartbeat(const bool _LED = false) {

					// Turn ON HeartBeat
					PORT_HEARTBEAT |= (1 << PIN_HEARTBEAT);

					// HeartBeat LED Blink
					if (_LED) this->LED(LED_GREEN, 1, 50);

					// Turn OFF HeartBeat
					PORT_HEARTBEAT &= ~(1 << PIN_HEARTBEAT);

					// Calculate Loop Time
					this->Time.Loop_Time = (millis() - this->Time.Last_Loop_Time);
					this->Time.Last_Loop_Time = millis();

				}

		};

		// Define Interrupt Variables Structure
		B107AA::Interrupt_Mask_Structure B107AA::Interrupt_Mask;
		B107AA::Interrupt_Status_Structure B107AA::Interrupt_Status;
		B107AA* B107AA::instance = nullptr;

		// Interrupt Routine TIMER5
		ISR(TIMER5_COMPA_vect) {

			// Call Timer Handler
			B107AA::TIMER5_Handler();

		}

		// RS485 Interrupt
		ISR(INT4_vect) {

			// Call INT4 Handler
			B107AA::INT4_Handler();

		}

		// Interrupt Routine PCMSK0 [PCINT0 - PCINT7]
		ISR(PCINT0_vect) {

			// PCMSK0 Handler
			B107AA::PCMSK0_Handler();

		}

		// Interrupt Routine PCMSK2 [PCINT16 - PCINT23]
		ISR(PCINT2_vect) {	

			// PCMSK2 Handler
			B107AA::Read_Inputs_Static();

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
