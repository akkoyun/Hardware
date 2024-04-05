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
		#ifndef __Console__
			#include <Console.h>
		#endif
		#ifndef SdFat_h
			#include <SdFat.h>
		#endif

		// Hardware Class
		class B107AA : public SdFat {

			// Private Context
			private:

				// Define Pointers
				PowerStat_Console* Terminal;

				// Define Interrupt Structure
				struct Interrupt_Struct {
					uint16_t Status = 0;
					uint16_t Buffer = 0;
				} Interrupt;

				// Module Pin Definitions
				void Set_PinOut(void) {

					// Set PIN_RELAY_START as Output with Pull-Down
					DDR_RELAY_START |= (1 << PIN_RELAY_START); 			// Output
					PORT_RELAY_START &= ~(1 << PIN_RELAY_START);		// Pull-Down

					// Set PIN_RELAY_STOP as Output with Pull-Down
					DDR_RELAY_STOP |= (1 << PIN_RELAY_STOP); 			// Output
					PORT_RELAY_STOP &= ~(1 << PIN_RELAY_STOP);			// Pull-Down

					// Set PIN_INT_ENERGY_1 as Input with Pull-Up
					DDR_INT_ENERGY_1 &= ~(1 << PIN_INT_ENERGY_1); 		// Input
					PORT_INT_ENERGY_1 |= (1 << PIN_INT_ENERGY_1);		// Pull-Up

					// Set PIN_INT_ENERGY_2 as Input with Pull-Up
					DDR_INT_ENERGY_2 &= ~(1 << PIN_INT_ENERGY_2); 		// Input
					PORT_INT_ENERGY_2 |= (1 << PIN_INT_ENERGY_2);		// Pull-Up

					// Set PIN_INT_ENV as Input with Pull-Up
					DDR_INT_ENV &= ~(1 << PIN_INT_ENV); 				// Input
					PORT_INT_ENV |= (1 << PIN_INT_ENV);					// Pull-Up

					// Set PIN_INT_RTC as Input with Pull-Up
					DDR_INT_RTC &= ~(1 << PIN_INT_RTC); 				// Input
					PORT_INT_RTC |= (1 << PIN_INT_RTC);					// Pull-Up

					// Set PIN_EN_3V8 as Output with Pull-Down
					DDR_EN_3V8 |= (1 << PIN_EN_3V8); 					// Output
					PORT_EN_3V8 &= ~(1 << PIN_EN_3V8);					// Pull-Down

					// Set PIN_RS485_DIR as Output with Pull-Down
					DDR_RS485_DIR |= (1 << PIN_RS485_DIR); 				// Output
					PORT_RS485_DIR &= ~(1 << PIN_RS485_DIR);			// Pull-Down

					// Set PIN_INT_RS485 as Input with Pull-Up
					DDR_INT_RS485 &= ~(1 << PIN_INT_RS485); 			// Input
					PORT_INT_RS485 |= (1 << PIN_INT_RS485);				// Pull-Up

					// Set PIN_INT_CHARGER as Input with Pull-Up
					DDR_INT_CHARGER &= ~(1 << PIN_INT_CHARGER); 		// Input
					PORT_INT_CHARGER |= (1 << PIN_INT_CHARGER);			// Pull-Up

					// Set PIN_INT_GAUGE as Input with Pull-Up
					DDR_INT_GAUGE &= ~(1 << PIN_INT_GAUGE); 			// Input
					PORT_INT_GAUGE |= (1 << PIN_INT_GAUGE);				// Pull-Up

					// Set PIN_BUZZER as Output with Pull-Down
					DDR_BUZZER |= (1 << PIN_BUZZER); 					// Output
					PORT_BUZZER &= ~(1 << PIN_BUZZER);					// Pull-Down

					// Set PIN_LCD_SENSE as Input with Pull-Up
					DDR_LCD_SENSE &= ~(1 << PIN_LCD_SENSE); 			// Input
					PORT_LCD_SENSE |= (1 << PIN_LCD_SENSE);				// Pull-Up

					// Set PIN_FOTA_POWER_EN as Output with Pull-Down
					DDR_FOTA_POWER_EN |= (1 << PIN_FOTA_POWER_EN); 		// Output
					PORT_FOTA_POWER_EN &= ~(1 << PIN_FOTA_POWER_EN);	// Pull-Down

					// Set PIN_SD_SENSE as Input with Pull-Up
					DDR_SD_SENSE &= ~(1 << PIN_SD_SENSE); 				// Input
					PORT_SD_SENSE |= (1 << PIN_SD_SENSE);				// Pull-Up

					// Set PIN_SD_EN as Output with Pull-Down
					DDR_SD_EN |= (1 << PIN_SD_EN); 						// Output
					PORT_SD_EN &= ~(1 << PIN_SD_EN);					// Pull-Down

					// Set PIN_TERMINAL_SENSE as Input with Pull-Up
					DDR_TERMINAL_SENSE &= ~(1 << PIN_TERMINAL_SENSE); 	// Input
					PORT_TERMINAL_SENSE |= (1 << PIN_TERMINAL_SENSE);	// Pull-Up

					// Set PIN_GSM_RING as Input with Pull-Down
					DDR_GSM_RING &= ~(1 << PIN_GSM_RING); 				// Input
					PORT_GSM_RING &= ~(1 << PIN_GSM_RING);				// Pull-Down

					// Set PIN_GSM_PMON as Input with Pull-Down
					DDR_GSM_PMON &= ~(1 << PIN_GSM_PMON); 				// Input
					PORT_GSM_PMON &= ~(1 << PIN_GSM_PMON);				// Pull-Down

					// Set PIN_GSM_SWREADY as Input with Pull-Down
					DDR_GSM_SWREADY &= ~(1 << PIN_GSM_SWREADY); 		// Input
					PORT_GSM_SWREADY &= ~(1 << PIN_GSM_SWREADY);		// Pull-Down

					// Set PIN_GSM_COMM_EN as Output with Pull-Up
					DDR_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN); 			// Output
					PORT_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN);			// Pull-Up

					// Set PIN_GSM_ONOFF as Output with Pull-Down
					DDR_GSM_ONOFF |= (1 << PIN_GSM_ONOFF); 				// Output
					PORT_GSM_ONOFF &= ~(1 << PIN_GSM_ONOFF);			// Pull-Down

					// Set PIN_GSM_SDOWN as Output with Pull-Down
					DDR_GSM_SDOWN |= (1 << PIN_GSM_SDOWN); 				// Output
					PORT_GSM_SDOWN &= ~(1 << PIN_GSM_SDOWN);			// Pull-Down

					// Set PIN_SENSE_1 as Input with Pull-Down
					DDR_SENSE_1 &= ~(1 << PIN_SENSE_1); 				// Input
					PORT_SENSE_1 &= ~(1 << PIN_SENSE_1);				// Pull-Down

					// Set PIN_SENSE_2 as Input with Pull-Down
					DDR_SENSE_2 &= ~(1 << PIN_SENSE_2); 				// Input
					PORT_SENSE_2 &= ~(1 << PIN_SENSE_2);				// Pull-Down

					// Set PIN_SENSE_3 as Input with Pull-Down
					DDR_SENSE_3 &= ~(1 << PIN_SENSE_3); 				// Input
					PORT_SENSE_3 &= ~(1 << PIN_SENSE_3);				// Pull-Down

					// Set PIN_SENSE_4 as Input with Pull-Down
					DDR_SENSE_4 &= ~(1 << PIN_SENSE_4); 				// Input
					PORT_SENSE_4 &= ~(1 << PIN_SENSE_4);				// Pull-Down

					// Set PIN_SENSE_5 as Input with Pull-Down
					DDR_SENSE_5 &= ~(1 << PIN_SENSE_5); 				// Input
					PORT_SENSE_5 &= ~(1 << PIN_SENSE_5);				// Pull-Down

					// Set PIN_SENSE_6 as Input with Pull-Down
					DDR_SENSE_6 &= ~(1 << PIN_SENSE_6); 				// Input
					PORT_SENSE_6 &= ~(1 << PIN_SENSE_6);				// Pull-Down

					// Set PIN_SENSE_7 as Input with Pull-Down
					DDR_SENSE_7 &= ~(1 << PIN_SENSE_7); 				// Input
					PORT_SENSE_7 &= ~(1 << PIN_SENSE_7);				// Pull-Down

					// Set PIN_SENSE_8 as Input with Pull-Down
					DDR_SENSE_8 &= ~(1 << PIN_SENSE_8); 				// Input
					PORT_SENSE_8 &= ~(1 << PIN_SENSE_8);				// Pull-Down

					// Set PIN_MCU_LED_RED as Output with Pull-Down
					DDR_MCU_LED_RED |= (1 << PIN_MCU_LED_RED); 			// Output
					PORT_MCU_LED_RED &= ~(1 << PIN_MCU_LED_RED);		// Pull-Down

					// Set PIN_MCU_LED_GREEN as Output with Pull-Down
					DDR_MCU_LED_GREEN |= (1 << PIN_MCU_LED_GREEN); 		// Output
					PORT_MCU_LED_GREEN &= ~(1 << PIN_MCU_LED_GREEN);	// Pull-Down

					// Set PIN_MCU_LED_BLUE as Output with Pull-Down
					DDR_MCU_LED_BLUE |= (1 << PIN_MCU_LED_BLUE); 		// Output
					PORT_MCU_LED_BLUE &= ~(1 << PIN_MCU_LED_BLUE);		// Pull-Down

					// Set PIN_HEARTBEAT as Output with Pull-Down
					DDR_HEARTBEAT |= (1 << PIN_HEARTBEAT); 				// Output
					PORT_HEARTBEAT &= ~(1 << PIN_HEARTBEAT);			// Pull-Down

				}

				// Interrupt Handler Function
				// --------------------------

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
					uint8_t _PCICR_Mask = PCICR;

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

				// EEPROM Functions
				// ----------------

				// Read PUBLISH Register from EEPROM Function
//				void Read_Register(void) {

					// Read Publish Register
//					this->Register.Publish = (((uint32_t)RV3028::Read_EEPROM(__EEPROM_PUBLISH_MASK_MSB_2__) << 24) | ((uint32_t)RV3028::Read_EEPROM(__EEPROM_PUBLISH_MASK_MSB_1__) << 16) | ((uint32_t)RV3028::Read_EEPROM(__EEPROM_PUBLISH_MASK_LSB_2__) << 8) | (uint32_t)RV3028::Read_EEPROM(__EEPROM_PUBLISH_MASK_LSB_1__));

					// Read Stop Register
//					this->Register.Stop = (((uint32_t)RV3028::Read_EEPROM(__EEPROM_STOP_MASK_MSB_2__) << 24) | ((uint32_t)RV3028::Read_EEPROM(__EEPROM_STOP_MASK_MSB_1__) << 16) | ((uint32_t)RV3028::Read_EEPROM(__EEPROM_STOP_MASK_LSB_2__) << 8) | (uint32_t)RV3028::Read_EEPROM(__EEPROM_STOP_MASK_LSB_1__));

//				}

				// Terminal Functions
				// ------------------

				// Terminal Display Controller
				void Terminal_Control(void) {

					// Control for Terminal Sense
					#if defined(_DEBUG_)

						// Control for Terminal Sense
						if (bitRead(PIN_REGISTER_TERMINAL_SENSE, PIN_TERMINAL_SENSE)) {

							// End Serial Stream
							if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_START)) {

								// End Serial Stream
								Serial.end();

								// Set Terminal Variable
								bitClear(this->Interrupt.Status, INTERRUPT_TERMINAL_START);

							}

							// Clear Terminal Variable
							bitClear(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE);

						} else {

							// Start Serial Stream
							if (!bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_START)) {

								// Start Serial Stream
								Serial.begin(115200);

								// Start Console
								Terminal->Begin();

								// Set Terminal Variable
								bitSet(this->Interrupt.Status, INTERRUPT_TERMINAL_START);

							}

							// Set Terminal Variable
							bitSet(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE);

						}

					#endif

				}

				// Handle Input Functions
				// ----------------------

				// Handle Input Functions
				bool Phase_R(void) {

					// Handle Phase R Status
					if (CONTROL_PHASE_R) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_PHASE_R__);

						// End Function
						return true;

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_PHASE_R__);

						// End Function
						return false;

					}

				}
				bool Phase_S(void) {

					// Handle Phase S Status
					if (CONTROL_PHASE_S) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_PHASE_S__);

						// End Function
						return true;

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_PHASE_S__);

						// End Function
						return false;

					}

				}
				bool Phase_T(void) {

					// Handle Phase T Status
					if (CONTROL_PHASE_T) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_PHASE_T__);

						// End Function
						return true;

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_PHASE_T__);

						// End Function
						return false;

					}

				}
				bool Thermic_Fault(void) {

					// Handle Thermic Fault Status
					if (CONTROL_TH) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_FAULT_TH__);

						// End Function
						return true;

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_FAULT_TH__);

						// End Function
						return false;

					}

				}
				bool Motor_Protection_Relay(void) {

					// Handle Motor Protection Relay Status
					if (CONTROL_MP) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_FAULT_MP__);

						// End Function
						return true;

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_FAULT_MP__);

						// End Function
						return false;

					}

				}
				bool Pump_Status(void) {

					// Handle for Pump Status
					if (CONTROL_M1 && CONTROL_M2 && !CONTROL_M3) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_PUMP__);

					} else {

						// Set Status Register
						bitClear(this->Register.Status, __STATUS_PUMP__);

					}

					// End Function
					return (bitRead(this->Register.Status, __STATUS_PUMP__));

				}
				bool System_Anomaly(void) {

					// Declare Current Status
					bool _SA_Status = bitRead(this->Register.Status, __STATUS_FAULT_SA__);

					// Just M1 Active
					if (CONTROL_M1 && !CONTROL_M2 && !CONTROL_M3) _SA_Status = true;

					// Just M2 Active
					else if (!CONTROL_M1 && CONTROL_M2 && !CONTROL_M3) _SA_Status = true;

					// Just M3 Active
					else if (!CONTROL_M1 && !CONTROL_M2 && CONTROL_M3) _SA_Status = true;

					// Pump Active and TH Active
					else if (CONTROL_M1 && CONTROL_M2 && !CONTROL_M3 && CONTROL_TH) _SA_Status = true;

					// Pump Active and MP Active
					else if (CONTROL_M1 && CONTROL_M2 && !CONTROL_M3 && CONTROL_MP) _SA_Status = true;

					// All Phases Active and MP Active
					else if (CONTROL_PHASE_R && CONTROL_PHASE_S && CONTROL_PHASE_T && CONTROL_MP) _SA_Status = true;

					// System Normal
					else _SA_Status = false;

					// Control for SA Status
					if (_SA_Status) {

						// Set Status Register
						bitSet(this->Register.Status, __STATUS_FAULT_SA__);

					} else {

						// Clear Status Register
						bitClear(this->Register.Status, __STATUS_FAULT_SA__);

					}

					// End Function
					return (_SA_Status);

				}

			// Public Context
			public:

				// Module Instance
				static B107AA* instance;

				// Define Register Structure
				struct Register_Struct {
					uint32_t Status = 0;
					uint32_t Buffer = 0;
					uint32_t Publish = 0;
					uint32_t Stop = 0;
				} Register;

				// Module Constructor
				explicit B107AA(PowerStat_Console* _Terminal) : SdFat(), Terminal(_Terminal) {

					// Set Pin Out
					this->Set_PinOut();

					// Clear Interrupt Variables
					this->Interrupt.Status = 0;
					this->Interrupt.Buffer = 0;

					// Set Instance
					instance = this;

				}

				// Begin Function
				void Begin(void) {

					// Control for Terminal
					this->Terminal_Control();

					// Print Firmware Version
					if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) Terminal->Text(6, 71, _Console_GRAY_, F(_FIRMWARE_));

					// Print Hardware Version
					// TODO: board düzeltilecek
					//Terminal->Text(7, 71, _Console_GRAY_, F(_HARDWARE_));

					// Enable SD Multiplexer
					this->SD_Multiplexer(true);

					// Start SD Card
					if (!SdFat::begin(53, SD_SCK_MHZ(50))) {

						// Set Status Register
						if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) bitSet(this->Register.Status, __STATUS_SYSTEM__);

					}

					// Disable SD Multiplexer
					this->SD_Multiplexer(false);

					// Read Register from EEPROM
//					this->Read_Register();

					// Read Boot Default Interrupt Status
					if (bitRead(PIN_REGISTER_INT_ENERGY_1, PIN_INT_ENERGY_1)) {

						// Set Energy 1 Interrupt
						bitSet(this->Interrupt.Buffer, INTERRUPT_ENERGY_1);

						// Clear Energy 1 Interrupt
						bitClear(this->Interrupt.Status, INTERRUPT_ENERGY_1);

					} else {

						// Clear Energy 1 Interrupt
						bitClear(this->Interrupt.Buffer, INTERRUPT_ENERGY_1);

						// Set Energy 1 Interrupt Status
						bitSet(this->Interrupt.Status, INTERRUPT_ENERGY_1);

					}
					if (bitRead(PIN_REGISTER_INT_ENERGY_2, PIN_INT_ENERGY_2)) {

						// Set Energy 2 Interrupt
						bitSet(this->Interrupt.Buffer, INTERRUPT_ENERGY_2);

						// Clear Energy 2 Interrupt
						bitClear(this->Interrupt.Status, INTERRUPT_ENERGY_2);

					} else {

						// Clear Energy 2 Interrupt
						bitClear(this->Interrupt.Buffer, INTERRUPT_ENERGY_2);

						// Set Energy 2 Interrupt Status
						bitSet(this->Interrupt.Status, INTERRUPT_ENERGY_2);

					}
					if (bitRead(PIN_REGISTER_INT_ENV, PIN_INT_ENV)) {

						// Set Environment Interrupt
						bitSet(this->Interrupt.Buffer, INTERRUPT_ENVIRONMENT);

						// Clear Environment Interrupt
						bitClear(this->Interrupt.Status, INTERRUPT_ENVIRONMENT);

					} else {

						// Clear Environment Interrupt
						bitClear(this->Interrupt.Buffer, INTERRUPT_ENVIRONMENT);

						// Set Environment Interrupt Status
						bitSet(this->Interrupt.Status, INTERRUPT_ENVIRONMENT);

					}
					if (bitRead(PIN_REGISTER_INT_RTC, PIN_INT_RTC)) {

						// Set RTC Interrupt
						bitSet(this->Interrupt.Buffer, INTERRUPT_RTC);

						// Clear RTC Interrupt
						bitClear(this->Interrupt.Status, INTERRUPT_RTC);

					} else {

						// Clear RTC Interrupt
						bitClear(this->Interrupt.Buffer, INTERRUPT_RTC);

						// Set RTC Interrupt Status
						bitSet(this->Interrupt.Status, INTERRUPT_RTC);

					}

					// Control for Inputs
					this->PCMSK2_Handler();

					// Set Buffer to Status
					this->Interrupt.Buffer = this->Interrupt.Status;

					// Disable Interrupts
					cli();

					// Set 1 Second Timer
					this->AVR_Timer();

					// Set INT4 as falling edge triggered Interrupt
					this->INTx_Interrupt(INT4, true, false);

					// Set PCINTxx Mask Register
					this->PCIEx_Mask(true, false, true);

					// Set PCINT4-23 Interrupts
					this->PCINTxx_Interrupt(INTERRUPT_PCINT4, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT5, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT6, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT7, true);

					// Set PCINT16-23 Interrupts
					this->PCINTxx_Interrupt(INTERRUPT_PCINT16, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT17, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT18, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT19, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT20, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT21, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT22, true);
					this->PCINTxx_Interrupt(INTERRUPT_PCINT23, true);

					// Enable Interrupts
					sei();

				}

				// SD Card Functions
				// -----------------

				// SD Multiplexer Function
				void SD_Multiplexer(const bool _State = false) {

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

				// FOTA Power Function
				void Burn_Firmware(void) {

					// Disable SD Multiplexer
					this->SD_Multiplexer(false);

					// Wait for 50ms
					delay(50);

					// Enable FOTA Power
					PORT_FOTA_POWER_EN |= (1 << PIN_FOTA_POWER_EN);

					// Wait for 50ms		
					delay(50);

					// Disable FOTA Power		
					PORT_FOTA_POWER_EN &= ~(1 << PIN_FOTA_POWER_EN);

				}

				// Relay Functions
				// ---------------

				void Relay(bool _Relay = RELAY_START, bool _Lock = RELAY_UNLOCK, uint16_t _Delay = 500) {

					// Start Relay
					if (_Relay) {

						// Set RELAY_START
						PORT_RELAY_START |= (1 << PIN_RELAY_START);

						// Work Delay
						delay(_Delay);

						// Clear RELAY_START
						PORT_RELAY_START &= ~(1 << PIN_RELAY_START);

					} else {

						// Set RELAY_STOP
						PORT_RELAY_STOP |= (1 << PIN_RELAY_STOP);

						// Work Delay
						delay(_Delay);

						// Control for Lock
						if (!_Lock) {

							// Clear RELAY_STOP
							PORT_RELAY_STOP &= ~(1 << PIN_RELAY_STOP);

						}

					}




				}

				// ISR Handler Functions
				// ---------------------

				// Timer Interrupt Function
				static void TIMER5_Handler_Static(void) {

					// Set Interrupt Handler
					if (instance) instance->TIMER5_Handler();

				}
				void TIMER5_Handler(void) {

					// Set Timer Interrupt
					bitSet(this->Interrupt.Status, INTERRUPT_TIMER);

					// Set Display Interrupt
					bitSet(this->Interrupt.Status, INTERRUPT_DISPLAY);

				}

				// INT4 Interrupt Function
				static void INT4_Handler_Static(void) {

					// Set Interrupt Handler
					if (instance) instance->INT4_Handler();

				}
				void INT4_Handler(void) {

					// Set RS485 Interrupt
					bitSet(this->Interrupt.Status, INTERRUPT_RS485);

				}

				// PCMSK0 Mask Handler Function
				static void PCMSK0_Handler_Static(void) {

					// Set Interrupt Handler
					if (instance) instance->PCMSK0_Handler();

				}
				void PCMSK0_Handler(void) {

					// Control for ENERGY Interrupt
					if (bitRead(this->Interrupt.Buffer, INTERRUPT_ENERGY_1) != !bitRead(PIN_REGISTER_INT_ENERGY_1, PIN_INT_ENERGY_1)) {

						// Set ENERGY 1 Interrupt
						if (bitRead(PIN_REGISTER_INT_ENERGY_1, PIN_INT_ENERGY_1)) {

							// Clear ENERGY 1 Interrupt Mask
							bitClear(this->Interrupt.Buffer, INTERRUPT_ENERGY_1);
							
						} else {

							// Set ENERGY 1 Interrupt Mask
							bitSet(this->Interrupt.Buffer, INTERRUPT_ENERGY_1);

						}

						// Set ENERGY 1 Interrupt
						bitSet(this->Interrupt.Status, INTERRUPT_ENERGY_1);

					}

					// Control for ENERGY Interrupt
					if (bitRead(this->Interrupt.Buffer, INTERRUPT_ENERGY_2) != !bitRead(PIN_REGISTER_INT_ENERGY_2, PIN_INT_ENERGY_2)) {

						// Set ENERGY Interrupt
						if (bitRead(PIN_REGISTER_INT_ENERGY_2, PIN_INT_ENERGY_2)) {

							// Clear ENERGY Interrupt Mask
							bitClear(this->Interrupt.Buffer, INTERRUPT_ENERGY_2);
							
						} else {
							
							// Set ENERGY Interrupt Mask
							bitSet(this->Interrupt.Buffer, INTERRUPT_ENERGY_2);

						}

						// Set ENERGY Interrupt
						bitSet(this->Interrupt.Status, INTERRUPT_ENERGY_2);

					}

					// Control for ENVIRONMENT Interrupt
					if (bitRead(this->Interrupt.Buffer, INTERRUPT_ENVIRONMENT) != !bitRead(PIN_REGISTER_INT_ENV, PIN_INT_ENV)) {

						// Set ENVIRONMENT Interrupt
						if (bitRead(PIN_REGISTER_INT_ENV, PIN_INT_ENV)) {

							// Clear ENVIRONMENT Interrupt Mask
							bitClear(this->Interrupt.Buffer, INTERRUPT_ENVIRONMENT);

						} else {
							
							// Set ENVIRONMENT Interrupt Mask	
							bitSet(this->Interrupt.Buffer, INTERRUPT_ENVIRONMENT);

						}

						// Set ENVIRONMENT Interrupt
						bitSet(this->Interrupt.Status, INTERRUPT_ENVIRONMENT);

					}

					// Control for RTC Interrupt
					if (bitRead(this->Interrupt.Buffer, INTERRUPT_RTC) != !bitRead(PIN_REGISTER_INT_RTC, PIN_INT_RTC)) {

						// Set RTC Interrupt
						if (bitRead(PIN_REGISTER_INT_RTC, PIN_INT_RTC)) {
							
							// Clear RTC Interrupt Mask
							bitClear(this->Interrupt.Buffer, INTERRUPT_RTC);
							
						} else {

							// Set RTC Interrupt Mask
							bitSet(this->Interrupt.Buffer, INTERRUPT_RTC);

							// Set RTC Interrupt
							bitSet(this->Interrupt.Status, INTERRUPT_RTC);

						}

					}

				}

				// Static Read Inputs Function
				static void PCMSK2_Handler_Static(void) {

					// Control for Instance
					if (instance) {

						// Read Inputs
						instance->PCMSK2_Handler();

					}

				}
				void PCMSK2_Handler(void) {

					// Handler Delay
					delay(5);

					// Handle Phase Status
					this->Phase_R();
					this->Phase_S();
					this->Phase_T();

					// Handle Defect Status
					this->Thermic_Fault();
					this->Motor_Protection_Relay();

					// Handle Pump Status
					this->Pump_Status();

					// Handle System Anomaly Status
					this->System_Anomaly();

					// Control for Buffer
					if (this->Interrupt.Status != this->Interrupt.Buffer) {

						// Control for Pump Start/Stop
						if ((bitRead(this->Register.Status, __STATUS_PUMP__) != bitRead(this->Register.Buffer, __STATUS_PUMP__))) {

							// Control for Pump Start
							if (bitRead(this->Register.Status, __STATUS_PUMP__)) bitSet(this->Interrupt.Status, INTERRUPT_PUMP_START);

							// Control for Pump Stop
							if (!bitRead(this->Register.Status, __STATUS_PUMP__)) bitSet(this->Interrupt.Status, INTERRUPT_PUMP_STOP);

						}

						// Set Interrupt to Status
						bitSet(this->Interrupt.Status, INTERRUPT_PINCHANGE);

						// Set Buffer to Status
						this->Interrupt.Buffer = this->Interrupt.Status;

					}

				}

				// Interrupt Handler Function
				inline bool Interrupt_Handler(const uint8_t _Type = INTERRUPT_RTC, const bool _Clean = false) {

					// Define Return Status
					bool _Status = false;

					// Control for Type
					switch (_Type) {

						// Timer
						case INTERRUPT_TIMER: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_TIMER);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_TIMER);

							// Return Display Interrupt
							return(_Status);

						}

						// Display
						case INTERRUPT_DISPLAY: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_DISPLAY);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_DISPLAY);

							// Return Display Interrupt
							return(_Status);

						}

						// Energy 1
						case INTERRUPT_ENERGY_1: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_ENERGY_1);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_ENERGY_1);

							// Return Energy 1 Interrupt
							return(_Status);

						}

						// Energy 2
						case INTERRUPT_ENERGY_2: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_ENERGY_2);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_ENERGY_2);

							// Return Energy 2 Interrupt
							return(_Status);

						}

						// Environment
						case INTERRUPT_ENVIRONMENT: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_ENVIRONMENT);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_ENVIRONMENT);

							// Return Environment Interrupt
							return(_Status);

						}

						// RS485
						case INTERRUPT_RS485: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_RS485);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_RS485);

							// Return RS485 Interrupt
							return(_Status);

						}

						// RTC
						case INTERRUPT_RTC: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_RTC);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_RTC);

							// Return RTC Interrupt
							return(_Status);

						}

						// Pin Change
						case INTERRUPT_PINCHANGE: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_PINCHANGE);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_PINCHANGE);

							// Return Pin Change Interrupt
							return(_Status);

						}

						// Pump Start
						case INTERRUPT_PUMP_START: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_PUMP_START);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_PUMP_START);

							// Return Pump Start Interrupt
							return(_Status);

						}

						// Pump Stop
						case INTERRUPT_PUMP_STOP: {

							// Read Status
							_Status = bitRead(this->Interrupt.Status, INTERRUPT_PUMP_STOP);

							// Control for Clean
							if (_Clean && _Status) bitClear(this->Interrupt.Status, INTERRUPT_PUMP_STOP);

							// Return Pump Stop Interrupt
							return(_Status);

						}

					}

					// End Function
					return(false);

				}

				// Terminal Status Function
				inline bool Terminal_Status(void) {

					// Return Terminal Status
					return(bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE));

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

					// Control for Terminal Sense
					this->Terminal_Control();

					// Turn ON HeartBeat
					PORT_HEARTBEAT |= (1 << PIN_HEARTBEAT);

					// HeartBeat LED Blink
					if (_LED) {

						// Blink LED
						if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) {this->LED(LED_GREEN, 1, 50);} else {this->LED(LED_RED, 1, 50);}

					}

					// Turn OFF HeartBeat
					PORT_HEARTBEAT &= ~(1 << PIN_HEARTBEAT);

					// Print Interrupt Status
					if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) {

						// Control for Display Interrupt
						if (this->Interrupt_Handler(INTERRUPT_DISPLAY)) {

							// Print Register Status
							Console.Show_Status(REGISTER_STATUS, this->Register.Status);
							Console.Show_Status(REGISTER_PUBLISH, this->Register.Publish);
							Console.Show_Status(REGISTER_STOP, this->Register.Stop);

						}

					}

				}

				// GSM Functions
				// -------------

				// Set Interrupt
				void Set_GSM_Interrupt(void) {

					// Set Interrupt Mask
					this->PCIEx_Mask(false, true, false);

					// Enable GSM Ring Interrupt
					this->PCINTxx_Interrupt(11, true);

				}

				// Power Switch
				void Power_Switch(const bool _State = false) {

					// Control for _State
					if (_State) {

						// Set PIN_EN_3V8 pin HIGH
						PORT_EN_3V8 |= (1 << PIN_EN_3V8);


					} else {

						// Set PIN_EN_3V8 pin LOW
						PORT_EN_3V8 &= ~(1 << PIN_EN_3V8);

					}

				}

				// Enable Communication Buffer.
				void Communication(const bool _State = false) {

					// Control for _State
					if (_State) {

						// Set GSM_COMM_EN pin LOW
						PORT_GSM_COMM_EN &= ~(1 << PIN_GSM_COMM_EN);

					} else {

						// Set GSM_COMM_EN pin HIGH
						PORT_GSM_COMM_EN |= (1 << PIN_GSM_COMM_EN);

					}

				}

				// On or Off Modem.
				void OnOff(const uint16_t _Time = 1500) {

					// Set PIN_GSM_ONOFF Signal HIGH
					PORT_GSM_ONOFF |= (1 << PIN_GSM_ONOFF);

					// Print Progress Bar
					for (uint8_t i = 84; i < 120; i++) {

						// Print Progress Bar
						if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) Terminal->Text(21, i, _Console_GRAY_, F("█"));

						// Command Delay
						delay(_Time / 35);

					}

					// Clear Progress Bar
					if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) Terminal->Space(21, 84, 36);

					// Set PIN_GSM_ONOFF Signal LOW
					PORT_GSM_ONOFF &= ~(1 << PIN_GSM_ONOFF);

				}

				// ShutDown Modem
				void ShutDown(const uint16_t _Time) {

					// Set PIN_GSM_SDOWN Signal HIGH
					PORT_GSM_SDOWN |= (1 << PIN_GSM_SDOWN);

					// Command Delay
					delay(_Time);

					// Set PIN_GSM_SDOWN Signal LOW
					PORT_GSM_SDOWN &= ~(1 << PIN_GSM_SDOWN);

				}

				// Get Power Monitor
				bool PowerMonitor(void) {

					// Return Power Monitor
					return (bitRead(PIN_REGISTER_GSM_PMON, PIN_GSM_PMON));

				}

				// Get Software Ready
				bool SWReady(void) {

					// Return Software Ready
					return (bitRead(PIN_REGISTER_GSM_SWREADY, PIN_GSM_SWREADY));

				}

				// Power ON Sequence of Modem
				bool ON(void) {

					// Get Start Time
					const uint32_t _Start_Time = millis();

					// Enable GSM Modem Power Switch
					this->Power_Switch(true);  

					// Power On Delay
					delay(10);

					// Set Communication Signal LOW
					this->Communication(true);

					// Communication Delay
					delay(10);

					// Turn On Modem
					if (this->PowerMonitor()) {

						// End Function
						return (true);

					} else {

						// Print Message
						if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) Terminal->Show_Message(_Console_CYAN_, F("Powering ON GSM Modem..."));

						// Send On Off Signal
						this->OnOff(1500);

						// Print Message
						if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) Terminal->Show_Message(_Console_CYAN_, F("Waiting for Power Monitor..."));

						// Wait for Power Monitor
						while (millis() - _Start_Time < 15000) {

							// Control for PWMon (PJ3)
							if (this->PowerMonitor()) {

								// Print Message
								if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) Terminal->Show_Message(_Console_CYAN_, F("Waiting for Ready..."));

								// Wait for Software Ready
								while (millis() - _Start_Time < 30000) {

									// Control for SWReady (PJ4)
									if (this->SWReady()) return (true);

									// Wait Delay
									delay(10);

								}

							}

							// Wait Delay
							delay(10);

						}

					}

					// End Function
					return (false);

				}

				// Power OFF Sequence of Modem
				bool OFF(void) {

					// Print Message
					if (bitRead(this->Interrupt.Status, INTERRUPT_TERMINAL_SENSE)) Terminal->Show_Message(_Console_CYAN_, F("Powering OFF GSM Modem..."));

					// Turn Off Modem
					if (this->PowerMonitor()) {

						// Turn Off Modem
						this->OnOff(2750);

						// Get Start Time
						const uint32_t _Start_Time = millis();

						// Wait for Power Monitor
						while (millis() - _Start_Time < 15000) {

							// Control for PowerMonitor
							if (!this->PowerMonitor()) {

								// Disable GSM Modem Voltage Translator
								this->Communication(false);

								// Disable GSM Modem Main Power Switch
								this->Power_Switch(false);  

								// End Function
								return (true);

							}

						}

						// End Function
						return (false);
						
					} else {

						// Disable GSM Modem Voltage Translator
						this->Communication(false);

						// Disable GSM Modem Main Power Switch
						this->Power_Switch(false);  

					}

					// End Function
					return (true);

				}

		};

		// Define Interrupt Variables Structure
		B107AA* B107AA::instance = nullptr;

		// Interrupt Routine TIMER5
		ISR(TIMER5_COMPA_vect) {

			// Call Timer Handler
			B107AA::TIMER5_Handler_Static();

		}

		// RS485 Interrupt
		ISR(INT4_vect) {

			// Call INT4 Handler
			B107AA::INT4_Handler_Static();

		}

		// Interrupt Routine PCMSK0 [PCINT0 - PCINT7]
		ISR(PCINT0_vect) {

			// PCMSK0 Handler
			B107AA::PCMSK0_Handler_Static();

		}

		// Interrupt Routine PCMSK2 [PCINT16 - PCINT23]
		ISR(PCINT2_vect) {	

			// PCMSK2 Handler
			B107AA::PCMSK2_Handler_Static();

		}

	#endif

#endif /* defined(Hardware) */
