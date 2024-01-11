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

		// Hardware Class
		class Hardware {

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

				// Define Interrupt Variables Structure
				struct Interrupt_Struct {

					// Define GSM MONI Interval
					uint32_t Timer = 0;

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

						// Define GSM Monitor Interrupt
						bool MONI = false;

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
					this->Hardware_Terminal.Sense = !bitRead(PIN_REGISTER_TERMINAL_SENSE, PIN_TERMINAL_SENSE);

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

					// Read Boot Default Interrupt Status
					this->Hardware_Interrupt.Mask.PCINT4_State = bitRead(PIN_REGISTER_INT_ENERGY_1, PIN_INT_ENERGY_1);
					this->Hardware_Interrupt.Mask.PCINT5_State = bitRead(PIN_REGISTER_INT_ENERGY_2, PIN_INT_ENERGY_2);
					this->Hardware_Interrupt.Mask.PCINT6_State = bitRead(PIN_REGISTER_INT_ENV, PIN_INT_ENV);
					this->Hardware_Interrupt.Mask.PCINT7_State = bitRead(PIN_REGISTER_INT_RTC, PIN_INT_RTC);

					// Set Interrupt Updater
					this->Hardware_Interrupt.Status.Energy = this->Hardware_Interrupt.Mask.PCINT4_State && this->Hardware_Interrupt.Mask.PCINT5_State;
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

					// Increase Timer
					Hardware_Interrupt.Timer++;

					// Handle MONI Interrupt every 300th Timer tick
					if (Hardware_Interrupt.Timer % 300 == 0) {
						Hardware_Interrupt.Status.MONI = true;
					}

					// Set Display Interrupts
					Hardware_Interrupt.Display.Uptime = true;
					Hardware_Interrupt.Display.Environment = true;
					Hardware_Interrupt.Display.Battery = true;

				}

				// INT4 Interrupt Function
				static void INT4_Handler(void) {

					// Set RS485 Interrupt
					Hardware_Interrupt.Status.RS485 = true;

				}

				// PCMSK0 Mask Handler Function
				static void PCMSK0_Handler(void) {

					// Define Interrupt Variables
					bool _Energy_Interrupt = false;
					bool _Environment_Interrupt = false;
					bool _RTC_Interrupt = false;

					// Control for ENERGY Interrupt
					if (Hardware_Interrupt.Mask.PCINT4_State != CONTROL_ENERGY_1) {
						Hardware_Interrupt.Mask.PCINT4_State = CONTROL_ENERGY_1;
						_Energy_Interrupt = true;
					}

					// Control for ENERGY Interrupt
					if (Hardware_Interrupt.Mask.PCINT5_State != CONTROL_ENERGY_2) {
						Hardware_Interrupt.Mask.PCINT5_State = CONTROL_ENERGY_2;
						_Energy_Interrupt = true;
					}

					// Control for ENVIRONMENT Interrupt
					if (Hardware_Interrupt.Mask.PCINT6_State != CONTROL_ENVIRONMENT) {
						Hardware_Interrupt.Mask.PCINT6_State = CONTROL_ENVIRONMENT;
						_Environment_Interrupt = true;
					}

					// Control for RTC Interrupt
					if (Hardware_Interrupt.Mask.PCINT7_State != CONTROL_RTC) {
						Hardware_Interrupt.Mask.PCINT7_State = CONTROL_RTC;
						_RTC_Interrupt = true;
					}

					// Set Interrupt Updater
					Hardware_Interrupt.Status.Energy = _Energy_Interrupt;
					Hardware_Interrupt.Status.Environment = _Environment_Interrupt;
					Hardware_Interrupt.Status.RTC = _RTC_Interrupt;

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

		// Interrupt Routine PCMSK0 [PCINT0 - PCINT7]
		ISR(PCINT0_vect, ISR_NOBLOCK) {

			// PCMSK0 Handler
			Hardware::PCMSK0_Handler();

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
