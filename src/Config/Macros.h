// Set Pin as Output with Pull-down
#define SET_PIN_OUTPUT_PULLDOWN(_PIN) DDR_##_PIN |= (1 << PIN_##_PIN); PORT_##_PIN &= ~(1 << PIN_##_PIN);

// Set Pin as Output with Pull-up
#define SET_PIN_OUTPUT_PULLUP(_PIN) DDR_##_PIN |= (1 << PIN_##_PIN); PORT_##_PIN |= (1 << PIN_##_PIN);

// Set Pin as Input with Pull-down
#define SET_PIN_INPUT_PULLDOWN(_PIN) DDR_##_PIN &= ~(1 << PIN_##_PIN); PORT_##_PIN &= ~(1 << PIN_##_PIN);

// Set Pin as Input with Pull-up
#define SET_PIN_INPUT_PULLUP(_PIN) DDR_##_PIN &= ~(1 << PIN_##_PIN); PORT_##_PIN |= (1 << PIN_##_PIN);

// B107AA Macro Definitions
// ------------------------
#ifdef _B107AA_

    // Define Terminal Sense Macro
    #define CONTROL_TERMINAL (((PIN_REGISTER_TERMINAL_SENSE) >> (PIN_TERMINAL_SENSE)) & 0x01)

    // Define Input Port Macros
    #define CONTROL_PHASE_R (((PIN_REGISTER_SENSE_1) >> (PIN_SENSE_1)) & 0x01)
    #define CONTROL_PHASE_S (((PIN_REGISTER_SENSE_2) >> (PIN_SENSE_2)) & 0x01)
    #define CONTROL_PHASE_T (((PIN_REGISTER_SENSE_3) >> (PIN_SENSE_3)) & 0x01)
    #define CONTROL_M1 (((PIN_REGISTER_SENSE_4) >> (PIN_SENSE_4)) & 0x01)
    #define CONTROL_M2 (((PIN_REGISTER_SENSE_5) >> (PIN_SENSE_5)) & 0x01)
    #define CONTROL_M3 (((PIN_REGISTER_SENSE_6) >> (PIN_SENSE_6)) & 0x01)
    #define CONTROL_TH (((PIN_REGISTER_SENSE_7) >> (PIN_SENSE_7)) & 0x01)
    #define CONTROL_MP (((PIN_REGISTER_SENSE_8) >> (PIN_SENSE_8)) & 0x01)

    // Define Interrupt Port Macros
    #define CONTROL_ENERGY_1 (((PIN_REGISTER_INT_ENERGY_1) >> (PIN_INT_ENERGY_1)) & 0x01)	// PCINT4
    #define CONTROL_ENERGY_2 (((PIN_REGISTER_INT_ENERGY_2) >> (PIN_INT_ENERGY_2)) & 0x01)	// PCINT5
    #define CONTROL_ENVIRONMENT (((PIN_REGISTER_INT_ENV) >> (PIN_INT_ENV)) & 0x01)			// PCINT6
    #define CONTROL_RTC (((PIN_REGISTER_INT_RTC) >> (PIN_INT_RTC)) & 0x01)					// PCINT7
    #define CONTROL_RS485 (((PIN_REGISTER_INT_RS485) >> (PIN_INT_RS485)) & 0x01)
    #define CONTROL_GSM_RING (((PIN_REGISTER_GSM_RING) >> (PIN_GSM_RING)) & 0x01)			// PCINT11
    #define CONTROL_GSM_PMON (((PIN_REGISTER_GSM_PMON) >> (PIN_GSM_PMON)) & 0x01)			// PCINT12
    #define CONTROL_GSM_SWREADY (((PIN_REGISTER_GSM_SWREADY) >> (PIN_GSM_SWREADY)) & 0x01)	// PCINT13

#endif // B107AA



