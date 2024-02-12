// Include Arduino Library
#ifndef Arduino_h
    #include <Arduino.h>
#endif

// Define Input Pins
// -----------------
#define __INPUT_PIN_R__			(uint8_t)0
#define __INPUT_PIN_S__			(uint8_t)1
#define __INPUT_PIN_T__			(uint8_t)2
#define __INPUT_PIN_M1__		(uint8_t)3
#define __INPUT_PIN_M2__		(uint8_t)4
#define __INPUT_PIN_M3__		(uint8_t)5
#define __INPUT_PIN_TH__		(uint8_t)6
#define __INPUT_PIN_MP__		(uint8_t)7

// Define Register Bits
// --------------------
#define __STATUS_PUMP__     	(uint8_t)0
#define __STATUS_PHASE_R__  	(uint8_t)1
#define __STATUS_PHASE_S__  	(uint8_t)2
#define __STATUS_PHASE_T__  	(uint8_t)3
#define __STATUS_FAULT_TH__ 	(uint8_t)5
#define __STATUS_FAULT_MP__ 	(uint8_t)6
#define __STATUS_FAULT_SA__		(uint8_t)7
#define __STATUS_INPUT__		(uint8_t)8
#define __STATUS_P_LOW__		(uint8_t)10
#define __STATUS_P_HIGH__		(uint8_t)11
#define __STATUS_P_DROP__		(uint8_t)12
#define __STATUS_P_RISE__		(uint8_t)13
#define __STATUS_V_LOW__		(uint8_t)20
#define __STATUS_V_HIGH__		(uint8_t)21
#define __STATUS_I_HIGH__		(uint8_t)22
#define __STATUS_FQ_LOW__		(uint8_t)23
#define __STATUS_FQ_HIGH__		(uint8_t)24
#define __STATUS_V_IMBALANCE__	(uint8_t)25
#define __STATUS_I_IMBALANCE__	(uint8_t)26
#define __STATUS_CT_R__			(uint8_t)28
#define __STATUS_CT_S__			(uint8_t)29
#define __STATUS_CT_T__			(uint8_t)30
#define __STATUS_SYSTEM__		(uint8_t)31
