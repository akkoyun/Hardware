// Include Arduino Library
#ifndef Arduino_h
    #include <Arduino.h>
#endif

// Define Input Pins
// -----------------
#ifndef __INPUT_PIN_R__
    #define __INPUT_PIN_R__			(uint8_t)0
#endif
#ifndef __INPUT_PIN_S__
    #define __INPUT_PIN_S__			(uint8_t)1
#endif
#ifndef __INPUT_PIN_T__
    #define __INPUT_PIN_T__			(uint8_t)2
#endif
#ifndef __INPUT_PIN_M1__
    #define __INPUT_PIN_M1__		(uint8_t)3
#endif
#ifndef __INPUT_PIN_M2__
    #define __INPUT_PIN_M2__		(uint8_t)4
#endif
#ifndef __INPUT_PIN_M3__
    #define __INPUT_PIN_M3__		(uint8_t)5
#endif
#ifndef __INPUT_PIN_TH__
    #define __INPUT_PIN_TH__		(uint8_t)6
#endif
#ifndef __INPUT_PIN_MP__
    #define __INPUT_PIN_MP__		(uint8_t)7
#endif

// Define Register Bits
// --------------------
#ifndef __STATUS_PUMP__
    #define __STATUS_PUMP__     	(uint8_t)0
#endif
#ifndef __STATUS_PHASE_R__
    #define __STATUS_PHASE_R__  	(uint8_t)1
#endif
#ifndef __STATUS_PHASE_S__
    #define __STATUS_PHASE_S__  	(uint8_t)2
#endif
#ifndef __STATUS_PHASE_T__
    #define __STATUS_PHASE_T__  	(uint8_t)3
#endif
#ifndef __STATUS_FAULT_TH__
    #define __STATUS_FAULT_TH__ 	(uint8_t)5
#endif
#ifndef __STATUS_FAULT_MP__
    #define __STATUS_FAULT_MP__ 	(uint8_t)6
#endif
#ifndef __STATUS_FAULT_SA__
    #define __STATUS_FAULT_SA__		(uint8_t)7
#endif
#ifndef __STATUS_INPUT__
    #define __STATUS_INPUT__		(uint8_t)8
#endif
#ifndef __STATUS_P_LOW__
    #define __STATUS_P_LOW__		(uint8_t)10
#endif
#ifndef __STATUS_P_HIGH__
    #define __STATUS_P_HIGH__		(uint8_t)11
#endif
#ifndef __STATUS_P_DROP__
    #define __STATUS_P_DROP__		(uint8_t)12
#endif
#ifndef __STATUS_P_RISE__
    #define __STATUS_P_RISE__		(uint8_t)13
#endif
#ifndef __STATUS_V_LOW__
    #define __STATUS_V_LOW__		(uint8_t)20
#endif
#ifndef __STATUS_V_HIGH__
    #define __STATUS_V_HIGH__		(uint8_t)21
#endif
#ifndef __STATUS_I_HIGH__
    #define __STATUS_I_HIGH__		(uint8_t)22
#endif
#ifndef __STATUS_FQ_LOW__
    #define __STATUS_FQ_LOW__		(uint8_t)23
#endif
#ifndef __STATUS_FQ_HIGH__
    #define __STATUS_FQ_HIGH__		(uint8_t)24
#endif
#ifndef __STATUS_V_IMBALANCE__
    #define __STATUS_V_IMBALANCE__	(uint8_t)25
#endif
#ifndef __STATUS_I_IMBALANCE__
    #define __STATUS_I_IMBALANCE__	(uint8_t)26
#endif
#ifndef __STATUS_ENVIRONMENT__
    #define __STATUS_ENVIRONMENT__	(uint8_t)30
#endif
#ifndef __STATUS_SYSTEM__
    #define __STATUS_SYSTEM__		(uint8_t)31
#endif
