// Set Pin as Output with Pull-down
#define SET_PIN_OUTPUT_PULLDOWN(_PIN) DDR_##_PIN |= (1 << PIN_##_PIN); PORT_##_PIN &= ~(1 << PIN_##_PIN);

// Set Pin as Output with Pull-up
#define SET_PIN_OUTPUT_PULLUP(_PIN) DDR_##_PIN |= (1 << PIN_##_PIN); PORT_##_PIN |= (1 << PIN_##_PIN);

// Set Pin as Input with Pull-down
#define SET_PIN_INPUT_PULLDOWN(_PIN) DDR_##_PIN &= ~(1 << PIN_##_PIN); PORT_##_PIN &= ~(1 << PIN_##_PIN);

// Set Pin as Input with Pull-up
#define SET_PIN_INPUT_PULLUP(_PIN) DDR_##_PIN &= ~(1 << PIN_##_PIN); PORT_##_PIN |= (1 << PIN_##_PIN);
