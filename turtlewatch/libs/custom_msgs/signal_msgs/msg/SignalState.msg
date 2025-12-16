# FILE: msg/SignalState.msg

# --- ENUM DEFINITIONS (Constants) ---
uint8 IDLE=0
uint8 GREETING=1
uint8 SPEAKING=2
uint8 MOVING=3
uint8 STOPPED=4
uint8 REVERSE=5
uint8 ERROR_MINOR=6
uint8 ERROR_CRITICAL=7
uint8 LOW_BATTERY=8

# --- MESSAGE FIELDS ---
Header header
uint8 state