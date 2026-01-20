# Aktueller Signalzustand des Roboters
# Published auf: /signals/current_state
# Team: Signals
Header header
# State-ID (entspricht SignalState Enum)
uint8 state_id
# State-Name als String für Debugging
string state_name
# LED aktiv?
bool led_active
# Audio aktiv?
bool audio_active
# Konstanten für state_id
uint8 GREETING = 0
uint8 IDLE = 1
uint8 BUSY = 2
uint8 STOP_BUSY = 3
uint8 ERROR_MINOR_STUCK = 4
uint8 ERROR_MINOR_NAV = 5
uint8 ROOM_NOT_FOUND = 6
uint8 ERROR_MAJOR = 7
uint8 LOW_BATTERY = 8
uint8 MOVE = 9
uint8 START_MOVE = 10
uint8 STOP_MOVE = 11
uint8 GOAL_REACHED = 12
uint8 REVERSE = 13
uint8 SPEAKING = 14