# Status-Update für Remote Monitoring und Display
# Published auf: /signals/status_update
# Team: Signals
Header header
# Aktueller State
uint8 current_state
string current_state_name
# Vorheriger State
uint8 previous_state
string previous_state_name
# Zeitstempel des State-Wechsels
time state_changed_at
# Zusätzliche Info (z.B. Fehlerdetails)
string info