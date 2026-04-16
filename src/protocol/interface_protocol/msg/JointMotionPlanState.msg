# Planner status
uint8 STATUS_DISABLED = 0
uint8 IDLE = 1
uint8 EXECUTING =2
uint8 EXITING = 3

int32 request_id
uint8  status

## Progress and time
float64 progress          # 0.0~1.0, overall planning/execution progress

