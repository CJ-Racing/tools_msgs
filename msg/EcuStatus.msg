Header header

uint8 TASK_IDLE             = 0
uint8 TASK_INSPECTION       = 1
uint8 TASK_MANUAL_DRIVING   = 2
uint8 TASK_EBS_TEST         = 3
uint8 TASK_ACCELERATION     = 4
uint8 TASK_SKIDPAD          = 5
uint8 TASK_TRACKDRIVER      = 6

uint8 AS_IDLE      = 0
uint8 AS_READY     = 1
uint8 AS_RUNNING   = 2
uint8 AS_COMPLETE  = 3
uint8 AS_EMERGENCY = 4

uint8 ami_out      # 任务编号
bool  go           # 启动确认 
uint8 as_state     # AS 主状态 
