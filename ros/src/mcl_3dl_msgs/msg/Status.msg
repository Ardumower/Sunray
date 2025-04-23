std_msgs/Header header

# status definitions

uint8 NORMAL = 0
uint8 GLOBAL_LOCALIZATION = 1
uint8 EXPANSION_RESETTING = 2

uint8 status

# error definitions

uint8 ERROR_NORMAL = 0
uint8 ERROR_POINTS_NOT_FOUND = 1

uint8 error

# convergence status definitions

uint8 CONVERGENCE_STATUS_NORMAL = 0
uint8 CONVERGENCE_STATUS_CONVERGED = 1
uint8 CONVERGENCE_STATUS_LARGE_STD_VALUE = 2

uint8 convergence_status

uint32 particle_size
float32 match_ratio
float32 entropy
