## Motion plan request message (via topic)
## request_type constants
uint8 REQUEST_PLAN_EXECUTE=0    # Plan and execute
uint8 REQUEST_CANCEL=1          # Cancel specified request_id
uint8 REQUEST_RESET=2           # Reset to default pose

int32 request_id
uint8  request_type

## Motion target (consistent with JointMotion.action)
bool      use_gravity_compensation
int32[]   joint_indices           # Empty array means reset to default
float64[] target_positions        # Target joint positions (rad)
float64[] target_velocities       # Target joint velocities (empty for auto-planning)
float64   execution_time          # Expected execution time (s)
float64[] stiffness               # Optional, empty array for default
float64[] damping                 # Optional, empty array for default