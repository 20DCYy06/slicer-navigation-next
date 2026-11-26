uint8 IDLE=0
uint8 PREPARING=1
uint8 PLANNING=2
uint8 NAVIGATING=3
uint8 VERIFYING=4
uint8 STOPPING=5

uint8 GNSS_UNKNOWN=0
uint8 GNSS_SINGLE=1
uint8 GNSS_FLOAT=2
uint8 GNSS_FIXED=3

uint8 ALIGNMENT_UNAVAILABLE=0
uint8 ALIGNMENT_DEGRADED=1
uint8 ALIGNMENT_READY=2

# The snapshot time can advance while individual source stamps remain stale.
std_msgs/Header header
geometry_msgs/PoseStamped pose
bool pose_valid
geometry_msgs/TwistStamped velocity
bool velocity_valid
uint8 gnss_fix_type
bool gnss_valid
time gnss_stamp
time gnss_received_stamp
uint8 alignment_state
uint64 transform_version
bool base_online
bool estop
bool manual_override
bool hold
bool stop_valid
bool stopped
string active_velocity_source
uint64 control_epoch
string nav_goal_id
string reason
string geographic_reason
string navigation_reason
string goal_id
uint8 phase
string last_goal_id
uint16 last_result_code
bool last_result_valid
time last_result_stamp
