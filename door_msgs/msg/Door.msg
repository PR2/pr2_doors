Header header
geometry_msgs/Point32 frame_p1  ## position of the door frame
geometry_msgs/Point32 frame_p2  ## position of the door frame
geometry_msgs/Point32 door_p1   ## Ground plane projection of a point on the plane of the door 
geometry_msgs/Point32 door_p2   ## Ground plane projection of a point on the plane of the door
geometry_msgs/Point32 handle    ## Position of the door handle
float32 height               ## Height of the door

int32 UNKNOWN=0

int32 HINGE_P1=1
int32 HINGE_P2=2
int32 hinge                  

int32 ROT_DIR_CLOCKWISE=1
int32 ROT_DIR_COUNTERCLOCKWISE=2
int32 rot_dir                

int32 LOCKED=1
int32 LATCHED=2
int32 UNLATCHED=3
int32 latch_state            

geometry_msgs/Vector3 travel_dir           ## vector pointing in the direction the robot is going to travel through the door
float32 weight               ## @Sachin: what do we use this for?


