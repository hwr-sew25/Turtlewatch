#Value if the robot is moving or not 

Header header 

bool is_moving                  #Can be [True,False]
string movement_status          #Can be [moving,standing,emergency_stop,reverse, has_a_goal, is_returning_to_startposition]

float32 linear_velocity         #m/s vorwärts/rückwärts 
float32 angular_velocity        #rad/s Drehung 

float32 left_wheel_velocity     #linkes rad rad/s 
float32 right_wheel_velocity    #rechtes rad rad/s
