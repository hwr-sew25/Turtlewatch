import logging
import random
import threading
import time
from typing import Callable

import genpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from bridge.types import Seconds

logger = logging.getLogger("BridgeLogger")

def mock_sub(topic_name:str, msg_class: type[genpy.Message], callback: Callable[[genpy.Message], None], interval: Seconds)-> None:
    thread = threading.Thread(target=dispatcher, args=(topic_name,msg_class,callback,interval))
    thread.start()

        
def dispatcher(topic_name:str, msg_class: type[genpy.Message], callback: Callable[[genpy.Message], None],interval: Seconds)-> None:
    topics = {
        "/cmd_vel": cmd_vel_handler,
        "/odom": odom_handler,
    }

    handler = topics.get(topic_name)
    if not handler:
        logger.error(f"Mock handler for {topic_name} not implemented yet")
    else:
        topics[topic_name](msg_class, callback, interval)

# TODO check if we can remove the timing stuff from the handlers (no hot looping)
def cmd_vel_handler(msg_class: type[genpy.Message], callback: Callable[[genpy.Message], None],interval: Seconds) -> None:
    """Simulates a robot moving forward and turning slightly."""
    iteration = 0
    start_time = time.time()
    while True:
        msg = Twist()
        
        msg.linear.x = 0.5 + random.uniform(-0.05, 0.05) 
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.1 
        
        callback(msg)
        
        iteration += 1
        next_time = start_time + (iteration * interval)
        sleep_duration = next_time - time.time()
        if sleep_duration > 0:
            time.sleep(sleep_duration)

def odom_handler(msg_class: type[genpy.Message], callback: Callable[[genpy.Message], None], interval: Seconds) -> None:
    """Simulates odometry data (position and velocity)."""
    
    # Keep track of simulated position
    x_pos = 0.0
    y_pos = 0.0
    
    iteration = 0
    start_time = time.time()
    while True:
        msg = Odometry()
        
        msg.header = Header()
        msg.header.stamp = genpy.Time.from_sec(time.time())
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        # 2. Simulate Position (Pose) - moving in a simple line for testing
        x_pos += 0.05 
        msg.pose.pose.position.x = x_pos
        msg.pose.pose.position.y = y_pos
        msg.pose.pose.position.z = 0.0
        
        # Valid Quaternion (identity = no rotation)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # 3. Simulate Velocity (Twist)
        msg.twist.twist.linear.x = 0.5
        msg.twist.twist.angular.z = 0.0

        callback(msg)
        
        iteration += 1
        next_time = start_time + (iteration * interval)
        sleep_duration = next_time - time.time()
        if sleep_duration > 0:
            time.sleep(sleep_duration)
