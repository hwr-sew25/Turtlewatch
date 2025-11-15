from fileinput import filelineno
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from influxdb_client_3 import InfluxDBClient3, Point, WriteOptions
from datetime import datetime
import time
import logging

# Setup Python logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger('InfluxLogger')

class InfluxLogger:
    def __init__(self):
        rospy.init_node('turtlewatch', anonymous=True)
        
        # Setup InfluxDB client
        logger.info("Reading InfluxDB token...")
        with open("../influxdb_token.txt", "r") as file:
            db_token = file.read().strip()
        
        logger.info("Connecting to InfluxDB...")
        self.client = InfluxDBClient3(
            host="http://localhost:8181",
            database="dev",
            token=db_token
        )
        
        logger.info("Successfully connected to InfluxDB (database: dev)")
        
        # Rate limiters - track last write time for each topic
        self.last_cmd_vel_time = 0
        self.last_odom_time = 0
        self.rate_limit = 1.0  # seconds
        
        # Statistics counters
        self.cmd_vel_received = 0
        self.cmd_vel_written = 0
        self.odom_received = 0
        self.odom_written = 0
        
        # Subscribe to topics
        logger.info("Subscribing to ROS topics...")
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        logger.info("Subscribed to /cmd_vel and /odom (rate limited to 1 Hz)")
        logger.info("TurtleWatch logger is running...")
        
        # Start statistics timer (log stats every 10 seconds)
        rospy.Timer(rospy.Duration(10.0), self.log_statistics)
    
    def should_write(self, last_time):
        """Check if enough time has passed since last write"""
        current_time = time.time()
        if current_time - last_time >= self.rate_limit:
            return True, current_time
        return False, last_time
    
    def cmd_vel_callback(self, msg):
        """Write cmd_vel data to InfluxDB"""
        self.cmd_vel_received += 1
        
        should_write, new_time = self.should_write(self.last_cmd_vel_time)
        
        if not should_write:
            logger.debug(f"[CMD_VEL] Skipped (rate limited) - received: {self.cmd_vel_received}")
            return
        
        self.last_cmd_vel_time = new_time
        
        try:
            point = (
                Point("cmd_vel")
                .tag("robot_id", "turtlebot_01")
                .field("linear_x", float(msg.linear.x))
                .field("linear_y", float(msg.linear.y))
                .field("linear_z", float(msg.linear.z))
                .field("angular_x", float(msg.angular.x))
                .field("angular_y", float(msg.angular.y))
                .field("angular_z", float(msg.angular.z))
            )
            
            self.client.write(point)
            self.cmd_vel_written += 1
            logger.info(f"[CMD_VEL] ✓ Written - linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}")
            
        except Exception as e:
            logger.error(f"[CMD_VEL] ✗ Failed to write: {e}", exc_info=True)
    
    def odom_callback(self, msg):
        """Write odometry data to InfluxDB"""
        self.odom_received += 1
        
        should_write, new_time = self.should_write(self.last_odom_time)
        
        if not should_write:
            logger.debug(f"[ODOM] Skipped (rate limited) - received: {self.odom_received}")
            return
        
        self.odom_time = new_time
        
        try:
            point = (
                Point("odometry")
                .tag("robot_id", "turtlebot_01")
                .tag("frame_id", msg.header.frame_id)
                .field("pos_x", float(msg.pose.pose.position.x))
                .field("pos_y", float(msg.pose.pose.position.y))
                .field("pos_z", float(msg.pose.pose.position.z))
                .field("vel_linear_x", float(msg.twist.twist.linear.x))
                .field("vel_angular_z", float(msg.twist.twist.angular.z))
                .time(msg.header.stamp.to_nsec())  # Use ROS timestamp
            )
            
            self.client.write(point)
            self.odom_written += 1
            logger.info(f"[ODOM] ✓ Written - pos=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}), vel={msg.twist.twist.linear.x:.2f}")
            
        except Exception as e:
            logger.error(f"[ODOM] ✗ Failed to write: {e}", exc_info=True)
    
    def log_statistics(self, event):
        """Log statistics periodically"""
        logger.info("=" * 60)
        logger.info("STATISTICS:")
        logger.info(f"  cmd_vel - received: {self.cmd_vel_received}, written: {self.cmd_vel_written}")
        logger.info(f"  odom    - received: {self.odom_received}, written: {self.odom_written}")
        logger.info("=" * 60)
    
    def run(self):
        logger.info("Spinning... Press Ctrl+C to stop")
        rospy.spin()
        logger.info("Shutting down TurtleWatch logger")

def main():
    try:
        influx_logger = InfluxLogger()
        influx_logger.run()
    except rospy.ROSInterruptException:
        logger.info("TurtleWatch logger interrupted")
    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        raise

if __name__ == "__main__":
    main()
