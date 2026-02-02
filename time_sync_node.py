import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

class ScanTimeSync(Node):
    def __init__(self):
        super().__init__('scan_time_sync')

        # QoS Settings (Best Effort is usually better for Sensors)
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Subscribe to the original (broken time) Scan
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_policy)

        # 2. Publish the new (fixed time) Scan
        self.pub_scan = self.create_publisher(LaserScan, '/scan_synced', qos_policy)
        
        # Time Sync Variables
        self.time_offset = Duration(seconds=0)
        self.time_synced = False
        
        self.get_logger().info("Waiting for first scan to calculate time offset...")

    def scan_callback(self, msg):
        # --- 1. Calculate Offset (Only runs once) ---
        if not self.time_synced:
            # Get the current time on THIS computer (WSL)
            pc_time = self.get_clock().now()
            
            # Get the time the Robot THINKS it is (from the message)
            robot_time = Time.from_msg(msg.header.stamp)
            
            # Calculate the difference so we can align them
            self.time_offset = pc_time - robot_time
            
            self.get_logger().info(f"Time Sync Locked! Offset: {self.time_offset.nanoseconds / 1e9}s")
            self.time_synced = True

        # --- 2. Fix the Timestamp ---
        # Take the original robot time
        original_stamp = Time.from_msg(msg.header.stamp)
        
        # Add the offset to make it match the PC time
        synced_stamp = original_stamp + self.time_offset
        
        # Update the message header
        msg.header.stamp = synced_stamp.to_msg()

        # --- 3. Republish ---
        self.pub_scan.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanTimeSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
