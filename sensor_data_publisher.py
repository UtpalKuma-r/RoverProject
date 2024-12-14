import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
import math
import serial
import time

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.publisher_ = self.create_publisher(Imu, 'imudata', 10)
        self.publisherMag_ = self.create_publisher(MagneticField, 'magdata', 10)
        self.publisherTemp_ = self.create_publisher(Temperature, 'tempdata', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)  # Publish every 1 second

    def timer_callback(self):
        # Create a new IMU message
        imu_msg = Imu()
        mag_msg = MagneticField()
        temp_msg = Temperature()

        # Set header
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        mag_msg.header = Header()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        
        temp_msg.header = Header()
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.arduino.in_waiting > 0:
            received = self.arduino.readline().decode("utf-8").rstrip()
            if(received not in ["0", "0Data Underflow", "0All is well"]):
                #self.get_logger().info(received)
                data = received.split(",");
                if len(data) < 10: return;
                for _ in data:
                    if _ != "":
                        data[data.index(_)] = float(_)
                        
                #self.get_logger().info(data)

                # Acceleration data (in m/s²), scaled from mg to m/s²
                imu_msg.linear_acceleration.x = data[0]/ 1000.0  # Convert mg to m/s²
                imu_msg.linear_acceleration.y = data[1]/ 1000.0
                imu_msg.linear_acceleration.z = data[2]/ 1000.0

                # Gyroscope data (in radians per second), scaled from DPS to rad/s
                imu_msg.angular_velocity.x = data[3]* (math.pi / 180.0)  # Convert degrees per second to rad/s
                imu_msg.angular_velocity.y = data[4]* (math.pi / 180.0)
                imu_msg.angular_velocity.z = data[5]* (math.pi / 180.0)

                # Magnetometer data (in microteslas), just passing the values directly
                mag_msg.magnetic_field.x = data[6]
                mag_msg.magnetic_field.y = data[7]
                mag_msg.magnetic_field.z = data[8]

                # Temperature data (in Celsius), directly from the input
                temp_msg.temperature = data[9]

                # Publish the IMU message
                self.publisher_.publish(imu_msg)
                self.publisherMag_.publish(mag_msg)
                self.publisherTemp_.publish(temp_msg)
                self.get_logger().info('Publishing IMU data: "%s"' % imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)

    # Clean up and shut down
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
