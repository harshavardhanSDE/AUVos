import rospy
import numpy as np
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion


class ThrusterController:
    def __init__(self):
        rospy.init_node('thruster_controller', anonymous=True)

        # Thruster Allocation Matrix (TAM)
        self.TAM = np.array([
            [1, -1, -1, 1, 1, -1],
            [1, 1, -1, -1, -1, -1],
            [1, -1, 1, -1, 1, 1],
            [1, 1, 1, 1, -1, 1],
            [1, -1, -1, -1, -1, 1],
            [1, 1, -1, 1, 1, 1],
            [1, -1, 1, 1, -1, -1],
            [1, 1, 1, -1, 1, -1]
        ])

        # PID Gains for Position Holding
        self.Kp = np.array([1.0, 1.0, 1.0, 0.8, 0.8, 1.2])
        self.Kd = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.2])
        self.prev_error = np.zeros(6)

        # Inputs
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Output
        self.thruster_pub = rospy.Publisher(
            '/thruster_forces', Float32MultiArray, queue_size=10)

        self.current_wrench = np.zeros(6)  # Store joystick command
        self.current_rates = np.zeros(6)   # Store angular velocity from IMU

    def joy_callback(self, msg):
        # Map joystick axes to forces and torques
        self.current_wrench = np.array([
            msg.axes[1],  # Surge (X)
            msg.axes[0],  # Sway (Y)
            msg.axes[3],  # Heave (Z)
            msg.axes[4],  # Roll
            msg.axes[2],  # Pitch
            msg.axes[5]   # Yaw
        ])

    def imu_callback(self, msg):
        # Extract angular velocity (for stabilization)
        self.current_rates = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # Compute PID correction
        error = -self.current_rates  # Assuming we want to stabilize to zero motion
        pid_correction = self.Kp * error + self.Kd * (error - self.prev_error)
        self.prev_error = error

        # Compute final wrench command (Joystick + PID correction)
        final_wrench = self.current_wrench + pid_correction

        # Compute thruster forces
        thruster_forces = np.linalg.pinv(self.TAM).dot(final_wrench)

        # Publish thruster forces
        thrust_msg = Float32MultiArray()
        thrust_msg.data = thruster_forces.tolist()
        self.thruster_pub.publish(thrust_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = ThrusterController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
