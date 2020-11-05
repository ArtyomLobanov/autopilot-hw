# #!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from turtlesim.srv import Spawn
from turtlesim.msg import Pose


class Position:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class Follower:

    def __init__(self, name, x, y, target_name, max_speed, update_delay=1.0):
        self.name = name
        self.position = Position(x, y, 0)
        self.target_position = None
        self.max_speed = max_speed
        self.update_delay = update_delay
        self.target_position_subscriber = rospy.Subscriber('/' + target_name + '/pose', Pose, self.update_target_position)
        self.self_position_subscriber = rospy.Subscriber('/' + name + '/pose', Pose, self.update_self_position)
        self.control_publisher = rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=10)

        rospy.wait_for_service('/spawn')
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(x, y, 0., name)
        self.send_control_message()

    def update_target_position(self, message):
        self.target_position = Position(message.x, message.y, message.theta)

    def update_self_position(self, message):
        self.position = Position(message.x, message.y, message.theta)

    def define_route(self):
        if not self.target_position:
            return 0, 0
        dx = self.target_position.x - self.position.x
        dy = self.target_position.y - self.position.y
        azimuth = math.atan2(dy, dx)
        distance = math.hypot(dx, dy)
        if distance < 0.00001:
            return 0, 0
        delta_angular = azimuth - self.position.theta
        if delta_angular < -math.pi:
            delta_angular += 2 * math.pi
        elif delta_angular > math.pi:
            delta_angular -= 2 * math.pi
        linear_speed = min(self.max_speed, distance / self.update_delay)
        angular_speed = delta_angular / self.update_delay
        return linear_speed, angular_speed

    def send_control_message(self):
        speed, angular_speed = self.define_route()
        msg = Twist()
        msg.linear = Vector3(speed, 0, 0)
        msg.angular = Vector3(0, 0, angular_speed)
        self.control_publisher.publish(msg)

    def run(self):
        rate = rospy.Rate(1 / self.update_delay)
        while not rospy.is_shutdown():
            self.send_control_message()
            rate.sleep()


rospy.init_node('follower_node')
Follower('follower', 0, 0, 'turtle1', 1).run()