#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class RB1():
    
    def __init__(self):
        self.rb1 = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.current_yaw = 0.0
        self.ctrl_c = False
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdownhook)

        self.target = 90
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("Waiting for odometry...")
        try:
            rospy.wait_for_message('/odom', Odometry, timeout=5.0)
            rospy.loginfo("Odometry received. Ready to rotate.")
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for odometry. Using default value.")

    def odom_callback(self, msg):
        _orientation = msg.pose.pose.orientation
        _orientation_list = [_orientation.x, _orientation.y, _orientation.z, _orientation.w]
        (_roll, _pitch, _yaw) = euler_from_quaternion(_orientation_list)
        self.current_yaw = _yaw
        
    def publish_cmd_vel(self):
        self.rb1.publish(self.cmd)
        
    def shutdownhook(self):
        self.stop()
        self.ctrl_c = True

    def stop(self):
        rospy.loginfo("Stopping robot")
        # Sending multiple stop commands to ensure it's received
        for i in range(5):
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            self.cmd.linear.z = 0.0
            self.cmd.angular.x = 0.0
            self.cmd.angular.y = 0.0
            self.cmd.angular.z = 0.0
            self.publish_cmd_vel()
            rospy.sleep(0.1)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def rotate(self, degrees):
        try:
            rospy.loginfo(f"Rotating {degrees} degrees")
            
            _initial_yaw = self.current_yaw
            _target_rotation = math.radians(degrees)
            target = self.normalize_angle(_initial_yaw + _target_rotation)
            _kP = 0.9
            _tolerance = math.radians(7)
            _max_angular_speed = 0.9
            _min_angular_speed = 0.15
            _min_speed_threshold = math.radians(15)

            while not rospy.is_shutdown() and not self.ctrl_c:
                angle_diff = self.normalize_angle(target - self.current_yaw)
                
                if abs(angle_diff) < _tolerance:
                    rospy.loginfo("Target reached!")
                    break
                
                _angular_velocity = _kP * angle_diff
                if abs(angle_diff) > _min_speed_threshold:
                    # Far from target: apply minimum speed to avoid crawling
                    # If close to target: slow speed
                    if abs(_angular_velocity) < _min_angular_speed:
                        _angular_velocity = _min_angular_speed if _angular_velocity > 0 else -_min_angular_speed
                
                # Apply maximum speed limit
                _angular_velocity = max(-_max_angular_speed,
                                        min(_max_angular_speed, _angular_velocity))
            
                self.cmd.angular.z = _angular_velocity
                
                # rospy.logdebug(f'Target: {math.degrees(target):.1f} Deg | '
                #             f'Current: {math.degrees(self.current_yaw):.1f} Deg | '
                #             f'Error: {math.degrees(angle_diff):.1f} Deg')
                
                self.publish_cmd_vel()
                self.rate.sleep()
            self.stop()
            return True
            
        except Exception as e:
            rospy.logerr(f"Rotation failed: {str(e)}")
            self.stop()
            return False

    def turn(self, linear_speed=0.2, angular_speed=0.2, degree=-90):
        # TODO
        pass

    def move(self, moving_time, linear_speed=0.2, angular_speed=0.2):
        # TODO
        # rospy.loginfo(f"Moving for {moving_time} seconds at {linear_speed} m/s")
        
        # self.cmd.linear.x = linear_speed
        # self.cmd.angular.z = angular_speed
        
        # start_time = rospy.Time.now()
        # while (rospy.Time.now() - start_time).to_sec() < moving_time and not self.ctrl_c:
        #     self.publish_cmd_vel()
        #     self.rate.sleep()
        
        # self.stop()
        pass
            
if __name__ == '__main__':
    rospy.init_node('my_rb1_test', anonymous=True)
    rb1_object = RB1()
    try:
        rb1_object.rotate(90)
        rospy.sleep(2)
        rb1_object.rotate(-45)
        rospy.sleep(2)
        rb1_object.rotate(-45)
    except rospy.ROSInterruptException:
        pass
