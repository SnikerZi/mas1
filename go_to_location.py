#time spended to write this code



import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
​

class tutrle:
    def __init__(self,x,y,yaw,pose_message,velocity_message,K_linear,distance,
                    linear_speed,K_angular,desired_angel_goal,angular_speed):
                    self.x = self.pose_message.x
                    self.y = self.pose_message.y
                    self.yaw = self.pose_message.theta
                    self.velocity_message = Twist()
                    self.cmd_vel_topic='/turtle1/cmd_vel'
                    self.velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
                    self.K_linear = 0.5
                    self.distance = abs(math.sqrt(((self.x_goal-x)**2)((self.y_goal-y)**2)))
                    self.linear_speed = distance*K_linear
                    self.K_angular
                    self.desired_angel_goal
                    self.angular_speed
                    self.velocity_message.linear.x= linear_speed
                    self.velocity_message.angular.z= angular_speed
                    self.velocity_publisher.publish(velocity_message)
                    self.position_topic = 'turtle1/pose'
                    self.pose_publisher = rospy.Subscriber(position_topic,Pose,poseCallback)

#TODO
#everything else

def go_to_location(self x_goal,y_goal):
    x,y,yaw
    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
    while(True):
        K_linear = 0.5
        distance = abs(math.sqrt(((x_goal-x)**2)((y_goal-y)**2)))
        linear_speed = distance*K_linear


        K_angular = 4.0
        desired_angel_goal = math.atan2(y_goal - y,x_goal-x)
        angular_speed = (desired_angel_goal-yaw)*K_angular

        velocity_message.linear.x= linear_speed
        velocity_message.angular.z= angular_speed


        velocity_publisher.publish(velocity_message)
        print( 'x=',x,'y=',y)

        if (distance<0.01):
            break

​


​
if __name__ == '__main__':
    try:
        rospy.init_node('pos_node', anonymous = True)

        cmd_vel_topic = '/turle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

        position_topic = 'turtle1/pose'
        pose_publisher = rospy.Subscriber(position_topic,Pose,poseCallback)
        time.Sleep(2)
#        move(1.0,2.0,True)
        go_to_location(1,1)
    except rospy.ROSInterruptException:
        rospy.loginfo('node terminated.')
