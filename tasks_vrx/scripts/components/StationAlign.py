#!/usr/bin/python3
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

input_location = Odometry()
current=Twist()
global goal
goal=Twist()
def quat2rpy(quat):
    euler=tf.transformations.euler_from_quaternion(quat)
    return euler[0],euler[1],euler[2]

def currentLocation(data):
    input_location=data
    current.linear.x=input_location.pose.pose.position.x
    current.linear.y=input_location.pose.pose.position.y
    current.linear.z=input_location.pose.pose.position.z
    quat1 = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    current.angular.x,current.angular.y,current.angular.z=quat2rpy(quat1)

def GoalLocation(data):
    goal=data
    #print(goal)
    Reach2Station(goal)

        

def moveAngle(goal,theta,speed):
    x2=goal.linear.x
    x1=current.linear.x
    y2=goal.linear.y
    y1=current.linear.y
    print("Goal : {:.3f},{:.3f}".format(x2,y2))
    print("Current : {:.3f},{:.3f}".format(x1,y1))
    #Work Something here
    if(theta>0):
        print("+ve is executed")
        if (x2<x1 and y2<y1):
            angle_lateral.publish(-1.0*(math.pi/2.0-theta))
            thrust_lateral.publish(1.0*speed)
        elif (x2>x1 and y2>y1):
            angle_lateral.publish((-1.0*math.pi/2.0-theta))
            thrust_lateral.publish(-1.0*speed)
    else:
        print("-ve is executed")
        if (x2<x1 and y2>y1):
            print("Moving correct engine !")
            angle_lateral.publish((math.pi/2.0-abs(theta)))
            thrust_lateral.publish(-1.0*speed)
        elif (x2>x1 and y2<y1):
            angle_lateral.publish((math.pi/2.0-abs(theta)))
            thrust_lateral.publish(1.0*speed)

def rotate(speed):
    if(speed<0):
        angle_lateral.publish(math.pi/4.0)
        angle_left.publish(-1*math.pi/4.0)
        angle_right.publish(-1*math.pi/4.0)
        thrust_lateral.publish(2*speed)
        thrust_left.publish(speed)
        thrust_right.publish(speed)
    else:
        angle_lateral.publish(-1*math.pi/4.0)
        angle_left.publish(1*math.pi/4.0)
        angle_right.publish(1*math.pi/4.0)
        thrust_lateral.publish(-2*speed)
        thrust_left.publish(speed)
        thrust_right.publish(speed)

def go_to_goal(goal):
    #cmd_vel_topic='/turtle1/cmd_vel'

    #Code to avoid divide by zero error
    K_linear=0.4

    #Code to avoid divide by zero error
    K_angular=0.382

    #Align to the Standard
    while(True):
        diff=(-3.14-current.angular.z)
        if(abs(diff)<0.5235):
            angular_speed= diff*K_angular
            if(angular_speed<0.1):
                angular_speed=0.1
        else:
            angular_speed= 0.3
        rotate(angular_speed)
        if abs(-3.14-current.angular.z)<0.1:
            angle_lateral.publish(0)
            angle_left.publish(0)
            angle_right.publish(0)
            thrust_lateral.publish(0)
            thrust_left.publish(0)
            thrust_right.publish(0)
            break
    #Move to 2D Coordinate
    while(True):
        distance = abs(math.sqrt(((goal.linear.x-current.linear.x) ** 2) + ((goal.linear.y-current.linear.y) ** 2)))
        if(distance<1):
            linear_speed = distance * K_linear
            if (linear_speed<0.25):
                linear_speed=0.25
        else:
            linear_speed=1.0
        goal_angle=math.atan((goal.linear.y-current.linear.y)/(goal.linear.x-current.linear.x))
        print("Home Slope: ",goal_angle*180.0/math.pi)
        moveAngle(goal,goal_angle,linear_speed)
        #velocity_publisher.publish(velocity_message)
        #print('x=', current.linear.x, 'y=',current.linear.y)
        if (distance <0.1):
            break
    diff=math.pi-(-goal.angular.z-current.angular.z)
    print(diff*180/math.pi)
    if(diff<0.1):
        #Align to the orientation
        while(True):
            diff=math.pi-(-goal.angular.z-current.angular.z)
            if(diff<0.5235):
                angular_speed= diff*K_angular
                if(angular_speed<0.1):
                    angular_speed=0.1
            else:
                angular_speed= 1.0
            rotate(angular_speed)
            if math.pi-(-goal.angular.z-current.angular.z)<0.1:
                angle_lateral.publish(0)
                angle_left.publish(0)
                angle_right.publish(0)
                thrust_lateral.publish(0)
                thrust_left.publish(0)
                thrust_right.publish(0)
                break


def Reach2Station(goal):
    go_to_goal(goal)

def Station_Keeping():
    global angle_lateral,angle_left,angle_right,thrust_lateral,thrust_left,thrust_right
    angle_lateral=rospy.Publisher('/wamv/thrusters/lateral_thrust_angle',Float32,queue_size=10)
    thrust_lateral=rospy.Publisher('/wamv/thrusters/lateral_thrust_cmd',Float32,queue_size=10)
    angle_left=rospy.Publisher('/wamv/thrusters/left_thrust_angle',Float32,queue_size=10)
    thrust_left=rospy.Publisher('/wamv/thrusters/left_thrust_cmd',Float32,queue_size=10)
    angle_right=rospy.Publisher('/wamv/thrusters/right_thrust_angle',Float32,queue_size=10)
    thrust_right=rospy.Publisher('/wamv/thrusters/right_thrust_cmd',Float32,queue_size=10)
    rospy.init_node('Station_Goal', anonymous=True)
    rospy.Subscriber("/wamv/robot_localization/odometry/filtered",Odometry , currentLocation)
    rospy.Subscriber("Station_Location",Twist,GoalLocation)
    rospy.spin()
if __name__ == '__main__':
    Station_Keeping()