#!/usr/bin/env python3

# This code is a part of the LoCO AUV project.
# Copyright (C) The Regents of the University of Minnesota

# Maintainer: Junaed Sattar <junaed@umn.edu> and the Interactive Robotics and Vision Laboratory

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from loco_pilot.msg import Command
from geometry_msgs.msg import Wrench
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Quaternion, Transform
from tf.transformations import *

apply_left=None
apply_vertical=None
apply_right=None
apply_drag=None

vertical_raw=0
right_raw=0
left_raw=0
maxvel=0

def commands_callback1(data):

    global vertical_raw
    global right_raw
    global left_raw

    max_force=23.15 # Newtons

    pitch=data.pitch
    yaw=data.yaw
    throttle=data.throttle

    vertical_raw=max_force*pitch
    if throttle==0 and yaw==0:
        left_raw=0
        right_raw=0
    else:
        if throttle>=0:
            if yaw>=0:
                left_raw=max_force*(throttle**2+yaw**2)**0.5
                right_raw=max_force*throttle
            else:
                right_raw=max_force*(throttle**2+yaw**2)**0.5
                left_raw=max_force*throttle
        else:
            if yaw>=0:
                left_raw=-max_force*(throttle**2+yaw**2)**0.5
                right_raw=max_force*throttle
            else:
                right_raw=-max_force*(throttle**2+yaw**2)**0.5
                left_raw=max_force*throttle

def commands_callback2(data):
    global apply_left
    global apply_vertical
    global apply_right
    global apply_drag

    global maxvel

    Area_z=.16    # m^2, approx
    p=1000  # water density
    drag_coef_z=1.2 # half circle

    #Area_y=0.14 # m^2, approx, cross section both cyl plus cross section betweeen tubes and thrusters
    Area_y=0.083
    drag_coef_y=1.2 # cyl, square tube rounded edges

    drag_paramx=20.578 # from data
    drag_paramy=drag_coef_y*Area_y*p/2 # calc
    drag_paramz=drag_coef_z*Area_z*p/2 # calc

    vertical=Wrench()
    left=Wrench()
    right=Wrench()
    drag=Wrench()

    for i in range (0,len(data.name)):
        if data.name[i]=="robot::right_thruster":
            rtindex=i
        if data.name[i]=="robot::left_thruster":
            ltindex=i
        if data.name[i]=="robot::vertical_thruster":
            vtindex=i
        if data.name[i]=="robot::loco_base_frame":
            drindex=i


    vertical_vec= [0,0,vertical_raw,0]
    right_vec= [0,0,right_raw,0]
    left_vec=[0,0,left_raw,0]

    # Unknown issue with using Quaternion constructor, this is a workaround.
    quat_rt=[data.pose[rtindex].orientation.x,data.pose[rtindex].orientation.y,data.pose[rtindex].orientation.z,data.pose[rtindex].orientation.w]
    quat_lt=[data.pose[ltindex].orientation.x,data.pose[ltindex].orientation.y,data.pose[ltindex].orientation.z,data.pose[ltindex].orientation.w]
    quat_vt=[data.pose[vtindex].orientation.x,data.pose[vtindex].orientation.y,data.pose[vtindex].orientation.z,data.pose[vtindex].orientation.w]
    quat_dr=[data.pose[drindex].orientation.x,data.pose[drindex].orientation.y,data.pose[drindex].orientation.z,data.pose[drindex].orientation.w]

    new_vertical=quaternion_multiply(quaternion_multiply(quat_vt,vertical_vec),quaternion_conjugate(quat_vt))
    new_right=quaternion_multiply(quaternion_multiply(quat_rt,right_vec),quaternion_conjugate(quat_rt))
    new_left=quaternion_multiply(quaternion_multiply(quat_lt,left_vec),quaternion_conjugate(quat_lt))

    new_dragx=quaternion_multiply(quaternion_multiply(quat_dr,[1,0,0,0]),quaternion_conjugate(quat_dr))
    new_dragy=quaternion_multiply(quaternion_multiply(quat_dr,[0,1,0,0]),quaternion_conjugate(quat_dr))
    new_dragz=quaternion_multiply(quaternion_multiply(quat_dr,[0,0,1,0]),quaternion_conjugate(quat_dr))

    vertical.force.x=new_vertical[0]
    vertical.force.y=new_vertical[1]
    vertical.force.z=new_vertical[2]
    left.force.x=new_left[0]
    left.force.y=new_left[1]
    left.force.z=new_left[2]
    right.force.x=new_right[0]
    right.force.y=new_right[1]
    right.force.z=new_right[2]

    vel_relx=data.twist[drindex].linear.x*new_dragx[0]+data.twist[drindex].linear.y*new_dragx[1]+data.twist[drindex].linear.z*new_dragx[2]
    vel_rely=data.twist[drindex].linear.x*new_dragy[0]+data.twist[drindex].linear.y*new_dragy[1]+data.twist[drindex].linear.z*new_dragy[2]
    vel_relz=data.twist[drindex].linear.x*new_dragz[0]+data.twist[drindex].linear.y*new_dragz[1]+data.twist[drindex].linear.z*new_dragz[2]
    ang_relx=data.twist[drindex].angular.x*new_dragx[0]+data.twist[drindex].angular.y*new_dragx[1]+data.twist[drindex].angular.z*new_dragx[2]
    ang_rely=data.twist[drindex].angular.x*new_dragy[0]+data.twist[drindex].angular.y*new_dragy[1]+data.twist[drindex].angular.z*new_dragy[2]
    ang_relz=data.twist[drindex].angular.x*new_dragz[0]+data.twist[drindex].angular.y*new_dragz[1]+data.twist[drindex].angular.z*new_dragz[2]

    drag_relx=-drag_paramx*vel_relx*abs(vel_relx)
    drag_rely=-drag_paramy*vel_rely*abs(vel_rely)
    drag_relz=-drag_paramz*vel_relz*abs(vel_relz)
    torque_relx=-drag_paramx*2*(0.577*0.344/2)**2*ang_relx*abs(ang_relx)
    torque_rely=-drag_paramy*2*(0.577*0.731/2)**2*ang_rely*abs(ang_rely)
    torque_relz=-drag_paramz*2*(0.577*0.731/2)**2*ang_relz*abs(ang_relz)

    drag.force.x=drag_relx*new_dragx[0]+drag_rely*new_dragy[0]+drag_relz*new_dragz[0]
    drag.force.y=drag_relx*new_dragx[1]+drag_rely*new_dragy[1]+drag_relz*new_dragz[1]
    drag.force.z=drag_relx*new_dragx[2]+drag_rely*new_dragy[2]+drag_relz*new_dragz[2]
    drag.torque.x=torque_relx*new_dragx[0]+torque_rely*new_dragy[0]+torque_relz*new_dragz[0]
    drag.torque.y=torque_relx*new_dragx[1]+torque_rely*new_dragy[1]+torque_relz*new_dragz[1]
    drag.torque.z=torque_relx*new_dragx[2]+torque_rely*new_dragy[2]+torque_relz*new_dragz[2]

    #print(data.twist[drindex])
    #vel=(data.twist[drindex].linear.x**2+data.twist[drindex].linear.y**2+data.twist[drindex].linear.z**2)**0.5
    #if vel>maxvel:
        #maxvel=vel
        #print(maxvel)

    apply_drag.publish(drag)
    apply_vertical.publish(vertical)
    apply_right.publish(right)
    apply_left.publish(left)


def thrust_apply():
    global apply_left
    global apply_vertical
    global apply_right
    global apply_drag

    rospy.init_node('sim_control_node', anonymous=False)
    rate=rospy.Rate(100) # 100 Hz, latency issues with lower rates

    apply_left=rospy.Publisher("/left_thrust",Wrench, queue_size=1)
    apply_vertical=rospy.Publisher("/vertical_thrust",Wrench, queue_size=1)
    apply_right=rospy.Publisher("/right_thrust",Wrench, queue_size=1)
    apply_drag=rospy.Publisher("/drag_force",Wrench, queue_size=1)

    rospy.Subscriber("/loco/command", Command, commands_callback1)
    rospy.Subscriber("/gazebo/link_states",LinkStates,commands_callback2)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        thrust_apply()
    except rospy.ROSInterruptException:
        pass
