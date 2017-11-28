#!/usr/bin/env python
import rospy
from mouth_detector.srv import *

def billy_server():
    rospy.init_node('billy_server')
    s = rospy.Service('move_robot', TriggerPhase, move_robot)
    rospy.spin()

def move_robot(request):
    phase_id = request.phase
    print "phase_id is %d"%phase_id

    # move the robot according to the phase

    success = True
    message = "son, it works!"
    return TriggerPhaseResponse(success, message)



if __name__ == '__main__':
    billy_server()
