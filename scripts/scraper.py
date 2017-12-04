#!/user/bin/env python
import urllib2
import time
import rospy
from mouth_detector.srv import TriggerPhase

brent_site = "https://brentyi.com/marshmallow/phase"

def serviceCall(value):
    try:
        phase = rospy.ServiceProxy('move_robot', TriggerPhase)
        response = phase(int(value))
        return response.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def parse(source):
    # parse
    print "Parsed as: %s"%source
    return source

def loop():
    ropy.wait_for_service('move_robot')
    most_recent_phase = -100
    while True:
        f = urllib2.urlopen(brent_site)
        page_source = f.read()

        value = int(parse(page_source))
        if most_recent_phase != value:
            print("detects " + str(value))
            most_recent_phase = value
            serviceCall(value)
        time.sleep(0.5)

loop()
