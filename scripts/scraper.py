#!/user/bin/env python
import urllib2
import time
import rospy
from mouth_detector.srv import TriggerPhase

brent_site = "https://brentyi.com/marshmellow"

def serviceCall():
    f = urllib2.urlopen(brent_site)
    page_source = f.read()
    value = parse(page_source)


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
    rospy.wait_for_service('move_robot')
    while True:
        serviceCall()
        time.sleep(1)

loop()
