#!/usr/bin/env python
import rospy
import sys
from rosbot_patrol_simulation.msg import EspTrigger
from sensor_msgs.msg import Range


class RangeToTrigger:

    def __init__(self):
        self.sub1 = rospy.Subscriber("sonar1",Range,self.sonar1)
        self.sub2 = rospy.Subscriber("sonar2",Range,self.sonar2)
        self.sub3 = rospy.Subscriber("sonar3",Range,self.sonar3)
        self.sub4 = rospy.Subscriber("sonar4",Range,self.sonar4)

        self.pub1 = rospy.Publisher("/motion_trigger",EspTrigger, queue_size=1)
        

        self.__treshold = 2.0


    def sonar1(self,data):
        msg = EspTrigger()
        if(data.range <= self.__treshold):
            msg.id = 1
            msg.move = True
            self.pub1.publish(msg)
        else:
            msg.id = 1
            msg.move = False
            self.pub1.publish(msg)
    
    def sonar2(self,data):
        msg = EspTrigger()
        if(data.range <= self.__treshold):
            msg.id = 2
            msg.move = True
            self.pub1.publish(msg)
        else:
            msg.id = 2
            msg.move = False
            self.pub1.publish(msg)
    
    def sonar3(self,data):
        msg = EspTrigger()
        if(data.range <= self.__treshold):
            msg.id = 3
            msg.move = True
            self.pub1.publish(msg)
        else:
            msg.id = 3
            msg.move = False
            self.pub1.publish(msg)

    def sonar4(self,data):
        msg = EspTrigger()
        if(data.range <= self.__treshold):
            msg.id = 4
            msg.move = True
            self.pub1.publish(msg)
        else:
            msg.id = 4
            msg.move = False
            self.pub1.publish(msg)


def main(args):
  obj = RangeToTrigger()
  rospy.init_node('sonar_to_esp', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)