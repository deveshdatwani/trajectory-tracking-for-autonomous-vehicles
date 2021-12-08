#!/usr/bin/env python

import rospy
import rospkg

from std_msgs.msg import String

class HelloWorldNode:

    def __init__(self):

        # variables
        self._string  = 'Hello World! - '
        self._count   = 0
        self._message = ''

        # 10Hz cycle rate
        self.rate = rospy.Rate(10)

        # publishers
        self._pubMessage = rospy.Publisher('/hello_world_message', String, queue_size=10)
        
        # subscribers
        rospy.Subscriber('/hello_world_message', String, self._callbackMessage)

        # initialize message
        self.updateMessage()

    def updateMessage(self):

        # increment count and update message to send
        self._count   += 1
        self._message = self._string + str(self._count)

    def _callbackMessage(self, dataString):

        print('received: ' + dataString.data)

        # check if message received is the same as previously sent
        if dataString.data == self._message:

            # update message to send
            self.updateMessage()

    def publishDataToROS(self):
        
        # publish next message
        self._pubMessage.publish(String(self._message))
        

def main():

    rospy.init_node('hello_world')
    node = HelloWorldNode()

    print('Hello World ROS Test\n')

    while not rospy.is_shutdown():
        
        node.publishDataToROS()
        node.rate.sleep()


if __name__ == "__main__":
    main()
