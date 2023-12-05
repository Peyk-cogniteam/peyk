#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import time
import os

class ImageSubscriberNode:
    def __init__(self, max_messages, time_interval):
        self.max_messages = max_messages
        self.time_interval = time_interval
        self.received_messages = 0
        self.start_time = time.time()

        # Subscribe to the image topic
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        # Create a timer to check the condition periodically
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)


    def image_callback(self, msg):
        # Increment the count of received messages
        self.received_messages += 1
      

    def timer_callback(self, event):
        # Check if the specified number of messages has been received within the time interval
        if self.received_messages >= self.max_messages:
            rospy.loginfo("Received {} messages in {} seconds. Stopping subscription.".format(
                self.max_messages, self.time_interval))
            self.timer.shutdown()  # Stop the timer
            rospy.signal_shutdown("Received {} messages in {} seconds.".format(
                self.max_messages, self.time_interval))

        # Check if the time interval has passed
        elif time.time() - self.start_time >= self.time_interval:
            rospy.loginfo("Did not receive {} messages in {} seconds. Rebooting the computer.".format(
                self.max_messages, self.time_interval))
            os.system("sudo reboot")
            #self.timer.shutdown()  # Stop the timer
def main():
    rospy.init_node('peyk_auto_reboot_node', anonymous=True)

    # Specify the maximum number of messages and time interval
    max_messages = 10
    time_interval = 30  # seconds

    # Create an instance of the ImageSubscriberNode class
    image_subscriber_node = ImageSubscriberNode(max_messages, time_interval)

    # Spin the ROS node
    rospy.spin()

if __name__ == '__main__':
    main()