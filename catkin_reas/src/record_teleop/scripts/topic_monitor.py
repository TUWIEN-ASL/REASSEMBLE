#!/usr/bin/env python3
import rospy
import time

def monitor_topics_availability(topics, check_interval=3.0):
    """
    Monitor if the given topics are available.

    :param topics: List of topic names to monitor.
    :param check_interval: Interval in seconds to check for topic availability.
    """
    rospy.init_node('topic_monitor', anonymous=True)

    while not rospy.is_shutdown():
        # Get the list of currently published topics
        published_topics = [topic for topic, _ in rospy.get_published_topics()]
        
        for topic in topics:
            if topic in published_topics:
                pass
                #rospy.loginfo(f"Topic '{topic}' is active.")
            else:
                rospy.logerr(f"Topic '{topic}' is not active!")
        
        time.sleep(check_interval)

if __name__ == '__main__':
    try:
        # Get the topics from the ROS parameter
        topics_to_monitor = rospy.get_param('/topic_monitor/topics')
        topics_to_monitor = topics_to_monitor.split()
        if not topics_to_monitor:
            rospy.logwarn("No topics provided for monitoring. Please pass them as a list in the 'topics' parameter.")
        else:
            print("Monitoring topics...")
            monitor_topics_availability(topics_to_monitor)
    except rospy.ROSInterruptException:
        pass
