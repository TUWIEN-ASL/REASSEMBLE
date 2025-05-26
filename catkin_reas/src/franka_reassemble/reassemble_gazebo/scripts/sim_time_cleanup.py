#!/usr/bin/env python3

import rospy

def shutdown_hook():
    rospy.set_param('/use_sim_time', False)
    rospy.loginfo('Set use_sim_time to false')

def main():
    rospy.init_node('set_use_sim_time_on_shutdown')
    rospy.on_shutdown(shutdown_hook)
    rospy.spin()

if __name__ == '__main__':
    main()
