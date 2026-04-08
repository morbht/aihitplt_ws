#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

class RelayController:
    def __init__(self):
        self.pub = rospy.Publisher('spray_control', Bool, queue_size=10)
        rospy.loginfo("Relay Controller started")
        rospy.loginfo("Commands: 1=ON, 0=OFF, q=quit")
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                cmd = input("Enter command (1/0/q): ").strip().lower()
                
                if cmd == 'q':
                    rospy.signal_shutdown("User quit")
                    break
                elif cmd == '1':
                    self.pub.publish(Bool(True))
                    rospy.loginfo("Sent ON command")
                elif cmd == '0':
                    self.pub.publish(Bool(False))
                    rospy.loginfo("Sent OFF command")
                else:
                    rospy.logwarn("Invalid command. Use 1 (ON), 0 (OFF), or q (quit)")
                    
            except EOFError:
                break
            except Exception as e:
                rospy.logerr("Error: %s", e)

def main():
    rospy.init_node('spray_controller')
    controller = RelayController()
    controller.run()

if __name__ == '__main__':
    main()
