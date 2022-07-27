import rospy
from tocabi_mobile import TocabiMobile, CommandBase
from geometry_msgs.msg import Twist


class RospyListener(CommandBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('tm_listener', anonymous=True, disable_signals=True)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)

    def callback(self,data):
        self.command[0] = data.linear.x
        self.command[1] = data.linear.y
        self.command[2] = data.angular.z

if __name__ == "__main__":
    rl = RospyListener()
    tm = TocabiMobile(rl)
    tm.connect()
    tm.run()