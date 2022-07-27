import rospy
from src.tocabi_mobile import TocabiMobile, CommandBase
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rl = CommandBase()
    tm = TocabiMobile(rl)
    tm.recovery()
    
    print ('recovery done !')