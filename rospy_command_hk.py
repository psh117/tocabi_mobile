import rospy
from src.tocabi_mobile import TocabiMobile, CommandBase
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
import sys

def set_param():
    rospy.set_param('~dev', '/dev/input/js1')
    rospy.set_param('~deadzone', '0.1')
    rospy.set_param('~autorepeat_rate', '10,0')


class RospyListener(CommandBase):
    def __init__(self):
        super().__init__()
        rospy.init_node('tm_listener', disable_signals=True)
        # rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.Subscriber("joy", Joy, self.pedalcallback)
        rospy.Subscriber("drive_mode", Int16, self.dmcallback)
        # self.MAF_size = 10
        # self.MAF_r = [0]*self.MAF_size
        # self.MAF_l = [0]*self.MAF_size
        # self.MAF_y = [0]*self.MAF_size
        self.threshold = 0.1
        self.pedal_r = 0
        self.pedal_l = 0
        self.pedal_y = 0
        self.dm = 0
        self.gui_control = False
        

    def dmcallback(self, msg):
        if msg.data == 3:
            self.gui_control = True

            print('mode : guicontrol')

            self.command[0] = 0
            self.command[1] = 0
            self.command[2] = 0

        elif msg.data == 2:
            self.gui_control = False

            print('mode : guicontrol deactivate')

            self.command[0] = 0
            self.command[1] = 0
            self.command[2] = 0
        else:
            if self.dm != msg.data:
                
                self.dm = msg.data

                if self.dm == 1:
                    print('mode : drive')
                elif self.dm == 0:
                    print('mode : neutral')
                elif self.dm == -1:
                    print('mode : reverse')

                self.command[0] = 0
                self.command[1] = 0
                self.command[2] = 0


        sys.stdout.flush()


            
            

        # print(self.dm)

    # def callback(self,data):
    #     self.command[0] = data.linear.x
    #     self.command[1] = data.linear.y
    #     self.command[2] = data.angular.z
    
    def pedalcallback(self, data):
        # print(self.dm)
        # input : data.axes[0~2]
        # output : self.command[0~2] = 0
        # for backward. using mode. (external button required)
        # mode = 0~1 (0:foward, 1:backward)
        scale = 3
        self.pedal_r = (data.axes[0]+1)/2/scale
        self.pedal_l = (data.axes[1]+1)/2/scale
        self.pedal_y = data.axes[2]/scale

        # # Buffer Moving Average Filter(MAF)
        # self.MAF_r.pop()
        # self.MAF_r.insert(0, self.pedal_r)
        # self.pedal_r = sum(self.MAF_r)/len(self.MAF_r)
        # self.MAF_l.pop()
        # self.MAF_l.insert(0, self.pedal_l)
        # self.pedal_l = sum(self.MAF_l)/len(self.MAF_l)
        # self.MAF_y.pop()
        # self.MAF_y.insert(0, self.pedal_y)
        # self.pedal_y = sum(self.MAF_y)/len(self.MAF_y)
        self.speed = 1.0

        if self.gui_control:
            self.command[0] = data.axes[0]/scale
            self.command[1] = data.axes[1]/scale
            self.command[2] = data.axes[2]/scale
        else:
            if self.dm == 0: # Parking
                self.command[0] = 0
                self.command[1] = 0
                self.command[2] = 0
            else:
                if self.pedal_r > 0 and self.pedal_l > 0:
                    diff = self.pedal_r - self.pedal_l
                    if diff > self.threshold:
                        # Right Diagonal Foward
                        self.command[0] = min(self.pedal_r, self.pedal_l)
                        # self.command[1] = diff
                        self.command[1] = 0
                        # self.command[2] = self.pedal_y
                    elif diff < -self.threshold:
                        # Left Diagonal Foward
                        self.command[0] = min(self.pedal_r, self.pedal_l)
                        # self.command[1] = diff
                        self.command[1] = 0
                        # self.command[2] = self.pedal_y
                    else:
                        # Foward
                        if self.dm == 1:
                            self.command[0] = min(self.pedal_r, self.pedal_l)
                            self.command[1] = 0
                        elif self.dm == -1:
                            self.command[0] = -min(self.pedal_r, self.pedal_l)
                            self.command[1] = 0
                        # else:
                        #     raise Exception("Drive mode is not in {-1,0,1}.")
                        # self.command[2] = self.pedal_y
                elif self.pedal_r > 0:
                    # Right
                    self.command[0] = 0
                    self.command[1] = self.pedal_r
                    # self.command[2] = self.pedal_y
                elif self.pedal_l > 0:
                    # LEFT
                    self.command[0] = 0
                    self.command[1] = -self.pedal_l
                    # self.command[2] = self.pedal_y
                # elif abs(self.pedal_y) > 0.001:
                #     self.command[0] = 0
                #     self.command[1] = 0
                #     # self.command[2] = self.pedal_y
                else:
                    self.command[0] = 0
                    self.command[1] = 0
                    # self.command[2] = 0
                
                if abs(self.pedal_y) > 0.001:
                    self.command[2] = -self.pedal_y
                else:
                    self.command[2] = 0

        # print("pedal_l : ")
        # print(rl.pedal_l)
        # print("\npedal_r : ")
        # print(rl.pedal_r)
        # print("\npedal_y : ")
        # print(rl.pedal_y)

if __name__ == "__main__":
    #    set_param()
    rl = RospyListener()
    tm = TocabiMobile(rl)
    for _ in range(10): # 10 trials
        try:
            tm.connect()
            break
        except AssertionError:
            print('fail to connect')
    tm.run()
