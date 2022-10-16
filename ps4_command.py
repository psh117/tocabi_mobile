from src.tocabi_mobile import TocabiMobile, CommandBase
import threading
from pyPS4Controller.controller import Controller

class PS4Listener(CommandBase, Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        CommandBase.__init__(self)
        self.command[0] = 0.0
        self.command[1] = 0.0
        self.command[2] = 0.0

        self.up=False
        self.down=False
        self.speed = 1

    def on_triangle_press(self):
        print('triangle!')
        pass

    def on_triangle_release(self):
        pass

    def on_x_press(self):
        self.kill_signal = True
        self.stop = True
        # sys.exit(0)

    def on_up_arrow_press(self):
        self.up=True

    def on_down_arrow_press(self):
        self.down=True

    def on_up_down_arrow_release(self):
        self.up = False
        self.down = False

    def disconnect(self):
        print("[PS4] controller disconnected")

    def on_L3_left(self, value):
        self.command[1] = value/32768

    def on_L3_right(self, value):
        self.command[1] = value/32768

    def on_L3_up(self, value):
        self.command[0] = -value/32768

    def on_L3_down(self,value):
        self.command[0] = -value/32768

    def on_L3_x_at_rest(self):
        self.command[1] = 0

    def on_L3_y_at_rest(self):
        self.command[0] = 0

    def on_R3_right(self, value):
        self.command[2] = -value/32768
    
    def on_R3_left(self, value):
        self.command[2] = -value/32768

    def on_R3_up(self, value):
        pass
    
    def on_R3_down(self, value):
        pass

    def on_R3_x_at_rest(self):
        self.command[2] = 0

    def on_R3_y_at_rest(self):
        pass

    def start_threaded_listen(self):
        self.t = threading.Thread(target=self.listen)
        self.t.start()

    def on_R2_press(self, value):
        self.speed = 1 + (33000 + value)*0.0001

    def on_R2_release(self):
        self.speed = 1

if __name__ == "__main__":
    rl = PS4Listener(interface="/dev/input/js0", connecting_using_ds4drv=False)
    rl.start_threaded_listen()
    tm = TocabiMobile(rl)
    tm.connect()
    tm.run()
    print('clean')
    rl.stop = True
    rl.t.join()