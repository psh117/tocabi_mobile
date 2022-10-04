"""
TOCABI MOBILE ACTIVATOR (tocabi_mobile.py)

@ Authors: Suhan Park (psh117@snu.ac.kr), Junewhee Ahn (june992@snu.ac.kr)
@ 2022 @ DYROS

@ Some source codes are from 
    dyros_pcv_canopen @ KIM-HC github repo 
    (https://github.com/KIM-HC/dyros_pcv_canopen)
    @ See script/elmo.py

@ Questions / Help
    CANOpen API -> psh117@snu.ac.kr
    cobra4812_node*.dcf -> june992@snu.ac.kr
"""

from curses import reset_shell_mode
import canopen
import time

from .utils import CtrlWord, gen_controlword
import numpy as np
import sys

class CommandBase:
    def __init__(self) -> None:
        self.command = [0,0,0]
        self.speed = 1
        self.kill_signal = False
        self.init_cnt = 0

class TocabiMobile():
    def __init__(self, cmd : CommandBase) -> None:
        """
        0. Getting CANOpen device
        """
        self.network = canopen.Network()
        self.network.connect(bustype='socketcan', channel='can0', bitrate=500000)
        self.print_cnt = 0
        self.print_rate = 100 #
        self.command_process_cnt = 0
        self.command_process_rate = 10
        self.cmd = cmd
        self.old_time = time.time()

        self.start_time = time.time()
        self.last_fault_time = [time.time(),time.time(),time.time(),time.time(),time.time()]
        self.fault_stop_delay = 5

        self.file = []
        
        """
        REAL Command (should be integer)
        IGNORE 0 INDEX (USE 1~4) for node_id

        self.command[1] ~ self.command[4] 
        """
        self.command = [0,0,0,0,0]

        """
        """
        self.cmd_LPF = [0,0,0]
        self.theta_LPF = 0.0
        self.theta_LPF_prev = 0.0
        self.theta_dot = 0.0
        self.vel_x_LPF = 0.0
        self.vel_x_LPF_prev = 0.0
        self.vel_x_dot = 0.0      
        self.vel_y_LPF = 0.0
        self.vel_y_LPF_prev = 0.0
        self.vel_y_dot = 0.0                
        self.intergral_ = 0.0

        

    def change_status(self, status,
                    sleep_time : float  = 0.08):
        network = self.network
        network.nmt.state = status

        for n in network:
            network[n].nmt.state = status
        time.sleep(sleep_time)

        print('master state: {0}'.format(network.nmt.state))
        for node_id in network:
            print('node {0} state: {1}'.format(node_id, network[node_id].nmt.state))
        print()
        sys.stdout.flush()

    def get_status_word(self):
        network = self.network
        stat_strs  = '[STAT-IN]\n'
        for node_id in network:
            # network[node_id].tpdo.r
            # network[node_id].tpdo.read()
            # network[node_id].rpdo.read()
            stat_strs += "n: {1} stat:{0:<32}\n".format(bin(network[node_id].tpdo['status word'].raw), node_id)
        print(stat_strs)

    def check_statusword(self, statusword):
        if type(statusword) is int:
            statusword = bin(statusword)
        if (statusword[:2] != '0b'):
            print('cannot check statusword: {0}'.format(statusword))
            return
        statusword = statusword[2:]
        try:
            print('    Ready to switch on [0]: {0}'.format(statusword[-1]))
            print('           Switched on [1]: {0}'.format(statusword[-2]))
            print('     Operation enabled [2]: {0}'.format(statusword[-3]))
            print('                 Fault [3]: {0}'.format(statusword[-4]))
            print('       Voltage enabled [4]: {0}'.format(statusword[-5]))
            print('            Quick stop [5]: {0}'.format(statusword[-6]))
            print('    Switch on disabled [6]: {0}'.format(statusword[-7]))
            print('               Warning [7]: {0}'.format(statusword[-8]))
            print(' Manufacturer specific [8]: {0}'.format(statusword[-9]))
            print('                Remote [9]: {0}'.format(statusword[-10]))
            print('       Target reached [10]: {0}'.format(statusword[-11]))
            print('Internal limit active [11]: {0}'.format(statusword[-12]))
            print('     Op mode specific [12]: {0}'.format(statusword[-13]))
            print('     Op mode specific [13]: {0}'.format(statusword[-14]))
            print('Manufacturer specific [14]: {0}'.format(statusword[-15]))
            print('Manufacturer specific [15]: {0}\n'.format(statusword[-16]))
        except:
            print('somethins wrong')

    def connect(self):
        """
        1. Connect to CAN Network
        """
        network = self.network
        network.scanner.search()
        time.sleep(0.5)
        print(len(network.scanner.nodes) )

        if len(network.scanner.nodes) == 3:
            time.sleep(1)
            print('searching again ...')
            network.scanner.search()


        
        assert (len(network.scanner.nodes) == 4)


        for node_id in network.scanner.nodes:
            print('node_id',node_id)
            node_made_ = canopen.RemoteNode(node_id=node_id, object_dictionary='/home/dyros/sh_ws/tocabi_mobile/dcf/cobra4812_node{}.dcf'.format(node_id))
            network.add_node(node_made_)
            node_made_.tpdo.read()
            node_made_.rpdo.read()

            network[node_id].nmt.state = 'INITIALISING'
            all_codes = [emcy.code for emcy in network[node_id].emcy.log]
            active_codes = [emcy.code for emcy in network[node_id].emcy.active]
            for code_ in all_codes:
                print('emcy: {0}'.format(code_))
            for code_ in active_codes:
                print('emcy: {0}'.format(code_))
            network[node_id].emcy.reset()

        ## check bus state
        print('\nstate: {0}'.format(network.bus.state))
        print('bus channel_info: {0}'.format(network.bus.channel_info))

        ## -----------------------------------------------------------------
        ## send out time message
        network.time.transmit()
        time.sleep(0.1)

        # trans_type_ = 1
        trans_type_ = 255
        event_timer = 10

        # PDO Mapping
        """
        Change PDO Mappings here 
        node_id: 1~4 integer
        """
        for node_id in network:
            ##change TPDO configuration
            id_ = 1
            network[node_id].tpdo[id_].clear()
            network[node_id].tpdo[id_].add_variable('modes of operation display')     ## 16bit
            network[node_id].tpdo[id_].add_variable('status word')             ## 16bit
            network[node_id].tpdo[id_].trans_type = trans_type_
            network[node_id].tpdo[id_].event_timer = event_timer
            network[node_id].tpdo[id_].inhibit_time = 0
            network[node_id].tpdo[id_].enabled = True
            id_ = 2
            network[node_id].tpdo[id_].clear()
            network[node_id].tpdo[id_].add_variable('position actual value')  ## 32bit
            network[node_id].tpdo[id_].add_variable('velocity actual value')  ## 16bit
            network[node_id].tpdo[id_].add_variable('current actual value')  ## 16bit
            network[node_id].tpdo[id_].trans_type = trans_type_
            network[node_id].tpdo[id_].event_timer = event_timer
            network[node_id].tpdo[id_].inhibit_time = 0
            network[node_id].tpdo[id_].enabled = True
            network[node_id].tpdo.save()

            id_ = 3
            network[node_id].tpdo[id_].clear()
            network[node_id].tpdo[id_].add_variable('velocity demand value')  ## 32bit
            network[node_id].tpdo[id_].trans_type = trans_type_
            network[node_id].tpdo[id_].event_timer = event_timer
            network[node_id].tpdo[id_].inhibit_time = 0
            network[node_id].tpdo[id_].enabled = True
            network[node_id].tpdo.save()

            ##change RPDO configuration
            id_ = 1
            network[node_id].rpdo[id_].clear()
            network[node_id].rpdo[id_].add_variable('motor_switch')  ##16bit
            network[node_id].rpdo[id_].add_variable('mode_of_operation') ##16bit
            #network[node_id].rpdo[id_].add_variable('motor enable/disable') ##16bit
            network[node_id].rpdo[id_].enabled = True

            id_ = 2
            network[node_id].rpdo[id_].clear()
            network[node_id].rpdo[id_].add_variable('start motion')
            network[node_id].rpdo[id_].add_variable('target velocity')
            network[node_id].rpdo[id_].add_variable('target current value')
            # network[node_id].rpdo[id_].add_variable('velocity demand value')
            network[node_id].rpdo[id_].enabled = True

            id_ = 3 
            network[node_id].rpdo[id_].clear()
            network[node_id].rpdo[id_].add_variable('clear error')
            network[node_id].rpdo[id_].add_variable('control word')
            network[node_id].rpdo[id_].enabled = True
            # id_ = 3
            # network[node_id].rpdo[id_].clear()
            # network[node_id].rpdo[id_].add_variable('motor enable/disable')
            # network[node_id].rpdo[id_].enabled = True

            network[node_id].rpdo.save()

            # ## dont know if it works or not
            network[node_id].nmt.state = 'PRE-OPERATIONAL'
            time.sleep(0.1)
            network[node_id].tpdo.read()
            network[node_id].rpdo.read()
            network[node_id].tpdo[1]
            time.sleep(0.1)
            network[node_id].rpdo['motor_switch'].raw = 1
            network[node_id].rpdo['mode_of_operation'].raw = 3
            network[node_id].rpdo['clear error'].raw = 1
            network[node_id].rpdo[1].transmit()
            time.sleep(0.1)

            network[node_id].nmt.state = 'OPERATIONAL'
            time.sleep(0.1)
            # print(network[node_id].tpdo)
            sword_bin = bin(network[node_id].tpdo['status word'].raw)
            str_stat = " stat:{0:<15}".format(sword_bin)
            print(str_stat)
            pos = " stat:{0:<15}".format(network[node_id].tpdo['position actual value'].raw)
            print(pos)
            network[node_id].rpdo['motor_switch'].raw = 1
            network[node_id].rpdo['mode_of_operation'].raw = 3
            network[node_id].rpdo['start motion'].raw = 1
            network[node_id].rpdo[1].transmit()
            time.sleep(0.1)

        self.file = open("TextFile.txt", "w")

        self.change_status(status='STOPPED')
        self.change_status(status='RESET')
        self.change_status(status='RESET COMMUNICATION')
        self.change_status(status='INITIALISING')
        self.change_status(status='PRE-OPERATIONAL')
        self.change_status(status='OPERATIONAL')
        sys.stdout.flush()

    def recovery(self):
        """
        1. Connect to CAN Network
        """
        network = self.network
        network.scanner.search()
        time.sleep(0.5)
        print(len(network.scanner.nodes) )
        assert (len(network.scanner.nodes) == 4)
        for node_id in network.scanner.nodes:
            print('node_id',node_id)
            node_made_ = canopen.RemoteNode(node_id=node_id, object_dictionary='dcf/cobra4812_node{}.dcf'.format(node_id))
            network.add_node(node_made_)

            network[node_id].nmt.state = 'INITIALISING'
            all_codes = [emcy.code for emcy in network[node_id].emcy.log]
            active_codes = [emcy.code for emcy in network[node_id].emcy.active]
            for code_ in all_codes:
                print('emcy: {0}'.format(code_))
            for code_ in active_codes:
                print('emcy: {0}'.format(code_))
            network[node_id].emcy.reset()

        ## check bus state
        print('\nstate: {0}'.format(network.bus.state))
        print('bus channel_info: {0}'.format(network.bus.channel_info))

        ## -----------------------------------------------------------------
        ## send out time message
        network.time.transmit()
        time.sleep(0.1)

        self.change_status(status='RESET')
        self.change_status(status='RESET COMMUNICATION')
        self.change_status(status='INITIALISING')
        sys.stdout.flush()

    def run(self):
        network = self.network
        try:
            while True:
                """
                Direction Reference:
                    network[1].rpdo['target velocity'].raw = cmd_fl
                    network[2].rpdo['target velocity'].raw = cmd_bl
                    network[3].rpdo['target velocity'].raw = -cmd_fr
                    network[4].rpdo['target velocity'].raw = -cmd_br
                """
                if self.cmd.kill_signal:
                    print ('detect kill signal. shutting down ...')
                    raise KeyboardInterrupt()

                for node_id in network:
                    network[node_id].rpdo['mode_of_operation'].raw = 3
                    sword_bin = bin(network[node_id].tpdo['status word'].raw)
                    
                    network[node_id].rpdo['target velocity'].raw =  self.command[node_id]
                    ctrl_wrd = gen_controlword(sword_bin)

                    if ctrl_wrd == CtrlWord.FAULT_RESET:
                        if time.time() > self.start_time + 5.0:
                            if  time.time() > self.last_fault_time[node_id] + self.fault_stop_delay:
                                print(f'ID {node_id} -- fault detected ! reboot retry in {self.fault_stop_delay} sec')
                                network[node_id].rpdo['motor_switch'].raw = 0
                                network[node_id].rpdo['target velocity'].raw = 0
                        self.last_fault_time[node_id] = time.time()

                    if time.time() <= self.last_fault_time[node_id] + self.fault_stop_delay:
                        network[node_id].rpdo['motor_switch'].raw = 0
                        network[node_id].rpdo['target velocity'].raw = 0
                    else:
                        network[node_id].rpdo['motor_switch'].raw = 1
                        network[node_id].rpdo['mode_of_operation'].raw = 3
                        network[node_id].rpdo['start motion'].raw = 1
                            
                    network[node_id].rpdo['control word'].raw = ctrl_wrd
                    network[node_id].rpdo[1].transmit()
                    network[node_id].rpdo[2].transmit()
                    network[node_id].rpdo[3].transmit()
                        
                self.periodic_print()
                self.process_command()

                remaining = max(0.005-(time.time() - self.old_time),0.0)
                time.sleep(remaining)
                self.old_time = time.time()
                self.network.sync.transmit()
                
                
        except KeyboardInterrupt:
            print('safely stopping')
            for node_id in network.scanner.nodes:
                network[node_id].rpdo['motor_switch'].raw = 0
                network[node_id].rpdo['target velocity'].raw = 0
                network[node_id].rpdo[1].transmit()
            
            self.change_status(status='PRE-OPERATIONAL')
            self.change_status(status='INITIALISING')
            # self.change_status(status='STOPPED') # PLEASE DON'T IT WILL TERMINATE CAN COMM
            
    def periodic_print(self):
        """
        A. Periodical Print
        """
        network = self.network
        self.print_cnt += 1
        if self.print_cnt >= self.print_rate:
            self.print_cnt = 0

            """ 
            Put some codes for display 
            """
            # sword_bin = bin(network[3].tpdo['status word'].raw)
            # print( "n: {1} stat:{0:<32}\n".format(sword_bin, 1))
            # print('')
            # print('raw   cmd:', self.cmd.command, self.cmd.speed)
            # print('motor cmd:', self.command[1:5])
            # print('')
            # sys.stdout.flush()
            #print('pav: {}'.format(network[1].tpdo['position actual value'].raw))
            # print('cav: {}'.format(network[1].tpdo['current actual value'].raw))

    def process_command(self):
        """ 
        for test,
        arbitrary values are assigned 
        THIS WILL BE REPLACED BY "REAL COMM" CODES or just keyboard for fun?
        """
        network = self.network
        
        vel_rad = float(network[1].tpdo['velocity actual value'].raw) / 9.55
        vel_rad2 = float(network[2].tpdo['velocity actual value'].raw) / 9.55
        vel_rad3 = -float(network[3].tpdo['velocity actual value'].raw) / 9.55
        vel_rad4 = -float(network[4].tpdo['velocity actual value'].raw) / 9.55
        #print(vel_rad,  vel_rad2, vel_rad3, vel_rad4)

        # v_x, v_y, w;


        cmd_x, cmd_y, cmd_theta = self.cmd.command 
        booster = self.cmd.speed
        
        #velocity trash cmd handle
        if cmd_x > 1.0 or cmd_x < -1.0:
            print('commmand x error')
            cmd_x = 0.0
            cmd_y = 0.0
            cmd_theta = 0.0
        
        if cmd_y > 1.0 or cmd_y < -1.0:
            print('commmand y error')
            cmd_x = 0.0
            cmd_y = 0.0
            cmd_theta = 0.0
        
        if cmd_theta > 1.0 or cmd_theta < -1.0:
            print('commmand theta error')
            cmd_x = 0.0
            cmd_y = 0.0
            cmd_theta = 0.0

        # command filter
        pi = 3.141592
        del_t = 0.005
        cutoff_freq = 2
        cutoff_freq2 = 20
        self.cmd_LPF[0] = 1/(1 + 2*pi*cutoff_freq*del_t)*self.cmd_LPF[0] + (2*pi*cutoff_freq*del_t)/(1 + 2*pi*cutoff_freq*del_t)*cmd_x
        self.cmd_LPF[1] = 1/(1 + 2*pi*cutoff_freq*del_t)*self.cmd_LPF[1] + (2*pi*cutoff_freq*del_t)/(1 + 2*pi*cutoff_freq*del_t)*cmd_y
        self.cmd_LPF[2] = 1/(1 + 2*pi*cutoff_freq*del_t)*self.cmd_LPF[2] + (2*pi*cutoff_freq*del_t)/(1 + 2*pi*cutoff_freq*del_t)*cmd_theta
        # print(cmd_x , self.cmd_LPF[0])
        # print(cmd_y , self.cmd_LPF[1])
        # print(cmd_theta , self.cmd_LPF[2])
        
        # velocity mapping
        vel_x_max = 0.5
        vel_y_max = 0.5
        vel_theta_max = 0.5

        vel_x_min = 0.2
        vel_y_min = 0.2
        vel_theta_min = 0.2

        cmd_x_dead = 0.2
        cmd_y_dead = 0.2
        cmd_theta_dead = 0.2

        R_wheel = 0.091
        Length_1 = 0.287
        Length_2 = 0.255
        l1_l2 = 1.0/(Length_1 + Length_2)
        vel_x_real = R_wheel/4.0*(vel_rad + vel_rad2 + vel_rad3 + vel_rad4)
        vel_y_real = R_wheel/4.0*(-vel_rad + vel_rad2 + vel_rad3 - vel_rad4)
        vel_th_real = R_wheel/4.0*l1_l2*(-vel_rad - vel_rad2 + vel_rad3 + vel_rad4)

        self.vel_x_LPF = 1/(1 + 2*pi*cutoff_freq2*del_t)*self.vel_x_LPF + (2*pi*cutoff_freq2*del_t)/(1 + 2*pi*cutoff_freq2*del_t)*vel_x_real
        self.vel_x_dot = (self.vel_x_LPF - self.vel_x_LPF_prev)/del_t
        self.vel_x_LPF_prev = self.vel_x_LPF

        self.vel_y_LPF = 1/(1 + 2*pi*cutoff_freq2*del_t)*self.vel_y_LPF + (2*pi*cutoff_freq2*del_t)/(1 + 2*pi*cutoff_freq2*del_t)*vel_y_real
        self.vel_y_dot = (self.vel_y_LPF - self.vel_y_LPF_prev)/del_t
        self.vel_y_LPF_prev = self.vel_y_LPF        


        self.theta_LPF = 1/(1 + 2*pi*cutoff_freq2*del_t)*self.theta_LPF + (2*pi*cutoff_freq2*del_t)/(1 + 2*pi*cutoff_freq2*del_t)*vel_th_real
        self.theta_dot = (self.theta_LPF - self.theta_LPF_prev)/del_t
        self.theta_LPF_prev = self.theta_LPF

        #velocity calc
        #vel_x = vel_x_max*(self.cmd_LPF[0])
        #if(self.cmd_LPF[0] < cmd_x_dead and self.cmd_LPF[0] > -cmd_x_dead ):
        #    vel_x = 0
        #else:
        #    vel_x = ((vel_x_max - vel_x_min)/(1 - cmd_x_dead))*(self.cmd_LPF[0] - cmd_x_dead) + vel_x_min
        #vel_x = 0
        vel_x = vel_x_max*(self.cmd_LPF[0])
        vel_y = vel_y_max*(-self.cmd_LPF[1])


        #vel_y = - 0.05

        ###############################################
        #kp = 0.5
        #ki = 0.04
        #desired_theta = 0.0
        #desired_x = 0.1        
        #error = desired_theta - self.theta_LPF
        #self.intergral_ += error
        
        ############## Keunwoo's best gain tuning ##############
        #vel_theta = desired_theta - 0.05*self.theta_dot#+ ki * self.intergral_# + kp*error + ki * self.intergral_  #vel_theta_max*(self.cmd_LPF[2])
        #vel_x = desired_x - 0.3*self.vel_x_dot
        #vel_y = desired_y - 0.3*self.vel_y_dot
        ###############################################

        #print("cmd x", self.cmd_LPF[0])
        #print("vel x", vel_x)

        vel_x = vel_x_max*(cmd_x)
        vel_y = vel_y_max*(-cmd_y)
        vel_theta = vel_theta_max*(cmd_theta)

        # kinematics


        # vel_x forward plus
        # vel_y left plus
        w_1 = (vel_x - vel_y - (Length_1 + Length_2)*vel_theta)/R_wheel
        w_2 = (vel_x + vel_y - (Length_1 + Length_2)*vel_theta)/R_wheel
        w_3 = (vel_x + vel_y + (Length_1 + Length_2)*vel_theta)/R_wheel
        w_4 = (vel_x - vel_y + (Length_1 + Length_2)*vel_theta)/R_wheel

        #lines = str(vel_rad) +' '+ str(vel_rad2) +' '+ str(vel_rad3)+' '+str(vel_rad4)+' '+ str(w_1)+' '+ str(w_2)+' '+ str(w_3)+' '+ str(w_4)
        # lines = str(vel_x) +' '+ str(vel_y) +' '+ str(vel_theta)+' '+str(vel_x_real)+' '+ str(vel_y_real)+' '+ str(vel_th_real) + ' '+ str(self.vel_x_dot) + ' '+ str(self.vel_y_dot) + ' '+ str(self.theta_dot) 

        

        # self.file.write(lines)
        # self.file.write('\n')



        # motor 3,4 axis direction mapping
        # rad/s -> rpm
        rad2rpm = 9.55
        self.command[1] =  rad2rpm * w_1#front_left
        self.command[2] =  rad2rpm * w_2#back_left  
        self.command[3] =  -rad2rpm * w_3#front_right    
        self.command[4] =  -rad2rpm * w_4#back_right

        #speed_x = cy * speed_mod
        #speed_y = cx * speed_mod
        #speed_a = -cz * speed_mod * angle_def

        #self.command[1] =  (speed_y + speed_x + speed_a) #front_left
        #self.command[2] =  (speed_y - speed_x + speed_a) #back_left  
        #self.command[3] = -(speed_y - speed_x - speed_a) #front_right    
        #self.command[4] = -(speed_y + speed_x - speed_a) #back_right
