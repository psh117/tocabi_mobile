"""
TOCABI MOBILE ACTIVATOR UTILITIES (utils.py)

@ Authors: Suhan Park (psh117@snu.ac.kr), Junewhee Ahn (june992@snu.ac.kr)
@ 2022 @ DYROS

@ Some source codes are from 
    dyros_pcv_canopen @ KIM-HC github repo 
    (https://github.com/KIM-HC/dyros_pcv_canopen)
    @ See script/elmo.py

@ Questions / Help
    CANOpen API -> psh117@snu.ac.kr
    cobra4812_node*.dcf -> june992@snu.ac.kr

@ TODOs
    TODO: ROS Comm or TCP/UDP Comm Implementation
    TODO: Mechanum Control
"""

class CtrlWord():
    FAULT_RESET =  0b10000000
    SWITCH_ON =    0b10000001
    SHUTDOWN =          0b110
    SWITCH_ON =         0b111
    # DISABLE_OPERATION = 0b111
    ENABLE_OPERATION = 0b1111
    ENABLE_OPERATION = 0b1111


def gen_controlword(statusword):
    global pc
    if statusword[-2:-1] == '11':
        return CtrlWord.ENABLE_OPERATION
    if statusword[-4] == '1': # fault
        # pc += 1
        # if pc > 100:
        #     pc = 0
        #     print('fault')
        # time.sleep(1)
        return CtrlWord.FAULT_RESET
    if statusword[-7] == '1': # quick stop
        # print('quick stop')
        # time.sleep(1)
        return CtrlWord.SHUTDOWN
    if statusword[-1] == '1':
        return CtrlWord.ENABLE_OPERATION
    if statusword[-6] == '1': # quick stop
        return CtrlWord.SHUTDOWN

    return CtrlWord.ENABLE_OPERATION