# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("c5_pickNplace", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_BASE,
            DR_MV_MOD_REL
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
            pass

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        #wait_digital_input(2)

    def grip():
        release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        #wait_digital_input(1)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([400, -50, 90, 0, -180, 0])
    pos2 = posx([400, 50, 90, 0, -180, 0])
    

    while rclpy.ok():
        set_tool("Tool Weight_2FG")
        set_tcp("2FG_TCP")

        # 초기 위치로 이동
        while True:
            release()
            print("movej")
            movej(JReady, vel=VELOCITY, acc=ACC)
            print("Move to Picking Spop")
            movel(pos1, vel=VELOCITY, acc=ACC)
            print("Grapping")
            movel(posx(0, 0, -50, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VELOCITY, acc=ACC)
            grip()
            print("Pull Up")
            movel(posx(0, 0, 50, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VELOCITY, acc=ACC)
            print("Move to Placing Spot")
            movel(pos2, vel=VELOCITY, acc=ACC)
            print("Placing")
            movel(posx(0, 0, -30, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VELOCITY, acc=ACC)
            release()
            movel(posx(0, 0, 30, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VELOCITY, acc=ACC)
            print("movej")
            movej(JReady, vel=VELOCITY, acc=ACC)
            print("Move to Picking Spop")
            movel(pos2, vel=VELOCITY, acc=ACC)
            print("Grapping")
            movel(posx(0, 0, -50, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VELOCITY, acc=ACC)
            grip()
            print("Pull Up")
            movel(posx(0, 0, 50, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VELOCITY, acc=ACC)
            print("Move to Placing Spot")
            movel(pos1, vel=VELOCITY, acc=ACC)
            print("Placing")
            movel(posx(0, 0, -30, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VELOCITY, acc=ACC)
            release()
            movel(posx(0, 0, 30, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VELOCITY, acc=ACC)
            print("movej")
            movej(JReady, vel=VELOCITY, acc=ACC)
            
            

if __name__ == "__main__":
    main()
