# error in move periodic(line 124)

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL_LLOW, ACC_LLOW = 20, 20
VEL_LOW, ACC_LOW = 50, 50
VEL_HIGH, ACC_HIGH = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("domino1", namespace=ROBOT_ID)

    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        # wait_digital_input(1, ON)
    def ungrip():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        # wait_digital_input(2, ON)


    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            amovel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            get_current_posx,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            set_digital_output,
            # wait_digital_input,
            move_periodic,

        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
       
    set_tool("Tool Weight_GR")
    set_tcp("onRobotRG2")

    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]

    # Gear Point
    p_gear = []
    p_gear1 = [457, -202, 70, 0, 180, 0]
    p_gear2 = [364, -156, 70, 0, 180, 0]
    p_gear3 = [449, -97, 70, 0, 180, 0]
    p_gearC = [425, -148, 70, 0, 180, 0]
    p_gear.append(p_gear1)
    p_gear.append(p_gear2)
    p_gear.append(p_gear3)
    p_gear.append(p_gearC)
    x_gear = []
    for i in range(4):
        x_gear.append(posx(p_gear[i]))

    p_gear_goal = []
    x_gear_goal = []
    for i in range(4):
        p_gear_goal = p_gear.copy()
        p_gear_goal[i][1] += 300
        p_gear_goal[i][2] += 30
        x_gear_goal.append(posx(p_gear_goal[i]))
    

    sequence = ["First", "Second", "Third", "Middle"]

    if rclpy.ok():
        ungrip()
        
        # 초기 위치로 이동
        print('Starting...')
        movel(posx(0,0,50,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW) 
        print('Move to Starting Point')
        movej(JReady, vel=20, acc=20)
        grip()

        for i in range(3,4):
            print(f"Move to {sequence[i]} gear.")
            movel(x_gear[i], mod=DR_MV_MOD_ABS, ref=DR_BASE, vel=VEL_HIGH, acc=ACC_HIGH)
            ungrip()
            
            print(f"Grap {sequence[i]} gear.")
            movel(posx(0, 0, -25, 0, 0, 0), mod=DR_MV_MOD_REL, ref=DR_BASE, vel=VEL_LOW, acc=ACC_LOW)
            grip()
            movel(posx(0, 0, 85, 0, 0, 0), mod=DR_MV_MOD_REL, ref=DR_BASE, vel=VEL_LOW, acc=ACC_LOW)

            print(f"Transmitting {sequence[i]} gear.")
            movel(x_gear_goal[i], mod=DR_MV_MOD_ABS, ref=DR_BASE, vel=VEL_HIGH, acc=ACC_HIGH)
        
            if i == 3:
                print(f"Put down {sequence[i]} gear.")
                task_compliance_ctrl(stx= [500, 500, 500, 100, 100, 100])
                set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                while get_current_posx()[0][2] > 50:
                    while check_force_condition(DR_AXIS_Z, max=10):
                        print('start move periodic')
                        move_periodic(amp =[0,0,0,0,0,30], period=[0,0,0,0,0,2], atime=1, repeat=1, ref=DR_BASE)
                        print('end move periodic')
                        break
                    print('current z is higher than 50')
                    pass
                print('release compliance ctrl')
                release_compliance_ctrl()
            else:    
                print(f"Put down {sequence[i]} gear.")
                movel(posx(0, 0, -15, 0, 0, 0), mod=DR_MV_MOD_REL, ref=DR_BASE, vel=VEL_LLOW, acc=ACC_LLOW)
            print("Ungrip")
            ungrip()
            movel(posx(0, 0, 85, 0, 0, 0), mod=DR_MV_MOD_REL, ref=DR_BASE, vel=VEL_LOW, acc=ACC_LOW)
            
            
            movej(JReady, vel=10, acc=10)
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
