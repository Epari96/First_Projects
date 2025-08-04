
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
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
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            get_current_posx,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            set_digital_output,
            # wait_digital_input,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return


    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]

    # Domino Storage Point
    p_storage = [560, 55, 150, 90, -180, 90]
    p_grip = [560, 55, 150, 90, 135, 90]
    j_grip = [-19.3, 34.4, 68.1, 41.9, 94.1, -22.4]
    p_ungrip = [560, -150, 80, 90, -135, 90]
    j_ungrip = [8.712, 33.328, 78.133, -45.173, 81.056, 21.398]
    domino_stack = 0
    p_domino_start = [450, -100, 150, 90, -180, 90]
       
    set_tool("Tool Weight_GR")
    set_tcp("onRobotRG2")

    if rclpy.ok():
        ungrip()
        # 초기 위치로 이동
        print('Starting...')
        movel(posx(0,0,50,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
        print('Move to Starting Point')
        movej(JReady, vel=20, acc=20)
        # Move to storage point
        grip()
        print(f'Move to Domino Storage Point', p_storage)
        movel(posx(p_storage), vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        # Move to stacked domino
        print('Reaching for domino')
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        # # for simulation 
        # while get_current_posx()[0][2] > 100:
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        print('Contact!')
        curr_z = get_current_posx()[0][2]
        release_compliance_ctrl()
        movel(posx(p_storage), vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        domino_stack = int((curr_z+2)//15)
        print('Number of Currently Stored Domino is',domino_stack)
        
        for i in range(0,domino_stack):
            print('Grip Sequence Initiating')
            print(f'Positiong {j_grip} for grip')
            movej(j_grip, vel=10, acc=10)
            ungrip()
            movel(posx(0,0,-(150-curr_z+15*(i+1)),0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
            grip()
            movel(posx(0,0,40,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
            print('Aligning gripper')
            movel(posx(p_storage), vel=VEL_LOW, acc=ACC_LOW, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            print('Move to domino aligning point')
            movel(posx(560, -150, 150, 90, -180, 90), vel=VEL_LOW, acc=ACC_LOW, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            print(f'Positiong {p_ungrip} for align domino')
            movej(j_ungrip, vel=10, acc=10)
            print('Domino aligning...')
            
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
            release_compliance_ctrl()
            ungrip()
            movel(posx(560, -150, 65, 90, -180, 90), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_LOW, acc=ACC_LOW)
            # movej([-15.7, 32.3, 69.1, 0, 78.6, -15.6], vel=10, acc=10)
            grip()
            
            p_domino_stack = p_domino_start
            movel(posx(0,0,150,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=VEL_HIGH)
            p_domino_stack[1] += 50
            movel(posx(p_domino_stack), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_HIGH, acc=VEL_HIGH)
            
            print('Domino Stacking...')
            movel(posx(0,0,-70,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=VEL_LOW)
            
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5):
                pass
            release_compliance_ctrl()
            ungrip()
            movel(posx(0,0,80,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
            print(f"{i+1} domino stacked!")
            
            if i < domino_stack-1:
                p_storage[2] -= 15
                movel(posx(p_storage), vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            else:
                movel(posx(0,40,-80,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
                grip()
                movel(posx(0,-50,0,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=VEL_HIGH)
                movej(JReady, vel=10, acc=10)
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
