
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL_LOW, ACC_LOW = 100, 100
VEL_HIGH, ACC_HIGH = 220, 220

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
        
    def apply_force_control():
        print('Sensing...')
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        print('Contact!')
        release_compliance_ctrl()


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
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            DR_TOOL,
            set_digital_output,
            mwait,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return


    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    
    # Cup storage coordinate
    P_cup_storage = [425, -75, 198, 0, 180, 0]
    H_cup = 95
    H_cup_grip = 9
    H_cup_diff = 11.5
    P_cup_stack = [575, 50, 230, 0, 180, 0]
    W_cup_offset = 80
    
    J_last_cup = [-57.7, 29.9, 122.4, 75.5, 95.4, -62]
    J_last_spot = [-22.247, 3.651, 104.603, 94.483, 79.429, -198.728]
    
       
    set_tool("Tool Weight_GR")
    set_tcp("onRobotRG2")

    if rclpy.ok():
        ungrip()
        # 초기 위치로 이동
        print('Starting...')
        movel(posx(0,0,20,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
        print('Move to Starting Point')
        movej(JReady, vel=20, acc=20)
        movel(posx(0,0,20,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=ACC_HIGH)
        grip()
        
        for i in range(10):
            # Make placing coordinate
            P_temp_goal = P_cup_storage.copy()
            # Z value offset for avoid contact
            P_temp_goal[2] += 20
            print(f'Move to cup storage point for cup number {i+1}')
            movel(posx(P_temp_goal), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_HIGH, acc=ACC_HIGH)
            ungrip()
            
            print(f"Grip cup number {i+1}")
            # Lower Z value of cup storage step by step
            P_temp_goal[2] -= 20 + (H_cup_diff * i)
            movel(posx(P_temp_goal), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_LOW, acc=ACC_LOW)
            grip()
            
            # Move Z for avoid contact with placed cup floor by floor
            if i >= 6:
                movel(posx(0, 0, 200, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=ACC_HIGH)
            else:
                movel(posx(0, 0, 100, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=ACC_HIGH)
            print("Move for avoid contact with stacked cup")
            movel(posx(0, 80, 0, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=ACC_HIGH)
            
            # Make Placing coordinate with P_cup_stack coordinate with math formula
            P_temp_goal = P_cup_stack.copy()
            # Calculating X, Z
            if i <= 5:
                P_temp_goal[0] -= (i//2.5) * (((W_cup_offset +2) * 1.73)/2)
                cup_layer = 0
            elif 5 < i <= 8:
                P_temp_goal[0] -= (((W_cup_offset +2) * 1.73)/6 + (i//8) * ((W_cup_offset +2) * 1.73)/2)
                cup_layer = 1
            else:
                P_temp_goal[0] -= (((W_cup_offset +2) * 1.73) / 3)
                cup_layer = 2
            P_temp_goal[2] = 100 + H_cup * cup_layer
            # Calculating Y
            if i <= 2:
                P_temp_goal[1] += (W_cup_offset + 2) * i
            elif i == 3 or i == 4 or i == 6 or i == 7:
                P_temp_goal[1] += (W_cup_offset + 2)/2 + (i % 3) * (W_cup_offset +2)
            else:
                P_temp_goal[1] += (W_cup_offset +2)
            
            # Make temp coordinate for avoid placed cup with height
            P_temp_Z_goal = P_temp_goal.copy()
            P_temp_Z_goal[2] += H_cup
            print(f"Move to placing point {i+1}")
            movel(posx(P_temp_Z_goal), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_HIGH, acc=ACC_HIGH)
            
            # Move Z with high speed with little gap
            print(f"Placing Cup {i+1}")
            movel(posx(P_temp_goal), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_LOW, acc=ACC_LOW)
            # start force control for softly ungrap cup
            apply_force_control()
            ungrip()
            movel(posx(0, 0, 30, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=ACC_HIGH)
            mwait()
        
        print("Move to grap last cup!")
        movel(posx(P_cup_storage), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_HIGH, acc=ACC_HIGH)
        movej(J_last_cup, vel=30, acc=30)
        
        print("Grapping last cup!!")
        movel(posx(0, 0, -50, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
        movel(posx(0, 0, -50, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=10, acc=10)
        movel(posx(0,0,50,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
        
        print("Move to put last cup!!!")
        movej(J_last_spot , vel=30, acc=30)
        
        print("Putting last cup!!!!")
        movel(posx(0, 0, -100, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
        movel(posx(0, 0, -50, 0, 0, 0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=10, acc=10)
        movel(posx(0,0,-80,0,0,0), ref=DR_TOOL, mod=DR_MV_MOD_REL, vel=100, acc=100)
        
        movej(JReady, vel=30, acc=30)
        print("Process finished.")
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
