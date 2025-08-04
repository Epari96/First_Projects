# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL_LOW, ACC_LOW = 40, 40
VEL_HIGH, ACC_HIGH = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

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
            set_digital_output,
            # wait_digital_input,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return


    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]

    point1 = (600, 150, 90, 90, -180, 90)
    point2 = (500, 150, 90, 90, -180, 90)
    point3 = (600, 50, 90, 90, -180, 90)
    point4 = (500, 50, 90, 90, -180, 90)
    Pallet_Point = []
    
    # Goal pallet position
    point5 = [450, 50, 150, 90, -180, 90]
    Pallet_Stacked = [0, 0, 0]
       
    # # Make grid pallet point
    # direction = 0 # Normal Pallet -> 0: Snake, 1: Zigzag / Rhombus Pallet -> 2: Snake, 3: Zigzag
    # row = 3
    # column = 3
    # stack = 1
    # thickness = 0
    # point_offset = [0, 0, 0] # Offset for calculated pose
    # if direction < 2: # Normal Pallet
    #     total_count = row * column * stack
    # else: # Rhombus Pallet
    #     total_count = (row * column - int(row/2)) * stack
    # for pallet_index in range(0, total_count):
    #     Pallet_Pose = get_pattern_point(pos1, pos2, pos3, pos4, pallet_index, direction, row, column, stack, thickness, point_offset)
    #     Pallet_Point.append(Pallet_Pose)
    
    # row = 3
    # column = 3
    # for i in range(row):
    #     for j in range(column):
    #         Pallet_Pose = posx(point1)
    #         Pallet_Pose[0] += (i-1) * (point2[0]-point1[0])/(column-1)
    #         Pallet_Pose[1] += (j-1) * (point3[1]-point1[1])/(row-1)
    #         Pallet_Point.append(Pallet_Pose)
    
    Pallet_Point.append(posx(598, 150, 90, 90, -180, 90))
    Pallet_Point.append(posx(548, 150, 90, 90, -180, 90))
    Pallet_Point.append(posx(498, 150, 90, 90, -180, 90))
    Pallet_Point.append(posx(598, 100, 90, 90, -180, 90))
    Pallet_Point.append(posx(548, 100, 90, 90, -180, 90))
    Pallet_Point.append(posx(498, 100, 90, 90, -180, 90))
    Pallet_Point.append(posx(598, 50, 90, 90, -180, 90))
    Pallet_Point.append(posx(548, 50, 90, 90, -180, 90))
    Pallet_Point.append(posx(498, 50, 90, 90, -180, 90))
    
            
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    ungrip()

    if rclpy.ok():
        # 초기 위치로 이동
        print('Start')
        movel(posx(0,0,100,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
        print('Move to Startimg Point')
        movej(JReady, vel=VEL_LOW, acc=ACC_LOW)
        # Move to First Position
        for i in range(0,8):
            grip()
            print(f'Move to Grap Point {i+1} ', Pallet_Point[i])
            movel(Pallet_Point[i], vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE)
            print('Reaching to Block')
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=5): # or get_current_posx()[2]>60:
                pass
            print('Contact!')
            curr_z = get_current_posx()[0][2]
            release_compliance_ctrl()
            
            if curr_z > 65:
                block_type = 0
                print("This block is long!")
            elif 55 < curr_z <= 65:
                block_type = 1
                print("This block is middle!")
            elif 45 < curr_z <= 55:
                block_type = 2
                print("This block is short!")
            else:
                movel(posx(0,0,150,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=ACC_HIGH)
                print("Gripper is not on the block!\nShutting Down.")
                rclpy.shutdown()
            
            print('Grip Sequence Initiating')
            movel(posx(0,0,25,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=ACC_HIGH)
            ungrip()
            movel(posx(0,0,-45,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
            grip()
            movel(posx(0,0,80,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)

            print('Move to Stacking Point')
            if block_type == 0:
                point_goal = point5
                point_goal[1] += 50 * Pallet_Stacked[0]
                pos_goal = posx(point_goal)
                print('Move to goal point: ', point_goal)
                movel(pos_goal, vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE)
                Pallet_Stacked[0] += 1
                point_goal = point5
            elif block_type == 1:
                point_goal = point5
                point_goal[0] -= 50
                point_goal[1] += 50 * Pallet_Stacked[1]
                pos_goal = posx(point_goal)
                print('Move to goal point: ', point_goal)
                movel(pos_goal, vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE)
                Pallet_Stacked[1] += 1
                point_goal = point5
            else:
                point_goal = point5
                point_goal[0] -= 100
                point_goal[1] += 50 * Pallet_Stacked[2]
                pos_goal = posx(point_goal)
                print('Move to goal point: ', point_goal)
                movel(pos_goal, vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE)
                Pallet_Stacked[2] += 1
                point_goal = point5
            print("Present Pallet Stack: ", Pallet_Stacked)
                
            print('Realese Sequence Initiating')
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            while not check_force_condition(DR_AXIS_Z, max=15): # or get_current_posx()[2]>60:
                pass
            release_compliance_ctrl()
            ungrip()
            movel(posx(0,0,60,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
        
        movej(JReady, vel=VEL_LOW, acc=ACC_LOW)
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
