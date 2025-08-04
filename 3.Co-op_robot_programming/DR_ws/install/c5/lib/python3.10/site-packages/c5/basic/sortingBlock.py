
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VEL_LOW, ACC_LOW = 50, 50
VEL_HIGH, ACC_HIGH = 120, 120

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

    # Make Grid Point With 3 Point
    point1 = (598, 150, 90, 90, -180, 90)
    point2 = (498, 150, 90, 90, -180, 90)
    point3 = (598, 50, 90, 90, -180, 90)

    rows = 3
    cols = 3

    Pallet_Point = []
    for i in range(rows):
        for j in range(cols):
            x = point1[0] + j * (point2[0] - point1[0]) / (cols - 1)
            y = point1[1] + i * (point3[1] - point1[1]) / (rows - 1)
            z, rx, ry, rz = point1[2], point1[3], point1[4], point1[5]
            Pallet_Point.append(posx(x, y, z, rx, ry, rz))
    
    # Goal pallet position
    point5 = [450, 50, 150, 90, -180, 90]
    Pallet_Stacked = [0, 0, 0]
       
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
        for i in range(0,9):
            grip()
            print(f'Move to Grap Point {i+1} ', Pallet_Point[i])
            movel(Pallet_Point[i], vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE)
            print('Reaching to Block')
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            # # for simulation 
            # while not check_force_condition(DR_AXIS_Z, max=5) or get_current_posx()[0][2] > 60:
            while not check_force_condition(DR_AXIS_Z, max=5):
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
            movel(posx(0,0,10,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_HIGH, acc=ACC_HIGH)
            ungrip()
            movel(posx(0,0,-30,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
            grip()
            movel(posx(0,0,80,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)

            print('Move to Stacking Point')
            point_goal = point5.copy()
            if block_type == 0:
                point_goal[1] += 50 * Pallet_Stacked[0]
                pos_goal = posx(point_goal)
                print('Move to goal point: ', point_goal)
                movel(pos_goal, vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE)
                movel(posx(0,0,-70,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
                Pallet_Stacked[0] += 1
            elif block_type == 1:
                point_goal[0] -= 50
                point_goal[1] += 50 * Pallet_Stacked[1]
                pos_goal = posx(point_goal)
                print('Move to goal point: ', point_goal)
                movel(pos_goal, vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE)
                movel(posx(0,0,-80,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
                Pallet_Stacked[1] += 1
            else:
                point_goal[0] -= 100
                point_goal[1] += 50 * Pallet_Stacked[2]
                pos_goal = posx(point_goal)
                print('Move to goal point: ', point_goal)
                movel(pos_goal, vel=VEL_HIGH, acc=ACC_HIGH, ref=DR_BASE)
                movel(posx(0,0,-90,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
                Pallet_Stacked[2] += 1
                
            print('Realese Sequence Initiating')
            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            # # for simulation
            # while not check_force_condition(DR_AXIS_Z, max=15): # or get_current_posx()[0][2] > 60:
            while not check_force_condition(DR_AXIS_Z, max=15):
                pass
            release_compliance_ctrl()
            ungrip()
            print("Present Pallet Stack: ", Pallet_Stacked)
            movel(posx(0,0,60,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
        
        movej(JReady, vel=VEL_LOW, acc=ACC_LOW)
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
