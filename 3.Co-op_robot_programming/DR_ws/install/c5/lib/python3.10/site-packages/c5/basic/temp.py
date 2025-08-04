

P_cup_storage = [425, -75, 198, 0, 180, 0]
H_cup = 95
H_cup_grip = 9
H_cup_diff = 11.5
curr_stack = [[], []]
P_cup_stack = [575, 85, 230, 0, 180, 0]
W_cup_offset = 80

for i in range(10):
    P_temp_goal = P_cup_storage.copy()
    P_temp_goal[2] += 20
    # movel(posx(P_temp_goal), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_HIGH, acc=ACC_HIGH)
    # ungrip()
    P_temp_goal[2] -= 20 + (H_cup_diff * i)
    # movel(posx(P_temp_goal), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_LOW, acc=ACC_LOW)
    # grip()
    # movel(posx(0,0,100,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=VEL_LOW, acc=ACC_LOW)
    
    P_temp_goal = P_cup_stack.copy()
    # Calculating X, Z
    if i <= 5:
        P_temp_goal[0] -= (i//2.5) * (W_cup_offset +2)
        cup_layer = 0
    elif 5 < i <= 8:
        P_temp_goal[0] -= (((W_cup_offset +2) * 1.73)/6 + (i//8) * ((W_cup_offset +2) * 1.73)/2)
        cup_layer = 1
    else:
        P_temp_goal[0] -= ((W_cup_offset +2) * 1.73) / 3
        cup_layer = 2
    P_temp_goal[2] = 110 + H_cup * cup_layer
    # Calculating Y
    if i <= 2:
        P_temp_goal[1] += (W_cup_offset + 2) * i
    elif i == 3 or i == 4 or i == 6 or i == 7:
        P_temp_goal[1] += (W_cup_offset + 2)/2 + (i % 3) * (W_cup_offset +2)
    else:
        P_temp_goal[1] += (W_cup_offset +2)
        
    P_temp_Z_goal = P_temp_goal.copy()
    P_temp_Z_goal[2] += H_cup
    # movel(posx(P_temp_Z_goal), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_HIGH, acc=ACC_HIGH)
    # movel(posx(P_temp_goal), ref=DR_BASE, mod=DR_MV_MOD_ABS, vel=VEL_LOW, acc=ACC_LOW)
    # ungrip()
    # mwait()
    print(f"point{i}")
    print(P_temp_Z_goal)
    print(P_temp_goal)