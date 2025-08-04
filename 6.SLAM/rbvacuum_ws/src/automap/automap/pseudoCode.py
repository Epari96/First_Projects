
def odom_callback(self, msg):
    self.현재위치 = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    if self.현재목표:
        distance = {'현위치-목표위치 유클리드 거리'}
        if distance < 0.3:
            old_goal = self.현재목표
            if old_goal not in self.방문목록:
                self.방문목록.append(old_goal)
            self.update_frontiers_after_goal()
        else:
            if '이동없음' and 'gaol_timeout발생시':
                old_goal = self.현재목표
                if old_goal not in self.방문목록:
                    self.방문목록.append(old_goal)
                self.update_frontiers_after_goal()
                
def map_callback(self, map_msg):
    frontiers = self.detect_frontiers(map_msg)
    if frontiers:
        target = self.select_best_frontier(frontiers)
        if target and not self.is_goal_in_progress():
            self.publish_goal(target)
    else:
        if self.현재목표:
            pass
        else:
            self.update_frontiers_after_goal()
            
def check_termination_conditions(self, map_msg):
    frontiers = self.detect_frontiers(map_msg)
    if '최대 탐색 시간 초과':
        '맵 저장'
    if '프론티어 없음':     
        '맵 저장'
        
def detect_frontiers(self, map_msg):
    '맵 메세지에서 0 인근 픽셀에 -1이 8픽셀 이상 모여있을 경우 프론티어로 저장'
    
def update_frontiers_after_goal(self):
    if not self.is_exploring:
        return
    frontiers = self.detect_frontiers(self.map_msg)
    if frontiers:
        target = self.select_best_frontier(frontiers)
        if target:
            self.publish_goal(target)
            
def select_best_frontier(self, frontiers):
    '거리기반, 방문목록에 없는 가장 가까운 프론티어 반환'
    

'''
# slam
resolution: 0.05 -> 0.02                       # type: ignore
loop_search_maximum_distance: 3.0 -> 0.5       # type: ignore

# nav2
'global_costmap'                               # type: ignore
  inflation_radius: 0.45 -> 0.6                # type: ignore
  update_frequency: 1.0 -> 10.0                # type: ignore
  robot_radius: 0.175 -> 0.155                 # type: ignore
  resolution: 0.6 -> 0.01                      # type: ignore
  
'local_costmap'                                # type: ignore
  inflation_radius: 0.45 -> 0.3                # type: ignore
  update_frequency: 5.0 -> 10.0                # type: ignore
  robot_radius: 0.175 -> 0.155                 # type: ignore
  resolution: 0.6 -> 0.01                      # type: ignore
  cost_scaling_factor: 4.0 -> 10.0             # type: ignore

'FollowPath'                                   # type: ignore
  PathAlign.scale: 32.0 -> 50.0                # type: ignore


'''

