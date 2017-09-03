
# Vehicle status in maze search

이 노드의 목적은 터널구간의 입구에서부터 출구까지 차량의 위치와 상태를 파악한 후 장애물들을 피해 최단경로를 찾는것이다. 최단경로 알고리즘으로는 A* 알고리즘을 이용하였다. 6개의 상태에 따라 동작하도록 정

#### subscribed topic
	* /map(nav_msgs/OccupancyGrid)
	* /odom(nav_msgs/Odometry)
	* /scan(sensor_msgs/LaserScan)
#### published topic
	* /cmd_vel(geometry_msgs/Twist)


## 1. State is stop
차량의 x,y,z 각속도와 선속도를 모두 0으로 설정하여 동작을 멈춘다.

```python
if self.state == 'stop':
	self.publishing_vel(0, 0, 0, 0, 0, 0)
```

		
## 2. Setting_start_and_goal
입구의 양쪽 모서리좌표까지의 최소거리값을 구한 후 두좌표 사이의 중앙지점과 차량의 위치가 수직이되는 지점을 시작점으로 잡는다
끝나는 지점의 경우 시작점에서 경기장대각선으로 좌표를 계산한다.
```python
        if self.state == "setting_start_and_goal":
            min_distance = 100
            for i in range(0,90):
                if self.scan.ranges[i] < min_distance:
                    min_distance = self.scan.ranges[i]
                    idx_1 = i
            min_distance = 100
            for i in range(270,360):
                if self.scan.ranges[i] < min_distance:
                    min_distance = self.scan.ranges[i]
                    idx_2 = i
            point1 = [self.position_now[0] + self.scan.ranges[idx_1] * math.cos(idx_1 * np.pi/180), self.position_now[1] + self.scan.ranges[idx_1] * math.sin(idx_1 * np.pi/180)]
            point2 = [self.position_now[0] + self.scan.ranges[idx_2] * math.cos(idx_2 * np.pi/180), self.position_now[1] + self.scan. math.sin(idx_2 * np.pi/180)]
            
            ########오류#############코드
            between_point1_point2 = [point1[0] + point2[0], point1[1] + point2[1]] 
            # defining start point
            angle = theta_dot2dot(self.position_now, between_point1_point2)
            self.start_point = [between_point1_point2[0] + math.cos(angle) * 0.1, between_point1_point2[1] + math.sin(angle) * 0.1]
            
            # defining end point
            angle = theta_dot2dot(point2, point1)
            self.theta_exit = angle
            distance_axis_x = [1.6*math.cos(angle), 1.6*math.sin(angle)]
            distance_axis_y = [1.6*math.cos(angle - np.pi/2), 1.6*math.sin(angle - np.pi/2)]
            self.end_point = [self.start_point[0] + distance_axis_x[0] + distance_axis_y[0], self.start_point[1] + distance_axis_x[1] + distance_axis_y[1]]
self.state = "move_to_start_point"
```
## 3. Move_to_start_point

위에서 구한 시작점과 현재좌표와 각도를 이용하여 시작점까지 이동한다.

```python 
        #미완성
        if self.state == "move_to_start_point":
            self.move_to_some_point(self.position_now, self.theta_now, self.start_point)
            distance_remain = distance_dot2dot(self.position_now[0], self.position_now[1], self.start_point[0], self.start_point[1])
            if distance_remain < 0.02:
				self.state = "path_finding" # now maze solve start!!
```


## 4. Path_finding
maze_solving_click_mode/readme 참고
```python
        if self.state == 'path_finding':
            solver = Solver([self.low_position, self.col_position], [self.destination_low, self.destination_col], map.data)
            solver.solve_distance()
            solver.find_shortest_path()
            self.shortest_path = solver.shortest_path
			self.state = 'direction_setting'
```
## 5. Direction_setting
slam을 통해 생선된 맵의 중앙좌표(192,192)와 turtlebot3의 현재위치를 같게 수정한다.
```python
        self.low_position = 192 + int((odometry.pose.pose.position.x) * 20) + 
        self.col_position = 192 + int((odometry.pose.pose.position.y) * 20) + 9
```
3개의 허수부와 1개의 실수부로 이루어진 Quaternion vector(qx,qy,qz,qw)를 통해 Euler angles의 z각도를 구한다.
 ```python

        self.theta = euler_from_quaternion([odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w])
        self.position_now = [odometry.pose.pose.position.x, odometry.pose.pose.position.y]
        #Adjust to the current angle.
        self.theta_now = self.theta+np.pi/2 
        if self.theta < 0:
            self.theta = self.theta + np.pi*2
 ```
현재좌표와 경유점의 좌표가 이루는 각도값을 구함
 
 ```python
        direction_desired = math.atan2(self.low_position - self.path[0], self.path[1] - self.col_position)
        if direction_desired < 0:  #Adjust to the current angle.
			direction_desired = direction_desired + np.pi*2
```
현재방향(z축)과 목표방향의 차이에 따른 방향조정한다.
```python
#theta : 현재각도 , direction_desired : 목표각도
if self.state == 'direction_setting':
            # calculate degree and direction
            if direction_desired > self.theta:
                if direction_desired - self.theta < np.pi:
                    turn_direction = 'left'
                else:
                    turn_direction = 'right'
            else:
                if self.theta - direction_desired < np.pi:
                    turn_direction = 'right'
                else:
                    turn_direction = 'left'
                    
            # publish topic : 방향조정을 위한 모터동작
            #현재 각도와 목표 각도의 차이 가 클 경우 회전속도를 다를게 설정
            difference = abs(direction_desired - self.theta)
            if difference > np.pi:
                difference = np.pi*2 - difference
            if difference > 0.3:
                turn_speed = 0.6
            elif difference > 0.2:
                turn_speed = 0.3
            elif difference > 0.1:
                turn_speed = 0
            elif difference
                turn_speed = 0.05
            else:
                turn_speed = 0
                self.state = 'going'

            if turn_direction =='left':
                angular_z = turn_speed
            else:
                angular_z = - turn_speed
                
self.publishing_vel(0, 0, angular_z, 0, 0, 0) # 정해진 각속도값을 출력
```
## 6. Going
3가지의 경우가 생기는데 지정된 경유점으로 똑바르게 가는경우, 잘못된 방향으로 가는 경우, 목표지점까지의 거리가 0인경우로 나뉜다.
잘못된 방향으로 가는 경우 직선과 경유점이 수직으로 만나는 거리를 구한 후 임계값 클 경우, 방향을 다시 조정한다.
똑바르게 갈 경우 계속해서 경로를 찾아간다. 목표지점에 도착하면 멈춘다.

```python

        if self.state == 'going':
            a = math.tan(self.theta + np.pi/2)
            b = -1
            c = -a*self.low_position + self.col_position
            distance_expected = distance_dot2line(a, b, c, self.path[0], self.path[1])
            distance_now = distance_dot2dot(self.low_position, self.col_position, self.path[0], self.path[1])
            distance_from_destination = distance_dot2dot(self.low_position, self.col_position, self.destination_low, self.destination_col)
            self.publishing_vel(0, 0, 0, 0.06, 0, 0)

            # print 'expected : ', distance_expected, 'now : ', distance_now
            if distance_expected > 1:
                self.state = 'direction_setting'
            if distance_from_destination == 0:
                self.state = 'stop'
            elif distance_now == 0:
                self.state = 'path_finding'
				self.publishing_vel(0, 0, 0, 0, 0, 0)
```





