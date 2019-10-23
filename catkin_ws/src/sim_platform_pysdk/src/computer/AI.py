#!/usr/bin/env python
# AI program
import time
import math
import os
import sys
import traceback

sys.path.append(os.path.abspath((os.path.dirname(__file__) + os.path.sep + '..')))
from common.obstacles import Obstacles

from computer_interface import ComputerInterface
from mymath import Math


class AI(ComputerInterface):
    EMERGENCY_RADIUS = 25

    def __init__(self, team_name='B'):
        super(AI, self).__init__(team_name)
        
        self.k_p_xy = 0.1
        self.k_p_z = 1.0
        self.k_p_yaw = 1.0

        self.v_max_xy_uav = 2.0
        self.v_min_xy_uav = 1.0
        self.v_max_xy_car = 2.0
        self.v_min_xy_car = 1.0
        self.v_max_z_uav = 1
        self.v_yaw_uav = 0.1
        self.v_yaw_car = 0.5

        self.attack_radius = 20
        self.arrive_radius = 10
        self.init_pose = {}
        self.plan_points_dict = {}

    def get_next_target(self, name):
        return self.plan_points_dict[name][0]

    def get_target_num(self, name):
        return len(self.plan_points_dict[name])

    def set_plan_by_2Dpoint(self, name, point):
        init_state = self.get_init_pose(name)
        self.plan_points_dict[name] = [
            point + init_state[2:3] + [0]
        ]

    def set_plan_by_2Dpoints(self, name, points):
        init_state = self.get_init_pose(name)
        self.plan_points_dict[name] = []
        for point in points:
            self.plan_points_dict[name].append(
                point + init_state[2:3] + [0]
            )

    def is_emergency(self):
        for name in self.get_enemy_alive_agent_name_list():
            if self.dist2basestation2D(name) <= self.EMERGENCY_RADIUS:
                return True
        return False
            
    def no_enemy_alive(self):
        return False if self.get_enemy_alive_agent_name_list() else True

    #-- Select target at different mode ---------------------------------------
    def select_basestation_as_target(self):
        for name in self.get_self_alive_agent_name_list():
            self.set_plan_by_2Dpoint(name, self.STATE_ENEMY_BASESTATION[:2])
    
    def select_target_from_emergency_zone(self):
        target = self.get_enemy_alive_agent_name_list()[0]
        dist_min = self.dist2basestation2D(target)
        for name in self.get_enemy_alive_agent_name_list():
            dist = self.dist2basestation2D(name)
            if dist < dist_min:
                dist_min = dist
                target = name
        
        target_state = self.get_pose(target)
        for name in self.get_self_alive_agent_name_list():
            self.set_plan_by_2Dpoint(name, target_state[:2])

    def select_target_from_anywhere(self):
        index_max = []
        for target in self.get_enemy_alive_agent_name_list():
            list_tmp = []
            for name in self.get_self_alive_agent_name_list():
                list_tmp.append((name, self.dist2agents2D(name, target), ))
            for name in self.get_enemy_alive_agent_name_list():
                list_tmp.append((name, self.dist2agents2D(name, target), ))
            list_tmp = sorted(list_tmp, key=lambda agent: agent[1])
            
            prefix = [-1] + [0] * (len(list_tmp)-1)
            for i in range(1, len(list_tmp)):
                prefix[i] = prefix[i-1] + 1 if list_tmp[i][0] in self.get_self_alive_agent_name_list() else prefix[i-1] - 1
            i_max = prefix.index(max(prefix))
            if i_max < 0:
                return
            index_max.append((target, i_max, list_tmp, ))
        
        tuple_max = index_max[0]
        i_max = tuple_max[1]
        
        for term in index_max:
            if term[1] > i_max:
                i_max = term[1]
                tuple_max = term
        
        list_tmp = tuple_max[2]
        target = tuple_max[0]
        state = self.get_pose(target)
        for term in list_tmp:
            name = term[0]
            if name in self.get_self_alive_agent_name_list():
                self.set_plan_by_2Dpoint(name, state[:2])


    #-- Deal with obstacles ---------------------------------------------------
    def calc_obstacles(self, pt1, pt2):
        rect_list = [rect for _, rect in Obstacles.items()]
        expanded_rect = Math.expand_rectangles(rect_list, 1.5)
        obstacles_points = []
        for rect in expanded_rect:
            vertices = [
                [rect[0], rect[2]],
                [rect[1], rect[2]],
                [rect[1], rect[3]],
                [rect[0], rect[3]]
            ]
            for i in range(4):
                if Math.linesintersect([pt1, pt2], [vertices[i], vertices[(i+1)%4]]):
                    obstacles_points.extend(vertices)
                    break
        return obstacles_points

    def update_target_avoiding_obstacle(self):
        for name in self.get_self_alive_agent_name_list():
            state = self.get_pose(name)
            target = self.get_next_target(name)
            points = self.calc_obstacles(state[:2], target[:2])
            if not points:
                return

            points.append(state[:2])
            points.append(target[:2])
            # import matplotlib.pyplot as plt
            # plt.scatter([x[0] for x in points], [x[1] for x in points], color='blue')
            convex_hull = Math.gen_convex_hull(points)
            # plt.plot([x[0] for x in convex_hull] + convex_hull[0][:1], [x[1] for x in convex_hull] + convex_hull[0][-1:], color='red')

            if int(name[-1]) % 2 == 0:
                new_target = Math.choose_points_at_right_hand(state[:2], target, convex_hull)
            else:
                new_target = Math.choose_points_at_left_hand(state[:2], target, convex_hull)

            # plt.scatter(new_target[0], new_target[1], color='green')
            # plt.show()
            
            self.set_plan_by_2Dpoints(name, new_target)
        

    #-- Main operations -------------------------------------------------------
    def Initialize(self):    
        print("Initializing...")  

        for _ in range(150):
            for name in self.get_self_alive_agent_name_list():
                state = self.get_pose(name)
                if 'uav' in name:
                    target = [state[0], state[1], 12 + int(name[-1]) * 1, 0]
                    linear_x = self.k_p_xy*(target[0] - state[0])
                    linear_y = self.k_p_xy*(target[1] - state[1])
                    linear_z = self.k_p_z*(target[2] - state[2])
                    # x = linear_x * math.cos(state[5]) + linear_y * math.sin(state[5])
                    # y = - linear_x * math.sin(state[5]) + linear_y * math.cos(state[5])
                    angular_z = self.k_p_yaw*(target[3] - state[5])

                    # cmd = [x, y, linear_z, angular_z]
                    cmd = [linear_x, linear_y, linear_z, angular_z]
                    cmd = self.uav_vel_limit(cmd)
                    self.set_move_cmd(name, cmd)
            self.move_cmd_send()
            time.sleep(1.0/30)
        
        self.time = {}
        for name in self.get_self_alive_agent_name_list():
            self.init_pose[name] = self.get_pose(name)
            self.time[name] = time.time()

        print("Finished initialization")

    def deal_attack(self, name):
        attack_list = []

        if not self.get_enemy_alive_agent_name_list():
            if Math.points_dist(self.STATE_ENEMY_BASESTATION, self.get_pose(name)[:3]) <= self.attack_radius:
                self.attack_cmd_send(name, self.STATE_ENEMY_BASESTATION[:3])
            return

        for enemy in self.get_enemy_alive_agent_name_list():
            attack_list.append((enemy, self.dist2agents2D(name, enemy), ))
        if not attack_list:
            return

        attack_list = sorted(attack_list, key=lambda x: x[1])
        if attack_list[0][1] <= self.attack_radius:
            target = attack_list[0][0]
        else:
            return

        target_state = self.get_pose(target)
        self.attack_cmd_send(name, target_state)

    def deal_move(self):
        for name in self.get_self_alive_agent_name_list():
            target = self.get_next_target(name)
            state = self.get_pose(name)
            if self.get_target_num(name) == 1:
                if Math.points_dist(state[:2], target[:2]) <= self.arrive_radius:
                    continue
            if 'uav' in name:
                linear_x = self.k_p_xy*(target[0] - state[0])
                linear_y = self.k_p_xy*(target[1] - state[1])
                linear_z = self.k_p_z*(target[2] - state[2])
                angular_z = self.k_p_yaw*(target[3] - state[5])

                cmd = [linear_x, linear_y, linear_z, angular_z]
                cmd = self.uav_vel_limit(cmd)
                self.set_move_cmd(name, cmd)
            elif 'car' in name:
                err_x = target[0] - state[0]
                err_y = target[1] - state[1]
                err_dist = math.sqrt(err_x**2 + err_y**2)
                heading_angle = math.atan2(target[1] - state[1], target[0] - state[0])
                err_angle = heading_angle - state[5]
                if math.fabs(err_angle) < 0.1:
                    linear_x = self.k_p_xy * err_dist
                    linear_y = 0
                    linear_z = 0
                    angular_z = 0
                else:
                    linear_x = 0
                    linear_y = 0
                    linear_z = 0
                    angular_z = self.k_p_yaw * err_angle
                cmd = [linear_x, linear_y, linear_z, angular_z]
                # print("cmd: " + str(cmd))
                cmd = self.car_vel_limit(cmd)
                self.set_move_cmd(name, cmd)

    def _process(self):
        self.STATE_SELF_BASESTATION = self.get_pose(self.get_self_basestation_name())
        self.STATE_ENEMY_BASESTATION = self.get_pose(self.get_enemy_basestation_name())

        for name in self.get_self_agent_name_list():
            self.plan_points_dict[name] = []
        # print("1: " + str(self.plan_points_dict['carB2']))

        if self.no_enemy_alive():
            self.select_basestation_as_target()
            # print("2: " + str(self.plan_points_dict['carB2']))
        elif self.is_emergency():
            self.select_target_from_emergency_zone()
            # print("3: " + str(self.plan_points_dict['carB2']))
        else:
            empty_num = self.no_plan_number()
            while empty_num > 0:
                self.select_target_from_anywhere()
                empty_num_new = self.no_plan_number()
                if empty_num_new == empty_num:
                    break
                elif empty_num_new > empty_num:
                    print("Error empty_num: %d empty_num_new: %d"%(empty_num, empty_num_new))
                    break
                else:
                    empty_num = empty_num_new
            for name in self.get_self_alive_agent_name_list():
                if not self.plan_points_dict[name]:
                    state = self.get_pose(name)
                    self.plan_points_dict[name] = [
                        state[:3] + [0]
                    ]
            # print("4: " + str(self.plan_points_dict['carB2']))
        self.update_target_avoiding_obstacle()
        # print("5: " + str(self.plan_points_dict['carB2']))

        self_alive_name_list = self.get_self_alive_agent_name_list()
        mod = 3 #len(self_alive_name_list)
        for i in range(len(self_alive_name_list)):
            name = self_alive_name_list[i]
            t = time.time()
            if int(t) % mod == i % mod and t - self.time[name] > 3:
                self.deal_attack(name)
                self.time[name] = t

        self.deal_move()
        self.move_cmd_send()

    def process(self):
        try:
            self._process()
        except:
            traceback.print_exc()

    # utils
    def dist2agents2D(self, name1, name2):
        state1 = self.get_pose(name1)
        state2 = self.get_pose(name2)
        return Math.points_dist(state1[:2], state2[:2])

    def dist2basestation2D(self, name):
        state = self.get_pose(name)
        dx = state[0] - self.STATE_SELF_BASESTATION[0]
        dy = state[1] - self.STATE_SELF_BASESTATION[1]
        return math.sqrt(dx**2 + dy**2)

    def get_self_alive_agent_name_list(self):
        list_tmp = []
        for name in self.get_self_agent_name_list():
            if self.is_alive(name):
                list_tmp.append(name)
        return list_tmp

    def get_enemy_alive_agent_name_list(self):
        list_tmp = []
        for name in self.get_enemy_agent_name_list():
            if self.is_alive(name):
                list_tmp.append(name)
        return list_tmp

    def no_plan_number(self):
        ct = 0
        for _, plan in self.plan_points_dict.items():
            if len(plan) == 0:
                ct += 1
        return ct

    def uav_vel_limit(self, cmd):
        if math.fabs(cmd[3]) > self.v_yaw_uav:
            cmd[3] = cmd[3] / math.fabs(cmd[3]) * self.v_yaw_uav

        if math.fabs(cmd[2]) > self.v_max_z_uav:
            cmd[2] = cmd[2] / math.fabs(cmd[2]) * self.v_max_z_uav

        v_mod = math.sqrt(cmd[0]**2 + cmd[1]**2)
        if v_mod == 0:
            return cmd
        
        if v_mod > self.v_max_xy_uav:
            for i in range(2):
                cmd[i] = cmd[i] / v_mod * self.v_max_xy_uav
        elif v_mod < self.v_min_xy_uav:
            for i in range(2):
                cmd[i] = cmd[i] / v_mod * self.v_min_xy_uav

        return cmd

    def car_vel_limit(self, cmd):
        if math.fabs(cmd[3]) > self.v_yaw_car:
            cmd[3] = cmd[3] / math.fabs(cmd[3]) * self.v_yaw_car

        v_mod = math.sqrt(cmd[0]**2 + cmd[1]**2)
        if v_mod == 0:
            return cmd
        
        if v_mod > self.v_max_xy_car:
            for i in range(2):
                cmd[i] = cmd[i] / v_mod * self.v_max_xy_car
        elif v_mod < self.v_min_xy_car:
            for i in range(2):
                cmd[i] = cmd[i] / v_mod * self.v_min_xy_car

        return cmd

    def get_init_pose(self, name):
        return self.init_pose[name]
