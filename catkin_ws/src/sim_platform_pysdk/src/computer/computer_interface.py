#!/usr/bin/env python
# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import time
import math

import rospy

from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import String 
from gazebo_msgs.msg import ModelStates
from judge_system_msgs.msg import Alive_models
from judge_system_msgs.msg import Move_cmd
from judge_system_msgs.msg import Attack_cmd


class ComputerInterface(object):
    _instance = None
    def __new__(cls, *args, **kwargs):
        if not ComputerInterface._instance:
            ComputerInterface._instance = super(ComputerInterface, cls).__new__(cls, *args, **kwargs)
        return ComputerInterface._instance

    def __init__(self, team_name):
        rospy.init_node("computer")
        self.__rate = rospy.Rate(30)

        self.__set_team_name(team_name)
        self.__name_list_a = []
        self.__name_list_b = []

        self.__pose_all_a = ModelStates()
        self.__pose_all_b = ModelStates()
        self.__pose_all_a_dict = {}
        self.__pose_all_b_dict = {}

        self.__alive_models_a = Alive_models()
        self.__alive_models_b = Alive_models()
        self.__alive_models_a_dict = {}
        self.__alive_models_b_dict = {}

        self.__move_cmd = Move_cmd()
        self.__move_cmd_dict = {}

        self.__attack_cmd = Attack_cmd()

        self.__sub_pose_a = rospy.Subscriber('/A/car_uav_pose', ModelStates, self.__callback_pose_a)
        self.__sub_aliv_model_a = rospy.Subscriber('/A/alive_model', Alive_models, self.__callback_alive_a)
        self.__sub_pose_b = rospy.Subscriber('/B/car_uav_pose', ModelStates, self.__callback_pose_b)
        self.__sub_aliv_model_b = rospy.Subscriber('/B/alive_model', Alive_models, self.__callback_alive_b)

        self.__pub_move_cmd = rospy.Publisher('/%s/move_cmd'%self.__team_name, Move_cmd, queue_size=10)
        self.__pub_attack_cmd = rospy.Publisher('/%s/attack_cmd'%self.__team_name, Attack_cmd, queue_size=10)

        while not self.__name_list_a or not self.__name_list_b:
            rospy.loginfo("Waiting for connecting to ros...")
            time.sleep(0.1)
        rospy.loginfo("Successfully connected to ros !")


    def __set_team_name(self, team_name):
        if team_name != 'A' and team_name != 'B':
            raise NameError(team_name)
        self.__team_name = team_name

    # Private method ----------------------------------------------------------
    def __callback_pose_a(self, msg):
        self.__pose_all_a = msg
        self.__name_list_a = msg.name
        for i in range(0, len(msg.name)):
            pose_tmp = []
            pose_tmp.append(msg.pose[i].position.x)
            pose_tmp.append(msg.pose[i].position.y)
            pose_tmp.append(msg.pose[i].position.z)
            atti = [0,0,0]
            self.__tf_quaternion2euler(msg.pose[i].orientation.x, 
                msg.pose[i].orientation.y, 
                msg.pose[i].orientation.z, 
                msg.pose[i].orientation.w, atti)
            pose_tmp = pose_tmp + atti
            pose_tmp.append(msg.twist[i].linear.x)
            pose_tmp.append(msg.twist[i].linear.y)
            pose_tmp.append(msg.twist[i].linear.z)
            self.__pose_all_a_dict[msg.name[i]] = pose_tmp

    def __callback_pose_b(self, msg):
        self.__pose_all_b = msg
        self.__name_list_b = msg.name
        for i in range(0, len(msg.name)):
            pose_tmp = []
            pose_tmp.append(msg.pose[i].position.x)
            pose_tmp.append(msg.pose[i].position.y)
            pose_tmp.append(msg.pose[i].position.z)
            atti = [0,0,0]
            self.__tf_quaternion2euler(msg.pose[i].orientation.x, 
                msg.pose[i].orientation.y, 
                msg.pose[i].orientation.z, 
                msg.pose[i].orientation.w, atti)
            pose_tmp = pose_tmp + atti
            pose_tmp.append(msg.twist[i].linear.x)
            pose_tmp.append(msg.twist[i].linear.y)
            pose_tmp.append(msg.twist[i].linear.z)
            self.__pose_all_b_dict[msg.name[i]] = pose_tmp

    def __callback_alive_a(self, msg):
        self.__alive_models_a_dict = {}
        self.__alive_models_a = msg
        for i in range(len(msg.body)):
            name = msg.body[i].name.data
            self.__alive_models_a_dict[name] = msg.body[i]

    def __callback_alive_b(self, msg):
        self.__alive_models_b_dict = {}
        self.__alive_models_b = msg
        for i in range(len(msg.body)):
            name = msg.body[i].name.data
            self.__alive_models_b_dict[name] = msg.body[i]


    # Send messages -----------------------------------------------------------
    # Send move cmd
    def set_move_cmd(self, name, cmd):
        self.__move_cmd_dict[name] = Twist()
        self.__move_cmd_dict[name].linear.x = cmd[0]
        self.__move_cmd_dict[name].linear.y = cmd[1]
        self.__move_cmd_dict[name].linear.z = cmd[2]
        self.__move_cmd_dict[name].angular.z = cmd[3]

    def __TfCommandDcit2Msg(self):
        self.__move_cmd = Move_cmd()
        if self.__team_name == 'A':
            self.__move_cmd.header.frame_id = "move_cmd_a"
        elif self.__team_name == 'B':
            self.__move_cmd.header.frame_id = "move_cmd_b"
        self.__move_cmd.header.stamp = rospy.Time.now()
        self.__move_cmd.team.data = self.__team_name
        for (name, cmd) in self.__move_cmd_dict.items():
            name_tmp = String()
            name_tmp.data = name
            self.__move_cmd.name.append(name_tmp)
            self.__move_cmd.cmd.append(cmd)

    def move_cmd_send(self):
        self.__TfCommandDcit2Msg()
        self.__pub_move_cmd.publish(self.__move_cmd)

    # Send attack cmd
    def __gen_attack_cmd(self, name, target):
        pose = self.get_pose(name)[:3]

        msg_attack = Attack_cmd()
        msg_attack.name.data = name
        msg_attack.team.data = self.__team_name
        msg_attack.start.x = pose[0]
        msg_attack.start.y = pose[1]
        msg_attack.start.z = pose[2]
        msg_attack.target.x = target[0]
        msg_attack.target.y = target[1]
        msg_attack.target.z = target[2]
        return msg_attack

    def attack_cmd_send(self, name, target):
        attack_cmd = self.__gen_attack_cmd(name, target)
        self.__pub_attack_cmd.publish(attack_cmd)


    # Get messages ---------------------------------------------------------
    # Get blood information
    def get_blood(self, name):
        self.__check_name(name)
        if 'A' in name:
            if name in self.__alive_models_a_dict.keys():
                return self.__alive_models_a_dict[name].blood
            else:
                # print("%s is not alive."%name)
                return 0
        elif 'B' in name:
            if name in self.__alive_models_b_dict.keys():
                return self.__alive_models_b_dict[name].blood
            else:
                # print("%s is not alive."%name)
                return 0

    # Get bullet information
    def get_remain_bullet(self, name):
        self.__check_name(name)
        if 'A' in name:
            if name in self.__alive_models_a_dict.keys():
                return self.__alive_models_a_dict[name].bullet
            else:
                print("%s is not alive."%name)
                return None
        if 'B' in name:
            if name in self.__alive_models_b_dict.keys():
                return self.__alive_models_b_dict[name].bullet
            else:
                print("%s is not alive."%name)
                return None

    # Get alive information
    def is_alive(self, name):
        if (name in self.__alive_models_a_dict.keys()) or (name in self.__alive_models_b_dict.keys()):
            return True
        else:
            return False
			
    # Get pose information
    def get_pose(self, name):
        self.__check_name(name)
        if 'A' in name:
            return self.__pose_all_a_dict[name]
        elif 'B' in name:
            return self.__pose_all_b_dict[name]

    def get_self_basestation_name(self):
        self_name_list = self.__name_list_a if self.__team_name == 'A' else self.__name_list_b
        for name in self_name_list:
            if ('uav' not in name) and ('car' not in name):
                return name
        return None

    def get_enemy_basestation_name(self):
        enemy_name_list = self.__name_list_b if self.__team_name == 'A' else self.__name_list_a
        for name in enemy_name_list:
            if ('uav' not in name) and ('car' not in name):
                return name
        return None

    # Get self name list
    def get_self_agent_name_list(self):
        list_tmp = []
        self_name_list = self.__name_list_a if self.__team_name == 'A' else self.__name_list_b
        for name in self_name_list:
            if ('uav' in name) or ('car' in name):
                list_tmp.append(name)
        return list_tmp

    # Get enemy name list
    def get_enemy_agent_name_list(self):
        list_tmp = []
        enemy_name_list = self.__name_list_b if self.__team_name == 'A' else self.__name_list_a
        for name in enemy_name_list:
            if ('uav' in name) or ('car' in name):
                list_tmp.append(name)
        return list_tmp


    # utils ----------------------------------------------------------------
    def __check_name(self, name):
        b1 = (name not in self.get_self_agent_name_list())
        b2 = (name not in self.get_enemy_agent_name_list())
        b3 = (name not in self.get_self_basestation_name())
        b4 = (name not in self.get_enemy_basestation_name())
        if b1 and b2 and b3 and b4 :
            raise Exception("Error name", name)

    def __tf_quaternion2euler(self, x, y, z, w, attitude):
        aSinInput = -2*(x*z-w*y)
        if aSinInput > 1:
            aSinInput = 1
        elif aSinInput < -1:
            aSinInput = -1
        attitude[2] = math.atan2(2.0*(x*y+w*z), w*w + x*x - y*y - z*z )
        attitude[1] = math.asin(aSinInput)
        attitude[0] = math.atan2(2.0*(y*z+w*x), w*w - x*x - y*y + z*z )


    # main --------------------------------------------------------------------
    def process(self):
        raise NotImplementedError

    def run(self):
        while not rospy.is_shutdown():
            self.process()
            self.__rate.sleep()