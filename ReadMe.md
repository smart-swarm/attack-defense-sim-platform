# *已经下载过，需要更新代码*
```
cd attack-defense-sim-platform
git pull
```
# *已经安装过裁判系统，需要更新裁判系统*
```
https://github.com/smart-swarm/update_judge_system
```
# 1. 配置环境
在 doc文件夹中 3_methods_to_configure_environment 文件夹中选取任意一种方式进行环境配置。
```
doc/主办方提供的SDK的接口.pdf  

doc/直接安装系统镜像手册.pdf  

doc/分布配置运行环境手册.pdf
```
# 2. 更新比赛平台程序

## 2.1 复制整个目录
```
git clone https://github.com/smart-swarm/attack-defense-sim-platform.git
cd ./attack-defense-sim-platform
```
## 2.2 安装裁判系统（judge system），很重要！！！
```
cd ./install_judge_system
chmod +x install.sh
./install.sh # or sh install.sh
cd -
```
## 2.3. 编译工作空间
```
cd ./catkin_ws
catkin_make
source devel/setup.bash
```

# 3. 运行比赛平台
# *如果你对执行命令不熟悉，可以只用步骤5中的GUI鼠标操作替代*

## 3.1 打开终端，输入
```
bash start_platform_6vs6.sh
```
直到屏幕输出"simulation platform ready..." ，方可进行下一步.
## 3.2 新开一终端, 运行时样例代码
```
./battle.sh
```
# 4 一局比赛结束,可不用关闭界面，再次运行
## 4.1 打开终端，输入
```
./reset.sh
```
等待小地图窗口打开即可。
## 4.2 新开一终端, 运行时样例代码
```
./battle.sh
```

# 5. 使用GUI执行以上命令
```
chmod +x use_gui.sh
./use_gui.sh
```


# 6. 问题咨询

在安装过程中如遇到其他问题，欢迎咨询技术人员，咨询时请备注“空地协同对抗赛-技术咨询”。

- 	咨询邮箱：smartswarm@qq.com
-	技术交流QQ群：278935440


# 1. clone this reposity
```
git clone https://github.com/smart-swarm/attack-defense-sim-platform.git
cd ./attack-defense-sim-platform
```
# 2. install the judge system (very important!)
```
cd ./install_judge_system
chmod +x install.sh
./install.sh # or sh install.sh
cd -
```
# 3. compile all the packages
```
cd ./catkin_ws
catkin_make
source devel/setup.bash
```
# 4. running the simulation platform
```
bash start_platform_6vs6.sh
```
wait for the terminal print "simulation platform ready..." .
# 5. open a new terminal, running the example code.
```
./battle.sh
```
# 6. keep gazebo running, and battle again.
## 6.1 open a terminal
```
./reset.sh
```
waiting for the mini map show.。
## 4.2 open a new terminal
```
./battle.sh
```

# **What you need** *is to replace the example code of sim_platform_pysdk with your code.*

# Welcome to discuss in the ISSUES zone, feel free to ask or send email to *smartswarm@qq.com.*

