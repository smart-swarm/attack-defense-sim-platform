
# 1. 配置环境
在 3_methods_to_configure_environment 文件夹中选取任意一种方式进行环境配置。

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

## 3.1 打开终端，输入
```
bash start_platform_6vs6.sh
```
直到屏幕输出"simulation platform ready..." .
## 3.2 新开一终端, 运行时样例代码
```
roslaunch sim_platform_pysdk start_computer_and_your_code.launch
```

# 4. 问题咨询

在安装过程中如遇到其他问题，欢迎咨询技术人员，咨询时请备注“赛事技术咨询”。

- 	咨询邮箱：\url{smartswarm@qq.com}
-	技术交流QQ群：\url{278935440}


