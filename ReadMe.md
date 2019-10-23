# 1. clone this reposity
```
https://github.com/smart-swarm/attack-defense-sim-platform.git
cd ./attack-defense-sim-platform
```
# 2. install the judge system (very important!)
```
cd ./install_judge_system
sh install.sh # or ./install.sh
cd -
```
# 3. compile all the packages
```
cd ./catkin_ws
catkin_make
```
# 4. running the simulation platform
```
bash start_platform_6vs6.sh
```
wait for the terminal print "simulation platform ready..." .
# 5. open a new terminal, running the example code.
```
roslaunch sim_platform_pysdk start_computer_and_your_code.launch
```

#* What you need is to replace the example code of sim_platform_pysdk with your code.*

#* Welcome to discuss in the ISSUE zone. or send email to smartswarm@qq.com.*


