## 객체지향모델링 합반 2조 ROS 기반의 터틀봇 자율주행

## 목차

1. [배경](#1-배경)
2. [개발 환경](#2-개발-환경)
3. [사용법](#3-사용법)
4. [팀원](#4-팀원)

## 1. 배경

&nbsp;자율주행은 '인간이 운전하는 것보다 안전한 운전'을 목표로 하는 기술로, 차량 사고를 줄여주어 운행의 안정성을 높여준다. 또한, 전동화를 통해 차의 부품 수가 줄어들어 고장률도 줄어들고, 호텔 산업, 승차 공유 산업, 항공 산업, 부동산 산업 등 우리 생활 전반에 영향을 미칠 것이다.

&nbsp; 실제 차량을 이용한 자율 주행 테스트는 위험성이 높고 비용적인 부담이 되기 때문에 본 프로젝트는 ROS 프레임워크를 사용하여 터틀봇을 사용한 자율 주행 환경을 제공한다.

## 2. 개발 환경

```
* OS : Ubuntu 18.04 LTS
* Programming Language : Python2.7
* IDE : Pycharm 2.7.17
* Framework : Ros melodic
```

## 3. 사용법

터틀봇에 좌, 우 카메라 추가  
[kobuki.urdf.xacro](https://github.com/ads0070/deu_car/blob/master/kobuki.urdf.xacro) 참고
```
$ cd ~/catkin_ws/src/kobuki_description/urdf
$ gedit kobuki.urdf.xacro
$ cd ~/catkin_ws
$ catkin_make
```

launch 파일 및 car_state_machine.py 실행

```
$ roscd deu_car
$ source ./gazebo_env.sh
$ chmod +x ./scripts/blocking_bar_control.sh
$ chmod +x ./scripts/obstacle_spawn.py
$ roslaunch deu_car car_test.launch
$ rosrun deu_car car_state_machine.py
```

## 4. 팀원
- 20173217 안대현
- 20194152 허세진
- 20173176 박진우
- 20153294 김두영


<a href="https://github.com/ads0070" title="20173217 안대현">
<img src="https://avatars.githubusercontent.com/u/73926856?v=4" height="50" alt="안대현"/></a>

<a href="https://github.com/Heosejin98" title="20194152 허세진">
<img src="https://avatars.githubusercontent.com/u/61305083?v=4" height="50" alt="허세진"/></a>

<a href="https://github.com/014787410"  title="20173176 박진우">
<img src="https://avatars.githubusercontent.com/u/93768331?v=4" height="50" alt="박진우"/></a>
