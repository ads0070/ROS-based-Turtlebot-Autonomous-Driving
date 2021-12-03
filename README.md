## 객체지향모델링 합반 2조 ROS 기반의 터틀봇 자율주행

## 목차

- [비전](#비전)
- [개발 환경](#개발환경)
- [사용법](#사용법)
- [팀원](#팀원)

## 1. 비전

본 프로젝트에 사용하는 자율주행 기술은 '인간이 운전하는 것보다 안전한 운전'으로 차량 사고를 줄여주어 운행의 안정성을 높여준다. 또한, 전동화를 통해 차의 부품 수가 줄어들어 고장률도 줄어들고, 호텔 산업, 승차 공유 산업, 항공 산업, 부동산 산업 등 우리 생활 전반에 영향을 미칠 것이다.

또한, 쉽게 배울 수 없는 ROS라는 로봇 소프트웨어를 제작하기 위한 프레임워클르 활용해 봄으로써 로봇 소프트웨어에 익숙해지는 계기가 될 것이다.

## 2. 개발 환경

```
* OS : Ubuntu 18.04 LTS
* Programming Language : Python2.7
* IDE : Pycharm 2.7.17
* Framework : Ros melodic
```

## 3. 사용법

```
$ roscd deu_car
$ source ./gazebo_env.sh
$ chmod +x ./scripts/blocking_bar_control.sh
$ chmod +x ./scripts/obstacle_spawn.py
$ roslaunch deu_car car_test.launch
$ rosrun deu_car car_state_machine.py
```

## 4. 팀원
This project exists thanks to all the people who contribute.

<a href="https://github.com/ads0070">
<img src="https://avatars.githubusercontent.com/u/73926856?v=4" height="50" alt="안대현"/></a>

<a href="https://github.com/Heosejin98">
<img src="https://avatars.githubusercontent.com/u/61305083?v=4" height="50" alt="허세진"/></a>

<a href="https://github.com/014787410">
<img src="https://avatars.githubusercontent.com/u/93768331?v=4" height="50" alt="박진우"/></a>
