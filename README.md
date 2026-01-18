#WEGO-MORAI 자율주행 경진대회 프로젝트
**MORAI 시뮬레이터 환경에서 Lidar 센서를 활용한 장애물 회피 및 로터리 주행 알고리즘 구현**

## 1. 프로젝트 개요
본 프로젝트는 WEGO 대회에서 제공된 MORAI 가상 환경 내에서 자율주행 미션을 수행하는 것을 목표로 합니다. 카메라를 통한 차선 인식 주행뿐만 아니라, Lidar 센서를 활용한 고난도 장애물 회피 및 교차로 주행 로직을 포함하고 있습니다.

## 2. 주요 담당 역할 (Lidar Module)
저는 이 프로젝트에서 **Lidar 센서 데이터 처리 및 판단 로직**을 담당하였습니다.

동적/정적 장애물 회피 (Obstacle Avoidance)
    * Lidar의 PointCloud 데이터를 분석하여 전방 및 측면의 장애물을 실시간 감지.
    * 장애물의 거리와 각도에 따른 주행 경로 보정 알고리즘 설계.
로터리(회전 교차로) 구간 주행 (Roundabout Navigation)
    * 로터리 진입 시 주변 차량 및 구조물을 Lidar로 탐지하여 진입 타이밍 판단.
    * 복잡한 곡률의 로터리 구간 내에서 안정적인 주행 유지.
센서 데이터 필터링
    * 노이즈 제거 및 효율적인 데이터 처리를 위한 ROI(Region of Interest) 설정 및 필터링 구현.

## 3. 개발 환경
* OS: Ubuntu 20.04 LTS
* Platform: ROS Noetic 
* Simulator: MORAI Simulator
* Language: Python 
