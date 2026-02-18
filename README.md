# ros2
All personal codings related ROS2

## Lidar
T-mini plus TOF Lidar
Simple main program for understanding ..
Simple ROS2 node

# YDLidar T-mini ROS2 Driver (Navigation2 & micro-ROS 호환)

본 패키지는 YDLidar T-mini 센서를 ROS2 환경에서 사용하기 위한 C++ 기반 드라이버입니다. 특히 자율주행(Navigation2)과의 완벽한 호환성과 향후 micro-ROS로의 포팅을 염두에 둔 데이터 정렬 로직이 적용되어 있습니다.

## 🚀 주요 특징

* **Navigation2 최적화**: 배열의 중간(`index 180`)이 로봇의 정면(0도)이 되도록 데이터를 $-\pi \sim \pi$ 범위로 자동 정렬합니다.
* **상하좌우 반전 교정**: 하드웨어 장착 방향에 따른 데이터 반전 문제를 드라이버 레벨에서 `180.0 - current_angle` 수식으로 완벽히 해결했습니다.
* **고속 데이터 처리**: 1ms 주기의 타이머를 통해 시리얼 데이터를 수신하며, 360도 한 바퀴 데이터가 완성된 시점에만 `LaserScan` 메시지를 발행하여 네트워크 부하를 줄이고 데이터 일관성을 확보했습니다.
* **micro-ROS 포팅 고려**: 임베디드 환경(STM32, ESP32 등)에서 재사용 가능한 정적 배열 구조와 상태 머신(State Machine) 기반 패킷 해석 로직을 사용합니다.

## 🛠 시스템 요구 사항

* **OS**: Ubuntu 22.04 LTS (또는 ROS2 지원 OS)
* **ROS2 버전**: Humble / Iron / Jazzy
* **하드웨어**: YDLidar T-mini, USB to Serial 변환기 (Baudrate: 230400)

## 📂 설치 및 빌드

```bash
# 워크스페이스 이동
cd ~/myproj/gemini

# 빌드 실행
colcon build --symlink-install --packages-select tmini_lidar_pkg

# 환경 설정 반영
source install/setup.bash

# run
ros2 run tmini_lidar_pkg tmini_node