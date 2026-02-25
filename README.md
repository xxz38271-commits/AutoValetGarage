# Auto Valet Garage 🚗
**TurtleBot4 기반 AMR 무인 자동 주차 관제 시스템**

> ROKEY 부트캠프 E-1조, F-2조 협업 프로젝트 (2024)

---

## 프로젝트 소개

도심 주차난과 이중주차 문제를 해결하기 위해 TurtleBot4 2대(robot1, robot5)가 협동하여 차량의 입차부터 출차까지 전 과정을 자동화하는 무인 주차 관제 시스템입니다.

YOLOv8으로 진입 차량의 종류를 인식하고, 미션 기반 아키텍처로 각 로봇에 작업을 분배합니다. 이중주차 상황에서는 FSM(유한 상태 머신)으로 4단계 협동 제어 시퀀스를 수행합니다.

---

## 주요 기능

**자율 입/출차 (TurtleBot4 × 2)**
- AMCL 기반 실시간 위치 추정 + Nav2로 웨이포인트 경유 자율 주행
- `ALIGN_BASE_FRONT` / `ALIGN_BASE_EXIT` 미션: OAK-D 카메라로 바닥 흰 선 인식 후 정밀 정렬
  - ROI 하단 1/3 영역 → HSV 필터링 → Contour 검출 → PID 기반 조향 제어
- Dock / Undock 액션 자동 처리, 미션 완료 후 60초 idle 시 자동 도킹

**이중주차 협동 제어 (4-Phase FSM)**
- 출차 요청 수신 시 SINGLE / DOUBLE 타입 자동 판별
- DOUBLE 출차 시 4단계 FSM으로 2대 로봇 시퀀스 제어:
  - Phase 1: robot1이 앞 차량을 buffer 구역으로 이동, robot5는 대기 위치로 이동
  - Phase 2: robot5가 안쪽 차량을 출차 구역으로 이송
  - Phase 3: robot1이 buffer 차량을 원위치로 복구
  - Phase 4: robot5가 대기 위치로 복귀
- 각 Phase 완료는 `mission_status` 토픽으로 동기화

---

## 시스템 구조

```
WebCam (입구)
    ↓
enter_process.py ─ YOLOv8 차종 인식 → Supabase Task 생성
    ↓
parking_manage_realtime.py ─ Task 감지 → /task_command/robot1,5 발행
    ↓                              ↓
robot_enter_node.py         convert_exit_task.py
(robot1 입차 실행)           (출차 Task 변환)
    ↓                              ↓
          WiFi 6 (FastDDS)
    ↓                              ↓
AMR1 (robot1)               AMR2 (robot5)
mission_executor.py         mission_executor.py
Nav2 + AMCL                 Nav2 + AMCL
Line Align (HSV)            Line Align (HSV)
    ↑                              ↑
mission_manager.py ─ 4-Phase FSM (DOUBLE 출차 협동 제어)
task_allocator.py  ─ raw_missions → robot1/robot5 분배
```

---

## 담당 역할

**전형준 - AMR 자율 주행 제어 및 협동 제어 시스템 구현**

**`parking_executor/mission_executor.py`** — AMR 온보드 실행 노드 (주담당)
- Nav2 `NavigateToPose` 액션 클라이언트 기반 자율 주행 구현
- 미션 타입별 분기: `MOVE_FRONT_CAR`, `EXIT_TARGET_CAR`, `ALIGN_BASE_*`, `BEEP_*`
- `ALIGN_BASE_FRONT` / `ALIGN_BASE_EXIT`: 카메라 기반 라인 트레이싱 정렬
  - ROI 하단 1/3 → HSV 흰색 필터 → Contour 검출 → `cmd_vel` PID 조향 제어
  - 라인 미검출 4초 타임아웃으로 안정성 확보
- Dock / Undock 비동기 액션 처리 (`is_docked` 실시간 동기화)
- 60초 idle 시 자동 도킹, robot5 `initialpose` 자동 설정
- `AudioNoteVector` 기반 픽업 완료 삐뽀 사운드 구현

**`parking_system/mission_manager.py`** — 협동 제어 FSM (주담당)
- SINGLE / DOUBLE 출차 판별 및 4-Phase 시퀀스 자동 생성
- `mission_status` 토픽으로 Phase 완료 감지 → 다음 Phase 트리거
- CSV 기반 블록 좌표 맵 로드, 주차 슬롯 ID → 실제 좌표 변환

**`final_project/robot_enter_node.py`** — robot1 입차 실행 (일부 기여)
- TurtleBot4Navigator 기반 입차 시퀀스 지원

---

## 기술 스택

![ROS2](https://img.shields.io/badge/ROS2_Humble-22314E?style=flat&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu_22.04-E95420?style=flat&logo=ubuntu&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=flat&logo=opencv&logoColor=white)

- **플랫폼**: TurtleBot4 (iRobot Create 3 + Raspberry Pi 4B)
- **자율 주행**: ROS2 Humble, Nav2 (AMCL + DWB Controller), SLAM Toolbox
- **비전**: YOLOv8n (차종 분류), OpenCV (HSV 라인 트레이싱)
- **센서**: RPLIDAR A1M8, OAK-D Pro, RPLIDAR A1M8
- **통신**: FastDDS, WiFi 6
- **DB/백엔드**: Supabase (PostgreSQL), React 웹 대시보드

---

## 패키지 구조

```
프로젝트 전체
├── final_project/               # PC 서버 측 (팀 공동)
│   ├── enter_process.py         # YOLOv8 입차 인식 + Task 생성
│   ├── robot_enter_node.py      # robot1 입차 실행
│   ├── parking_manage_realtime.py  # 중앙 Task 관리자
│   ├── convert_exit_task.py     # 출차 Task 변환
│   ├── robot1_status.py         # robot1 상태 모니터링
│   └── robot2_status.py         # robot5 상태 모니터링
│
└── parking_ws/src/              # AMR 온보드 (주담당)
    ├── parking_executor/
    │   └── mission_executor.py  # ★ AMR 주행/정렬/도킹 제어
    ├── parking_system/
    │   ├── mission_manager.py   # ★ 협동 제어 FSM (4-Phase)
    │   ├── task_allocator.py    # 미션 → robot1/robot5 분배
    │   └── parking_slot_manager.py
    └── parking_msgs/            # 커스텀 메시지 정의
```

---

## 팀 구성

| 이름 | 역할 |
|------|------|
| **전형준** | AMR 자율 주행 제어 (`mission_executor`), 이중주차 협동 제어 FSM (`mission_manager`), 입차 실행 일부 기여 |
| 팀원 2 | 입차 관제 서버 (YOLOv8, 주차 배정) |
| 팀원 3 | 출차 관제 서버, DB 설계 (Supabase) |
| 팀원 4 | 웹 대시보드, 결제 시스템 |
| 팀원 5~8 | F-2조 협업 |
