#!/usr/bin/env python3
import os
import math
import csv
import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from parking_msgs.msg import MissionArray, Mission, MissionStatus
from ament_index_python.packages import get_package_share_directory


class MissionManager(Node):
    """
    MissionManager (DOUBLE: Phase1~4 전체 구현)

    - /parking/exit_type (std_msgs/String, JSON payload) 를 구독해서
      exit_type(SINGLE/DOUBLE), parking_spot(A_1_1 등)을 파싱
    - SINGLE: 단순 출차 (robot5만 이동)
    - DOUBLE: 전체 Phase 수행
        * 예시 (A_1_1 안쪽차 기준, 컬럼 메타에 따라 일반화 가능)
          - Phase1:
              robot1: W_r1 -> base_slot(A_1) -> front_slot(A_1_2) -> buffer_slot(A_3)
              robot5: W_r5 -> side_slot(A_2)
          - Phase2:
              robot5: side_slot(A_2) -> base_slot(A_1) -> inner_slot(A_1_1) -> EXIT
          - Phase3:
              robot1: buffer_slot(A_3) -> front_slot(A_1_2) -> W_r1
          - Phase4:
              robot5: EXIT -> W_r5
    - 생성된 미션들을 MissionArray 로 raw_missions 토픽에 퍼블리시
    """

    def __init__(self):
        super().__init__('mission_manager')

        # 미션 ID 카운터
        self.mission_id_counter = 1

        # 1) 블럭 좌표/각도 CSV 로드 (name -> x,y,yaw_deg)
        self.block_map = self.load_block_map()

        # 2) 컬럼(A,B,C) 및 슬롯 메타 정보
        self.COLUMN_META = self.create_column_meta()

        # 3) 실제 출차 요청 토픽 구독 (/parking/exit_type)
        self.exit_sub = self.create_subscription(
            String,
            '/task_command/robot5',        # 외부 시스템이 퍼블리시하는 토픽
            self.exit_type_callback,
            10
        )

        # ✅ mission_status 구독 (robot1, robot5)
        self.status_sub_r1 = self.create_subscription(
            MissionStatus,
            '/robot1/mission_status',
            self.mission_status_callback_robot1,
            10
        )
        self.status_sub_r5 = self.create_subscription(
            MissionStatus,
            '/robot5/mission_status',
            self.mission_status_callback_robot5,
            10
        )

        # 4) TaskAllocator 가 구독하는 raw_missions 퍼블리셔
        self.mission_pub = self.create_publisher(
            MissionArray,
            'raw_missions',
            10
        )


        # 이미 처리한 task_id 모음 (중복 방지용)
        self.processed_task_ids = set()

        # ✅ DOUBLE 출차용 FSM 상태 저장
        #  - key: command_id (또는 task_id)
        #  - value: 각 phase 상태 / 마지막 mission_id 등
        self.double_fsm = {}         # { command_id: {...} }
        self.mission_to_task = {}    # { mission_id: command_id }
        # ★ 새로 추가
        self.active_command_id = None  # 지금 실행 중인 command_id (없으면 None)
        from collections import deque
        self.pending_tasks = deque() 


        self.get_logger().info('MissionManager (DOUBLE Phase1~4 + /parking/exit_type) started.')

    # -------------------------------------------------------------
    #  블럭 좌표 CSV 로드
    # -------------------------------------------------------------
    def load_block_map(self):
        block_map = {}

        try:
            # parking_system 패키지의 share 디렉토리 경로 얻기
            pkg_share = get_package_share_directory('parking_system')
            csv_path = os.path.join(pkg_share, 'config', 'parking_blocks.csv')
        except Exception as e:
            self.get_logger().error(f'Failed to get package share directory: {e}')
            return block_map

        if not os.path.exists(csv_path):
            self.get_logger().warn(
                f'parking_blocks.csv not found: {csv_path}. '
                f'Pose lookup will fail until this file is created.'
            )
            return block_map

        self.get_logger().info(f'Loading block map from: {csv_path}')

        try:
            with open(csv_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    name = row['name'].strip()
                    x = float(row['x'])
                    y = float(row['y'])
                    yaw_deg = float(row['yaw_deg'])

                    block_map[name] = {
                        'x': x,
                        'y': y,
                        'yaw_deg': yaw_deg
                    }
        except Exception as e:
            self.get_logger().error(f'Failed to load block map: {e}')
            return {}

        self.get_logger().info(f'Loaded {len(block_map)} blocks.')
        return block_map

    def start_double_fsm(self, parking_spot: str, command_id: str):
        self.get_logger().info(
            f'[FSM] Start DOUBLE task: cmd_id={command_id}, spot={parking_spot}'
        )

        # 방어용: active_command_id가 비어 있으면 여기서도 설정
        if self.active_command_id is None:
            self.active_command_id = command_id
        elif self.active_command_id != command_id:
            self.get_logger().warn(
                f'[FSM] start_double_fsm called while active_command_id={self.active_command_id}, '
                f'new cmd_id={command_id}. (logic check 필요)'
            )
        # 상태 초기화
        self.double_fsm[command_id] = {
            "parking_spot": parking_spot,
            "state": "PHASE1",

            # ✅ 각 페이즈에서 "골포즈"에 해당하는 mission_id들을 담는 집합
            "phase1_goal_ids": set(),
            "phase2_goal_ids": set(),
            "phase3_goal_ids": set(),
        }

        # Phase1 미션 생성 + publish
        self.start_double_phase1(command_id)

    # -------------------------------------------------------------
    #  컬럼 / 슬롯 메타 정보 (A/B/C)
    # -------------------------------------------------------------
    def create_column_meta(self):
        meta = {
            "A": {
                "exit": "EXIT",   # TODO: A열 출구 블럭 이름

                # 슬롯 번호별 side / buffer 정보
                "slots": {
                    # A_1_1 안쪽차 -> 옆블럭 A_2, 뒤쪽버퍼 A_3
                    1: {"side": "A_2", "buffer": "A_3"},
                    # A_2_1 안쪽차 -> 옆블럭 B_1, 뒤쪽버퍼 A_4 (예시)
                    2: {"side": "B_1", "buffer": "A_4"},
                    3: {"side": "A_4", "buffer": "A_1"},  # TODO: 맵 규칙에 맞게 변경
                    4: {"side": "B_3", "buffer": "A_2"},  # TODO: 맵 규칙에 맞게 변경
                },

                # 슬롯 그룹별 wait 위치 (W_1/2 vs W_3/4)
                "wait_groups": [
                    {
                        "slots": [1, 2],        # A_1, A_2
                        "robot1_wait": "W_1",   # TODO: A_1, A_2에서 robot1 대기 위치
                        "robot5_wait": "W_2",   # TODO: A_1, A_2에서 robot5 대기 위치
                    },
                    {
                        "slots": [3, 4],        # A_3, A_4
                        "robot1_wait": "W_3",   # TODO: A_3, A_4에서 robot1 대기 위치
                        "robot5_wait": "W_4",   # TODO: A_3, A_4에서 robot5 대기 위치
                    },
                ],
            },

            # =======================================
            # B 열 (러프하게 A 열 패턴 복붙, 나중에 수정)
            # =======================================
            "B": {
                "exit": "EXIT",   # TODO: B열 출구 블럭 이름

                "slots": {
                    # B_1_1 안쪽차 -> 옆블럭 B_2, 뒤쪽버퍼 B_3 정도로 가정
                    1: {"side": "B_2", "buffer": "B_3"},
                    2: {"side": "C_1", "buffer": "B_4"},
                    3: {"side": "B_2", "buffer": "B_1"},  # TODO
                    4: {"side": "C_3", "buffer": "B_2"},  # TODO
                },

                "wait_groups": [
                    {
                        "slots": [1, 2],
                        "robot1_wait": "W_1",   # TODO: 실제 대기 블럭으로 수정
                        "robot5_wait": "W_2",
                    },
                    {
                        "slots": [3, 4],
                        "robot1_wait": "W_3",
                        "robot5_wait": "W_4",
                    },
                ],
            },

            # =======================================
            # C 열 (러프하게 작성)
            # =======================================
            "C": {
                "exit": "EXIT",   # TODO: C열 출구 블럭 이름

                "slots": {
                    # C_1_1 안쪽차 -> 옆블럭 C_2, 뒤쪽버퍼 C_3 정도로 가정
                    1: {"side": "C_2", "buffer": "C_3"},
                    2: {"side": "W_2", "buffer": "C_4"},
                    3: {"side": "C_2", "buffer": "C_1"},  # TODO
                    4: {"side": "W_4", "buffer": "C_2"},  # TODO
                },

                "wait_groups": [
                    {
                        "slots": [1, 2],
                        "robot1_wait": "W_1",   # TODO: C열 기준으로 맞게 수정
                        "robot5_wait": "W_2",
                    },
                    {
                        "slots": [3, 4],
                        "robot1_wait": "W_3",
                        "robot5_wait": "W_4",
                    },
                ],
            },
        }

        return meta

    # -------------------------------------------------------------
    #  (col, slot_num) 에 해당하는 robot1/robot5 wait 위치 찾기
    # -------------------------------------------------------------
    def get_wait_positions(self, col, slot_num):
        meta = self.COLUMN_META.get(col)
        if meta is None:
            self.get_logger().warn(f'No COLUMN_META for col="{col}"')
            return None, None

        for group in meta["wait_groups"]:
            if slot_num in group["slots"]:
                return group["robot1_wait"], group["robot5_wait"]

        self.get_logger().warn(f'No wait group found for col={col}, slot={slot_num}')
        return None, None

    # -------------------------------------------------------------
    #  name 으로 Pose 생성 (CSV 기반)
    # -------------------------------------------------------------
    def get_waypoint(self, name, yaw_override_deg=None):
        info = self.block_map.get(name)
        if info is None:
            self.get_logger().warn(f'No block "{name}" in block_map')
            return None

        x = info['x']
        y = info['y']

        if yaw_override_deg is None:
            yaw_deg = info['yaw_deg']
        else:
            yaw_deg = yaw_override_deg

        yaw = math.radians(yaw_deg)

        pose = Pose()
        pose.position = Point(x=x, y=y, z=0.0)
        pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0),
        )
        return pose

    # -------------------------------------------------------------
    #  JSON 형식의 /parking/exit_type 콜백
    # -------------------------------------------------------------
    def exit_type_callback(self, msg):
        raw = msg.data.strip()
        self.get_logger().info(f'Received /task_command/robot5: {raw}')

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'JSON parse error: {e}')
            return

        # 🔹 새 JSON 포맷 매핑
        # task_type: "EXIT_SINGLE", "EXIT_DOUBLE" 등
        task_type = str(data.get('task_type', '')).upper()

        if task_type == 'EXIT_SINGLE':
            exit_type = 'SINGLE'
        elif task_type == 'EXIT_DOUBLE':
            exit_type = 'DOUBLE'
        else:
            self.get_logger().warn(f'Unsupported task_type="{task_type}"')
            return

        # 예전 parking_spot → 지금은 start_location 사용
        parking_spot = str(data.get('start_location', ''))

        # 예전 command_id → 지금은 task_id 사용
        command_id = data.get('task_id', '')

        # 예전 license_plate → 지금은 vehicle_plate 사용
        license_plate = str(data.get('vehicle_plate', ''))

        # ✅ task_id 유효성 체크
        if not command_id:
            self.get_logger().warn('Received task without task_id. Ignoring.')
            return

        # ✅ 이미 처리한 task_id면 중복 요청 → 무시
        if command_id in self.processed_task_ids:
            self.get_logger().warn(
                f'Duplicated task_id="{command_id}" received. Ignoring this request.'
            )
            return

        # ✅ 처음 보는 task_id면 기록만 해두고 계속 진행
        self.processed_task_ids.add(command_id)


        self.get_logger().info(
            f'Parsed task: exit_type={exit_type}, parking_spot={parking_spot}, '
            f'cmd_id={command_id}, plate={license_plate}'
        )
        missions = []

        if exit_type == 'SINGLE':
            # 기존 단일 출차는 그대로 한 방에 처리
            self.handle_single_spot(parking_spot, missions)

            if missions:
                arr = MissionArray()
                arr.missions = missions
                self.mission_pub.publish(arr)
                self.get_logger().info(f'Published {len(missions)} missions for {parking_spot}.')
            return

        elif exit_type == 'DOUBLE':
            # 이미 다른 DOUBLE task 수행 중이면 → 큐에 쌓기만 하고 리턴
            if self.active_command_id is not None:
                self.get_logger().info(
                    f'[FSM] already running cmd_id={self.active_command_id}, '
                    f'queue new cmd_id={command_id}, spot={parking_spot}'
                )
                # 나중에 다시 시작할 수 있도록 필요한 정보만 저장
                self.pending_tasks.append((parking_spot, command_id))
                return

            # 아무것도 실행 중이 아니면 바로 시작
            self.active_command_id = command_id
            self.start_double_fsm(parking_spot, command_id)
            return

        else:
            self.get_logger().warn(f'Unknown exit_type="{exit_type}"')
            return

        if missions:
            arr = MissionArray()
            arr.missions = missions
            self.mission_pub.publish(arr)
            self.get_logger().info(f'Published {len(missions)} missions for {parking_spot}.')

    def mission_status_callback_robot1(self, msg: MissionStatus):
        self._handle_mission_status('robot1', msg)

    def mission_status_callback_robot5(self, msg: MissionStatus):
        self._handle_mission_status('robot5', msg)

    def _finish_double_task(self, cmd_id: str):
        """DOUBLE FSM이 PHASE3까지 다 끝났을 때 호출해서
        - double_fsm 정리
        - active_command_id 해제
        - pending 큐에 쌓인 다음 task 시작
        """
        task = self.double_fsm.pop(cmd_id, None)
        if task is None:
            self.get_logger().warn(f'[FSM] finish called but no task for cmd_id={cmd_id}')
        else:
            self.get_logger().info(f'[FSM] cmd_id={cmd_id} DONE. Cleanup.')

        # 현재 active_command_id가 이 커맨드면 해제
        if self.active_command_id == cmd_id:
            self.active_command_id = None

        # 대기 중인 다음 DOUBLE task가 있으면 바로 시작
        if self.pending_tasks:
            next_parking_spot, next_cmd_id = self.pending_tasks.popleft()
            self.get_logger().info(
                f'[FSM] start next queued DOUBLE task: cmd_id={next_cmd_id}, '
                f'spot={next_parking_spot}'
            )
            self.active_command_id = next_cmd_id
            self.start_double_fsm(next_parking_spot, next_cmd_id)
        else:
            self.get_logger().info('[FSM] no pending DOUBLE task. Idle.')


    def _handle_mission_status(self, robot_name: str, msg: MissionStatus):
        mission_id = msg.mission_id
        state = msg.state
        # 이 mission_id가 어떤 DOUBLE task(command_id)에 속하는지 조회
        cmd_id = self.mission_to_task.get(mission_id)
        if not cmd_id:
            # DOUBLE FSM이 관리하지 않는 미션이면 무시
            return

        task = self.double_fsm.get(cmd_id)
        if not task:
            return

        self.get_logger().info(
            f'[FSM] status from {robot_name}: '
            f'mission_id={mission_id}, state={state}, cmd_id={cmd_id}, phase={task["state"]}'
        )
    # 성공한 미션만 페이즈 진행에 사용
        if state != 'SUCCEEDED':
            return

        # -------------------------
        # PHASE1 → PHASE2
        # -------------------------
        if task["state"] == "PHASE1":
            goal_set = task.get("phase1_goal_ids", set())
            if mission_id in goal_set:
                goal_set.discard(mission_id)
                self.get_logger().info(
                    f'[FSM] cmd_id={cmd_id} PHASE1 goal reached by {robot_name}. '
                    f'remaining={len(goal_set)}'
                )

                # 골포즈 모두 도달했으면 Phase2 시작
                if not goal_set:
                    self.get_logger().info(
                        f'[FSM] cmd_id={cmd_id}: Phase1 all goal poses reached. Start Phase2 (robot5).'
                    )
                    task["state"] = "PHASE2"
                    self.start_double_phase2(cmd_id)
            return

        # -------------------------
        # PHASE2 → PHASE3
        # -------------------------
        if task["state"] == "PHASE2":
            goal_set = task.get("phase2_goal_ids", set())
            if mission_id in goal_set:
                goal_set.discard(mission_id)
                self.get_logger().info(
                    f'[FSM] cmd_id={cmd_id} PHASE2 goal reached by {robot_name}. '
                    f'remaining={len(goal_set)}'
                )

                if not goal_set:
                    self.get_logger().info(
                        f'[FSM] cmd_id={cmd_id}: Phase2 all goal poses reached. Start Phase3 (robot1 & robot5).'
                    )
                    task["state"] = "PHASE3"
                    self.start_double_phase3(cmd_id)
            return

        # -------------------------
        # PHASE3 → DONE
        # -------------------------
        if task["state"] == "PHASE3":
            goal_set = task.get("phase3_goal_ids", set())
            if mission_id in goal_set:
                goal_set.discard(mission_id)
                self.get_logger().info(
                    f'[FSM] cmd_id={cmd_id} PHASE3 goal reached by {robot_name}. '
                    f'remaining={len(goal_set)}'
                )

                if not goal_set:
                    self.get_logger().info(
                        f'[FSM] cmd_id={cmd_id}: Phase3 all goal poses reached. DOUBLE sequence completed.'
                    )
                    task["state"] = "DONE"
                    # 🔴 여기서 직접 pop 하지 말고 헬퍼로 마무리
                    self._finish_double_task(cmd_id)
            return


    # -------------------------------------------------------------
    #  SINGLE 출차 로직 (간단 버전)
    #   - robot5: wait -> base_slot -> parking_spot -> EXIT -> wait
    # -------------------------------------------------------------
    def handle_single_spot(self, parking_spot, missions):
        parts = parking_spot.split('_')
        if len(parts) < 2:
            self.get_logger().warn(f'SINGLE invalid parking_spot: "{parking_spot}"')
            return

        col = parts[0]  # A/B/C
        if col not in self.COLUMN_META:
            self.get_logger().warn(f'No COLUMN_META for col="{col}"')
            return

        try:
            num = int(parts[1])
        except ValueError:
            self.get_logger().warn(f'Invalid slot number in "{parking_spot}"')
            return

        meta = self.COLUMN_META[col]
        _, robot5_wait = self.get_wait_positions(col, num)
        if robot5_wait is None:
            return

        meta = self.COLUMN_META[col]

        # 1) side / buffer 가져오기
        slot_info = meta["slots"].get(num)
        if slot_info is None:
            self.get_logger().warn(f'No slot meta for col={col}, num={num}')
            return

        side_slot   = slot_info["side"]    # 예: A_2
        buffer_slot = slot_info["buffer"]  # 예: A_3

        # 2) 이 슬롯 번호에 대한 wait 위치
        robot1_wait, robot5_wait = self.get_wait_positions(col, num)
        if robot1_wait is None or robot5_wait is None:
            return

        exit_name = meta["exit"]
        base_slot  = f"{col}_{num}"      # 예: "A_1"
        front_slot = f"{col}_{num}_2"    # 예: "A_1_2"
        inner_slot = parking_spot        # 예: "A_1_1"

        # 🔹 각 로봇이 base_slot에 "처음" 도착했는지 여부
        align_added_robot1 = False
        align_added_robot5 = False


        # robot5 route: wait -> base_slot -> parking_spot -> exit -> wait
        r1_phase1 = [
            (robot1_wait, 90),
            (base_slot,   0),
            (front_slot,  180),
            (base_slot, 90),
            (robot5_wait, 180),
            (exit_name,  180),
            (exit_name,  0),
            (robot5_wait, 270),
            (robot1_wait, 90),
        ]

        for name, yaw_deg in r1_phase1:
            pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
            if pose is None:
                return
            m = Mission()
            m.mission_id = self.next_mission_id()
            m.mission_type = 'MOVE_FRONT_CAR'
            m.car_id = 0
            m.from_slot_id = 0
            m.to_slot_id = 0
            m.target_pose = pose
            missions.append(m)

            # ✅ robot1이 base_slot에 "처음" 도착했을 때만 ALIGN 미션 1개 추가
            if (name == base_slot) and (not align_added_robot1):
                m_align = Mission()
                m_align.mission_id = self.next_mission_id()
                m_align.mission_type = 'ALIGN_BASE_FRONT'   # robot1용 정렬 미션
                m_align.car_id = 0
                m_align.from_slot_id = 0
                m_align.to_slot_id = 0
                m_align.target_pose = self.get_waypoint(base_slot, yaw_override_deg=yaw_deg)
                missions.append(m_align)

                align_added_robot1 = True

            if name == front_slot and yaw_deg == 180:
                b = Mission()
                b.mission_id = self.next_mission_id()
                b.mission_type = 'BEEP_R1'
                b.car_id = 0
                b.from_slot_id = 0
                b.to_slot_id = 0
                # 굳이 pose 안 써도 되지만, 형식 맞춰서 동일 pose 넣어줌
                b.target_pose = pose
                missions.append(b)

        self.get_logger().info(
            f'[AUTO SINGLE] spot={parking_spot}, robot5 route={r1_phase1}'
        )
        # -------------------------------------------------------------
    #  DOUBLE 출차 FSM 시작: Phase1부터 시작
    # -------------------------------------------------------------
    


    # -------------------------------------------------------------
    #  Phase1: robot1만 실행 (r1_phase1)
    #   robot1: wait_r1 -> base_slot -> front_slot -> buffer_slot (+ ALIGN_BASE_FRONT)
    # -------------------------------------------------------------
    def start_double_phase1(self, command_id: str):
        task = self.double_fsm.get(command_id)
        if not task:
            self.get_logger().warn(f'[FSM] Phase1 called but no task for cmd_id={command_id}')
            return

        parking_spot = task["parking_spot"]

        # 공통 슬롯/위치 계산
        parts = parking_spot.split('_')
        if len(parts) != 3:
            self.get_logger().warn(f'DOUBLE expects inner slot like A_1_1, got "{parking_spot}"')
            return

        col = parts[0]
        if col not in self.COLUMN_META:
            self.get_logger().warn(f'No COLUMN_META for col="{col}"')
            return

        try:
            num = int(parts[1])
        except ValueError:
            self.get_logger().warn(f'Invalid slot number in "{parking_spot}"')
            return

        inner_flag = parts[2]
        if inner_flag != '1':
            self.get_logger().warn(
                f'DOUBLE only supports *_1 inner slots for now, got "{parking_spot}"'
            )
            return

        meta = self.COLUMN_META[col]

        slot_info = meta["slots"].get(num)
        if slot_info is None:
            self.get_logger().warn(f'No slot meta for col={col}, num={num}')
            return

        side_slot   = slot_info["side"]
        buffer_slot = slot_info["buffer"]

        robot1_wait, robot5_wait = self.get_wait_positions(col, num)
        if robot1_wait is None or robot5_wait is None:
            return

        exit_name = meta["exit"]
        base_slot  = f"{col}_{num}"
        front_slot = f"{col}_{num}_2"
        inner_slot = parking_spot
        # ✅ Phase1 골포즈 집합 초기화
        task["phase1_goal_ids"] = set()

        align_added_robot1 = False
        missions = []

        last_id = None

        r1_phase1 = [
            (robot1_wait, 90),
            (base_slot,   0),
            (front_slot,  180),
            (buffer_slot, 0),
        ]

        for name, yaw_deg in r1_phase1:
            pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
            if pose is None:
                return

            m = Mission()
            m.mission_id = self.next_mission_id()
            m.mission_type = 'MOVE_FRONT_CAR'
            m.command_id = command_id 
            m.car_id = 0
            m.from_slot_id = 0
            m.to_slot_id = 0
            m.target_pose = pose
            missions.append(m)

            # 이 mission_id가 어떤 task의 것인지 기록
            self.mission_to_task[m.mission_id] = command_id
            if name == buffer_slot:
                task["phase1_goal_ids"].add(m.mission_id)

            # base_slot에 처음 도착하는 순간 ALIGN 미션 추가
            if (name == base_slot) and (not align_added_robot1):
                m_align = Mission()
                m_align.mission_id = self.next_mission_id()
                m_align.mission_type = 'ALIGN_BASE_FRONT'
                m_align.car_id = 0
                m_align.from_slot_id = 0
                m_align.to_slot_id = 0
                m_align.target_pose = self.get_waypoint(base_slot, yaw_override_deg=yaw_deg)
                missions.append(m_align)

                self.mission_to_task[m_align.mission_id] = command_id
                

                align_added_robot1 = True
            if name == front_slot and yaw_deg == 180:
                b = Mission()
                b.mission_id = self.next_mission_id()
                b.mission_type = 'BEEP_R1'
                b.car_id = 0
                b.from_slot_id = 0
                b.to_slot_id = 0
                # 굳이 pose 안 써도 되지만, 형식 맞춰서 동일 pose 넣어줌
                b.target_pose = pose
                missions.append(b)

        r5_sequence = [
            (robot5_wait, 270),   # r5_phase1
            (side_slot,   270),
        ]

        for name, yaw_deg in r5_sequence:
            pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
            if pose is None:
                return

            m = Mission()
            m.mission_id = self.next_mission_id()
            m.mission_type = 'EXIT_TARGET_CAR'
            m.car_id = 0
            m.from_slot_id = 0
            m.to_slot_id = 0
            m.target_pose = pose
            missions.append(m)

            self.mission_to_task[m.mission_id] = command_id
            # ✅ robot5가 side_slot에 도착하는 이 미션도 Phase1 골포즈로 등록
            if name == side_slot:
                task["phase1_goal_ids"].add(m.mission_id)

            
        if not missions:
            self.get_logger().warn(f'[FSM] Phase1 generated no missions for cmd_id={command_id}')
            return

        arr = MissionArray()
        arr.missions = missions
        self.mission_pub.publish(arr)
        self.get_logger().info(
            f'[FSM] Phase1 published {len(missions)} missions for cmd_id={command_id}, '
            f'parking_spot={parking_spot}'
        )

        task["phase1_last_id"] = last_id

        # -------------------------------------------------------------
    #  Phase2: robot5 실행 (r5_phase1 + r5_phase2 + r5_phase4)
    #   robot5: wait_r5 -> side_slot -> base_slot -> inner_slot -> base_slot
    #           -> robot5_wait -> EXIT -> EXIT(orientation 0) -> robot5_wait(복귀)
    # -------------------------------------------------------------
    def start_double_phase2(self, command_id: str):
        task = self.double_fsm.get(command_id)
        if not task:
            self.get_logger().warn(f'[FSM] Phase2 called but no task for cmd_id={command_id}')
            return

        parking_spot = task["parking_spot"]

        # 공통 슬롯/위치 계산 (Phase1과 동일)
        parts = parking_spot.split('_')
        if len(parts) != 3:
            self.get_logger().warn(f'DOUBLE expects inner slot like A_1_1, got "{parking_spot}"')
            return

        col = parts[0]
        if col not in self.COLUMN_META:
            self.get_logger().warn(f'No COLUMN_META for col="{col}"')
            return

        try:
            num = int(parts[1])
        except ValueError:
            self.get_logger().warn(f'Invalid slot number in "{parking_spot}"')
            return

        inner_flag = parts[2]
        if inner_flag != '1':
            self.get_logger().warn(
                f'DOUBLE only supports *_1 inner slots for now, got "{parking_spot}"'
            )
            return

        meta = self.COLUMN_META[col]

        slot_info = meta["slots"].get(num)
        if slot_info is None:
            self.get_logger().warn(f'No slot meta for col={col}, num={num}')
            return

        side_slot   = slot_info["side"]
        buffer_slot = slot_info["buffer"]

        robot1_wait, robot5_wait = self.get_wait_positions(col, num)
        if robot1_wait is None or robot5_wait is None:
            return

        exit_name = meta["exit"]
        base_slot  = f"{col}_{num}"
        front_slot = f"{col}_{num}_2"
        inner_slot = parking_spot

        # ✅ Phase2 골포즈 집합 초기화
        task["phase2_goal_ids"] = set()

        align_added_robot5 = False
        missions = []
        last_id = None

        # r5_phase1 + r5_phase2 + r5_phase4를 하나로 묶어서 순차 실행
        r5_sequence = [
            (base_slot,   0),     # r5_phase2
            (inner_slot,  180),
            (base_slot,   90),
            (robot5_wait, 180),
        ]

        for name, yaw_deg in r5_sequence:
            pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
            if pose is None:
                return

            m = Mission()
            m.mission_id = self.next_mission_id()
            m.mission_type = 'EXIT_TARGET_CAR'
            m.car_id = 0
            m.from_slot_id = 0
            m.to_slot_id = 0
            m.target_pose = pose
            missions.append(m)

            self.mission_to_task[m.mission_id] = command_id
            # ✅ robot5가 robot5_wait에 도착하는 미션을 Phase2 골포즈로 등록
            if name == robot5_wait:
                task["phase2_goal_ids"].add(m.mission_id)

            # base_slot에 처음 도착하는 순간 ALIGN 미션 추가
            if (name == base_slot) and (not align_added_robot5):
                m_align = Mission()
                m_align.mission_id = self.next_mission_id()
                m_align.mission_type = 'ALIGN_BASE_EXIT'
                m_align.car_id = 0
                m_align.from_slot_id = 0
                m_align.to_slot_id = 0
                m_align.target_pose = self.get_waypoint(base_slot, yaw_override_deg=yaw_deg)
                missions.append(m_align)

                self.mission_to_task[m_align.mission_id] = command_id
                

                align_added_robot5 = True
            if name == inner_slot and yaw_deg == 180:
                b = Mission()
                b.mission_id = self.next_mission_id()
                b.mission_type = 'BEEP_R5'
                b.car_id = 0
                b.from_slot_id = 0
                b.to_slot_id = 0
                # 굳이 pose 안 써도 되지만, 형식 맞춰서 동일 pose 넣어줌
                b.target_pose = pose
                missions.append(b)

        if not missions:
            self.get_logger().warn(f'[FSM] Phase2 generated no missions for cmd_id={command_id}')
            return

        arr = MissionArray()
        arr.missions = missions
        self.mission_pub.publish(arr)
        self.get_logger().info(
            f'[FSM] Phase2 published {len(missions)} missions for cmd_id={command_id}, '
            f'parking_spot={parking_spot}'
        )

        task["phase2_last_id"] = last_id


        # -------------------------------------------------------------
    #  Phase3: 다시 robot1 실행 (r1_phase3)
    #   robot1: buffer_slot -> inner_slot(원래 자리) -> base_slot -> wait_r1
    # -------------------------------------------------------------
    def start_double_phase3(self, command_id: str):
        task = self.double_fsm.get(command_id)
        if not task:
            self.get_logger().warn(f'[FSM] Phase3 called but no task for cmd_id={command_id}')
            return

        parking_spot = task["parking_spot"]

        # 공통 슬롯/위치 계산
        parts = parking_spot.split('_')
        if len(parts) != 3:
            self.get_logger().warn(f'DOUBLE expects inner slot like A_1_1, got "{parking_spot}"')
            return

        col = parts[0]
        if col not in self.COLUMN_META:
            self.get_logger().warn(f'No COLUMN_META for col="{col}"')
            return

        try:
            num = int(parts[1])
        except ValueError:
            self.get_logger().warn(f'Invalid slot number in "{parking_spot}"')
            return

        inner_flag = parts[2]
        if inner_flag != '1':
            self.get_logger().warn(
                f'DOUBLE only supports *_1 inner slots for now, got "{parking_spot}"'
            )
            return

        meta = self.COLUMN_META[col]

        slot_info = meta["slots"].get(num)
        if slot_info is None:
            self.get_logger().warn(f'No slot meta for col={col}, num={num}')
            return

        side_slot   = slot_info["side"]
        buffer_slot = slot_info["buffer"]

        robot1_wait, robot5_wait = self.get_wait_positions(col, num)
        if robot1_wait is None or robot5_wait is None:
            return

        exit_name = meta["exit"]
        base_slot  = f"{col}_{num}"
        front_slot = f"{col}_{num}_2"
        inner_slot = parking_spot

        # ✅ Phase3 골포즈 집합 초기화
        task["phase3_goal_ids"] = set()

        missions = []
        last_id = None

        r5_sequence = [
            (exit_name,   180),
            (exit_name,   0),
            (robot5_wait, 270),   # r5_phase4 (복귀)
        ]

        for name, yaw_deg in r5_sequence:
            pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
            if pose is None:
                return

            m = Mission()
            m.mission_id = self.next_mission_id()
            m.mission_type = 'EXIT_TARGET_CAR'
            m.car_id = 0
            m.from_slot_id = 0
            m.to_slot_id = 0
            m.target_pose = pose
            missions.append(m)

            self.mission_to_task[m.mission_id] = command_id

            if name == robot5_wait:
                task["phase3_goal_ids"].add(m.mission_id)

        r1_phase3 = [
            (buffer_slot, 0),
            (inner_slot,  180),
            (base_slot,   270),
            (robot1_wait, 90),
        ]

        for name, yaw_deg in r1_phase3:
            pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
            if pose is None:
                return

            m = Mission()
            m.mission_id = self.next_mission_id()
            m.mission_type = 'MOVE_FRONT_CAR'
            m.car_id = 0
            m.from_slot_id = 0
            m.to_slot_id = 0
            m.target_pose = pose
            missions.append(m)

            self.mission_to_task[m.mission_id] = command_id

            if name == robot1_wait:
                task["phase3_goal_ids"].add(m.mission_id)

            if name == inner_slot and yaw_deg == 180:
                b = Mission()
                b.mission_id = self.next_mission_id()
                b.mission_type = 'BEEP_R1'
                b.car_id = 0
                b.from_slot_id = 0
                b.to_slot_id = 0
                # 굳이 pose 안 써도 되지만, 형식 맞춰서 동일 pose 넣어줌
                b.target_pose = pose
                missions.append(b)

        if not missions:
            self.get_logger().warn(f'[FSM] Phase3 generated no missions for cmd_id={command_id}')
            return

        arr = MissionArray()
        arr.missions = missions
        self.mission_pub.publish(arr)
        self.get_logger().info(
            f'[FSM] Phase3 published {len(missions)} missions for cmd_id={command_id}, '
            f'parking_spot={parking_spot}'
        )

        task["phase3_last_id"] = last_id

    # -------------------------------------------------------------
    #  DOUBLE 출차 로직 - Phase1~4 전체 구현
    #  - parking_spot = "A_1_1" 처럼 안쪽차 기준
    # -------------------------------------------------------------
    # def handle_double_all_phases(self, parking_spot, missions):
    #     parts = parking_spot.split('_')
    #     if len(parts) != 3:
    #         self.get_logger().warn(f'DOUBLE expects inner slot like A_1_1, got "{parking_spot}"')
    #         return

    #     col = parts[0]  # A/B/C
    #     if col not in self.COLUMN_META:
    #         self.get_logger().warn(f'No COLUMN_META for col="{col}"')
    #         return

    #     try:
    #         num = int(parts[1])
    #     except ValueError:
    #         self.get_logger().warn(f'Invalid slot number in "{parking_spot}"')
    #         return

    #     inner_flag = parts[2]
    #     if inner_flag != '1':
    #         self.get_logger().warn(
    #             f'DOUBLE only supports *_1 inner slots for now, got "{parking_spot}"'
    #         )
    #         return

    #     meta = self.COLUMN_META[col]

    #     # 1) side / buffer 가져오기
    #     slot_info = meta["slots"].get(num)
    #     if slot_info is None:
    #         self.get_logger().warn(f'No slot meta for col={col}, num={num}')
    #         return

    #     side_slot   = slot_info["side"]    # 예: A_2
    #     buffer_slot = slot_info["buffer"]  # 예: A_3

    #     # 2) 이 슬롯 번호에 대한 wait 위치
    #     robot1_wait, robot5_wait = self.get_wait_positions(col, num)
    #     if robot1_wait is None or robot5_wait is None:
    #         return

    #     exit_name = meta["exit"]
    #     base_slot  = f"{col}_{num}"      # 예: "A_1"
    #     front_slot = f"{col}_{num}_2"    # 예: "A_1_2"
    #     inner_slot = parking_spot        # 예: "A_1_1"

    #     # 🔹 각 로봇이 base_slot에 "처음" 도착했는지 여부
    #     align_added_robot1 = False
    #     align_added_robot5 = False

    #     # ---------------------------------------------------------
    #     #  Phase1
    #     #   robot1: wait_r1 -> base_slot -> front_slot -> buffer_slot
    #     #   robot5: wait_r5 -> side_slot
    #     # ---------------------------------------------------------
    #     r1_phase1 = [
    #         (robot1_wait, 90),
    #         (base_slot,   0),
    #         (front_slot,  180),
    #         (buffer_slot, 0),
    #     ]

    #     for name, yaw_deg in r1_phase1:
    #         pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
    #         if pose is None:
    #             return
    #         m = Mission()
    #         m.mission_id = self.next_mission_id()
    #         m.mission_type = 'MOVE_FRONT_CAR'
    #         m.car_id = 0
    #         m.from_slot_id = 0
    #         m.to_slot_id = 0
    #         m.target_pose = pose
    #         missions.append(m)

    #         # ✅ robot1이 base_slot에 "처음" 도착했을 때만 ALIGN 미션 1개 추가
    #         if (name == base_slot) and (not align_added_robot1):
    #             m_align = Mission()
    #             m_align.mission_id = self.next_mission_id()
    #             m_align.mission_type = 'ALIGN_BASE_FRONT'   # robot1용 정렬 미션
    #             m_align.car_id = 0
    #             m_align.from_slot_id = 0
    #             m_align.to_slot_id = 0
    #             m_align.target_pose = self.get_waypoint(base_slot, yaw_override_deg=yaw_deg)
    #             missions.append(m_align)

    #             align_added_robot1 = True

    #         if name == front_slot and yaw_deg == 180:
    #             b = Mission()
    #             b.mission_id = self.next_mission_id()
    #             b.mission_type = 'BEEP_R1'
    #             b.car_id = 0
    #             b.from_slot_id = 0
    #             b.to_slot_id = 0
    #             # 굳이 pose 안 써도 되지만, 형식 맞춰서 동일 pose 넣어줌
    #             b.target_pose = pose
    #             missions.append(b)

    #     r5_phase1 = [
    #         (robot5_wait, 270),
    #         (side_slot,   270),
    #     ]

    #     for name, yaw_deg in r5_phase1:
    #         pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
    #         if pose is None:
    #             return
    #         m = Mission()
    #         m.mission_id = self.next_mission_id()
    #         m.mission_type = 'EXIT_TARGET_CAR'
    #         m.car_id = 0
    #         m.from_slot_id = 0
    #         m.to_slot_id = 0
    #         m.target_pose = pose
    #         missions.append(m)

    #     # ---------------------------------------------------------
    #     #  Phase2
    #     #   robot5: side_slot -> base_slot -> inner_slot(요청차) -> EXIT
    #     #   (이미 Phase1 의 마지막 pose 가 side_slot 이라고 가정)
    #     # ---------------------------------------------------------
    #     r5_phase2 = [
    #         (base_slot,  0),
    #         (inner_slot, 180),
    #         (base_slot,  90),
    #         (robot5_wait, 180),
    #         (exit_name,  180),
    #         (exit_name,  0),
    #     ]

    #     for name, yaw_deg in r5_phase2:
    #         pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
    #         if pose is None:
    #             return
    #         m = Mission()
    #         m.mission_id = self.next_mission_id()
    #         m.mission_type = 'EXIT_TARGET_CAR'
    #         m.car_id = 0
    #         m.from_slot_id = 0
    #         m.to_slot_id = 0
    #         m.target_pose = pose
    #         missions.append(m)

    #         # ✅ robot5가 base_slot에 "처음" 도착했을 때만 ALIGN 미션 1개 추가
    #         if (name == base_slot) and (not align_added_robot5):
    #             m_align = Mission()
    #             m_align.mission_id = self.next_mission_id()
    #             m_align.mission_type = 'ALIGN_BASE_EXIT'   # robot5용 정렬 미션
    #             m_align.car_id = 0
    #             m_align.from_slot_id = 0
    #             m_align.to_slot_id = 0
    #             m_align.target_pose = self.get_waypoint(base_slot, yaw_override_deg=yaw_deg)
    #             missions.append(m_align)

    #             align_added_robot5 = True

    #         if name == inner_slot and yaw_deg == 180:
    #             b = Mission()
    #             b.mission_id = self.next_mission_id()
    #             b.mission_type = 'BEEP_R5'
    #             b.car_id = 0
    #             b.from_slot_id = 0
    #             b.to_slot_id = 0
    #             b.target_pose = pose
    #             missions.append(b)

    #     # ---------------------------------------------------------
    #     #  Phase3
    #     #   robot1: buffer_slot -> front_slot(원래 자리 복귀) -> wait_r1
    #     # ---------------------------------------------------------
    #     r1_phase3 = [
    #         (buffer_slot, 0),
    #         (inner_slot, 180),
    #         (base_slot,  270),
    #         (robot1_wait, 90),
    #     ]

    #     for name, yaw_deg in r1_phase3:
    #         pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
    #         if pose is None:
    #             return
    #         m = Mission()
    #         m.mission_id = self.next_mission_id()
    #         m.mission_type = 'MOVE_FRONT_CAR'
    #         m.car_id = 0
    #         m.from_slot_id = 0
    #         m.to_slot_id = 0
    #         m.target_pose = pose
    #         missions.append(m)

    #     # ---------------------------------------------------------
    #     #  Phase4
    #     #   robot5: EXIT -> wait_r5 (복귀)
    #     # ---------------------------------------------------------
    #     r5_phase4 = [
    #         (robot5_wait, 270),
    #     ]

    #     for name, yaw_deg in r5_phase4:
    #         pose = self.get_waypoint(name, yaw_override_deg=yaw_deg)
    #         if pose is None:
    #             return
    #         m = Mission()
    #         m.mission_id = self.next_mission_id()
    #         m.mission_type = 'EXIT_TARGET_CAR'
    #         m.car_id = 0
    #         m.from_slot_id = 0
    #         m.to_slot_id = 0
    #         m.target_pose = pose
    #         missions.append(m)

    #     self.get_logger().info(
    #         f'[DOUBLE ALL PHASES] spot={parking_spot}, '
    #         f'r1_phase1={r1_phase1}, r5_phase1={r5_phase1}, '
    #         f'r5_phase2={r5_phase2}, r1_phase3={r1_phase3}, r5_phase4={r5_phase4}'
    #     )

    # -------------------------------------------------------------
    #  미션 ID 생성
    # -------------------------------------------------------------
    def next_mission_id(self):
        mid = self.mission_id_counter
        self.mission_id_counter += 1
        return mid


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
