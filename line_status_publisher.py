from dataclasses import dataclass
from typing import Optional, Tuple

from rclpy.node import Node
from msgs.msg import LineResult

class LineStatus:
    Forward = 1
    Left_Half_Forward = 2
    Right_Half_Forward = 3
    Left_Forward = 4
    Right_Forward = 5
    Left_Turn = 6
    Right_Turn = 7
    Forward_half = 8
    Backward_half = 9
    Left_Move = 10
    Right_Move = 11
    Follow_Point = 30
    Line_Lost = 99

@dataclass
class LineFeatures:
    # 검출된 라인 점 개수
    point_count: int

    # 점 2개일 때 또는 직선 판단일 때 사용하는 선 각도
    line_angle: Optional[float] = None

    # 점 3개 이상일 때 이차함수의 a값
    curve_a: Optional[float] = None

    # 곡선일 때 두 번째 점에서의 접선 각도
    tangent_angle: Optional[float] = None

    # 로봇 중심선과 라인 사이 거리
    # 왼쪽(-), 오른쪽(+)
    line_distance: Optional[float] = None

    # 좌표 따라 이동을 쓸 경우 사용할 목표 좌표
    target_x: Optional[float] = None
    target_y: Optional[float] = None

    #로봇 중심점 좌표
    robot_center_x: float = 320.0
    robot_center_y: float = 480.0

    #목표 좌표까지의 각도,거리
    follow_angle: Optional[float] = None
    follow_distance: Optional[float] = None



class LineDecision:
    def __init__(self):
        #직진, 미세회전, 회전 각도 기준 설정
        self.forward_angle = 10.0
        self.turn_angle = 30.0

        #곡선 판단하는 a값의 기준 - 나중에 수정
        self.curve_a = 0.15

        #거리기준 - 픽셀 단위로 맞춰서 수정하기
        self.move_distance = 60.0
        self.out_distance = 120.0

        #이탈 후 복귀여부 (좌표따라가기)
        self.follow_point = False

        self.follow_distance = 40.0
       

    def decide(self, features: LineFeatures) -> Tuple[int, float]:
        # 점 개수 0~1개이면 라인을 놓쳤다고 판단
        if features.point_count <= 1:
            return LineStatus.Line_Lost, 0.0
        
        #거리 기준 판단
        # line_distance값이 들어왔는지 판단
        if features.line_distance is not None:
            distance = features.line_distance

            if abs(distance) >= self.out_distance:
                return LineStatus.Follow_Point, 0.0

            # 2-2. 라인이 중심에서 어느 정도 벗어나 있으면 좌/우 이동
            if abs(distance) >= self.move_distance:
                if distance < 0:
                    return LineStatus.Left_Forward, 0.0
                else:
                    return LineStatus.Right_Forward, 0.0

                
        # 점 2개면 직선으로 판단
        if features.point_count == 2:
            return self._status_from_angle(features.line_angle)

        #점 개수가 3개 이상이면 a값으로 직선/곡선 판단
        if features.point_count >= 3:
            # a값이 없으면 일단 직선 각도 기준으로 판단
            if features.curve_a is None:
                return self._status_from_angle(features.line_angle,)
            
            #직선
            if abs(features.curve_a) < self.curve_a:
                return self._status_from_angle(features.line_angle)
            
            #곡선
            return self._status_from_angle(features.tangent_angle,)

        return LineStatus.Line_Lost, 0.0
    
    def _status_from_angle(self, angle: Optional[float]) -> Tuple[int, float]:
        if angle is None:
            return LineStatus.Line_Lost, 0.0
        
        abs_angle = abs(angle)

        #10도 이하: 직진
        if abs_angle <= self.forward_angle:
            return LineStatus.Forward, 0.0
        
        #10~30도: 미세회전
        if abs_angle <= self.turn_angle:
            if angle < 0:
                return LineStatus.Left_Half_Forward, abs_angle
            else:
                return LineStatus.Right_Half_Forward, abs_angle
            
        #30도 이상: 회전
        if angle < 0:
            return LineStatus.Left_Forward, abs_angle
        else:
            return LineStatus.Right_Forward, abs_angle
        

class LineStatusPublisher:
    def __init__(self, node:Node):
        self.node = node
        self.line_decision = LineDecision()
        self.line_pub = self.node.create_publisher(LineResult, 'line_result', 10)

    #라인 상태를 판단하고 Publish하는 함수    
    def publish_line_status(
        self,
        point_count: int,
        line_angle: Optional[float] = None,
        curve_a: Optional[float] = None,
        tangent_angle: Optional[float] = None,
        line_distance: Optional[float] = None,
        follow_angle: Optional[float] = None,
        follow_distance: Optional[float] = None,
    ) -> Tuple[int, float]:

        #LineFeatures 객체 생성
        features = LineFeatures(
            point_count=point_count,
            line_angle=line_angle,
            curve_a=curve_a,
            tangent_angle=tangent_angle,
            line_distance=line_distance,
            follow_angle=follow_angle,
            follow_distance=follow_distance,
        )

        #라인 상태를 판단
        status, angle = self.line_decision.decide(features)

        #라인 상태를 Publish
        msg = LineResult()
        msg.status = int(status)
        msg.angle = float(angle)

        self.line_pub.publish(msg)

        return status, angle