import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import cv2
import mediapipe as mp
import math

class HandControlNode(Node):
    def __init__(self):
        super().__init__('hand_control_node')

        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.GRIPPER_JOINT  = ['gripper_left_finger_joint']
        self.GRIPPER_OPEN   = 0.06
        self.GRIPPER_CLOSED = 0.0

        self.ARM_JOINTS = [
            'base_to_shoulder_joint',
            'shoulder_to_arm_joint',
            'arm_to_elbow_joint',
            'elbow_to_forearm_joint',
            'forearm_to_wrist_joint',
            'wrist_to_hand_joint',
        ]

        self.current_positions = {j: 0.0 for j in self.ARM_JOINTS}
        self.SHOULDER_MIN = -1.57
        self.SHOULDER_MAX =  1.57
        self.STEP         =  0.05

        self.mp_hands = mp.solutions.hands
        self.hands    = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.get_logger().info('Hand Control Node başlatıldı!')

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]

    def detect_gesture(self, lm):
        """
        STOP    : Tüm avuç açık (5 parmak açık) → robot durur
        UP      : Baş parmak yukarı → kol yukarı
        DOWN    : Baş parmak aşağı → kol aşağı
        NEUTRAL : Diğer durumlar
        """
        # Tüm avuç açık mı?
        fingers_open = (
            lm[8].y  < lm[6].y  and
            lm[12].y < lm[10].y and
            lm[16].y < lm[14].y and
            lm[20].y < lm[18].y
        )

        if fingers_open:
            return 'STOP'

        # Baş parmak yönü
        diff = lm[2].y - lm[4].y
        if diff > 0.08:
            return 'UP'
        elif diff < -0.08:
            return 'DOWN'
        else:
            return 'NEUTRAL'

    def compute_gripper_openness(self, lm, w, h):
        """
        Sadece baş parmak ucu (4) ile işaret parmağı ucu (8)
        arasındaki mesafeye göre gripper açıklığı.
        """
        thumb_index_dist = math.hypot(
            (lm[4].x - lm[8].x) * w,
            (lm[4].y - lm[8].y) * h
        )
        ref_dist = math.hypot(
            (lm[0].x - lm[9].x) * w,
            (lm[0].y - lm[9].y) * h
        ) + 1e-6


        val = thumb_index_dist / ref_dist
        openness = max(0.0, min(1.0, (val - 0.2) / 0.8))
        return 1.0 - openness 


    def send_gripper(self, openness: float):
        pos              = openness * self.GRIPPER_OPEN
        traj             = JointTrajectory()
        traj.joint_names = self.GRIPPER_JOINT
        pt               = JointTrajectoryPoint()
        pt.positions     = [pos]
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points      = [pt]
        self.gripper_pub.publish(traj)

    def send_arm(self, shoulder_pos: float):
        traj             = JointTrajectory()
        traj.joint_names = self.ARM_JOINTS
        pt               = JointTrajectoryPoint()
        pt.positions     = [
            self.current_positions['base_to_shoulder_joint'],
            shoulder_pos,
            self.current_positions['arm_to_elbow_joint'],
            self.current_positions['elbow_to_forearm_joint'],
            self.current_positions['forearm_to_wrist_joint'],
            self.current_positions['wrist_to_hand_joint'],
        ]
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
        traj.points        = [pt]
        self.arm_pub.publish(traj)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)

            ret, frame = node.cap.read()
            if not ret or frame is None:
                continue

            h, w = frame.shape[:2]
            result = node.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            if result.multi_hand_landmarks:
                hand = result.multi_hand_landmarks[0]
                lm   = hand.landmark
                node.mp_draw.draw_landmarks(frame, hand, node.mp_hands.HAND_CONNECTIONS)

                gesture  = node.detect_gesture(lm)
                current  = node.current_positions['shoulder_to_arm_joint']

                if gesture == 'UP':
                    node.send_arm(min(current + node.STEP, node.SHOULDER_MAX))
                elif gesture == 'DOWN':
                    node.send_arm(max(current - node.STEP, node.SHOULDER_MIN))
                elif gesture == 'STOP':
                    node.send_arm(current)  

                openness = node.compute_gripper_openness(lm, w, h)
                node.send_gripper(openness)

                thumb_px = (int(lm[4].x * w), int(lm[4].y * h))
                index_px = (int(lm[8].x * w), int(lm[8].y * h))
                cv2.line(frame, thumb_px, index_px, (0, 255, 255), 2)
                cv2.circle(frame, thumb_px, 8, (0, 200, 255), -1)
                cv2.circle(frame, index_px, 8, (0, 200, 255), -1)

            cv2.imshow('Robot Kol Kontrolu', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()