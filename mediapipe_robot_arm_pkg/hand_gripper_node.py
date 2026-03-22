import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import cv2
import mediapipe as mp
import math

class HandGripperNode(Node):
    def __init__(self):
        super().__init__('hand_gripper_node')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        # URDF'den: gripper_left_finger_joint upper=0.06
        self.JOINT_NAMES    = ['gripper_left_finger_joint']
        self.GRIPPER_OPEN   = 0.06
        self.GRIPPER_CLOSED = 0.0

        self.mp_hands = mp.solutions.hands
        self.hands    = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Kamera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.get_logger().info('Hand Gripper Node başlatıldı!')

    def _dist(self, lm, a, b, w, h):
        return math.hypot(
            (lm[a].x - lm[b].x) * w,
            (lm[a].y - lm[b].y) * h
        )

    def compute_openness(self, hand_landmarks, w, h):
        """
        Baş parmak ucu (4) ile orta parmak ucu (12) arası mesafeyi
        bilek (0) ile orta MCP (9) referansına normalize eder.
        Döndürür: 0.0 = kapalı, 1.0 = açık
        """
        lm  = hand_landmarks.landmark
        val = self._dist(lm, 4, 12, w, h) / (self._dist(lm, 0, 9, w, h) + 1e-6)
        return max(0.0, min(1.0, (val - 0.5) / 1.5))

    def send_gripper_command(self, openness: float):
        position = self.GRIPPER_CLOSED + openness * (self.GRIPPER_OPEN - self.GRIPPER_CLOSED)

        traj             = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES

        point                 = JointTrajectoryPoint()
        point.positions       = [position]
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points           = [point]

        self.publisher.publish(traj)
        self.get_logger().info(f'Gripper: {position:.4f} m  (açıklık: {openness:.2f})')

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandGripperNode()

    try:
        while rclpy.ok():
            # ROS mesajlarını işle
            rclpy.spin_once(node, timeout_sec=0.001)

            # Kamera karesi
            ret, frame = node.cap.read()
            if not ret or frame is None:
                continue

            h, w = frame.shape[:2]
            result = node.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

            openness = 0.0
            if result.multi_hand_landmarks:
                hand     = result.multi_hand_landmarks[0]
                node.mp_draw.draw_landmarks(frame, hand, node.mp_hands.HAND_CONNECTIONS)
                openness = node.compute_openness(hand, w, h)
                node.send_gripper_command(openness)

            # Görsel geri bildirim
            bar_h = int(openness * h)
            cv2.rectangle(frame, (w-35, h-bar_h), (w-10, h), (0, 255, 0), -1)
            cv2.putText(frame, f'Aciklik: {openness:.2f}',
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2)
            cv2.putText(frame, 'q = cikis',
                        (10, h-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)

            cv2.imshow('El Takibi - Gripper Kontrol', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()