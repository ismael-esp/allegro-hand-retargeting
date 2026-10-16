import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DexToAllegroMapper(Node):
    def __init__(self):
        super().__init__('la_dex_to_allegro_mapper')
        # Subscriber to dexterous hand joints
        self.sub = self.create_subscription(
            JointState,
            '/la_dex_joint_states',
            self.joint_callback,
            10)
        # Publisher for Allegro hand joints
        self.pub = self.create_publisher(
            JointState,
            '/la_/allegroHand_1/joint_cmd',
            10)

        # Dexterous hand (24 DoF) joint names
        self.dex_names = [
            'la_wrist_pitch_lower',    # 0
            'la_wrist_pitch_upper',    # 1
            'la_wrist_yaw',            # 2
            'la_index_yaw',            # 3
            'la_middle_yaw',           # 4
            'la_ring_yaw',             # 5
            'la_pinky_yaw',            # 6
            'la_index_pitch',          # 7
            'la_index_knuckle',        # 8
            'la_index_tip',            # 9
            'la_middle_pitch',         # 10
            'la_middle_knuckle',       # 11
            'la_middle_tip',           # 12
            'la_ring_pitch',           # 13
            'la_ring_knuckle',         # 14
            'la_ring_tip',             # 15
            'la_pinky_pitch',          # 16
            'la_pinky_knuckle',        # 17
            'la_pinky_tip',            # 18
            'la_thumb_yaw',            # 19
            'la_thumb_roll',           # 20
            'la_thumb_pitch',          # 21
            'la_thumb_knuckle',        # 22
            'la_thumb_tip'             # 23
        ]

        # Allegro hand (16 DoF) joint names
        self.allegro_names = [
            'la_joint_0_0',  # index_yaw
            'la_joint_1_0',  # index_pitch
            'la_joint_2_0',  # index_knuckle
            'la_joint_3_0',  # index_tip
            'la_joint_4_0',  # middle_yaw
            'la_joint_5_0',  # middle_pitch
            'la_joint_6_0',  # middle_knuckle
            'la_joint_7_0',  # middle_tip
            'la_joint_8_0',  # ring_yaw
            'la_joint_9_0',  # ring_pitch
            'la_joint_10_0', # ring_knuckle
            'la_joint_11_0', # ring_tip
            'la_joint_12_0', # thumb_roll
            'la_joint_13_0', # thumb_pitch
            'la_joint_14_0', # thumb_knuckle
            'la_joint_15_0'  # thumb_tip
        ]

        # Mapping from Dexterous index to Allegro index
        # invert of previous mapping: dex_idx -> allegro_idx
        self.dex2alg = {
            3: 0,   # index_yaw -> joint_0
            7: 1,   # index_pitch -> joint_1
            8: 2,   # index_knuckle -> joint_2
            9: 3,   # index_tip -> joint_3
            4: 4,   # middle_yaw -> joint_4
            10:5,   # middle_pitch -> joint_5
            11:6,   # middle_knuckle -> joint_6
            12:7,   # middle_tip -> joint_7
            5: 8,   # ring_yaw -> joint_8
            13:9,   # ring_pitch -> joint_9
            14:10,  # ring_knuckle -> joint_10
            15:11,  # ring_tip -> joint_11
            20:12,  # thumb_roll -> joint_12
            21:13,  # thumb_pitch -> joint_13
            22:14,  # thumb_knuckle -> joint_14
            23:15   # thumb_tip -> joint_15
        }

    def joint_callback(self, msg: JointState):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.allegro_names

        # Initialize positions with zeros
        positions = [0.0] * len(self.allegro_names)

        # Map dexterous positions into allegro positions
        for dex_idx, alg_idx in self.dex2alg.items():
            if dex_idx < len(msg.position) and alg_idx < len(positions):
                positions[alg_idx] = msg.position[dex_idx]

        out.position = positions
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DexToAllegroMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()