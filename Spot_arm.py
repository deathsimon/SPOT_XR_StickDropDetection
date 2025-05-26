import websocket as ws
# from websockets.sync.client import connect
import struct

from ApiManager import ApiRequests

class SpotArm:
    def __init__(self, target_IP="0.0.0.0", target_port="8888"):
        """Establish WebSocket link to target IP and port"""
        try:
            self.client = ws.create_connection(f"ws://{self.target_IP}:{self.target_port}/")
            # self.client = connect(f"ws://0.0.0.0:8888/")
        except Exception as e:
            # Handle the exception if the connection fails
            self.client = None
            raise ConnectionError("Could not connect to Spot.")
    
    def __del__(self):
        if self.client is not None:
            try:
                self.client.close()
            except Exception as e:
                # Handle the exception if closing the connection fails
                print(f"Error closing connection: {e}")        
    
    def open_gripper_at_angle(self, angle_in_degree):
        self.client.send_binary(
            ApiRequests("GRIPPER_OPEN_ANGLE").id.to_bytes(1, "big")
            + struct.pack(">f", angle_in_degree)
            )
    
    def open_gripper(self):
        self.client.send_binary(
            ApiRequests("GRIPPER_OPEN").id.to_bytes(1, "big")
            )
    
    def close_gripper(self):
        self.client.send_binary(
            ApiRequests("GRIPPER_CLOSE").id.to_bytes(1, "big")
            )
    
    def stand(self):
        self.client.send_binary(
            ApiRequests("STAND").id.to_bytes(1, "big")
            )
    
    def sit(self):
        self.client.send_binary(
            ApiRequests("STAND").id.to_bytes(1, "big")
            )

    def arm_stow(self):
        self.client.send_binary(
            ApiRequests("ARM_STOW").id.to_bytes(1, "big")
            )
        
    def set_arm_joints(self, sh0, sh1, el0, el1, wr0, wr1):
        self.client.send_binary(
            ApiRequests("SET_ARM_JOINTS").id.to_bytes(1, "big")
            + struct.pack(">6f", sh0, sh1, el0, el1, wr0, wr1)
            )
    
    def set_arm_velocity(self, x, y, z):
        self.client.send_binary(
            ApiRequests("ARM_VEL").id.to_bytes(1, "big")
            + struct.pack(">3f", x, y, z)
            )


# Spot_arm = SpotArm()

# if __name__ == "__main__":
#     main = Main()
#     main.stand()
#     main.open_gripper_at_angle(27)
#     main.set_arm_joints(0.0, -1.2, 1.9, 0.0, -0.7, 1.57)
#     main.set_arm_velocity(0.0, 0.2, 0.0) # nudge left
    # main.set_arm_velocity(0.0, -0.2, 0.0) # nudge right


