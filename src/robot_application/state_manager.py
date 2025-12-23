class StateManager:
    def __init__(self):
        self.joint_states = None
        self.robot_mode = "IDLE"

    def update_joint_states(self, joint_state):
        self.joint_states = joint_state

    def get_state(self):
        return {
            "mode": self.robot_mode,
            "joint_states": self.joint_states
        }

