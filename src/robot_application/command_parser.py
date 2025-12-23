class CommandParser:
    def parse(self, command: dict) -> str:
        """
        Input example:
        { "type": "jog", "joint": 1, "value": 0.1 }
        """

        if command["type"] == "home":
            return "HOME"

        if command["type"] == "jog":
            joint = command["joint"]
            value = command["value"]
            return f"JOG J{joint} {value}"

        if command["type"] == "pose":
            x, y, z = command["x"], command["y"], command["z"]
            return f"MOVE {x} {y} {z}"

        return "UNKNOWN"

