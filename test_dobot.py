from pydobot import Dobot
import time

port = "/dev/ttyACM0"   # change if needed

dobot = Dobot(port=port, verbose=True)

print("Connected to Dobot")

while True:
    pose = dobot.pose()
    print(pose)
    time.sleep(1)
