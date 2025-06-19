#from .roboclaw import Roboclaw
from roboclaw_python.roboclaw_3 import Roboclaw

rclaw = Roboclaw('/dev/ttyAMA0', 57600)
rclaw.open()

rclaw.forward_m1(0x80, 50)