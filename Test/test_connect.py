from os import getenv
from time import sleep

from dotenv import load_dotenv
import dronekit
load_dotenv()

board = getenv("BOARD")
baud = int(getenv("BAUDRATE"))
print("BOARD: %s BAUD: %d"%(board, baud))
vehicle = dronekit.connect(board, baud=baud, wait_ready=False)
print(vehicle.mode)
print("CONNECTED")
