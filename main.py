# Created by Philip Smith                           %                                                                               
#       Email: psmail147@gmail.com                  %                                                    
#       Github: https://github.com/psmail147        %                         
# --------------------------------------------------%
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from robot import Robot
from pid_controller import PID_Controller
import time 
import math

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

robot = Robot(sim)
controller = PID_Controller()

desired_distance = 0.5
corner_turn_speed = 0.7
previous_error = 0
previous_time = time.time()

while True:
    robot.random_wander()

sim.endSimulation()