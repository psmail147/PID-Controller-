# Created by Philip Smith                           %                                                                               
#       Email: psmail147@gmail.com                  %                                                    
#       Github: https://github.com/psmail147        %                         
# --------------------------------------------------%
from mealpy import FloatVar, PSO
from pid_controller import PID_Controller
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from robot import Robot
import time

desired_distance = 0.5

def objective_func(solution):

    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.stopSimulation()
    sim.startSimulation()
    time.sleep(1)

    kp, ki, kd = solution 

    robot = Robot(sim)
    controller = PID_Controller()
    controller.proportional_gain_constant = kp
    controller.integral_gain_constant = ki
    controller.derivative_gain_constant = kd
    controller.accumulated_error = 0

    previous_error = 0
    previous_time = time.time()

    total_error = 0
    count = 0

    start_time = time.time()

    while time.time() - start_time < 10:
        detectstate_front, frontsensor_dist, _, _, _ = robot.read_sensor("front_sensor")
        detectstate_left, _, _, _, _ = robot.read_sensor("left_sensor")
        
        current_time = time.time()
        delta_time = current_time - previous_time

        if detectstate_front and frontsensor_dist < 0.5:
            robot.turn_corner(detectstate_front)

        elif detectstate_left:
            _, current_dist, _, _, _ = robot.read_sensor("left_sensor")
            error = desired_distance - current_dist
            controller.accumulated_error  += error
            delta_error = error - previous_error

            proportional_output = controller.proportional_response(error)
            integral_output = controller.integral_response()
            derivative_output = controller.derivative_response(delta_error, delta_time)

            action = proportional_output + derivative_output #+ integral_output
            base_speed = 0.7 
            left_wheel_speed = base_speed + action
            right_wheel_speed = base_speed - action 

            robot.move(left_wheel_speed, right_wheel_speed)

            total_error += abs(error)
            count += 1
        sim.wait(0.05)
    sim.stopSimulation()
    time.sleep(1)

    average_error = total_error / max(count, 1)
    print(f"error: {average_error}")
    return average_error

problem = {
    "obj_func": objective_func, 
    "bounds": FloatVar(lb=[0.0, 0.0, 0.0], ub=[2.0, 0.1, 2.0]),
    "minmax": min,
    }

optimizer = PSO.OriginalPSO(epoch=1, pop_size=10)
best_solution, best_fitness = optimizer.solve(problem)
optimizer.logger.disabled = True
print("Best PID parameters:", best_solution)
print("With fitness (average error):", best_fitness)