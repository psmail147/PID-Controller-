# Created by Philip Smith                           %                                                                               
#       Email: psmail147@gmail.com                  %                                                    
#       Github: https://github.com/psmail147        %                         
# --------------------------------------------------%
import random
import math

class Robot:
    """
    A class to control a Pioneer P3DX robot in simulation.
    
    Provides basic motion commands, sensor readings, and autonomous wandering behavior
    using a Braitenberg-inspired obstacle avoidance algorithm.
    """

    def __init__(self, sim):
        """
        Initialize the robot by obtaining handles for actuators and proximity sensors.
        
        Args:
            sim: Simulation environment interface providing object access and control methods.
        """
        self.sim = sim
        self.pioneer_handle = sim.getObject('/PioneerP3DX')
        self.left_actuator = sim.getObject('/leftMotor')
        self.right_actuator = sim.getObject('/rightMotor')
        self.front_sensor = sim.getObject("/proximitySensor[0]")
        self.left_sensor = sim.getObject("/proximitySensor[1]")
        self.right_sensor = sim.getObject('/proximitySensor[2]')
        self.rear_sensor = sim.getObject('/proximitySensor[3]')
        self.front_right = sim.getObject('/proximitySensor[4]')
        self.front_left = sim.getObject('/proximitySensor[5]')

    def move(self, left_speed, right_speed):
        """
        Set the velocities of the left and right motors.
        
        Args:
            left_speed (float): Target speed for the left wheel.
            right_speed (float): Target speed for the right wheel.
        """
        self.sim.setJointTargetVelocity(self.left_actuator, left_speed)
        self.sim.setJointTargetVelocity(self.right_actuator, right_speed)

    def stop(self):
        """Stop the robot by setting both wheel speeds to zero."""
        self.move(0.0, 0.0)

    def get_position(self, relative_point):
        """
        Get the robot's position relative to a given reference frame.
        
        Args:
            relative_point: Handle or reference point relative to which position is measured.
        
        Returns:
            List[float]: (x, y, z) position coordinates.
        """
        return self.sim.getObjectPosition(self.pioneer_handle, relative_point)

    def get_orientation(self, relative_point):
        """
        Get the robot's orientation relative to a given reference frame.
        
        Args:
            relative_point: Handle or reference point relative to which orientation is measured.
        
        Returns:
            List[float]: (roll, pitch, yaw) orientation angles.
        """
        return self.sim.getObjectOrientation(self.pioneer_handle, relative_point)

    def read_sensor(self, sensor_name):
        """
        Read data from a specified proximity sensor.
        
        Args:
            sensor_name (str): Attribute name of the sensor (e.g., 'front_sensor').
        
        Returns:
            Tuple: (detection_state, distance, detected_point, detected_object_handle, normal_vector)
        """
        sensor = getattr(self, sensor_name)
        state, distance, point, detected_obj_handle, normal_vec = self.sim.readProximitySensor(sensor)
        return state, distance, point, detected_obj_handle, normal_vec

    def random_wander(self):
        """
        Perform random wandering behavior with obstacle avoidance.
        
        Uses a simple Braitenberg vehicle approach where sensor activations 
        adjust wheel speeds based on proximity to obstacles.
        """
        sensors = [
            ("front_sensor", self.front_sensor),
            ("rear_sensor", self.rear_sensor),
            ("left_sensor", self.left_sensor),
            ("front_left", self.front_left),
            ("front_right", self.front_right),
            ("right_sensor", self.right_sensor)
        ]
        
        detect_distance = 0.8   # Maximum distance to start reacting to obstacles
        max_distance = 0.2       # Minimum distance to normalize reaction strength
        detect_list = [0, 0, 0, 0, 0, 0]

        # Sensor-to-wheel influence mappings (Braitenberg coefficients)
        braitenberg_left = [-0.2, -0.4, -0.6, -0.8, -1.0, 0.0]  
        braitenberg_right = [-1.6, -1.4, -1.2, -1.0, -0.8, 0.0]
        
        base_speed = 2.0  # Base forward speed
        
        while True:
            # Update detection list based on sensor readings
            for i in range(len(sensors)):
                state, dist, _, _, _ = self.read_sensor(sensors[i][0])
                if state > 0 and dist < detect_distance:
                    dist = max(dist, max_distance)  # Clamp distance to avoid division by zero
                    detect_list[i] = 1 - ((dist - max_distance) / (detect_distance - max_distance))
                else:
                    detect_list[i] = 0

            # Compute adjusted wheel speeds
            left_speed = base_speed
            right_speed = base_speed
            for i in range(6):
                left_speed += braitenberg_left[i] * detect_list[i]
                right_speed += braitenberg_right[i] * detect_list[i]
            
            self.move(left_speed, right_speed)

    def turn_corner(self, detectstate_front):
        """
        Turn the robot in place until the front sensor no longer detects an obstacle.
        
        Args:
            detectstate_front (bool): Initial detection state of the front sensor.
        """
        while detectstate_front:
            detectstate_front, frontsensor_dist, _, _, _ = self.read_sensor("front_sensor")
            self.move(0.2, -0.2)  # Rotate in place to the right


#from coppeliasim_zmqremoteapi_client import RemoteAPIClient
#client = RemoteAPIClient()
#sim = client.require('sim')
#sim.startSimulation()
#r = Robot(sim)
#r.random_wander()