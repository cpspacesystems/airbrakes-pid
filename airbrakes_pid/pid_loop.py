from enum import Enum
import csv

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from sensor_msgs.msg import NavSatFix


class State(Enum):
    COAST = 0
    AIRBRAKE = 1
    APOGEE = 2

def feet_to_meters(feet):
    return feet / 3.281

def meters_to_feet(meters):
    return meters * 3.281



class PID(Node):
    def __init__(self):
        super().__init__('pid_controller')    
        filepath = 'src/airbrakes-pid/airbrakes_pid'

        # Hyperparameters TODO set as parameters
        self.TRIGGER_ALT = feet_to_meters(1_690)   # Starting altitude of PID loop
        self.MAX_ALT = feet_to_meters(5_000)     # our target maximum altitude
        self.EPSILON = 2.0  # if we are descending for this many seconds, stop airbrake

        self.SAMPLE_RATE = 100  # samples/s
        self.Kp = 250       # Proportional gain
        self.Ki = 5         # Integral gain
        self.Kd = 3        # Derivative gain

        self.MAX_STEPS = 4962   # Maximum possible steps
        MAX_RPM = 120  # determined experimentally (limits speed output can change)  
        self.MAX_STEP_SPEED = MAX_RPM*200/60   # Steps/s

        # Initialization
        self.integral_sum = 0    # Integral of error signal
        self.ddz = 0             # Deriv of error signal
        self.current_alt = 0     # Current altitude
        self.current_max_alt = 0  # The highest altitude we've reached this flight
        self.max_alt_time = 0     # The time we reached our highest altitude
        self.current_err = 0     # Current error
        self.current_time = 0   # Running time (since starting pid)
        self.starting_time = 0  # epoch time considered 0 for PID controller
        self.prev_err = 0        # Error signal in previous sample (for deriv)
        self.ideal_traj_t = []   # Ideal trajectory time series
        self.ideal_traj_z = []   # Ideal trajectory altitude values
        self.prev_resp = 0       # limits the rate of change 
        self.current_output = 0 
        #self.xnm1 = 0 
        #self.ynm1 = 0

        self.phase = State.COAST
        self.on = False

        # Subscribe to kalmann filter output
        self.create_subscription(NavSatFix, 'altimeter/fused', self.altimeter_callback, 10)

        self.DELTA_T = 1/self.SAMPLE_RATE    # clock period for PID output
        self.publisher_ = self.create_publisher(Int64, 'motor_steps', 10)
        self.timer = self.create_timer(self.DELTA_T, self.timer_callback)


        # Populate timeseries for ideal trajectory
        with open(filepath +'/Traj_0_degrees.csv', 'r', newline='') as ideal_traj_data:
            reader = csv.reader(ideal_traj_data)
            for row in reader:
                self.ideal_traj_t.append(float(row[0]))
                self.ideal_traj_z.append(float(row[1]))

        # TEMPORARY - gets simulated output to develop error signal
        self.sim_traj_t = []
        self.sim_traj_z = []
        with open(filepath + '/Traj_resp_0_degrees.csv', 'r', newline='') as simulated_traj:
            reader = csv.reader(simulated_traj)
            for row in reader:
                self.sim_traj_t.append(float(row[0]))
                self.sim_traj_z.append(float(row[1]))
    
        self.get_logger().info('[STATE COAST] Ready! waiting...')  

    # When message received
    def altimeter_callback(self, msg: NavSatFix):
        self.current_alt = msg.altitude 

    # Interrupt for PID loop timer
    def timer_callback(self):
        # perform sate transitions
        if self.phase == State.COAST and self.current_alt >= self.TRIGGER_ALT:
            self.phase = State.AIRBRAKE
            # initialize time for PID controller
            self.current_time = 0
            seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
            self.starting_time = seconds + (nanoseconds / 10**9)

        elif self.phase == State.AIRBRAKE: 
            # update time
            seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
            #self.current_time = (seconds + (nanoseconds / 10**9)) - self.starting_time
            self.current_time += self.DELTA_T

            if self.current_alt >= self.current_max_alt:
                self.current_max_alt = self.current_alt
                self.max_alt_time = self.current_time

            self.run_pid()

            # check for end of airbrake phase
            time_descending = self.current_time - self.max_alt_time
            if (self.current_alt >= self.MAX_ALT or time_descending > self.EPSILON):
                self.phase = State.APOGEE
                self.lock_break_open()
                self.get_logger().info('[STATE APOGEE] finished!')  
        

    # Send out the current response of the PID controller
    def run_pid(self):
        response = Int64()
        self.current_err = self.error()            # Calculate current error
        prev_resp = self.prev_resp
        response.data = int(self.Kp*self.current_err + self.Ki*self.integral_sum + self.Kd*self.ddz) #Kp + Ki/s + Kds
        # Deal with "negative" value responses

        # clamping motor output 
        if response.data < 0:
            response.data = 0
        if response.data > self.MAX_STEPS:    # Do not go beyond maximum steps
            response.data = self.MAX_STEPS
        if response.data > prev_resp + self.MAX_STEP_SPEED*self.DELTA_T:# If we're trying to go too fast
            response.data = int(prev_resp + self.MAX_STEP_SPEED*self.DELTA_T)    # Don't :)
        elif response.data < prev_resp - self.MAX_STEP_SPEED*self.DELTA_T:
            response.data = int(prev_resp - self.MAX_STEP_SPEED*self.DELTA_T)    # Still don't :)

        # TEMPORARY: publish error response for debugging
        #response.data = float(self.current_err)
        self.publisher_.publish(response)   # Publish response  
        self.get_logger().info(f'[STATE AIRBRAKE] mapping altitude {meters_to_feet(self.current_alt):.2f}ft -> motor angle {response.data}')  # Log input
        self.prev_resp = response.data

    # locks the break in the open position once we have reached apogee
    def lock_break_open(self):
        msg = Int64()
        msg.data = self.MAX_STEPS
        self.publisher_.publish(msg)

    # Integrate error signal 
    def integrate_signal(self):
        self.integral_sum += self.DELTA_T*self.current_err

    # Differentiate error signal
    def differentiate_signal(self):
        self.ddz = (self.current_err - self.prev_err)/self.DELTA_T
        self.prev_err = self.current_err
    
    # TODO: make faster by ignoring parts of the list that have already passed
    # Finds the closest time in input file 
    # Returns data point associated with closest time
    def find_closest_point(self, time, time_lst, data_lst):
        closest_time = time_lst[0]
        ind = 0
        for num in time_lst:
            if abs(num - time) < abs(closest_time - time):
                closest_time = num
            if (num > time):
                break
            if (ind < len(time_lst)-1):
                ind += 1
        return (time_lst[ind], data_lst[ind])

    # Returns current error signal
    def error(self):
        self.prev_err = self.current_err    # Set previous error
        # Get ideal altitude at given time
        desired_alt = feet_to_meters(self.find_closest_point(self.current_time, self.ideal_traj_t, self.ideal_traj_z)[1])
        return -(desired_alt-self.current_alt) 


def main(args=None):
    rclpy.init(args=args)
    pid = PID()
    rclpy.spin(pid)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
