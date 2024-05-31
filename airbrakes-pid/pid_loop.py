import rclpy
import csv

from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

class PID(Node):


    def __init__(self):
        super().__init__('pid_loop')    #Constructor
        self.Kp = 250       # Proportional gain
        self.Ki = 5         # Integral gain
        self.Kd = 0        # Derivative gain
        # Subscribe to kalmann filter output
        self.subscription = self.create_subscription(Pose,'sensor_fusion/state',
        self.listener_callback,10)
        self.sample_rate = 100  # samples/s
        timer_period = 1/self.sample_rate    # clock period for PID output
        self.delta_t = timer_period
        self.publisher_ = self.create_publisher(Float64, 'motor_steps', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription  # prevent unused variable warning
        self.integral_sum = 0   # Integral of error signal
        self.ddz = 0            # Deriv of error signal
        self.current_alt = 0    # Current altitude
        self.current_err = 0    # Current error
        self.current_time = 0   # Running time (since starting pid)
        self.prev_err = 0       # Error signal in previous sample (for deriv)
        self.alt_start = 1690   # Starting altitude of PID loop
        self.ideal_traj_t = []  # Ideal trajectory time series
        self.ideal_traj_z = []  # Ideal trajectory altitude values
        self.max_steps = float(4962)   # Maximum possible steps
        self.max_rpm = 120
        self.max_step_speed = float(self.max_rpm*200/60)   # Steps/s
        self.prev_resp = 0
        self.xnm1 = 0 
        self.ynm1 = 0
        self.current_output = 0 
        # Populate timeseries for ideal trajectory
        filepath = 'src/py_pubsub/py_pubsub'
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

    # Interrupt for PID loop timer
    def timer_callback(self):
        # TEMPORARY - simulated error signal
        self.current_alt = self.find_closest_point(self.current_time, self.sim_traj_t, self.sim_traj_z)[1]
        self.run_pid()
        pass

    # When message received
    def listener_callback(self, msg):
        self.get_logger().info('Kalmann input: "%s"' % msg.position.z)  # Log input
        self.current_alt = msg.position.z 

    # Send out the current response of the PID controller
    def run_pid(self):
        response = Float64()
        self.error()            # Calculate current error
        prev_resp = self.prev_resp
        response.data = self.Kp*self.current_err + self.Ki*self.integral_sum + self.Kd*self.ddz #Kp + Ki/s + Kds
        # Deal with "negative" value responses
        if (response.data < 0):
            response.data = float(0)
        if (response.data > self.max_steps):    # Do not go beyond maximum steps
            response.data = self.max_steps
        if (response.data > prev_resp + self.max_step_speed*self.delta_t):# If we're trying to go too fast
            response.data = prev_resp + self.max_step_speed*self.delta_t    # Don't :)
        elif (response.data < prev_resp - self.max_step_speed*self.delta_t):
            response.data = prev_resp - self.max_step_speed*self.delta_t    # Still don't :)
        # TEMPORARY: publish error response for debugging
        #response.data = float(self.current_err)
        self.publisher_.publish(response)   # Publish response  
        self.current_time += self.delta_t   # Increment time
        self.prev_resp = response.data

    # Integrate error signal 
    def integrate_signal(self):
        self.integral_sum += self.delta_t*self.current_err

    # Differentiate error signal
    def differentiate_signal(self):
        self.ddz = (self.current_err - self.prev_err)/self.delta_t
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
        return [time_lst[ind], data_lst[ind]]

    # Returns current error signal
    def error(self):
        self.prev_err = self.current_err    # Set previous error
        # Get ideal altitude at given time
        desired_alt = self.find_closest_point(self.current_time, self.ideal_traj_t, self.ideal_traj_z)[1]   
        self.current_err = -(desired_alt - self.current_alt)/3.281 # <- CONVERT TO M
        return self.current_err


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
