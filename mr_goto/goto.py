import rclpy
import numpy as np
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg._odometry import Odometry 
from tf_transformations import euler_from_quaternion
class MRGoto(Node):

    def __init__(self):
        super().__init__('goto')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.declare_parameter('mode', 'demo')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.cmd = Twist()
        self.turned = True
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_callback()
        #GoTo
        self.ground_x = -7.0
        self.ground_y = -7.0
        self.ground_angle = 45
        self.cell_size = 0.1
        self.visited = []
        self.explored = {}
        self.chosen = Cell(-1,-1,-1, 45.0,-1)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.callback_laser,
            10)
        self.subscription 
        self.sub_ground_truth_ = self.create_subscription(
        Odometry,
        "ground_truth",
        self.callback_ground_truth,
        10
        )

    def timer_callback(self):
        self.param_mode = self.get_parameter('mode').get_parameter_value().string_value
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Publishing: "{0}, {1}"'.format(self.cmd.linear.x, self.cmd.angular.z))

    #GoTo
    def callback_ground_truth(self,msg):
        #TODO: REPLACE WITH LOCALIZATION
        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w))
        self.ground_angle = math.degrees(yaw)
        self.ground_x = msg.pose.pose.position.x
        self.ground_y = msg.pose.pose.position.y


    def callback_laser(self, msg: LaserScan):
        #GoTo
        if(self.param_mode == 'plan'):
            self.plan(msg)
        else: 
            self.cmd = Twist()
            self.get_logger().info('Unknown mode value "%s"' % self.param_mode)

    #GoTo
    def plan(self,msg:LaserScan):
        self.blocked = []
        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        nr_of_scans = len(msg.ranges)
        self.cmd.angular.z = 0.0
        self.cmd.linear.x = 0.0
        x = round(self.ground_x,1)
        y = round(self.ground_y,1)
        a = self.ground_angle
        goal_x = self.get_parameter('x').get_parameter_value().double_value
        goal_y = self.get_parameter('y').get_parameter_value().double_value
        current_cell = Cell(round(x,1),round(y,1),math.dist([x,y],[goal_x,goal_y]),self.round_to_closest(a,[0,45,-45,180,90,-90,135,-135],45), -1)
        self.turned = abs(self.chosen.deg - self.ground_angle) < 0.01 

        goal_cell = Cell(round(goal_x,1),round(goal_y,1),0,0,-1)
        if goal_cell.x == current_cell.x and goal_cell.y == current_cell.y:
            self.get_logger().info('At goal')
            self.cmd.angular.z = 0.0
            self.cmd.linear.x = 0.0
        else:
            #Generate neighbourhood
            neighbourhood = [Cell(round(x,1)+self.cell_size,round(y,1)+self.cell_size,math.dist([round(x,1)+self.cell_size,round(y,1)+self.cell_size],[goal_x,goal_y]) ,45.0,1),
                            Cell(round(x,1)-self.cell_size,round(y,1)+self.cell_size,math.dist([round(x,1)-self.cell_size,round(y,1)+self.cell_size],[goal_x,goal_y]) ,135.0,7),
                            Cell(round(x,1)+self.cell_size,round(y,1)-self.cell_size,math.dist([round(x,1)+self.cell_size,round(y,1)-self.cell_size],[goal_x,goal_y]) ,-45.0,3),
                            Cell(round(x,1)-self.cell_size,round(y,1)-self.cell_size,math.dist([round(x,1)-self.cell_size,round(y,1)-self.cell_size],[goal_x,goal_y]) ,-135.0,5),
                            Cell(round(x,1)+self.cell_size,round(y,1),math.dist([round(x,1)+self.cell_size,round(y,1)],[goal_x,goal_y]) ,0.0,2),
                            Cell(round(x,1)-self.cell_size,round(y,1),math.dist([round(x,1)-self.cell_size,round(y,1)],[goal_x,goal_y]) ,180.0,5),
                            Cell(round(x,1),round(y,1)+self.cell_size,math.dist([round(x,1),round(y,1)+self.cell_size],[goal_x,goal_y]) ,90.0,0),
                            Cell(round(x,1),round(y,1)-self.cell_size,math.dist([round(x,1),round(y,1)-self.cell_size],[goal_x,goal_y]) ,-90.0,4)]
            #Increase cost of cells in direction of obstacles and decrease cost of cells away
            #TODO does not work correctly
            blocked = False
            for i in range(nr_of_scans):
                if msg.ranges[i] < 1.5:
                    blocked = True
                    angle = current_cell.deg + (135 - (270-i))
                    angle = (angle + 180) % 360 - 180

                    angle = self.round_to_closest(angle,[0,45,-45,180,90,-90,135,-135],10)
                    for cell in neighbourhood:
                        if (cell.x,cell.y,cell.deg) in self.explored:
                            cell = self.explored[cell.x,cell.y,cell.deg] 
                        if cell.deg == angle and not cell.penalised and cell.distance > 1:
                            cell.penalised = True
                            self.visited.append((cell.x,cell.y,cell.deg))
            #Get neighbourhood if explored
                if self.turned and blocked:
                    blocked = False
                    for i in range (-15,16):
                        blocked = msg.ranges[round(nr_of_scans/2) + i] < 1.
                        if blocked:
                            break
                    if not blocked and self.chosen.deg in [0,45,-45,180,90,-90,135,-135]:
                        c = [cell for cell in neighbourhood if cell.deg == self.chosen.deg and cell.distance > 1.]
                        if len(c) > 0:
                            if c[0] not in self.explored:
                                self.explored[(c[0].x,c[0].y,c[0].deg)] = c[0]

                            self.explored[(c[0].x,c[0].y,c[0].deg)].distance -= (self.explored[(c[0].x,c[0].y,c[0].deg)].distance * 0.00025)

            for i in range(8):
                cell = neighbourhood[i]
                if (cell.x, cell.y, cell.deg) not in self.explored:
                    self.explored[cell.x,cell.y, cell.deg] = cell
                #If already encountered, get cost value 
                neighbourhood[i] = self.explored[cell.x,cell.y, cell.deg]
            sorted_neighbourhood = sorted(neighbourhood, key=lambda cell: cell.distance)

            #Pick best node that is not blocked
            blocked = False
            for cell in sorted_neighbourhood:
                
                if not (cell.x,cell.y,cell.deg) in self.visited and self.turned:
                    self.chosen = cell
                    break

            #Turned, move towards goal cell if not blocked, else add to visited(blocked)
            if abs(self.chosen.deg - self.ground_angle) < 0.01:
                for i in range (-15,16):
                    if(msg.ranges[round(nr_of_scans/2) + i] < 1.5 - (abs(i))/50) and self.chosen.distance > 1.5 - (abs(i))/50 :
                        self.get_logger().info('Blocked')

                        blocked = True
                if not blocked:
                    self.cmd.linear.x = 0.2
                else:
                    self.visited.append((self.chosen.x,self.chosen.y,cell.deg))
                #Turn towards goal angle
            elif not self.turned:
                self.cmd.linear.x = 0.0
                angle_difference = self.chosen.deg - self.ground_angle

                # Adjust the angle difference to be within the range of -180 to 180 degrees
                if angle_difference > 180:
                    angle_difference -= 360
                elif angle_difference < -180:
                    angle_difference += 360

                # Calculate the turning direction
                if angle_difference < 0:
                    # Turn counterclockwise
                    self.cmd.angular.z = clamp(angle_difference * 0.05, -0.4, 0)
                else:
                    # Turn clockwise
                    self.cmd.angular.z = clamp(angle_difference * 0.05, 0, 0.4)

                

        #GoTo
    def round_to_closest(self,alpha, angles, tolerance):
        closest = None
        min_diff = float('inf')

        for num in angles:
            diff = abs(alpha - num)
            if diff < min_diff and diff < tolerance:

                closest = num
                min_diff = diff

        return closest

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MRGoto()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

class Cell:
    def __init__(self,x,y,cost,deg, index):
        self.x = x
        self.y = y
        self.distance = cost
        self.open = True
        self.deg = deg
        self.index = index
        self.penalised = False