import itertools
import numpy as np
from matplotlib import pyplot as plt
from numpy.core.fromnumeric import size
from shapely.geometry import Point, LineString
#import MapEnvironment
class Robot(object):
    
    def __init__(self):

        # define robot properties
        self.links = np.array([80.0,70.0,40.0,40.0])
        self.dim = len(self.links)

        # robot field of fiew (FOV) for inspecting points, from [-np.pi/6, np.pi/6]
        self.ee_fov = np.pi/3

        # visibility distance for the robot's end-effector. Farther than that, the robot won't see any points.
        self.vis_dist = 60.0

    def compute_forward_kinematics(self, given_config):
        '''
        Compute the 2D position (x,y) of each one of the links (including end-effector) and return.
        @param given_config Given configuration.
        '''
        # TODO: Task 2.2
        # Create a list to store the positions of each link
        link_positions = []
        # Define the origin position
        origin = np.array([0.0, 0.0])
        # Compute the position of each link
        position = origin
        for i in range(len(given_config)): 
            angle=given_config[i]
            if i > 0:
                angle = sum(given_config[0:i+1]) # in case of relative angle
            # Compute the position of the current link
            position += np.array([self.links[i] * np.cos(angle),
                                  self.links[i] * np.sin(angle)])
            # Append the position to the list of link positions
            link_positions.append(position.copy())
        # Return the list of link positions
        return np.asarray(link_positions)
        #pass

    def compute_distance(self, prev_config, next_config):
        '''
        Compute the euclidean distance betweeen two given configurations.
        @param prev_config Previous configuration.
        @param next_config Next configuration.
        '''
        # TODO: Task 2.2
        # Create two LineString objects
        # a=self.compute_forward_kinematics(prev_config)
        # b=self.compute_forward_kinematics(next_config)
        #diff=b-a
        diff=next_config-prev_config
        distance=np.linalg.norm(diff)

        #line1 = LineString(self.compute_forward_kinematics(prev_config))
        #line2 = LineString(self.compute_forward_kinematics(next_config))
        # Calculate the Euclidean distance between the two LineStrings
        #distance = line1.distance(line2)
        #print(distance)
        return distance
        #pass


    def compute_ee_angle(self, given_config):
        '''
        Compute the 1D orientation of the end-effector w.r.t. world origin (or first joint)
        @param given_config Given configuration.
        '''
        ee_angle = given_config[0]
        for i in range(1,len(given_config)):
            ee_angle = self.compute_link_angle(ee_angle, given_config[i])

        return ee_angle

    def compute_link_angle(self, link_angle, given_angle):
        '''
        Compute the 1D orientation of a link given the previous link and the current joint angle.
        @param link_angle previous link angle.
        @param given_angle Given joint angle.
        '''
        if link_angle + given_angle > np.pi:
            return link_angle + given_angle - 2*np.pi
        elif link_angle + given_angle < -np.pi:
            return link_angle + given_angle + 2*np.pi
        else:
            return link_angle + given_angle
        
    def validate_robot(self, robot_positions):
        '''
        Verify that the given set of links positions does not contain self collisions.
        @param robot_positions Given links positions.
        '''
        # TODO: Task 2.2
        line = LineString(robot_positions)
        return line.is_simple
        # # Iterate over all links
        # for i in range(len(robot_positions) - 1):
        #     for j in range(i + 1, len(robot_positions)):
        #         segment_1 = robot_positions[i], robot_positions[i + 1]
        #         segment_2 = robot_positions[j], robot_positions[j + 1]
        #         if self.segments_intersect(segment_1, segment_2):
        #             #print("Self-collision detected between links", i, "and", j)
        #             return False
        # return True
        #pass

    # def segments_intersect(self,segment_1, segment_2):
    #     # # Determine the intersection point of the two segments
    #     # x1, y1 = segment_1[0]
    #     # x2, y2 = segment_1[1]
    #     # x3, y3 = segment_2[0]
    #     # x4, y4 = segment_2[1]
    #     # denominator = ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4))
    #     # if denominator == 0:
    #     #     return False
    #     # else:
    #     #     return True
    #     # Define two line segments
    #     line1 = LineString(segment_1)
    #     line2 = LineString(segment_2)
    #
    #     # Check if the lines intersect
    #     if line1.intersects(line2):
    #         return True
    #     else:
    #         return False