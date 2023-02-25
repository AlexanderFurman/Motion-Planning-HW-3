import numpy as np
from RRTTree import RRTTree
import time
import math
import random

from collections import deque
from Robot import Robot
from MapEnvironment import MapEnvironment

class RRTMotionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.step_size = 0.5

        self.goal_flag = False

        #statistics
        self.stats = {}
        self.stats['E1_0.05'] = []
        self.stats['E2_0.05'] = []
        self.stats['E1_0.2'] = []
        self.stats['E2_0.2'] = []
        self.current_stats_mode = ext_mode + "_" + str(goal_prob)

    def reset(self, ext_mode, goal_prob):
        self.tree = RRTTree(self.planning_env)
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.goal_flag = False
        self.current_stats_mode = ext_mode + "_" + str(goal_prob)


    def generate_random_array(self):
        array = []
        for i in range(4):
            array.append(random.uniform(-math.pi, math.pi))
        array[0]=random.uniform(0, math.pi/2)
        #array[1]=random.uniform(-array[0], math.pi/2-array[0])
        return np.asarray(array)

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.3

        self.tree.add_vertex(self.planning_env.start)
        #update area limits
        x_range=self.planning_env.xlimit
        y_range = self.planning_env.ylimit
        #sample state:
        # for i in range(2000): #number of iterations
        while not self.goal_flag:
            if (np.random.uniform(0, 1) < self.goal_prob): #bias goal
                #take target as sample
                random_array=self.planning_env.goal
            else:
                random_array = self.generate_random_array()
            new_state=random_array
            x_near=self.tree.get_nearest_config(new_state)
            x_ext=self.extend(x_near[1],new_state)

            if np.isnan(x_ext[0]): #same node
                continue
            ans2=(MapEnvironment.edge_validity_checker(self.planning_env,x_near[1],x_ext) and self.planning_env.config_validity_checker(x_ext))
            if ans2:
                x_new_idx=self.tree.add_vertex(x_ext)
                edge_cost=self.compute_cost([x_near[1],x_ext])
                self.tree.add_edge(x_near[0],x_new_idx,edge_cost)

        #calculate plan
            if self.tree.is_goal_exists(self.planning_env.goal):
                self.goal_flag = True
                # print('goal exist')
                # plan = self.dijkstra()

        plan = self.find_plan()

        self.stats[self.current_stats_mode].append([self.compute_cost(plan), time.time()-start_time])
        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.3
        total_cost=0
        # TODO: Task 4.4
        for i in range(len(plan)-1):
            #self.planning_env.

            total_cost+=Robot.compute_distance(self.planning_env.robot,plan[i],plan[i+1])
            #self.planning_env.co
        return total_cost
        #pass

    def extend(self, near_config, rand_config):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        # TODO: Task 2.3

        # threshold = 0.3
        if self.ext_mode == 'E1':
            return rand_config

        # diff_config=rand_config-near_config
        # for i in range(len(diff_config)):
        #     if diff_config[i] > threshold:
        #         diff_config[i] = threshold
        #     if diff_config[i] < (-threshold):
        #         diff_config[i] = -threshold
        #diff_config=diff_config*0.1+near_config
        step_dir = np.array(rand_config) - np.array(near_config)
        length = np.linalg.norm(step_dir)
        step = (step_dir / length) * min(self.step_size, length)
        extended_config = near_config + step

        return extended_config

    def find_plan(self):
        start_idx = self.tree.get_root_id()
        current_idx, _ = self.tree.get_nearest_config(self.planning_env.goal)

        plan = [self.tree.vertices[current_idx].config]

        while current_idx != start_idx:
            current_idx = self.tree.edges[current_idx]
            plan.append(self.tree.vertices[current_idx].config)
        # print("plan =\n", plan)
        plan.reverse()
        return plan








        #pass
    # def dijkstra(self):
    #     '''
    #     shortest path in the tree
    #     '''
    #     srcIdx = self.tree.get_root_id()
    #     dstIdx = self.tree.get_nearest_config(self.planning_env.goal)[0]

    #     # build dijkstra
    #     #edges = self.tree.get_edges_as_states()
    #     edges=self.tree.edges
    #     nodes = self.tree.vertices# list(G.neighbors.keys())
    #     nodes_list=list(nodes.keys())
    #     dist = {node: float('inf') for node in nodes_list}
    #     prev = {node: None for node in nodes_list}
    #     dist[srcIdx] = 0

    #     while nodes_list:
    #         curNode = min(nodes_list, key=lambda node: dist[node])
    #         nodes_list.remove(curNode)
    #         if dist[curNode] == float('inf'):
    #             break

    #         neighbors = [key for key, value in edges.items() if value == curNode ]
    #         for neighbor in neighbors:
    #             newCost = dist[curNode] + nodes[neighbor].cost
    #             if newCost < dist[neighbor]:
    #                 dist[neighbor] = newCost
    #                 prev[neighbor] = curNode

    #     #path
    #     path = deque()
    #     curNode = dstIdx
    #     while prev[curNode] is not None:
    #         path.appendleft(nodes[curNode].config)
    #         curNode = prev[curNode]
    #     path.appendleft(nodes[curNode].config)
    #     return list(path)

    