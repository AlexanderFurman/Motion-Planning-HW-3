import numpy as np
from RRTTree import RRTTree
import time

np.random.seed(0)
# np.random.seed(1)

class RRTInspectionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, coverage):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env, task="ip")

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.coverage = coverage

        self.shift = (np.pi/3)/2
        self.step_size = 0.5
        self.current_coverage = 0
        self.saved_configs = []

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 2.4

        # your stopping condition should look like this: 
        # while self.tree.max_coverage < self.coverage:

        #add start node
        self.tree.add_vertex(self.planning_env.start, inspected_points=[])

        #iteratively add vertices to the rrt
        while self.tree.max_coverage < self.coverage:
            new_config = [np.random.uniform(0, 2*np.pi) for _ in range(self.planning_env.robot.dim)]
            # print("new sample = ", new_config)
            # goal biased sample:
            if self.tree.max_coverage > 0:
                if (np.random.uniform(0, 1) < self.goal_prob):
                    # config_shift = np.zeros(len(self.planning_env.robot.links))
                    # config_shift[-1] = np.random.uniform(-self.shift, self.shift) #maybe change this to take larger numbers, not uniform around zero
                    # new_config = self.tree.vertices[self.tree.max_coverage_id].config + config_shift
                    random_saved_config_idx = np.random.choice([i for i in range(len(self.saved_configs))])
                    random_saved_config = self.saved_configs[random_saved_config_idx]
                    config_shift = np.zeros(len(self.planning_env.robot.links))
                    config_shift[-1] = np.random.uniform(-self.shift, self.shift) #maybe change this to take larger numbers, not uniform around zero
                    new_config = random_saved_config + config_shift

            
            near_config_idx, near_config = self.tree.get_nearest_config(new_config)
            extended_config = self.extend(near_config, new_config)
            # print(f"config check = {self.planning_env.config_validity_checker(extended_config)} and edge check = {self.planning_env.edge_validity_checker(near_config, extended_config)}" )
            no_collision = (self.planning_env.config_validity_checker(extended_config) and self.planning_env.edge_validity_checker(near_config, extended_config))
            # no_collision = self.planning_env.edge_validity_checker(near_config, extended_config)
            if no_collision:
                old_inspected_points = self.tree.vertices[near_config_idx].inspected_points
                current_inspected_points = self.planning_env.compute_union_of_points(old_inspected_points, self.planning_env.get_inspected_points(extended_config))
                if len(old_inspected_points) < len(current_inspected_points):
                    self.save_config(extended_config)
                extended_config_idx = self.tree.add_vertex(extended_config, inspected_points=current_inspected_points)
                edge_cost = np.linalg.norm(near_config - extended_config)
                self.tree.add_edge(near_config_idx, extended_config_idx, edge_cost)
                if self.tree.max_coverage > self.current_coverage:
                    print(f"current max coverage is {self.tree.max_coverage}")
                    self.current_coverage = self.tree.max_coverage


        plan = self.find_plan()

        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.4
        cost = 0
        for i in range(len(plan)-1):
            cost += np.linalg.norm(plan[i] - plan[i+1])
        return cost


    def extend(self, near_config, rand_config):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        # TODO: Task 2.4
        if self.ext_mode == 'E1':
            return rand_config

        step_dir = np.array(rand_config) - np.array(near_config)
        length = np.linalg.norm(step_dir)
        step = (step_dir / length) * min(self.step_size, length)
        extended_config = near_config + step

        return extended_config

    def find_plan(self):
        start_idx = self.tree.get_root_id()
        current_idx = self.tree.max_coverage_id

        plan = [self.tree.vertices[current_idx].config]

        while current_idx != start_idx:
            current_idx = self.tree.edges[current_idx]
            plan.append(self.tree.vertices[current_idx].config)
        print("plan =\n", plan)
        plan.reverse()
        return plan

    def save_config(self, config):
        self.saved_configs.append(config)


    