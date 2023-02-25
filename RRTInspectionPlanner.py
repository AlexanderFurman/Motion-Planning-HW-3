import numpy as np
from RRTTree import RRTTree
import time

# np.random.seed(0)
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

        self.initial_goal = [0.3, 0.15, 1.0, 0]

        #statistics
        self.stats = {}
        self.stats['E1_0.05_0.5'] = []
        self.stats['E2_0.05_0.5'] = []
        self.stats['E1_0.2_0.5'] = []
        self.stats['E2_0.2_0.5'] = []
        self.stats['E1_0.05_0.75'] = []
        self.stats['E2_0.05_0.75'] = []
        self.stats['E1_0.2_0.75'] = []
        self.stats['E2_0.2_0.75'] = []
        self.current_stats_mode = ext_mode + "_" + str(goal_prob) + "_" + str(coverage)

    def reset(self, ext_mode, goal_prob, coverage):
        self.tree = RRTTree(self.planning_env, task="ip")
        self.saved_configs = []
        self.coverage = coverage
        self.current_coverage = 0
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.current_stats_mode = ext_mode + "_" + str(goal_prob) + "_" + str(coverage)


    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()
        
        current_start_time = start_time
        # initialize an empty plan.
        plan = []

        # TODO: Task 2.4

        #add start node
        self.tree.add_vertex(self.planning_env.start, inspected_points=[])

        #iteratively add vertices to the rrt
        while self.tree.max_coverage < self.coverage:
            if (time.time() - current_start_time > 30):
                self.reset(self.ext_mode, self.goal_prob, self.coverage)
                self.tree.add_vertex(self.planning_env.start, inspected_points=[])
                current_start_time = time.time()

            # new_config = np.array([np.random.uniform(0, 2*np.pi) for _ in range(self.planning_env.robot.dim)])
            new_config = np.array([np.random.uniform(0, np.pi/2) for _ in range(self.planning_env.robot.dim)])
            new_config[-1] = np.random.uniform(-np.pi, np.pi)
            # print(new_config)
            # print("new sample = ", new_config)

            # goal biased sample:
            if (np.random.uniform(0, 1) < self.goal_prob):
                if self.tree.max_coverage > 0:
                    # random_saved_config_idx = np.random.choice([i for i in range(len(self.saved_configs))])
                    # random_saved_config = self.saved_configs[random_saved_config_idx]
                    # config_shift = np.zeros(len(self.planning_env.robot.links))
                    # config_shift[-1] = np.random.uniform(-self.shift, self.shift) #maybe change this to take larger numbers, not uniform around zero
                    # new_config = random_saved_config + config_shift
                    max_config = self.tree.vertices[self.tree.max_coverage_id].config
                    config_shift = np.zeros(len(self.planning_env.robot.links))
                    config_shift[-1] = -self.shift
                    new_config = max_config + config_shift
                    if (new_config[-1] >= np.pi) or (new_config[-1] <= -np.pi):
                        new_config[-1] = np.random.uniform(-np.pi, np.pi)

                else:
                    new_config = self.initial_goal

            
            near_config_idx, near_config = self.tree.get_nearest_config(new_config)

            extended_config = self.extend(near_config, new_config)
            # print('SHMER extended = ', extended_config)
            if extended_config is None:
                # print('fucked config =', extended_config)
                continue
            extended_config = np.array(extended_config)
            # print(f'extended_config = {extended_config}')
            # print(f'type of extended config = {type(extended_config)}')
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
        if length < 0.001:
            return None
        step = (step_dir / length) * min(self.step_size, length)
        extended_config = near_config + step

        return np.array(extended_config)

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


    