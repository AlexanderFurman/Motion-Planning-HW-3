import argparse
from MapEnvironment import MapEnvironment
from RRTMotionPlanner import RRTMotionPlanner
from RRTInspectionPlanner import RRTInspectionPlanner
import numpy as np

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map_mp.json', help='Json file name containing all map information')
    parser.add_argument('-task', '--task', type=str, default='mp', help='choose from mp (motion planning) and ip (inspection planning)')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1', help='edge extension mode')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.05, help='probability to draw goal vertex')
    parser.add_argument('-coverage', '--coverage', type=float, default=0.5, help='percentage of points to inspect (inspection planning)')
    args = parser.parse_args()

    # prepare the map
    planning_env = MapEnvironment(json_file=args.map, task=args.task)

    # setup the planner
    if args.task == 'mp':
        planner = RRTMotionPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob)
    elif args.task == 'ip':
        planner = RRTInspectionPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, coverage=args.coverage)
    else:
        raise ValueError('Unknown task option: %s' % args.task);

    # execute plan
    # plan = planner.plan()

    # plan = [np.array([0.29026598, 0.37271903, 0.41952193, 0.71161095]), np.array([0.29026598, 0.37271903, 0.41952193, 0.87025345]), np.array([0.29026598, 0.37271903, 0.41952193, 1.04460431]), np.array([0.30822079, 0.32169569, 0.77156153, 1.14120411]), np.array([0.02463325, 0.12999273, 0.61732615, 0.81099277]), np.array([0.01235372, 0.08872267, 0.49428101, 0.32828593]), np.array([ 0.32746313, -0.15540204,  0.20170615,  0.37423892]), np.array([ 0.61077512, -0.53596493,  0.04594407,  0.39962353]), np.array([ 0.78, -0.78,  0.  ,  0.  ])]
    # plan.reverse()
    # plan = np.array(plan)
    planning_env = MapEnvironment(json_file='map_ip.json', task='ip')
    planner = RRTInspectionPlanner(planning_env=planning_env, ext_mode='E2', goal_prob=0.2, coverage=0.75)
    plan = planner.plan()
    # Visualize the final path.

    planner.planning_env.visualize_plan(plan)