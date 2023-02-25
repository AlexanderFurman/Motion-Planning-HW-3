from RRTMotionPlanner import RRTMotionPlanner
from RRTInspectionPlanner import RRTInspectionPlanner
from MapEnvironment import MapEnvironment
import json
from matplotlib import pyplot as plt
import numpy as np

def create_planner(string):
    if string == 'motion':
        map = 'map_mp.json'
        task = 'mp'
        planner = RRTMotionPlanner(MapEnvironment(map, task), 'E1', 0)
    elif string == 'inspection':
        map = 'map_ip.json'
        task = 'ip'
        planner = RRTInspectionPlanner(MapEnvironment(map, task), 'E1', 0, 1)
    else:
        print('invalid string')
        return
    return planner


def get_data(planner):
    
    ext_modes = ['E1', 'E2']
    goal_probs = [0.05, 0.2]
    coverages = [0.5, 0.75]


    # ext_modes = [ 'E2']
    # goal_probs = [0.2]
    # coverages = [0.5]

    for i in range(10):
        print(f"run {i}")
        for ext_mode in ext_modes:
            for goal_prob in goal_probs:

                if isinstance(planner, RRTInspectionPlanner):
                    for coverage in coverages:
                        planner.reset(ext_mode, goal_prob, coverage)
                        planner.plan()

                else:
                    planner.reset(ext_mode, goal_prob)
                    planner.plan()

    if isinstance(planner, RRTInspectionPlanner):
        outfile = 'data_ip_improvements.json'
    else:
        outfile = 'data_mp.json'
    with open(outfile, "w") as outfile:
        json.dump(planner.stats, outfile)

def get_stats(planner, filename):
    json_file = filename
    with open(json_file, 'r') as file:
        stats = json.load(file)


    costs = {}
    times = {}
    
    
    for k,v in stats.items():
        costs[k] = []
        times[k] = []
        for pair in v:
            costs[k].append(pair[0])
            times[k].append(pair[1])

    #create cumulative distribution function

    if isinstance(planner, RRTMotionPlanner):
        time_steps = np.linspace(0,1000,500)
    else:
        time_steps = np.linspace(0,70,100)


    if isinstance(planner, RRTMotionPlanner):
        stats_time_number_successes = []

        for _,v in times.items():
            count = 0
            sub_array = []
            for i in range(len(time_steps)):
                for time_to_success in v:
                    if time_to_success <= time_steps[i]:
                        count += 1
                sub_array.append(count/10)
                count = 0
            stats_time_number_successes.append(sub_array)

        for successes in stats_time_number_successes:
            plt.plot(time_steps, successes)
        
        plt.xlabel('time [s]')
        plt.ylabel('success fraction')
        plt.legend([k for k,_ in times.items()])

        plt.show()
        for k,v in times.items():
            print(f"average time taken for {k} = {sum(v)/len(v)}")
        for k,v in costs.items():
            print(f"average cost for {k} = {sum(v)/len(v)}")

    
    else:
        stats_time_number_successes_05 = []
        stats_time_number_successes_075 = []

        for k,v in times.items():
            count = 0
            sub_array = []
            for i in range(len(time_steps)):
                for time_to_success in v:
                    if time_to_success <= time_steps[i]:
                        count += 1
                sub_array.append(count/10)
                count = 0
            if '0.5' in k:
                stats_time_number_successes_05.append(sub_array)
            else:
                stats_time_number_successes_075.append(sub_array)

        for successes in stats_time_number_successes_05:
            plt.plot(time_steps, successes)
        
        plt.xlabel('time [s]')
        plt.ylabel('success fraction')
        plt.legend([k for k,_ in times.items() if '0.5' in k])

        plt.show()

        for successes in stats_time_number_successes_075:
            plt.plot(time_steps, successes)
        
        plt.xlabel('time [s]')
        plt.ylabel('success fraction')
        plt.legend([k for k,_ in times.items() if '0.75' in k])

        plt.show()

        for k,v in times.items():
            print(f"average time taken for {k} = {sum(v)/len(v)}")
        for k,v in costs.items():
            print(f"average cost for {k} = {sum(v)/len(v)}")


if __name__=='__main__':
    planner = create_planner('inspection')
    # get_data(planner)
    get_stats(planner, 'data_ip_improvements.json')








    # filename = 'data_ip_fail.json'
    # json_file = filename
    # with open(json_file, 'r') as file:
    #     stats = json.load(file)

    # for k in list(stats):
    #     old_key = k + '_0.5'
    #     new_key = k + '_0.75'
    #     stats[old_key] = stats[k][0:10]
    #     stats[new_key] = stats[k][10:]
    #     del stats[k]

    # # print(stats)
    # outfile = 'data_ip.json'
    # with open(outfile, "w") as outfile:
    #     json.dump(stats, outfile)

    # with open('data_ip.json', 'r') as inputfile:
    #     stats = json.load(inputfile)

    # for k,v in stats.items():
    #     print(f"length of stats[{k}] = {len(v)}")
    


    