import numpy as np
import os
import sys
import pickle
import ruamel.yaml
import itertools
import time
import keyboard
import traceback
from subprocess import Popen, PIPE, STDOUT, DEVNULL
from capture import OdometryCapture
from tqdm import tqdm

yaml = ruamel.yaml.YAML()
yaml.preserve_quotes = True

def interpolate_and_average(timestamps_to_interpolate, values_to_interpolate, timestamps, values):

    if timestamps.size == 0 or timestamps is None:
        return values_to_interpolate

    below = np.searchsorted(timestamps_to_interpolate, timestamps, side='left') - 1
    above = np.searchsorted(timestamps_to_interpolate, timestamps, side='right')

    valid_indices = (below >= 0) & (above < len(timestamps_to_interpolate))
    below = below[valid_indices]
    above = above[valid_indices]
    interpolated = (
        (values_to_interpolate[above] - values_to_interpolate[below]) /
        (timestamps_to_interpolate[above] - timestamps_to_interpolate[below])
    ) * (timestamps[valid_indices] - timestamps_to_interpolate[below]) + values_to_interpolate[below]

    return_values = np.zeros_like(values)
    return_values[valid_indices] = (interpolated + values[valid_indices]) / 2
    return_values[~valid_indices] = values[~valid_indices]
    return return_values

def call_command(nrepeats, iteration_dict, iteration_bar, command_bar, roscore_process):
    last_results = ""
    this_repeats = nrepeats
    for i in range(this_repeats):
        # Call command
        command_bar.reset()
        capture = OdometryCapture(loop=iteration_dict['use_loop'])
        iteration_bar.set_description("Iteration %d" % (i+1) + last_results)
        params  = capture.run(command_bar, roscore_process)
        
        
        iteration_bar.update(1)
        if params is None or params[0].size == 0 or params[1].size == 0 or params[2].size == 0 or params[3].size == 0:
            last_results = " - Last Failed..." + "1:{},2:{},3:{},4:{}".format(params[0].size, params[1].size, 
                                                                              params[2].size, params[3].size) if params is not None else "NONE"
            iteration_dict["fail_number"] += 1
            this_repeats += 1
            if this_repeats > nrepeats + 5:
                iteration_bar.set_description("Iteration %d" % (i+1) + last_results)
                return iteration_dict
            continue

        # Save the measurements
        timestamps, cpu_usage, memory_usage, poses = params
        iteration_dict["cpu_usage"]     = interpolate_and_average(timestamps, cpu_usage, iteration_dict["timestamps"], iteration_dict["cpu_usage"])
        iteration_dict["memory_usage"]  = interpolate_and_average(timestamps, memory_usage, iteration_dict["timestamps"], iteration_dict["memory_usage"])
        poses_from_dict                 = iteration_dict["poses"]
        if poses_from_dict.size == 0:
            poses_from_dict = poses
        else:
            poses_from_dict[:, 0]           = interpolate_and_average(timestamps, poses[:, 0], iteration_dict["timestamps"], poses_from_dict[:, 0])
            poses_from_dict[:, 1]           = interpolate_and_average(timestamps, poses[:, 1], iteration_dict["timestamps"], poses_from_dict[:, 1])
            poses_from_dict[:, 2]           = interpolate_and_average(timestamps, poses[:, 2], iteration_dict["timestamps"], poses_from_dict[:, 2])
        iteration_dict["poses"]         = poses_from_dict
        iteration_dict["timestamps"]    = interpolate_and_average(timestamps, timestamps, iteration_dict["timestamps"], iteration_dict["timestamps"])
        # Last results shows the number of poses, average cpu usage and average memory usage
        last_results = " Poses {} - CPU: {:.2f}% - Avg Memory: {:.2f}%".format(
            iteration_dict["poses"].shape[0],
            np.mean(iteration_dict["cpu_usage"]), np.mean(iteration_dict["memory_usage"])
        )
    return iteration_dict

def update_yaml(iteration_dict):

    # Check if the .yaml file exists
    config_path = './configs/test_config.yaml'
    if not os.path.isfile(config_path):
        print('The .yaml file does not exist. Exiting...')
        sys.exit()

    else:
        # Copy yaml without the first line
        with open(config_path, 'r') as f:
            lines = f.readlines()

        with open(config_path, 'w') as f:
            f.writelines(lines[1:])

        # Load the config file
        config = yaml.load(open(config_path, 'r'))

    # Update the relevant parameters in the config dictionary
    config['multiple_thread'] = iteration_dict['multiple_thread']
    config['max_cnt'] = iteration_dict['max_cnt']
    config['min_dist'] = iteration_dict['min_dist']
    config['max_solver_time'] = iteration_dict['max_solver_time']
    config['max_num_iterations'] = iteration_dict['max_num_iterations']

    # Write the updated config back to the YAML file
    with open(config_path, 'w') as yaml_file:
        yaml.dump(config, yaml_file)

    # Add the first line back
    with open(config_path, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write("%YAML:1.0\n" )
        f.write(content)
        f.close()

def main():
    
    iterations_list = []
    iteration_dict_default = {
        'iteration_index'       : 0,
        'multiple_thread'       : 1,
        'max_cnt'               : 80,
        'min_dist'              : 10,
        'max_solver_time'       : 0.02,
        'max_num_iterations'    : 8,
        'use_loop'              : False,
        "timestamps"            : np.array([]),
        "cpu_usage"             : np.array([]),
        "memory_usage"          : np.array([]),
        "poses"                 : np.empty((0, 3)),
        "fail_number"           : 0
    }
    iteration_dict = iteration_dict_default.copy()
    # Check if some previous iteration has been done and load it, in pickle format
    save_state_path = './scripts/odometry_tests/results/save_state.pkl'
    if os.path.isfile(save_state_path):
        iterations_list, iteration_dict = pickle.load(open(save_state_path, 'rb'))

    # Define the grid search
    thread_list             = [1, 2]
    cnt_list                = [80, 110, 150, 200]
    dist_list               = [30]
    solver_time_list        = [0.02, 0.04, 0.06]
    num_iterations_list     = [8, 12, 16]
    loop_list               = [False]

    start_index     = iteration_dict['iteration_index']
    # Create an iterator over all combinations
    combinations    = itertools.product(thread_list, cnt_list, dist_list, solver_time_list, num_iterations_list, loop_list)
    combinations    = list(combinations)
    combinations.sort()
    combinations    = combinations[start_index + 1:] if len(iterations_list) > 0 else combinations
    nrepeats        = 5
    
    print("-" * 50)
    print("Starting testing with {} combinations, from {}".format(len(combinations) + start_index, start_index))
    roscore_log = open("./scripts/odometry_tests/logs/roscore.log", "w+")
    rqt_log     = open("./scripts/odometry_tests/logs/rqt.log", "w+")
    # Run ROSCORE
    roscore_process = Popen("roscore", stdout=roscore_log, stderr=STDOUT)
    rqt_process     = Popen("rqt", stdout=rqt_log, stderr=STDOUT)
    time.sleep(5)
    # Create progress bars
    disable_bars = False
    ended = False
    main_bar        = tqdm(total=len(combinations) + start_index, position = 0, desc = "Testing:", initial=start_index, 
                    bar_format='{desc:70} -> {percentage:3.0f}%|{bar:50}{r_bar}', dynamic_ncols=True, colour='green', disable=disable_bars)
    iteration_bar   = tqdm(total=nrepeats, position = 1, desc = "Iterations", 
                    bar_format='{desc:70} -> {percentage:3.0f}%|{bar:50}{r_bar}', dynamic_ncols=True, colour='blue', disable=disable_bars)
    command_bar     = tqdm(total=100, position = 2, desc = "Odometry Process", 
                    bar_format='{desc:70} -> {percentage:3.0f}%|{bar:50}{r_bar}', dynamic_ncols=True, colour='red', unit_scale=True, disable=disable_bars)
    #print("\n\n\n")
    try:
        
        for index, combination in enumerate(combinations):
            if roscore_process.poll() is None:
                index += start_index
                multiple_thread, max_cnt, min_dist, max_solver_time, max_num_iterations, use_loop = combination
                # Update iteration_dict with new parameters
                iteration_dict['iteration_index']       = index
                iteration_dict['multiple_thread']       = multiple_thread
                iteration_dict['max_cnt']               = max_cnt
                iteration_dict['min_dist']              = min_dist
                iteration_dict['max_solver_time']       = max_solver_time
                iteration_dict['max_num_iterations']    = max_num_iterations
                iteration_dict['use_loop']              = use_loop
                main_bar.set_description("Params: {}".format(combination))
                # Update the .yaml file with the new parameters
                update_yaml(iteration_dict)

                # Call command and save the outputs and measures
                iteration_bar.reset()
                iteration_dict = call_command(nrepeats, iteration_dict, iteration_bar, command_bar, roscore_process)
                # Save the iteration data to the list
                iterations_list.append(iteration_dict.copy())

                # Save iterations_list to pickle file
                pickle.dump((iterations_list, iteration_dict), open(save_state_path, 'wb'))

                # Reset iteration_dict for the next iteration
                iteration_dict = iteration_dict_default.copy()

                # Update progress bar or print progress
                main_bar.update(1)
            else:
                raise Exception("Roscore process ended unexpectedly")
        ended = True

    except Exception as e:
        print("Saving progress...")
        update_yaml(iteration_dict_default)
        print("#" * 50)
        traceback.print_exc()
        print("#" * 50)
    
    main_bar.close()
    iteration_bar.close()
    command_bar.close()
    roscore_process.terminate()
    rqt_process.terminate()
    roscore_log.close()
    rqt_log.close()
    # dump results if ended
    if ended:
        pickle.dump((iterations_list, iteration_dict), open("./scripts/odometry_tests/results/finalized_{}.pkl".format(time.strftime("%Y%m%d-%H%M%S")), 'wb'))
        # Delete the save_state file if it exists
        if os.path.isfile(save_state_path):
            os.remove(save_state_path)

if __name__ == '__main__':
    main()

