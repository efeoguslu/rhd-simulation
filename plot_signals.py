import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from typing import List
import accelLogParser

# global variables:

default_target_folder_path = "C:\\Users\\efeog\\OneDrive\\Masaüstü\\road-hazard-detection-simulation"
sampling_rate = 75.0

def load_data(filename):
    return np.loadtxt(filename)

def plot_data(x, y, label, ax=None):
    if ax is None:
        fig, ax = plt.subplots()
    ax.plot(x, y, label=label)
    ax.legend()

def update_target_folder_path():
    arguments = sys.argv

    # Check if there are 2 or more command-line arguments
    if len(arguments) > 2:
        print("Warning: Please put your file path in \" \" and try again.")
        sys.exit()

    # Check if command-line arguments were provided
    elif len(arguments) == 2:
        # If an argument is provided, use it as the target folder path
        if not os.path.isdir(arguments[1]):
            print("Error: The specified folder path is not valid.")
            sys.exit()
        return arguments[1]
    else:
        # Otherwise, use the default target folder path
        return default_target_folder_path

def prepare_data_from_logs(target) -> List[dict]:
    logsDictList = accelLogParser.parseLinesFromLog(os.path.join(target, "allSensorLogFile.txt"))
    return logsDictList

def extract_data_by_keys(list_of_dicts, keys):
    # Initialize an empty list to hold the result
    result = []
    
    # Iterate over each key in the keys list
    for key in keys:
        # Use list comprehension to extract values for the current key from all dictionaries
        values = [float(d[key]) for d in list_of_dicts if key in d]
        # Append the list of values to the result
        result.append(values)
    
    return result

def get_time(data):
    step_sec = (1.0 / sampling_rate)
    return np.arange(start=0, step=step_sec, stop=step_sec*len(data))

def plot_buttons_lines(ax, time, button, button_sets, colors):
    for i in range(len(button_sets)):
        for j in range(len(time)):
            if (button[i][j] == 1) and (not button_sets[i]):
                ax.axvline(time[j], color=colors[i])
                button_sets[i] = True
            if (button[i][j] == 0) and button_sets[i]:
                button_sets[i] = False

def detect_state_changes_and_write_to_file(name, lst):
    # Open a new text file in write mode
    with open(name, 'w') as f:
        # Initialize variables to track the previous value and the current index
        prev_value = lst[0]
        for i, value in enumerate(lst):
            # Check for a state change from 1.0 to 0.0
            if prev_value == 1.0 and value == 0.0:
                # Write the index of the state change to the file
                f.write('1\n')
            else:
                # If no state change, write 0
                f.write('0\n')
            # Update the previous value for the next iteration
            prev_value = value

def modify_bumps(input_list):
    # Iterate over the list using enumerate to get both index and value
    for i, val in enumerate(input_list):
        if val == 2:
            input_list[i] = 0  # Set it to 0 if the current value is 2
        elif val == 0:
            input_list[i] = 1  # Set it to 1 if the current value is 0
    return input_list

def modify_potholes(input_list):
    # Iterate over the list using enumerate to get both index and value
    for i, val in enumerate(input_list):
        if val == 2:
            input_list[i] = 0  # Set it to 0 if the current value is 2
        elif val == 1:
            input_list[i] = -1  # Set it to 1 if the current value is 0
    return input_list

def main():
    target_folder_path = update_target_folder_path()
    dataList = prepare_data_from_logs(target_folder_path)

    button_keys = ['pothole_button_state', 'bump_button_state']

    #for newer datasets:
    state_change_keys = ['state_bump', 'state_pothole']

    buttons = extract_data_by_keys(dataList, button_keys)  # list of lists
    state_changes = extract_data_by_keys(dataList, state_change_keys)

    pothole_button_presses = buttons[0]
    bump_button_presses = buttons[1]

    bump_states = state_changes[0]
    pothole_states = state_changes[1]

    #detect_state_changes_and_write_to_file("pothole_buttons.txt", pothole_button_presses)
    #detect_state_changes_and_write_to_file("bump_buttons.txt", bump_button_presses)
    
    time_vector = get_time(buttons[0])

    bump_button_set = False
    pothole_button_set = False

    button_sets = [bump_button_set, pothole_button_set]
    button_colors = ['red', 'gold']

    #y2 = load_data('output_state_signals.txt')
    #active_filter_output = load_data('active_filter_output.txt')
    #active_filter_output_timevector = np.arange(len(active_filter_output)) / sampling_rate

    #filtered_signal = load_data('filtered_signal.txt')
    #unfiltered_signal = load_data('unfiltered_signal.txt')
    #unfiltered_signal_timevector = np.arange(len(unfiltered_signal)) / sampling_rate

    #x2 = np.arange(len(y2)) / sampling_rate  # Convert to time scale

    compound_acceleration_vector = load_data('compound_acceleration_vector.txt')
    active_filter_output = load_data('active_filter_output.txt')
    runtime_state_change = load_data('runtime_state_deque.txt')

    fig, ax = plt.subplots()  # Create a new figure and axis

    # print(pothole_states)

    plot_data(time_vector, modify_potholes(pothole_states), "pothole", ax=ax)
    plot_data(time_vector, modify_bumps(bump_states), "bump", ax=ax)
    plot_data(time_vector, compound_acceleration_vector, "CAV", ax=ax)
    plot_data(time_vector, active_filter_output, "active filter output", ax=ax)
    plot_data(time_vector, runtime_state_change, "runtime_state_change", ax=ax)

    #plot_data(x2, y2, "peak detection", ax=ax)
    #plot_buttons_lines(ax, time_vector, buttons, button_sets, button_colors)
    #plot_data(active_filter_output_timevector, active_filter_output, "active filter output", ax=ax)
    #plot_data(unfiltered_signal_timevector, unfiltered_signal, "unfiltered signal", ax=ax)

    plt.legend()

    # Show the plot
    plt.show()

if __name__ == "__main__":
    main()