import matplotlib.pyplot as plt
import numpy as np
import sys 
import os
from typing import List
import accelLogParser

# global variables:

default_target_folder_path = "C:\\Users\\efeog\\OneDrive\\Masaüstü\\road-hazard-detection-simulation"
sampling_rate = 75.0
fig, ax1 = plt.subplots()

def load_data(filename):
    return np.loadtxt(filename)

def plot_data(x, y, label):
    plt.plot(x, y, label=label)

def update_target_folder_path():

    arguments = sys.argv

    # Check if there are 2 or more command-line arguments
    if len(arguments) > 2:
        print("Warning: Please put your file path in \" \" and try again.")
        sys.exit()

    # Check if command-line arguments were provided
    elif len(arguments) == 2:
        # If an argument is provided, use it as the target folder path
        if not os.path.isdir(sys.argv[1]):
            print("Error: The specified folder path is not valid.")
            sys.exit()

        return sys.argv[1]
    else:
        # Otherwise, use the default target folder path
        return default_target_folder_path

def prepare_data_from_logs(target) -> List[dict]:
    logsDictList = accelLogParser.parseLinesFromLog(os.path.join(target,"allSensorLogFile.txt"))
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
    return np.arange(start = 0, step = step_sec, stop = step_sec*len(data))

def plot_buttons_lines(time, button, button_sets, colors):

    for i in range(3):
        for j in range(len(time)):
            if(button[i][j] == 1) and (not button_sets[i]):
                plt.axvline(time[j], color=colors[i])
                button_sets[i] = True
            if(button[i][j] == 0) and button_sets[i]:
                button_sets[i] = False

def main():

    target_folder_path = update_target_folder_path()
    dataList = prepare_data_from_logs(target_folder_path)

    button_keys = ['pothole_button_state', 'bump_button_state', 'mode_button_state']
    buttons = extract_data_by_keys(dataList, button_keys) # list of lists

    time_vector = get_time(buttons[0])

    bump_button_set = False
    pothole_button_set = False
    mode_button_set = False 

    button_sets = [bump_button_set, pothole_button_set, mode_button_set]
    button_colors = ['red', 'gold', 'blue']

    plot_buttons_lines(time_vector, buttons, button_sets, button_colors)



    # Load data from the first file
    # y1 = load_data('ulas1_active_filtered_vec.txt')

    # Load data from the second file
    # // y2 = load_data('output_signals.txt')

    y3 = load_data('output_state_signals.txt')

    # Generate x values based on the length of y1 and y2
    # x1 = np.arange(len(y1))
    # x2 = np.arange(len(y2))
    x3 = np.arange(len(y3))

    # Plot both datasets
    # plot_data(x1, y1, "acceleration")
    # plot_data(x2, y2, "peak detection") # Bunu şimdilik kullanma
    plot_data(time_vector, y3, "state detection")


    # Customize the plot
    plt.xlabel('X Axis Label')
    plt.ylabel('Y Axis Label')
    # plt.title('Title of the Graph')
    plt.legend()

    # Show the plot
    plt.show()












if __name__ == "__main__":
    main()