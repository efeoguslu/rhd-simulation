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

    for i in range(len(button_sets)):
        for j in range(len(time)):
            if(button[i][j] == 1) and (not button_sets[i]):
                plt.axvline(time[j], color=colors[i])
                button_sets[i] = True
            if(button[i][j] == 0) and button_sets[i]:
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

def main():

    target_folder_path = update_target_folder_path()
    dataList = prepare_data_from_logs(target_folder_path)

    button_keys = ['pothole_button_state', 'bump_button_state', 'mode_button_state']
    buttons = extract_data_by_keys(dataList, button_keys) # list of lists

    pothole_button_presses = buttons[0]
    bump_button_presses = buttons[1]

    #detect_state_changes_and_write_to_file("pothole_buttons.txt", pothole_button_presses)
    #detect_state_changes_and_write_to_file("bump_buttons.txt", bump_button_presses)
    
    # print(pothole_button_presses)

    # exit()

    time_vector = get_time(buttons[0])

    bump_button_set = False
    pothole_button_set = False
    # mode_button_set = False 

    # button_sets = [bump_button_set, pothole_button_set, mode_button_set]
    button_sets = [bump_button_set, pothole_button_set]
    # button_colors = ['red', 'gold', 'blue']
    button_colors = ['red', 'gold']

    plot_buttons_lines(time_vector, buttons, button_sets, button_colors)

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



    exit()

    ## SONRA KULLANACAKSIN: TESPİTLERİ AYIRINCA:

    # Load data from the first file
    # y1 = load_data('ulas1_active_filtered_vec.txt')

    # Load data from the second file
    # // y2 = load_data('output_signals.txt')

    bump = load_data('bump_state_signal.txt')
    pothole = load_data('pothole_state_signal.txt')

    filtered_signal = load_data('filtered_signal.txt')


    # Generate x values based on the length of y1 and y2
    # x1 = np.arange(len(y1))
    # x2 = np.arange(len(y2))
    x3 = np.arange(len(bump))
    x4 = np.arange(len(pothole))

    # Plot both datasets
    # plot_data(x1, y1, "acceleration")
    # plot_data(x2, y2, "peak detection") # Bunu şimdilik kullanma
    plot_data(time_vector, bump, "bump state")
    plot_data(time_vector, pothole, "pothole state")

    # plt.title('Title of the Graph')
    plt.legend()

    # Show the plot
    plt.show()












if __name__ == "__main__":
    main()