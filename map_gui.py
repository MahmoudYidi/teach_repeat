import tkinter as tk
from tkinter import messagebox
import os
import matplotlib.pyplot as plt
import csv
from tkinter import ttk

# Function to list directories containing .db3 files
def get_map_names(directory="/home/ros/aoc_strawberry_scenario_ws/"):
    """Scans the given directory for folders that contain .db3 files."""
    map_names = []
    for folder in os.listdir(directory):
        folder_path = os.path.join(directory, folder)
        if os.path.isdir(folder_path):
            # Check if .db3 files exist in the folder
            for file in os.listdir(folder_path):
                if file.endswith(".db3"):
                    map_names.append(folder)
                    break
    return map_names

# Update map names dropdown
def update_map_name_dropdown():
    """Updates the map names dropdown with directories containing .db3 files."""
    map_names = get_map_names()
    map_name_dropdown['values'] = map_names
    if map_names:
        map_name_dropdown.set(map_names[0])  

def start_mapping():
    map_name = map_name_dropdown.get()
    record_interval = record_interval_slider.get()

    if not map_name or not record_interval:
        messagebox.showerror("Input Error", "Please enter both a map name and a record interval.")
        return

    # Build the ROS 2 action command with output_folder, record_interval, and start_map
    action_command = f"ros2 action send_goal /record_map teachrepeat/action/MapRecord '{{output_folder: \"{map_name}\", record_interval: {record_interval}, start_map: true}}'"
    os.system(action_command)
    messagebox.showinfo("Command Sent", f"Started mapping with map name '{map_name}' and record interval '{record_interval}'.")

def stop_mapping():
    map_name = map_name_dropdown.get()
    record_interval = record_interval_slider.get()

    if not map_name or not record_interval:
        messagebox.showerror("Input Error", "Please enter both a map name and a record interval.")
        return

    # Build the ROS 2 action command with output_folder, record_interval, and start_map set to false
    action_command = f"ros2 action send_goal /record_map teachrepeat/action/MapRecord '{{output_folder: \"{map_name}\", record_interval: {record_interval}, start_map: false}}'"
    os.system(action_command)
    messagebox.showinfo("Command Sent", f"Stopped mapping with map name '{map_name}' and record interval '{record_interval}'.")

def repeat_mapping():
    map_name = map_name_dropdown.get()
    record_interval = record_interval_slider.get()

    if not map_name or not record_interval:
        messagebox.showerror("Input Error", "Please enter both a map name and a record interval.")
        return

    # Build the ROS 2 action command with map_name and record_interval
    action_command = f"ros2 action send_goal /repeat_map teachrepeat/action/MapRepeat '{{output_folder: \"{map_name}\", record_interval: {record_interval}}}'"
    os.system(action_command)
    messagebox.showinfo("Command Sent", f"Repeated mapping with map name '{map_name}' and record interval '{record_interval}'.")

def read_odom_data(file_path):
    """Reads odometry data from a CSV file and returns lists of timestamps, x, and y positions."""
    timestamps = []
    x_positions = []
    y_positions = []

    try:
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            next(reader)   
            for row in reader:
                timestamps.append(float(row[0]))
                x_positions.append(float(row[1]))
                y_positions.append(float(row[2]))
    except FileNotFoundError:
        messagebox.showerror("File Error", f"File not found: {file_path}")
        return None, None, None

    return timestamps, x_positions, y_positions

def plot_odom_data():
    """Reads two odometry CSV files and plots the x, y positions."""
    map_name = map_name_dropdown.get()
    record_interval = record_interval_slider.get()

    if not map_name or not record_interval:
        messagebox.showerror("Input Error", "Please enter both a map name and a record interval.")
        return

    # Construct the paths to the CSV files using the map name and record interval
    file1 = f"/home/ros/aoc_strawberry_scenario_ws/{map_name}/odom_data.csv"
    file2 = f"/home/ros/aoc_strawberry_scenario_ws/{map_name}/odom_data2.csv"

    # Read odometry data from both files
    timestamps1, x1, y1 = read_odom_data(file1)
    timestamps2, x2, y2 = read_odom_data(file2)

    if timestamps1 is None or timestamps2 is None:
        return  

    # Create a figure and axis for plotting
    plt.figure(figsize=(10, 6))

    # Plot the data for both odometry files
    plt.plot(x1, y1, label=f'Ground truth', color='b', marker='o', markersize=4, linestyle='-', linewidth=1)
    plt.plot(x2, y2, label=f'Repeat', color='r', marker='x', markersize=4, linestyle='--', linewidth=1)

    
    plt.title('Odometry Data Comparison')
    plt.xlabel('X Position (meters)')
    plt.ylabel('Y Position (meters)')
    plt.legend()
    plt.grid(True)
    plt.show()


# GUI Setup


root = tk.Tk()
root.title("ROS 2 Map Command GUI")
root.geometry("400x500")

# Map name dropdown (with .db3 files in folders)
map_name_label = tk.Label(root, text="Map Name:")
map_name_label.pack(pady=5)

# Create dropdown for map names
map_name_dropdown = ttk.Combobox(root, width=30)
map_name_dropdown.pack(pady=5)

# Update the map name dropdown with directories containing .db3 files
update_map_name_dropdown()

# Record interval label and slider
record_interval_label = tk.Label(root, text="Record Interval (seconds):")
record_interval_label.pack(pady=5)

# Create a slider for record interval (1.0 to 10.0)
record_interval_slider = tk.Scale(root, from_=1.0, to=10.0, orient='horizontal', resolution=0.1, length=300)
record_interval_slider.set(1.0)  # Default value is 1.0
record_interval_slider.pack(pady=5)

# Start/Stop Section
start_stop_frame = tk.LabelFrame(root, text="Start/Stop Mapping", padx=10, pady=10)
start_stop_frame.pack(padx=10, pady=10, fill="both")

# Start Mapping Button (Green)
start_button = tk.Button(start_stop_frame, text="Start Mapping", command=start_mapping, bg="green", fg="white", width=15)
start_button.pack(pady=5)

# Stop Mapping Button (Red)
stop_button = tk.Button(start_stop_frame, text="Stop Mapping", command=stop_mapping, bg="red", fg="white", width=15)
stop_button.pack(pady=5)

# Repeat Section
repeat_frame = tk.LabelFrame(root, text="Repeat Mapping", padx=10, pady=10)
repeat_frame.pack(padx=10, pady=10, fill="both")

# Repeat Mapping Button (Green)
repeat_button = tk.Button(repeat_frame, text="Repeat Mapping", command=repeat_mapping, bg="green", fg="white", width=15)
repeat_button.pack(pady=5)

# Plot Odometry Section
plot_frame = tk.LabelFrame(root, text="Plot Map", padx=10, pady=10)
plot_frame.pack(padx=10, pady=10, fill="both")

# Plot Odometry Button (Blue)
plot_button = tk.Button(plot_frame, text="Plot Map", command=plot_odom_data, bg="blue", fg="white", width=15)
plot_button.pack(pady=5)

# Start the GUI loop
root.mainloop()

