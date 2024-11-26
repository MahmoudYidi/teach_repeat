import csv
import matplotlib.pyplot as plt

def read_odom_data(file_path):
    """Reads odometry data from a CSV file and returns lists of timestamps, x, and y positions."""
    timestamps = []
    x_positions = []
    y_positions = []

    with open(file_path, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header
        for row in reader:
            timestamps.append(float(row[0]))
            x_positions.append(float(row[1]))
            y_positions.append(float(row[2]))

    return timestamps, x_positions, y_positions

def plot_odom_data(file1, file2):
    """Reads two odometry CSV files and plots the x, y positions."""
    # Read odometry data from both files
    timestamps1, x1, y1 = read_odom_data(file1)
    timestamps2, x2, y2 = read_odom_data(file2)

    # Create a figure and axis for plotting
    plt.figure(figsize=(10, 6))

    # Plot the data for both odometry files
    plt.plot(x1, y1, label=f'Ground truth', color='b', marker='o', markersize=4, linestyle='-', linewidth=1)
    plt.plot(x2, y2, label=f'Repeat', color='r', marker='x', markersize=4, linestyle='--', linewidth=1)

    # Adding labels and title
    plt.title('Odometry Data Comparison')
    plt.xlabel('X Position (meters)')
    plt.ylabel('Y Position (meters)')

    # Add a legend to differentiate between the two datasets
    plt.legend()

    # Show grid
    plt.grid(True)

    # Display the plot
    plt.show()

if __name__ == "__main__":
    # Define the paths to the two odometry CSV files
    odom_data_file1 = '/home/ros/aoc_strawberry_scenario_ws/rosbag_data/odom_data.csv'  # Path to the first odometry CSV file
    odom_data_file2 = '/home/ros/aoc_strawberry_scenario_ws/rosbag_data/odom_data2.csv'  # Path to the second odometry CSV file

    # Call the function to plot the odometry data
    plot_odom_data(odom_data_file1, odom_data_file2)
