import rosbag
import matplotlib.pyplot as plt
import math

# Kinematic model to update (x, y, theta)
def update_odometry(x, y, theta, v, omega, dt):
    scaling_factor = 10.0  # Apply scaling to the velocity
    v *= scaling_factor
    
    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt
    theta_new = theta + omega * dt
    return x_new, y_new, theta_new

# Initialize variables
x, y, theta = 0.0, 0.0, 0.0  # initial position and orientation
times = []
positions_x = []
positions_y = []

# Load the rosbag
bag = rosbag.Bag('/home/kirilio3/Desktop/412/ros_project_exercies_2/csc22911_straight_line_odometry.bag', 'r')

# Check if messages are in the bag
message_count = 0
for topic, msg, t in bag.read_messages(topics=['/csc22911/car_cmd_switch_node/cmd']):
    message_count += 1
    # Extract velocity and omega
    v = msg.v
    omega = msg.omega
    print(topic, msg)
    
    # Update odometry using the kinematic model
    dt = 0.1  # Time step (10Hz rate)
    x, y, theta = update_odometry(x, y, theta, v, omega, dt)
    
    # Print the values of x, y, and theta at each time step
    print(f"Time: {t.to_sec()}s, x: {x:.2f}, y: {y:.2f}, theta: {theta:.2f} rad")
    
    # Store the data for plotting
    times.append(t.to_sec())
    positions_x.append(x)
    positions_y.append(y)

# Close the rosbag file
bag.close()

# Check if there were any messages
if message_count == 0:
    print("No messages found in the rosbag for the topic '/cmd'.")

# Optionally, scale the positions for better visualization
positions_x = [pos  for pos in positions_x]
positions_y = [pos  for pos in positions_y]

# Plot the trajectory
plt.plot(positions_x, positions_y, label="Trajectory")
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Trajectory of Duckiebot')
plt.legend()

# Set axis limits to ensure better visualization
plt.xlim(0, 25)
plt.ylim(-10, 20)

# Display the plot
plt.show()

# Optionally, close the plot to release resources
plt.close()
