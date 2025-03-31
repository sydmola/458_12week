import time
import roslibpy

# ROS Connection Details
ip = '192.168.8.104'
port = 9012
robot_name = 'foxtrot'

# Establish connection to ROS
ros = roslibpy.Ros(host=ip, port=port)
ros.run()
print(f"Connected to ROS: {ros.is_connected}")

# Topics for IR sensors and robot velocity
ir_topic = roslibpy.Topic(ros, f'/{robot_name}/ir_intensity', 'irobot_create_msgs/IrIntensityVector')
cmd_vel_topic = roslibpy.Topic(ros, f'/{robot_name}/cmd_vel', 'geometry_msgs/msg/Twist')

# Proportional gain for steering control
Kp = 0.035  

# Thresholds
desired_distance = 50
right_drop_threshold = 10  # Significant drop in right IR sensor value
sharp_turn_factor = 0.01    # Scaling for sharper turns

# Variable to track previous right IR sensor reading
prev_right_distance = None
prev_time = time.time()

def callback_ir(message):
    """Processes IR sensor data to navigate through walls and sharp corners."""
    global prev_right_distance, prev_time

    if 'readings' in message:
        ir_readings = message['readings']

        if len(ir_readings) == 7:
            # Extract IR sensor values
            left_distance = ir_readings[0].get('value', 0)   # Leftmost sensor
            right_distance = ir_readings[6].get('value', 0)  # Rightmost sensor

            left = ir_readings[1].get('value', 0)
            left_middle = ir_readings[2].get('value', 0)
            front_distance = ir_readings[3].get('value', 0)  # Front sensor
            right_middle = ir_readings[4].get('value', 0)
            right = ir_readings[5].get('value', 0)

            # Validate readings
            if left_distance is None or right_distance is None:
                print("Invalid readings, skipping control update.")
                return

            # Default values
            linear_x = 0.3  # Default speed
            angular_z = -Kp * ((0.5 * left_distance + 0.4 * left + 0.1 * left_middle) - 
                               (0.5 * right_distance + 0.4 * right + 0.1 * right_middle))  # Normal steering

            # --- 🕒 Detect Right IR Sensor Drop Over 0.4 Seconds ---
            current_time = time.time()
            right_drop = 0

            if prev_right_distance is not None and (current_time - prev_time >= 0.2):
                right_drop = prev_right_distance - right_distance  # Compute change in right IR reading

                if right_drop > right_drop_threshold:  # Sharp drop detected
                    print(f"⚠️ Sharp Right Turn Detected! (Right IR Drop: {right_drop})")

                    # Adaptive sharp turn logic
                    angular_z = min(-0.5, max(-3.0, -(Kp + sharp_turn_factor) * right_drop))  # Scaled sharp right turn
                    linear_x = max(0.15, 0.3 - (right_drop * 0.05))  # Slow down adaptively

                # Update previous right distance **only when a large drop occurs**
                prev_right_distance = right_distance
                prev_time = current_time
            else:
                prev_right_distance = right_distance  # Store initial reading

            # --- 🛑 FRONT OBSTACLE AVOIDANCE ---
            if any(dist > desired_distance for dist in [front_distance, right_middle, left_middle, left, right]):  
                print("🚧 Obstacle Ahead! Adjusting...")

                # Reduce speed adaptively
                linear_x = max(0.05, 0.3 - (front_distance * 0.05))  

                # Adaptive turning based on obstacle position
                if front_distance > desired_distance:  # Obstacle directly ahead
                    angular_z = -1.0 if left_middle > right_middle else 1.0  # Turn away from the closer side
                    linear_x = max(0.05, 0.3 - (front_distance * 0.05)) 
                elif left_middle > desired_distance:  # Obstacle on the left
                    angular_z = -1.0  # Turn right
                    linear_x = max(0.05, 0.3 - (left_middle * 0.05)) 
                elif right_middle > desired_distance:  # Obstacle on the right
                    angular_z = 1.0  # Turn left
                    linear_x = max(0.05, 0.3 - (right_middle * 0.05)) 
                else:
                    angular_z = angular_z
                    linear_x = linear_x


            # Create and publish velocity command
            twist = roslibpy.Message({
                'linear': {'x': linear_x, 'y': 0, 'z': 0},
                'angular': {'x': 0, 'y': 0, 'z': angular_z}
            })
            cmd_vel_topic.publish(twist)

            # Print sensor readings and control values
            print(f"Front: {front_distance:.2f} | Left: {left_distance:.2f} | Right: {right_distance:.2f} | "
                  f"Right Drop: {right_drop:.2f} | "
                  f"Angular Z: {angular_z:.2f} | Speed: {linear_x:.2f}")

        else:
            print(f"Warning: Expected 7 IR readings, but received {len(ir_readings)}")
    else:
        print("No IR readings received.")

# Subscribe to IR topic
ir_topic.subscribe(callback_ir)

# Keep running until interrupted
try:
    while ros.is_connected:
        time.sleep(1)
except KeyboardInterrupt:
    print("Disconnecting from ROS...")
    ir_topic.unsubscribe()
    cmd_vel_topic.unadvertise()
    ros.close()
    print("Disconnected.")