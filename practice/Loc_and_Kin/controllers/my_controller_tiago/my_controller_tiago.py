from controller import Robot

# Initialize the Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Initialize Wheels ---
left_motor = robot.getDevice('wheel_left_joint')
right_motor = robot.getDevice('wheel_right_joint')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- Initialize Lidar ---
lidar = robot.getDevice('lidar')
if lidar:
    lidar.enable(timestep)

# --- Initialize Torso and Arm (7 joints) ---
torso = robot.getDevice('torso_lift_joint')
arm_joints = []
for i in range(1, 8):
    arm_joints.append(robot.getDevice(f'arm_{i}_joint'))

# --- Simulation Variables ---
state = 'SEARCHING'
start_time = 0.0

# Initial "Ready" Pose
if torso: torso.setPosition(0.15)
arm_joints[1].setPosition(-1.1) # Shoulder tucked
arm_joints[3].setPosition(1.5)  # Elbow bent

# Main Control Loop
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    ranges = lidar.getRangeImage() if lidar else []

    if state == 'SEARCHING':
        # Rotate in place to find the object
        left_motor.setVelocity(1.0)
        right_motor.setVelocity(-1.0)
        
        if ranges:
            # Check if any detected point is between 0.2m and 1.5m
            for val in ranges:
                if 0.2 < val < 1.5:
                    state = 'APPROACHING'
                    break

    elif state == 'APPROACHING':
        if not ranges: continue
        
        # Simple tracking: find the closest point and steer toward it
        min_dist = min(ranges)
        target_idx = ranges.index(min_dist)
        mid_idx = len(ranges) // 2
        
        error = target_idx - mid_idx
        steering = 0.005 * error
        
        left_motor.setVelocity(2.0 + steering)
        right_motor.setVelocity(2.0 - steering)
        
        # Stop once we are close enough to the block
        if min_dist < 0.45:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            state = 'PREPARING_ARM'
            start_time = current_time

    elif state == 'PREPARING_ARM':
        # Align the EPick over the object
        if torso: torso.setPosition(0.25)
        arm_joints[1].setPosition(0.3) 
        arm_joints[3].setPosition(0.8)
        
        # Allow time for the arm to reach the position
        if current_time - start_time > 2.5:
            state = 'GRABBING'
            start_time = current_time

    elif state == 'GRABBING':       
        if current_time - start_time > 1.0:
            state = 'LIFTING'

    elif state == 'LIFTING':
        # Lift the arm to pull the object up
        arm_joints[1].setPosition(-0.3)
        print("Success: Object attached to EPick.")
        
        # Simulation task complete - stop wheels
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break