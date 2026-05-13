from controller import Robot
import math
import pygame

class MooseUR10Controller:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Deadzone threshold for stick drift
        self.deadzone = 0.1
        
        # UR10e Joint positions: [Shoulder Pan, Shoulder Lift, Elbow, Wrist 1, Wrist 2, Wrist 3]
        self.joint_positions = [0.0, -1.0, 1.5, -0.5, 0.0, 0.0]
        
        self.setup_devices()
        self.setup_pygame()

    def setup_devices(self):
        print("--- Initializing Moose + UR10e Assembly ---")
        
        # 1. MOOSE WHEELS (6-Wheel Skid Steer)
        self.left_motors = []
        self.right_motors = []
        for i in range(1, 4):
            l_motor = self.robot.getDevice(f"left motor {i}")
            r_motor = self.robot.getDevice(f"right motor {i}")
            if l_motor and r_motor:
                l_motor.setPosition(float('inf'))
                l_motor.setVelocity(0.0)
                r_motor.setPosition(float('inf'))
                r_motor.setVelocity(0.0)
                self.left_motors.append(l_motor)
                self.right_motors.append(r_motor)
        
        # 2. RECURSIVE ARM SEARCH
        # This finds joints regardless of "UR10e::" or "UR10e(1)::" prefixes
        arm_keywords = ["shoulder_pan", "shoulder_lift", "elbow_joint", "wrist_1", "wrist_2", "wrist_3"]
        self.arm_motors = [None] * 6
        
        n_devices = self.robot.getNumberOfDevices()
        for i in range(n_devices):
            device = self.robot.getDeviceByIndex(i)
            d_name = device.getName()
            
            for idx, keyword in enumerate(arm_keywords):
                if keyword in d_name:
                    device.setVelocity(1.0) # Set max velocity for joint movement
                    self.arm_motors[idx] = device
                    print(f"Matched Arm Joint: {d_name}")

        # 3. GRIPPER SEARCH
        self.left_finger = None
        for i in range(n_devices):
            device = self.robot.getDeviceByIndex(i)
            if "finger" in device.getName().lower() and "left" in device.getName().lower():
                self.left_finger = device
                print(f"Matched Gripper Finger: {device.getName()}")
                break

    def setup_pygame(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Controller: {self.joystick.get_name()} | Deadzone: {self.deadzone}")
        else:
            print("ERROR: No DualSense controller found.")
            self.joystick = None

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        # Optional: Rescale value after deadzone to keep movement smooth
        return (value - (self.deadzone if value > 0 else -self.deadzone)) / (1.0 - self.deadzone)

    def process_input(self):
        pygame.event.pump()
        if not self.joystick: return

        # --- MOOSE DRIVE (Right Stick) ---
        # Axis 3: Y-axis (Fwd/Back), Axis 2: X-axis (Turn)
        fwd = self.apply_deadzone(-self.joystick.get_axis(3))
        turn = self.apply_deadzone(-self.joystick.get_axis(2))
        
        max_speed = 6.0 
        l_speed = (fwd + turn) * max_speed
        r_speed = (fwd - turn) * max_speed
        
        for m in self.left_motors:
            m.setVelocity(l_speed)
        for m in self.right_motors:
            m.setVelocity(r_speed)

        # --- ARM CONTROL (D-Pad & Triggers) ---
        arm_sensitivity = 0.02
        
        # D-Pad for Shoulder (Pan = Left/Right, Lift = Up/Down)
        if self.joystick.get_numhats() > 0:
            hat = self.joystick.get_hat(0)
            self.joint_positions[0] += hat[0] * arm_sensitivity 
            self.joint_positions[1] += hat[1] * arm_sensitivity 
        else:
            # DualSense Button IDs: 11=Up, 12=Down, 13=Left, 14=Right
            if self.joystick.get_button(11): self.joint_positions[1] += arm_sensitivity
            if self.joystick.get_button(12): self.joint_positions[1] -= arm_sensitivity
            if self.joystick.get_button(13): self.joint_positions[0] -= arm_sensitivity
            if self.joystick.get_button(14): self.joint_positions[0] += arm_sensitivity

        # Elbow Control (Triggers)
        l2 = self.apply_deadzone((self.joystick.get_axis(4) + 1) / 2)
        r2 = self.apply_deadzone((self.joystick.get_axis(5) + 1) / 2)
        self.joint_positions[2] += (r2 - l2) * arm_sensitivity

        # Clamp arm joints to avoid gimbal lock/rotational errors
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = max(-6.28, min(6.28, self.joint_positions[i]))

        # --- GRIPPER (L1 / R1) ---
        if self.joystick.get_button(4): # L1: Open
            if self.left_finger: self.left_finger.setPosition(0.0)
        elif self.joystick.get_button(5): # R1: Close
            if self.left_finger: self.left_finger.setPosition(0.5)

    def run(self):
        while self.robot.step(self.time_step) != -1:
            self.process_input()
            
            # Update motors that were successfully found
            for i, motor in enumerate(self.arm_motors):
                if motor:
                    motor.setPosition(self.joint_positions[i])

if __name__ == "__main__":
    controller = MooseUR10Controller()
    controller.run()