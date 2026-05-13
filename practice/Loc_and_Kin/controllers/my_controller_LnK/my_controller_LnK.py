from controller import Robot
import math
import pygame

class KukaController:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Link lengths (meters)
        self.L1, self.L2, self.L3 = 0.155, 0.155, 0.135
        
        # Internal State (Initial Arm Position)
        self.target_x, self.target_y, self.target_z = 0.20, 0.0, 0.10
        
        self.setup_devices()
        self.setup_pygame()

    def setup_devices(self):
        # Wheels: 0:FL, 1:FR, 2:RL, 3:RR
        self.wheels = []
        for i in range(1, 5):
            wheel = self.robot.getDevice(f"wheel{i}")
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)
            self.wheels.append(wheel)
            
        # Arm Motors
        self.arm_motors = []
        for i in range(1, 6):
            motor = self.robot.getDevice(f"arm{i}")
            motor.setVelocity(1.5)
            self.arm_motors.append(motor)

        # Gripper
        self.left_finger = self.robot.getDevice("finger::left")
        self.right_finger = self.robot.getDevice("finger::right")
        self.left_finger.setVelocity(0.05)
        self.right_finger.setVelocity(0.05)

    def setup_pygame(self):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Controller Connected: {self.joystick.get_name()}")
        else:
            print("No Controller Detected! Please connect via Bluetooth or USB.")
            self.joystick = None

    def process_input(self):
        pygame.event.pump()
        if not self.joystick:
            return

        # --- WHEELS (Right Stick) ---
        # Right Stick Y (Axis 3), Right Stick X (Axis 2)
        # Left Stick X (Axis 0) for Rotation
        fwd = -self.joystick.get_axis(3)   
        strafe = self.joystick.get_axis(2) 
        turn = self.joystick.get_axis(0)   
        
        speed = 10.0
        # Mecanum Kinematics
        self.wheels[0].setVelocity(speed * (fwd + strafe + turn)) # Front Left
        self.wheels[1].setVelocity(speed * (fwd - strafe - turn)) # Front Right
        self.wheels[2].setVelocity(speed * (fwd - strafe + turn)) # Rear Left
        self.wheels[3].setVelocity(speed * (fwd + strafe - turn)) # Rear Right

        # --- ARM (D-Pad / Arrows) ---
        arm_sensitivity = 0.006
        
        # Safe Hat Check (Prevents the crash you saw)
        if self.joystick.get_numhats() > 0:
            hat = self.joystick.get_hat(0)
            self.target_x += hat[1] * arm_sensitivity
            self.target_y -= hat[0] * arm_sensitivity
        else:
            # Fallback: Many DS5 drivers map D-Pad to buttons 11-14
            if self.joystick.get_numbuttons() >= 15:
                if self.joystick.get_button(11): self.target_x += arm_sensitivity # Up
                if self.joystick.get_button(12): self.target_x -= arm_sensitivity # Down
                if self.joystick.get_button(13): self.target_y += arm_sensitivity # Left
                if self.joystick.get_button(14): self.target_y -= arm_sensitivity # Right

        # Z-Axis Elevation (Triggers)
        # Scale triggers from [-1, 1] to [0, 1]
        l2_trigger = (self.joystick.get_axis(4) + 1) / 2
        r2_trigger = (self.joystick.get_axis(5) + 1) / 2
        self.target_z += (r2_trigger - l2_trigger) * arm_sensitivity

        # Workspace Safety Limits
        self.target_x = max(0.12, min(0.35, self.target_x))
        self.target_y = max(-0.25, min(0.25, self.target_y))
        self.target_z = max(-0.05, min(0.35, self.target_z))

        # --- GRIPPER (L1 / R1) ---
        if self.joystick.get_button(4): # L1: Release
            self.left_finger.setPosition(0.025)
            self.right_finger.setPosition(0.025)
        elif self.joystick.get_button(5): # R1: Grip
            self.left_finger.setPosition(0.0)
            self.right_finger.setPosition(0.0)

    def solve_arm_ik(self, tx, ty, tz):
        """Standard Analytic IK for the 5-DOF YouBot arm"""
        try:
            q1 = math.atan2(ty, tx)
            r = math.sqrt(tx**2 + ty**2)
            z = tz - self.L1
            d_sq = r**2 + z**2
            d = math.sqrt(d_sq)
            
            c3 = (self.L2**2 + self.L3**2 - d_sq) / (2 * self.L2 * self.L3)
            q3 = -(math.pi - math.acos(max(-1, min(1, c3))))
            
            a1 = math.atan2(z, r)
            a2 = math.acos(max(-1, min(1, (self.L2**2 + d_sq - self.L3**2) / (2 * self.L2 * d))))
            q2 = a1 + a2
            q4 = -(q2 + q3)
            
            return [q1, q2, q3, q4, 0.0]
        except Exception:
            return None

    def run(self):
        print("Controller Loop Started.")
        while self.robot.step(self.time_step) != -1:
            self.process_input()
            
            # Calculate and apply IK
            angles = self.solve_arm_ik(self.target_x, self.target_y, self.target_z)
            if angles:
                for i, pos in enumerate(angles):
                    # Motor 2 visual offset correction
                    if i == 1: 
                        pos = max(0.0, min(1.57, pos + 1.57))
                    self.arm_motors[i].setPosition(pos)

if __name__ == "__main__":
    controller = KukaController()
    controller.run()