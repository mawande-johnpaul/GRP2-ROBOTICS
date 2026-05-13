from controller import Robot
import numpy as np

class NumpySpot:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Physical constants
        self.L1, self.L2 = 0.11, 0.11
        
        self.leg_names = [
            ["front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor"],
            ["front left shoulder abduction motor", "front left shoulder rotation motor", "front left elbow motor"],
            ["rear right shoulder abduction motor", "rear right shoulder rotation motor", "rear right elbow motor"],
            ["rear left shoulder abduction motor", "rear left shoulder rotation motor", "rear left elbow motor"]
        ]
        
        self.motors = []
        for names in self.leg_names:
            leg_set = []
            for n in names:
                m = self.robot.getDevice(n)
                m.setVelocity(4.0) # Keep it smooth
                leg_set.append(m)
            self.motors.append(leg_set)

    def solve_ik_batch(self, x, z):
        """
        Uses NumPy to solve IK for all legs at once.
        x and z are NumPy arrays of length 4.
        """
        d_sq = x**2 + z**2
        d = np.sqrt(d_sq)
        
        # Law of Cosines for Elbow
        # We use np.clip to prevent NaN errors if math goes out of bounds
        cos_elbow = (self.L1**2 + self.L2**2 - d_sq) / (2 * self.L1 * self.L2)
        elbow_angles = -(np.pi - np.arccos(np.clip(cos_elbow, -1, 1)))
        
        # Law of Cosines for Shoulder
        alpha1 = np.arctan2(x, -z)
        alpha2 = np.arccos(np.clip((self.L1**2 + d_sq - self.L2**2) / (2 * self.L1 * d), -1, 1))
        shoulder_angles = alpha1 + alpha2
        
        return shoulder_angles, elbow_angles

    def run(self):
        while self.robot.step(self.time_step) != -1:
            t = self.robot.getTime() * 1.5
            
            # 1. Define Gait Parameters
            default_z = -0.17
            offsets = np.array([0.0, 0.5, 0.25, 0.75])
            phases = (t + offsets) % 1.0
            
            # 2. Initialize target arrays for all 4 legs
            x_targets = np.zeros(4)
            z_targets = np.full(4, default_z)

            # 3. Calculate path for all legs using NumPy conditions
            # Swing Phase (0.0 to 0.25)
            swing_mask = phases < 0.25
            p_swing = phases[swing_mask] / 0.25
            z_targets[swing_mask] = default_z + (np.sin(np.pi * p_swing) * 0.04)
            x_targets[swing_mask] = -0.03 + (p_swing * 0.06)

            # Stance Phase (0.25 to 1.0)
            stance_mask = phases >= 0.25
            p_stance = (phases[stance_mask] - 0.25) / 0.75
            x_targets[stance_mask] = 0.03 - (p_stance * 0.06)

            # 4. Solve IK in one batch
            shoulders, elbows = self.solve_ik_batch(x_targets, z_targets)

            # 5. Apply positions
            for i in range(4):
                self.motors[i][0].setPosition(0.0)
                self.motors[i][1].setPosition(shoulders[i])
                self.motors[i][2].setPosition(elbows[i])

if __name__ == "__main__":
    # Note: Ensure numpy is installed in your Webots python environment
    mover = NumpySpot()
    mover.run()