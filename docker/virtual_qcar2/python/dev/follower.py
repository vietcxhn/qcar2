import math
import numpy as np
import threading
import time
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets

class Follower:
    def __init__(self, qcar, k_steering=0.5, max_steering=0.6, max_speed=1, safe_distance=1):
        self.qcar = qcar
        self.k_steering = k_steering
        self.max_steering = max_steering
        self.max_speed = max_speed
        self.safe_distance = safe_distance
        
        self.running = False
        self.thread = None
        
        lidarPlot = pg.plot(title="LIDAR")

        squareSize = 10

        lidarPlot.setXRange(-squareSize, squareSize)

        lidarPlot.setYRange(-squareSize, squareSize)

        self.lidarData = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='r', symbolPen=None, symbolSize=2)
        self.detected = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='g', symbolPen=None, symbolSize=2)

        
    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def follow(self, point):
        
        # _, pos_leader, rot_leader, _ = leader.get_world_transform()    
        _, pos_follower, rot_follower, _ = self.qcar.get_world_transform()
        
        # Vector from follower to leader
        dx = point[0] - pos_follower[0]
        dy = point[1] - pos_follower[1]
        distance = math.hypot(dx, dy)

        # Follower heading and target direction
        follower_heading = rot_follower[2]
        target_angle = math.atan2(dy, dx)
        heading_error = target_angle - follower_heading
        heading_error = self.wrap_to_pi(heading_error)

        # Steering command
        steering_cmd = -self.k_steering * heading_error
        steering_cmd = max(-self.max_steering, min(self.max_steering, steering_cmd))

        # Speed control: Stop if too close
        if distance < self.safe_distance:
            speed_cmd = 0.0
        else:
            speed_cmd = min(self.max_speed, 0.5 * (distance - self.safe_distance))

        # Move follower
        self.qcar.set_velocity_and_request_state(
            forward=speed_cmd,
            turn=steering_cmd,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )
        
    def avoid_obstacle_lidar(self, min_distance=1, sample_points=400):
        """
        Reads LiDAR and applies basic obstacle avoidance by steering away.

        Args:
            qcar: the QCar object
            min_distance (float): distance to start avoiding
            max_turn (float): maximum steering rate
            sample_points (int): LiDAR resolution
        """
        success, angles, distances = self.qcar.get_lidar(samplePoints=sample_points)
        if not success:
            print("not success")
            return
        
        x = np.sin(angles)*distances

        y = np.cos(angles)*distances


        self.lidarData.setData(x,y)

        QtWidgets.QApplication.instance().processEvents()
        
        # Focus on front-facing region (±45°)
        fov_mask1 = np.abs(angles) < np.radians(45)
        fov_mask2 = np.abs(angles) > np.radians(315)
        fov_mask = np.logical_or(fov_mask1, fov_mask2)
        
        close_mask1 = distances < min_distance
        close_mask2 = distances > 0
        close_mask = np.logical_and(close_mask1, close_mask2)
        
        danger_mask = np.logical_and(fov_mask, close_mask)
    
        if not np.any(danger_mask):
            self.detected.setData([],[])
            print("not np.any(danger_mask)")
            return False

        print("detected")
        # Obstacle detected — steer away
        danger_angles = angles[danger_mask]
        
        #
        danger_distances = distances[danger_mask]
        x = np.sin(danger_angles)*danger_distances

        y = np.cos(danger_angles)*danger_distances
        self.detected.setData(x,y)
        #
        
        # Compute mean angle of obstacles
        mean_angle = self.wrap_to_pi(np.mean(danger_angles))

        # Turn away: if obstacle on left (angle > 0), turn right (negative)
        turn_rate = -np.clip(mean_angle, -np.radians(60), np.radians(60)) / np.radians(60) * self.max_steering

        print(self.k_steering *turn_rate)
        # Slow down near obstacles
        self.qcar.set_velocity_and_request_state(
            forward=0.6, 
            turn=self.k_steering *turn_rate,
            headlights=False,
            leftTurnSignal=False,
            rightTurnSignal=False,
            brakeSignal=False,
            reverseSignal=False
        )
        
        return True
        
    def run(self, leader):
        """Start following leader in a loop."""
        self.running = True
        while self.running:        
            if self.avoid_obstacle_lidar():
                continue
            
            _, pos_leader, _, _ = leader.get_world_transform()    
            self.follow(pos_leader)
            time.sleep(0.1)  # Small delay to simulate a realistic loop, adjust as needed

    def start(self, leader):
        """Start the follower loop in a new thread."""
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.run, args=(leader,))
            self.thread.start()

    def stop(self):
        """Stop the follower loop."""
        self.running = False
        if self.thread is not None:
            self.thread.join()