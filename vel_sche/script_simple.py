import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import cv2
import os
import webbrowser

class PathGenerator:
    def __init__(self, speed_straight, speed_turn):
        self.speed_straight = speed_straight
        self.speed_turn = speed_turn

    def generate_straight(self, start_point, length=100):
        x = np.linspace(start_point[0], start_point[0] + length, length)
        y = np.ones_like(x) * start_point[1]
        speed = np.ones_like(x) * self.speed_straight
        return x, y, speed

    def generate_left_turn(self, start_point, radius=20, num_points=100):
        angle = np.linspace(0, np.pi/2, num_points)
        x = start_point[0] + radius * np.sin(angle)   
        y = start_point[1] + radius * (1 - np.cos(angle))      
        speed = np.ones_like(x) * self.speed_turn
        return x, y, speed

class SpeedPlanner:
    def path_advanced_deceleration(self, x, y, speed):
        path_point_num = len(speed)
        dec_start_index = path_point_num  
        target_speed = speed[0]

        for i in range(path_point_num):
            if speed[i] < speed[0] - 1.0:
                dec_start_index = i
                target_speed = speed[i]
                break
        
        if dec_start_index == path_point_num:
            return speed
        else:
            dec = 0.6
            dist = 0.0
            adv_dec_dist = 3 * target_speed
            for i in range(dec_start_index - 1, -1, -1):
                dist += np.sqrt((x[i + 1] - x[i]) ** 2 + (y[i + 1] - y[i]) ** 2)
                if dist < adv_dec_dist:
                    speed[i] = target_speed
                else:
                    v = np.sqrt(2 * dec * (dist - adv_dec_dist) + target_speed ** 2)
                    speed[i] = min(speed[i], v)
            return speed

def visualize_path(x, y, speed, filename="output_with_speed_curve.mp4"):
    # Setup video writer
    height, width = 600, 800
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(filename, fourcc, 20.0, (width, height))

    fig, ax = plt.subplots(2, 1, figsize=(10, 8))
    
    for i in range(len(x)):
        ax[0].clear()
        ax[1].clear()

        # Draw path
        ax[0].plot(x, y, 'b-')
        ax[0].plot(x[:i], y[:i], 'ro')
        ax[0].set_xlim([min(x) - 10, max(x) + 10])
        ax[0].set_ylim([min(y) - 10, max(y) + 10])
        ax[0].set_title(f"Vehicle Path - Speed: {speed[i]:.2f} m/s")

        # Draw speed curve
        ax[1].plot(range(i+1), speed[:i+1], 'g-', label='Speed')
        ax[1].set_xlim([0, len(x)])
        ax[1].set_ylim([0, max(speed) + 1])
        ax[1].set_xlabel('Time Step')
        ax[1].set_ylabel('Speed (m/s)')
        ax[1].legend()
        ax[1].set_title('Speed Variation Over Time')
        
        fig.canvas.draw()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        out.write(cv2.resize(img, (width, height)))
    
    out.release()
    plt.close()

if __name__ == "__main__":
    map_reader = PathGenerator(speed_straight=10, speed_turn=5)
    speed_planner = SpeedPlanner()

    # Generate extended straight path
    x_straight, y_straight, speed_straight = map_reader.generate_straight(start_point=(0, 0), length=100)
    
    # Generate left turn path with smooth transition
    turn_start_x = x_straight[-1]
    turn_start_y = y_straight[-1]
    x_turn, y_turn, speed_turn = map_reader.generate_left_turn(start_point=(turn_start_x, turn_start_y))

    # Combine paths
    x = np.concatenate([x_straight, x_turn])
    y = np.concatenate([y_straight, y_turn])
    speed = np.concatenate([speed_straight, speed_turn])

    # Apply advanced deceleration
    speed_smoothed = speed_planner.path_advanced_deceleration(x, y, speed)

    # Visualize the path and speed curve
    visualize_path(x, y, speed_smoothed, filename="output_with_speed_curve.mp4")
