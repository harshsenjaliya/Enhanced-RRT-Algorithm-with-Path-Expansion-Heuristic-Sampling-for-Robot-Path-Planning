import numpy as np
import matplotlib.pyplot as plt

def general_map(width, height):
    # Generating General Space
    general_map = np.full((height, width), 0)

    for y in range(0, height):
        for x in range(0, width):
            # Wall Obstacles
            w1 = (y) - 5
            w2 = (y) - (height - 5)
            w3 = (x) - 5
            w4 = (x) - (width - 5)

            # Rectangle 1 Obstacle
            r11 = (x) - 100
            r12 = (y) - 100
            r13 = (x) - 175
            r14 = (y) - height

            # Rectangle 2 Obstacle
            r21 = (x) - 275
            r22 = (y) - 0
            r23 = (x) - 350
            r24 = (y) - 400

            # Hexagon Obstacle
            h6 = (y) + 0.58 * (x) - 475.098
            h5 = (y) - 0.58 * (x) + 275.002
            h4 = (x) - 779.9
            h3 = (y) + 0.58 * (x) - 775.002
            h2 = (y) - 0.58 * (x) - 24.92
            h1 = (x) - 520.1

            # Block Obstacle
            t1 = (x) - 900
            t2 = (x) - 1020
            t3 = (x) - 1100
            t4 = (y) - 50
            t5 = (y) - 125
            t6 = (y) - 375
            t7 = (y) - 450

            if ((h6 > 0 and h5 > 0 and h4 < 0 and h3 < 0 and h2 < 0 and h1 > 0) or
                (r11 > 0 and r12 > 0 and r13 < 0 and r14 < 0) or
                (r21 > 0 and r23 < 0 and r24 < 0 and r22 > 0) or
                (t1 > 0 and t2 < 0 and t4 > 0 and t5 < 0) or
                (t2 > 0 and t3 < 0 and t4 > 0 and t7 < 0) or
                (t6 > 0 and t7 < 0 and t1 > 0 and t2 < 0) or
                (w1 < 0) or (w2 > 0) or (w3 < 0) or (w4 > 0)):
                general_map[y, x] = 2

    return general_map

def narrow_map(width, height):
    narrow_map = np.full((height, width), 0)
    for y in range(0, height):
        for x in range(0, width):
            # Walls
            w1 = (y) - 1
            w2 = (x) - 1
            w3 = (y) - (height - 1)
            w4 = (x) - (width - 1)

            # Obstacle 1
            r11 = (y) - 475
            r12 = (y) - height
            r13 = (x)
            r14 = (x) - 275
            
            # Obstacle 2
            r21 = (y) - 875
            r22 = (y) - height
            r23 = (x) - 275
            r24 = (x) - 625
            
            # Obstacle 3
            r31 = (y) - 75
            r32 = (y) - height
            r33 = (x) - 625
            r34 = (x) - width
            
            # Obstacle 4
            r41 = (y)
            r42 = (y) - 425
            r43 = (x)
            r44 = (x) - 275
            
            # Obstacle 5
            r51 = (y)
            r52 = (y) - 825
            r53 = (x) - 325
            r54 = (x) - 575
            
            # Obstacle 6
            r61 = (y)
            r62 = (y) - 25
            r63 = (x) - 575
            r64 = (x) - width
            
            if ((r11 > 0 and r12 < 0 and r13 > 0 and r14 < 0) or
                (r21 > 0 and r22 < 0 and r23 > 0 and r24 < 0) or
                (r31 > 0 and r32 < 0 and r33 > 0 and r34 < 0) or
                (r41 > 0 and r42 < 0 and r43 > 0 and r44 < 0) or
                (r51 > 0 and r52 < 0 and r53 > 0 and r54 < 0) or
                (r61 > 0 and r62 < 0 and r63 > 0 and r64 < 0) or
                (w1 < 0) or (w2 < 0) or (w3 > 0) or (w4 > 0)):
                narrow_map[y, x] = 2
                
    return narrow_map

def cluster_map(width, height, num_squares, square_size):
    cluster_map = np.full((height, width), 0)
    np.random.seed(69)  # Fixed the random seed for reproducibility
    
    for _ in range(num_squares):
        x = np.random.randint(0, width - square_size)
        y = np.random.randint(0, height - square_size)
        cluster_map[y:y+square_size, x:x+square_size] = 2
    
    # Setting up the walls
    cluster_map[0, :] = cluster_map[-1, :] = 2  # Horizontal walls
    cluster_map[:, 0] = cluster_map[:, -1] = 2  # Vertical walls

    return cluster_map

def maze_map(width, height):
    # Create an empty map
    maze_map = np.full((height, width), 0)
    for y in range(0, height):
        for x in range(0, width):
            # Walls
            w1 = (y) - 1
            w2 = (x) - 1
            w3 = (y) - (height - 1)
            w4 = (x) - (width - 1)
            
            # Obstacle 1
            r11 = (y) - 1200
            r12 = (y) - 3000
            r13 = (x) - 1500
            r14 = (x) - 1520
            
            # Obstacle 2
            r21 = (y) 
            r22 = (y) - 1800
            r23 = (x) - 3000
            r24 = (x) - 3020
            
            # Obstacle 3 
            r31 = (y) - 1200
            r32 = (y) - 3000
            r33 = (x) - 4500
            r34 = (x) - 4520
            
            if ((r11>0 and r12<0 and r13>0 and r14<0) or
                (r21>0 and r22<0 and r23>0 and r24<0) or
                (r31>0 and r32<0 and r33>0 and r34<0) or
                (w1 < 0) or (w2 < 0) or (w3 > 0) or (w4 > 0)):
                maze_map[y,x] = 2
    
    # Defining square obstacles by providing coordinates of centers of square obstacles
    square_centers = [
        (750, 550), (2250, 550), (3750, 550),
        (5250, 550), (2250, 2250), (3750, 2250), (5250, 2250)
    ]
    square_size = 300  # Size of the square
    
    # Draw square obstacles
    for center in square_centers:
        cx, cy = center
        # Compute top-left corner of square given its center
        top_left_x = cx - square_size // 2
        top_left_y = cy - square_size // 2
        # Draw the square on the map
        maze_map[top_left_y:top_left_y + square_size, top_left_x:top_left_x + square_size] = 2
    
    # Define and draw boundary walls
    maze_map[0, :] = 2  # Top boundary
    maze_map[-1, :] = 2  # Bottom boundary
    maze_map[:, 0] = 2  # Left boundary
    maze_map[:, -1] = 2  # Right boundary

    return maze_map       

# Plotting the maps
plt.figure(figsize=(18, 12))

plt.subplot(1, 4, 1)
plt.title("General Map")
plt.imshow(general_map(1200, 500), cmap='gray', origin='lower')
# plt.show()

plt.subplot(1, 4, 2)
plt.title("Narrow Map")
plt.imshow(narrow_map(900, 900), cmap='gray', origin='lower') 
# plt.show()

plt.subplot(1, 4, 3)
plt.title("Cluster Map")
plt.imshow(cluster_map(900, 900, 75, 30), cmap='gray', origin='lower')
# plt.show()

plt.subplot(1, 4, 4)
plt.title("Maze Map")
plt.imshow(maze_map(6000, 3000), cmap='gray', origin='lower')
plt.tight_layout()
plt.show()
