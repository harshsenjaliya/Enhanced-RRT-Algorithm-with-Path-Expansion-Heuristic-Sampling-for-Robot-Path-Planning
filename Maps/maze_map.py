import numpy as np

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
                (r31>0 and r32<0 and r33>0 and r34<0)):
                maze_map[y,x] = 2
    
    # Define square obstacles
    # Provided coordinates of centers of square obstacles
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

# Map dimensions
width, height = 6000, 3000

# Generate the map
map_result = maze_map(width, height)

# If needed to visualize the map (requires matplotlib)
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    plt.figure(figsize=(12, 6))
    plt.imshow(map_result, cmap='gray')
    plt.colorbar()
    plt.show()
