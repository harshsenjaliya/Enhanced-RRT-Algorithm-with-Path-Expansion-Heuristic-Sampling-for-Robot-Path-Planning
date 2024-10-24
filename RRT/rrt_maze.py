import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, point, parent=None):
        self.point = np.array(point)
        self.parent = parent
        self.cost = 0 if parent is None else parent.cost + np.linalg.norm(self.point - parent.point)

def plot_map(width, height):
    obstacle_space = np.full((height, width), 0)  # No obstacle: 0
    
    # Define rectangular obstacles
    obstacle_space[1200:3000, 1500:1520] = 1  # Obstacle 1
    obstacle_space[0:1800, 3000:3020] = 1     # Obstacle 2
    obstacle_space[1200:3000, 4500:4520] = 1  # Obstacle 3

    # Square obstacles
    square_centers = [(750, 550), (2250, 550), (3750, 550), (5250, 550), (2250, 2250), (3750, 2250), (5250, 2250)]
    square_size = 300
    for cx, cy in square_centers:
        tl_x = cx - square_size // 2
        tl_y = cy - square_size // 2
        obstacle_space[tl_y:tl_y + square_size, tl_x:tl_x + square_size] = 1
    
    # Boundary walls
    obstacle_space[0, :] = 1
    obstacle_space[-1, :] = 1
    obstacle_space[:, 0] = 1
    obstacle_space[:, -1] = 1

    return obstacle_space

def check_collision(line_seg, obstacle_space):
    x0, y0 = map(int, line_seg[0])
    x1, y1 = map(int, line_seg[1])
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = np.sign(x1 - x0), np.sign(y1 - y0)
    err = dx - dy
    while True:
        if obstacle_space[y0, x0] == 1:  # Check if it hits an obstacle
            return True
        if (x0 == x1) and (y0 == y1):
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return False

def nearest(nodes, point):
    return min(nodes, key=lambda node: np.linalg.norm(node.point - point))

def steer(from_node, to_point, step_size):
    direction = np.array(to_point) - from_node.point
    norm = np.linalg.norm(direction)
    direction = (direction / norm) * min(step_size, norm)
    new_point = from_node.point + direction
    return Node(new_point, from_node)

def plot_tree(nodes, ax, goal, obstacle_space):
    ax.imshow(obstacle_space, cmap='Greys', origin='lower')  # Show obstacle map
    for node in nodes:
        if node.parent:
            ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'b-')
    # ax.plot([node.point[0] for node in nodes], [node.point[1] for node in nodes], 'ro')
    ax.plot(goal[0], goal[1], 'gx')
    plt.pause(0.01)

def backtrack_path(goal_node):
    path = []
    current_node = goal_node
    while current_node is not None:
        path.append(current_node.point)
        current_node = current_node.parent
    path.reverse()  # Reverse to start from the initial point to the goal
    return path

def rrt_star(start, goal, obstacle_space, num_iterations=5500, step_size=100, threshold=20, goal_bias=0.4):
    if obstacle_space[int(start[1]), int(start[0])] == 1 or obstacle_space[int(goal[1]), int(goal[0])] == 1:
        raise ValueError("Start or goal is inside an obstacle.")

    nodes = [Node(start)]
    fig, ax = plt.subplots()
    ax.set_xlim(0, obstacle_space.shape[1])
    ax.set_ylim(0, obstacle_space.shape[0])
    ax.imshow(obstacle_space, cmap='Greys', origin='lower')  # Initial plot of the obstacle space

    for i in range(num_iterations):
        if np.random.rand() < goal_bias:  # With goal_bias probability, sample the goal
            random_point = goal
        else:
            random_point = np.random.rand(2) * obstacle_space.shape[::-1]
        
        if obstacle_space[int(random_point[1]), int(random_point[0])] == 1:
            continue
        
        nearest_node = nearest(nodes, random_point)
        new_node = steer(nearest_node, random_point, step_size)
        if not check_collision((nearest_node.point, new_node.point), obstacle_space):
            nodes.append(new_node)
            if i % 100 == 0:
                # plot_tree(nodes, ax, goal, obstacle_space)
                pass 
            if np.linalg.norm(new_node.point - goal) <= threshold:
                print("Goal Reached")
                final_path = backtrack_path(new_node)
                for idx in range(len(final_path) - 1):
                    ax.plot([final_path[idx][0], final_path[idx+1][0]], [final_path[idx][1], final_path[idx+1][1]], 'g-')
                plot_tree(nodes, ax, goal, obstacle_space)
                return nodes, final_path
    return nodes, None

if __name__ == '__main__':
    width, height = 6000, 3000
    obstacle_space = plot_map(width, height)
    start = (100, 1500)
    goal = (5900, 1500)
    start_time = time.time()
    nodes, path = rrt_star(start, goal, obstacle_space)
    end_time = time.time()
    plt.show()
    
    print(f"Time Taken: {end_time - start_time} seconds")
    if path:
        print("Path length:", len(path))
