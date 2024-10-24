import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, point, parent=None):
        self.point = np.array(point)
        self.parent = parent
        self.cost = 0 if parent is None else parent.cost + np.linalg.norm(self.point - parent.point)

def plot_map(width, height, num_squares, square_size):
    np.random.seed(42)  # Fix the random seed for reproducibility
    narrow_map = np.full((height, width), 0)  # Initialize the map grid with zeros
    squares = []
    for _ in range(num_squares):
        # Generate a fixed random position for each square
        x = np.random.randint(0, width - square_size + 1)
        y = np.random.randint(0, height - square_size + 1)
        squares.append((x, y))
    
    for y in range(height):
        for x in range(width):
            obstacle = False
            for (sx, sy) in squares:
                # Define half-plane equations for the square
                in_square = (sx <= x < sx + square_size) and (sy <= y < sy + square_size)
                if in_square:
                    obstacle = True
                    break
            
            if obstacle:
                narrow_map[y, x] = 1  # Mark square interiors as obstacle
            
            # Setting up the walls
            if y == 0 or y == height - 1 or x == 0 or x == width - 1:
                narrow_map[y, x] = 1  # Mark the edges as walls

    return narrow_map

def check_collision(line_seg, obstacle_space):
    x0, y0 = map(int, line_seg[0])
    x1, y1 = map(int, line_seg[1])
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = np.sign(x1 - x0), np.sign(y1 - y0)
    err = dx - dy
    while True:
        if obstacle_space[y0, x0] == 1:
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

def extend(from_node, to_point, step_size, obstacle_space):
    path = []
    current_node = from_node
    while True:
        direction = np.array(to_point) - current_node.point
        distance = np.linalg.norm(direction)
        if distance < step_size:
            return path, Node(to_point, current_node)
        direction = (direction / distance) * step_size
        new_point = current_node.point + direction
        if check_collision((current_node.point, new_point), obstacle_space):
            return path, None
        current_node = Node(new_point, current_node)
        path.append(current_node)
    return path, current_node

def plot_tree(nodes, ax, goal, obstacle_space):
    ax.imshow(obstacle_space, cmap='Greys', origin='lower')
    for node in nodes:
        if node.parent:
            ax.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'b-')
    ax.plot(goal[0], goal[1], 'gx')
    plt.pause(0.01)

def backtrack_path(goal_node):
    path = []
    current_node = goal_node
    while current_node is not None:
        path.append(current_node.point)
        current_node = current_node.parent
    path.reverse()
    return path

def rrt_star_connect(start, goal, obstacle_space, num_iterations=3500, step_size=25, threshold=20, goal_bias=0.4):
    if obstacle_space[int(start[1]), int(start[0])] == 1 or obstacle_space[int(goal[1]), int(goal[0])] == 1:
        raise ValueError("Start or goal is inside an obstacle.")

    nodes = [Node(start)]
    fig, ax = plt.subplots()
    ax.set_xlim(0, obstacle_space.shape[1])
    ax.set_ylim(0, obstacle_space.shape[0])
    ax.imshow(obstacle_space, cmap='Greys', origin='lower')

    for i in range(num_iterations):
        if np.random.rand() < goal_bias:
            random_point = goal
        else:
            random_point = np.random.rand(2) * obstacle_space.shape[::-1]
        
        if obstacle_space[int(random_point[1]), int(random_point[0])] == 1:
            continue
        
        nearest_node = nearest(nodes, random_point)
        new_node = steer(nearest_node, random_point, step_size)
        if not check_collision((nearest_node.point, new_node.point), obstacle_space):
            nodes.append(new_node)
            extend_path, final_node = extend(new_node, goal, step_size, obstacle_space)
            nodes.extend(extend_path)
            if final_node and np.linalg.norm(final_node.point - goal) <= threshold:
                print("Goal Reached")
                final_path = backtrack_path(final_node)
                for idx in range(len(final_path) - 1):
                    ax.plot([final_path[idx][0], final_path[idx+1][0]], [final_path[idx][1], final_path[idx+1][1]], 'g-')
                plot_tree(nodes, ax, goal, obstacle_space)
                return nodes, final_path
    return nodes, None

if __name__ == '__main__':
    width, height, num_squares, square_size = 900, 900, 75, 30
    obstacle_space = plot_map(width, height, num_squares, square_size)
    start = (50, 850)
    goal = (850, 50)
    start_time = time.time()
    nodes, path = rrt_star_connect(start, goal, obstacle_space)
    end_time = time.time()
    plt.show()
    print(f"Time Taken: {end_time - start_time} seconds")
    if path:
        print("Path length:", len(path))