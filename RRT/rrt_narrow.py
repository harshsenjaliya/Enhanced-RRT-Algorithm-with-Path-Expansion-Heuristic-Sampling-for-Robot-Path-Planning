import numpy as np
import matplotlib.pyplot as plt
import time

class Node:
    def __init__(self, point, parent=None):
        self.point = np.array(point)
        self.parent = parent
        self.cost = 0 if parent is None else parent.cost + np.linalg.norm(self.point - parent.point)

def plot_map(width, height):
    obstacle_space = np.full((height, width), 0)
    for y in range(0, height):
        for x in range(0, width):
            # Define obstacles and walls based on simplified conditions
            # Obstacle checks
            if ((475 <= y <= 900 and 0 <= x <= 275) or
                (875 <= y <= 900 and 275 <= x <= 625) or
                (75 <= y <= 900 and 625 <= x <= 900) or
                (0 <= y <= 425 and 0 <= x <= 275) or
                (0 <= y <= 825 and 325 <= x <= 575) or
                (0 <= y <= 25 and 575 <= x <= 900) or
                (y == 0 or x == 0 or y == 899 or x == 899)):
                obstacle_space[y, x] = 1
    return obstacle_space

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

def rrt_star(start, goal, obstacle_space, num_iterations=10000, step_size=25, threshold=20, goal_bias=0.4):
    if obstacle_space[int(start[1]), int(start[0])] == 1 or obstacle_space[int(goal[1]), int(goal[0])] == 1:
        raise ValueError("Start or goal is inside an obstacle.")
    print("Start and goal are clear of obstacles.")
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
            if i % 100 == 0:
                plot_tree(nodes, ax, goal, obstacle_space)
            if np.linalg.norm(new_node.point - goal) <= threshold:
                print("Goal Reached")
                final_path = backtrack_path(new_node)
                for idx in range(len(final_path) - 1):
                    ax.plot([final_path[idx][0], final_path[idx+1][0]], [final_path[idx][1], final_path[idx+1][1]], 'g-')
                plot_tree(nodes, ax, goal, obstacle_space)
                return nodes, final_path
    print("No path found after completion of iterations.")
    return nodes, None

if __name__ == '__main__':
    width, height = 900, 900
    obstacle_space = plot_map(width, height)
    start = (50, 450)
    goal = (850, 50)
    start_time = time.time()
    nodes, path = rrt_star(start, goal, obstacle_space)
    end_time = time.time()
    plt.show()
    print(f"Time Taken: {end_time - start_time} seconds")
    if path:
        print("Path length:", len(path))
