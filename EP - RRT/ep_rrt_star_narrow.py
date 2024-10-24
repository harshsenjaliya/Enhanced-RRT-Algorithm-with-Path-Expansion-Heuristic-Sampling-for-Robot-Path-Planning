import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, point, parent=None):
        self.point = np.array(point)
        self.parent = parent
        self.cost = 0 if parent is None else parent.cost + np.linalg.norm(self.point - parent.point)

def plot_map(width, height):
    obstacle_space = np.full((height, width), 0)
    for y in range(height):
        for x in range(width):
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
    # Bresenham's Line Algorithm for checking collision in grid space
    x0, y0 = map(int, line_seg[0])
    x1, y1 = map(int, line_seg[1])
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        if obstacle_space[y0, x0] == 1:
            return True
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return False

def nearest(nodes, point):
    return min(nodes, key=lambda node: np.linalg.norm(node.point - point))

def steer(from_node, to_point, step_size):
    direction = np.array(to_point) - from_node.point
    norm = np.linalg.norm(direction)
    if norm == 0:
        return from_node  # No need to move if the point is the same
    direction = (direction / norm) * min(step_size, norm)
    new_point = from_node.point + direction
    return Node(new_point, from_node)

def sample_random_point(obstacle_space, goal, goal_bias=0.2):
    if np.random.rand() < goal_bias:  # Bias towards the goal
        return goal
    else:
        while True:
            point = np.random.randint(0, obstacle_space.shape[1]), np.random.randint(0, obstacle_space.shape[0])
            if obstacle_space[point[1], point[0]] == 0:
                return point

def rrt_star(start, goal, obstacle_space, num_iterations=1000, step_size=30):
    nodes = [Node(start)]
    for _ in range(num_iterations):
        random_point = sample_random_point(obstacle_space, goal)
        nearest_node = nearest(nodes, random_point)
        new_node = steer(nearest_node, random_point, step_size)
        if not check_collision((nearest_node.point, new_node.point), obstacle_space):
            nodes.append(new_node)
            if np.linalg.norm(new_node.point - goal) <= step_size:
                print("Goal reached")
                return nodes, new_node
    return nodes, None

def plot(nodes, goal_node, obstacle_space):
    plt.figure(figsize=(10, 10))
    plt.imshow(obstacle_space, origin='lower', interpolation='none', cmap='gray')  # Show the obstacle map
    for node in nodes:
        if node.parent:
            plt.plot([node.point[0], node.parent.point[0]], [node.point[1], node.parent.point[1]], 'b-')  # Blue lines for tree connections
    if goal_node:
        # Trace back the path to the start from the goal node
        current = goal_node
        while current.parent:
            plt.plot([current.point[0], current.parent.point[0]], [current.point[1], current.parent.point[1]], 'r-')  # Red line for the path
            current = current.parent
    plt.plot(nodes[0].point[0], nodes[0].point[1], 'go')  # Start point in green
    plt.plot(goal[0], goal[1], 'rx')  # Goal point in red
    plt.grid(True)
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.title('RRT Star Path Planning')
    plt.show()

if __name__ == '__main__':
    width, height = 900, 900
    obstacle_space = plot_map(width, height)
    start, goal = (50, 450), (850, 50)
    nodes, goal_node = rrt_star(start, goal, obstacle_space)
    plot(nodes, goal_node, obstacle_space)
