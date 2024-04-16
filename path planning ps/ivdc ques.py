import matplotlib.pyplot as plt
import math as np
import random as rd

# Node class definition
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


def node_2_node_distance(node1, node2):
# This function [Assumes nodes are in 2D plane] defenition is to compute the euclidean distance between two nodes.
    return np.sqrt(((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2))

def check_collision_free(new_node, obstacles, obstacle_radius):
# This is a collision checking function. By default it assumes True, ie. no collision.
    for obstacle in obstacles:
        distance = node_2_node_distance(new_node, obstacle)
        if distance < obstacle_radius:
            return False  # Collision
    return True  # Default -> collision free

def get_nearest_node(tree, point):
    nearest_node(tree, point):
    nearest_node = tree[0]
    min_distance = node_2_node_distance(nearest_node, point)
    for node in tree:
         distance = node_2_node_distance(node, point)
         if distance < min_distance:
              min_distance = distance
              nearest_node = node
    return nearest_node

def move_node_2_node(from_node, to_node, max_distance):

        distance = node_2_node_distance(from_node, to_node)
        if distance <= max_distance:
             return to_node
        else: 
             th= np.atan2(to_node.y - from_node.y, to_node.x - from_node.y)
             new_x = from_node.x + max_distance * np.cos(th)
             new_y = from_node.y + max_distance * np.sin(th)
             return Node(new_x, new_y)

def rewire_tree(tree, new_node, max_distance, obstacles, obstacle_radius):
     for node in tree:
          if node_2_node_distance(node, new_node) < obstacle_radius and \
          check_collision_free(node,new_node, obstacles):
               if new_node.cost + node_2_node_distance(new_node, node) < node.cost :
                    node.parent = new_node.cost = new_node.cost + node_2_node_distance(new_node, node)

def rrt_star(start, goal, x_range, y_range, obstacles, max_iter=1000, max_distance=0.4, obstacle_radius=0.2):

    tree=[start]
    start.cost = 0
    x_max = max(x_range)
    y_max= max(y_range)
    for i in range(max_iter):
         if rd.random()< 0.1:
              sampled_point = goal 
    else:
        sampled_x = rd.random() * x_max
        sampled_y = rd.random() * y_max
        sampled_point = Node(sampled_x,sampled_y)
    nearest_node = get_nearest_node(tree, sampled_point)
    new_node = move_node_2_node(nearest_node, sampled_point, max_distance)
    if check_collision_free(nearest_node, sampled_point, obstacles):
         new_node.parent = nearest_node.new_node.cost = nearest_node.cost + node_2_node_distance(nearest_node, new_node)
         tree.append(new_node)
         rewire_tree(tree, new_node,obstacle_radius, obstacles)
         if node_2_node_distance(new_node, goal) < max_distance:
              goal.parent = new_node
              goal.cost = new_node.cost + node_2_node_distance(new_node, goal)
              return goal
    return None 



# Here I have set up the start and goal nodes, state space, obstacles and radius of obstacle(assumed circular).
start_node = Node(0, 0)
goal_node = Node(5, 5)
x_range = (-1, 6)
y_range = (-1, 6)
obstacle1 = Node(1, 1)
obstacle2 = Node(2, 0.5)
obstacle3 = Node(2, 2)
obstacle4 = Node(3, 4)
obstacle5 = Node(3, 0)
obstacle6 = Node(4, 1)
obstacle7 = Node(3, 3)
obstacle8 = Node(1.5, 3)
obstacle9 = Node(4, 4)
obstacle10 = Node(0, 1)
obstacle11 = Node(1.3, 2)
obstacle12 = Node(2.5, 1.3)
obstacle13 = Node(3.5, 1.5)
obstacle14 = Node(4, 2)
obstacle15 = Node(4.5, 3)
obstacle16 = Node(5, 4)
obstacles = [obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11, obstacle12, obstacle13, obstacle14, obstacle15, obstacle16]
obstacle_radius = 0.2

# Running the RRT* algorithm.
path = rrt_star(start_node, goal_node, x_range, y_range, obstacles)

# Plotting results for Visualization.
plt.scatter(start_node.x, start_node.y, color='green', marker='o', label='Start')
plt.scatter(goal_node.x, goal_node.y, color='red', marker='o', label='Goal')
plt.scatter(*zip(*[(obstacle.x, obstacle.y) for obstacle in obstacles]), color='black', marker='x', label='Obstacle')
plt.plot(*zip(*path), linestyle='-', marker='.', color='blue', label='Path')
plt.legend()
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('RRT* Algorithm')
plt.show()

