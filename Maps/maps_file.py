import numpy as np
import matplotlib.pyplot as plt

def general_map(width, height):
    width = 1200
    height = 500
    # Generating General Space
    general_map = np.full((height, width), 0)

    for y in range(0, height):
        for x in range(0, width):

            # Wall Obstacles
            w1 = (y) - 5
            w2 = (y) - 495
            w3 = (x) - 5
            w4 = (x) - 1195

            # Rectangle 1 Obstacle
            r11 = (x) - 100
            r12 = (y) - 100
            r13 = (x) - 175
            r14 = (y) - 500

            # Rectangle 2 Obstacle
            r21 = (x) - 275
            r22 = (y) - 0
            r24 = (x) - 350
            r23 = (y) - 400

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

            # Setting the line constrain to obtain the obstacle space with buffer
            if ((h6 > 0 and h5 > 0 and h4 < 0 and h3 < 0 and h2 < 0 and h1 > 0) or (
                    r11 > 0 and r12 > 0 and r13 < 0 and r14 < 0) or (
                    r21 > 0 and r23 < 0 and r24 < 0 and r22 > 0) or (
                    t1 > 0 and t2 < 0 and t4 > 0 and t5 < 0) or (
                    t2 > 0 and t3 < 0 and t4 > 0 and t7 < 0) or (
                    t6 > 0 and t7 < 0 and t1 > 0 and t2 < 0) or (
                    w1 < 0) or (
                    w2 > 0) or (
                    w3 < 0) or (
                    w4 > 0)):
                general_map[y, x] = 2

    return general_map

def narrow_map(width, height):
    width = 900
    height = 900
    narrow_map = np.full((height, width), 0)
    for y in range(0, height):
        for x in range(0, width):
            
            # Walls
            w1 = (y) - 1
            w2 = (x) - 1
            w3 = (y) - 899
            w4 = (x) - 899
            
            # Obstacle 1
            r11 = (y) - 475
            r12 = (y) - 900
            r13 = (x) 
            r14 = (x) - 275
            
            # Obstacle 2
            r21 = (y) - 875
            r22 = (y) - 900
            r23 = (x) - 275
            r24 = (x) - 625
            
            # Obstacle 3
            r31 = (y) - 75
            r32 = (y) - 900
            r33 = (x) - 625
            r34 = (x) - 900
            
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
            r64 = (x) - 900
            
            if((r11>0 and r12<0 and r13>0 and r14<0) or
                (r21>0 and r22<0 and r23>0 and r24<0) or
                (r31>0 and r32<0 and r33>0 and r34<0) or
                (r41>0 and r42<0 and r43>0 and r44<0) or
                (r51>0 and r52<0 and r53>0 and r54<0) or
                (r61>0 and r62<0 and r63>0 and r64<0) or
                (w1<0) or (w2<0) or (w3>0) or (w4>0)):
                narrow_map[y,x] = 2
                
    return narrow_map

def cluster_map(width, height, num_squares, square_size):
    np.random.seed(42)  # Fix the random seed for reproducibility
    narrow_map = np.full((height, width), 0)
    
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
                cluster_map[y, x] = 2  # Mark square interiors as obstacle
            
            # Setting up the walls
            if y == 0 or y == height - 1 or x == 0 or x == width - 1:
                cluster_map[y, x] = 2  # Mark the edges as walls

    return cluster_map


