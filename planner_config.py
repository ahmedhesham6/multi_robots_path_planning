import sys
import os
sys.path.append(os.path.dirname(__file__) + "/../")
try:
    from PathPlanning.AStar import a_star
except:
    raise
try:
    from PathPlanning.Dijkstra import dijkstra 
except:
    raise


def config():
    grid_size = 1.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstable positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 20):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 20):
        ox.append(30.0)
        oy.append(60.0 - i)
    for i in range(15,31):
        ox.append(i)
        oy.append(40)
    for i in range(-10,10):
        ox.append(i)
        oy.append(20.0)
    for i in range(20,30):
        ox.append(i)
        oy.append(20)
    for i in range(45,60):
        ox.append(i)
        oy.append(20)
        ox.append(i)
        oy.append(10)

    configs = []
    a_star.show_animation=False
    configs.append(a_star.AStarPlanner(ox, oy, grid_size, robot_radius))

    dijkstra.show_animation = False
    configs.append(dijkstra.Dijkstra(ox, oy, grid_size, robot_radius))



    return configs