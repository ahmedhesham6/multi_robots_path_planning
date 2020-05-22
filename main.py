import matplotlib.pyplot as plt
import itertools
import time
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
try:
    from PathPlanning.PotentialFieldPlanning import potential_field_planning
except: 
    raise
from PathPlanning.ProbabilisticRoadMap import probabilistic_road_map
from PathPlanning.VoronoiRoadMap import voronoi_road_map

from planner_config import config
from graph import graph

from matplotlib.ticker import FuncFormatter
import matplotlib.pyplot as plt
import numpy as np

x = np.arange(4)
money = [1.5e5, 2.5e6, 5.5e6, 2.0e7]


def millions(x, pos):
    'The two args are the value and tick position'
    return '$%1.1fM' % (x * 1e-6)






def main():
    print(__file__ + " start!!")
    show_animation = False

    # start and goal position
    sx1 = 0.0  # [m]
    sy1 = 0.0  # [m]
    gx1 = 50.0  # [m]
    gy1 = 50.0  # [m]

    sx2 = 50.0  # [m]
    sy2 = 0.0  # [m]
    gx2 = 0.0  # [m]
    gy2 = 50.0  # [m]

    sx3 = 0.0  # [m]
    sy3 = 30.0  # [m]
    gx3 = 50.0  # [m]
    gy3 = 30.0  # [m]

    sx4 = 10.0  # [m]
    sy4 = 50.0  # [m]
    gx4 = 40.0  # [m]
    gy4 = 50.0  # [m]

    sx5 = 20.0  # [m]
    sy5 = 50.0  # [m]
    gx5 = 50.0  # [m]
    gy5 = 15.0  # [m]

    sx6 = 0.0  # [m]
    sy6 = 10.0  # [m]
    gx6 = 30.0  # [m]
    gy6 = 10.0  # [m]

    sx7 = 40.0  # [m]
    sy7 = 40.0  # [m]
    gx7 = 0.0  # [m]
    gy7 = 40.0  # [m]

    sx8 = 40.0  # [m]
    sy8 = 30.0  # [m]
    gx8 = 0.0  # [m]
    gy8 = 40.0  # [m]

    grid_size = 1  # [m]
    robot_radius = 5.0  # [m]

    r = [
        [sx1,sy1,gx1,gy1],
        [sx2,sy2,gx2,gy2],
        [sx3,sy3,gx3,gy3],
        [sx4,sy4,gx4,gy4],
        [sx5,sy5,gx5,gy5],
        [sx6,sy6,gx6,gy6],
        [sx7,sy7,gx7,gy7],
        [sx8,sy8,gx8,gy8]]

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

    timings = [0,0,0,0]
    dispalcment = [0,0,0,0]
    points = [0,0,0,0]
    for i,robot in  enumerate(itertools.combinations(r,4)):
    # set obstable positions
        if show_animation:  # pragma: no cover
            x= plt.figure(i)
            plt.plot(ox, oy, ".k")
            plt.plot(robot[0][0], robot[0][1], "or" , label="Bot 1" ,markersize=12)
            plt.plot(robot[0][2], robot[0][3], "xr" ,markersize=10)

            plt.plot(robot[1][0], robot[1][1], "og", label="Bot 2" ,markersize=12)
            plt.plot(robot[1][2], robot[1][3], "xg" ,markersize=10)

            plt.plot(robot[2][0], robot[2][1], "ob", label="Bot 3" ,markersize=12)
            plt.plot(robot[2][2], robot[2][3], "xb" ,markersize=10)

            plt.plot(robot[3][0], robot[3][1], "oy", label="Bot 4" ,markersize=12)
            plt.plot(robot[3][2], robot[3][3], "xy" ,markersize=10)

            # plt.grid(True)
            plt.axis("equal")
            plt.legend(loc='upper left', bbox_to_anchor=(0.7, 1.16),
            ncol=2, fancybox=True, shadow=True)

        a_star_config = config()[0]
        dijkstra_config = config()[1]

        for point in robot:
         # Astar Cooperative planning
            start = time.time()
            rx, ry = a_star_config.planning(point[0], point[1], point[2], point[3])
            timings[0] += time.time() - start
            if show_animation:
                plt.plot(rx, ry, "-r")
            points[0] += len(rx)
            #Calculate displacment
            x = np.array(rx)
            y = np.array(ry)
            dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
            dispalcment[0] += np.sum(np.sqrt(dist_array)) 

        # Dijkstra Cooperative planning
            start = time.time()
            rx, ry = dijkstra_config.planning(point[0], point[1], point[2], point[3])
            timings[1] += time.time() - start
            if show_animation:
                plt.plot(rx, ry, "-r")
            points[1] += len(rx)
            #Calculate displacment
            x = np.array(rx)
            y = np.array(ry)
            dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
            dispalcment[1] += np.sum(np.sqrt(dist_array)) 

        #Deadlock Problem

        #     print("Potential Field")
        # # Potential Field Cooperative planning
        #     start = time.time()
        #     rx, ry = potential_field_planning.potential_field_planning(point[0], point[1], point[2], point[3],ox,oy,grid_size,robot_radius)
        #     timings[2] += time.time() - start
        #     if show_animation:
        #         plt.plot(rx, ry, "-r")
        #     points[2] += len(rx)
        #     #Calculate displacment
        #     x = np.array(rx)
        #     y = np.array(ry)
        #     dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
        #     dispalcment[2] += np.sum(np.sqrt(dist_array)) 

        # PRM Cooperative planning
            start = time.time()
            rx, ry = probabilistic_road_map.PRM_planning(point[0], point[1], point[2], point[3],ox,oy,robot_radius)
            timings[2] += time.time() - start
            if show_animation:
                plt.plot(rx, ry, "-r")
            points[2] += len(rx)
            #Calculate displacment
            x = np.array(rx)
            y = np.array(ry)
            dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
            dispalcment[2] += np.sum(np.sqrt(dist_array)) 

        # Voroni Cooperative planning
            start = time.time()
            rx, ry = voronoi_road_map.VoronoiRoadMapPlanner().planning(point[0], point[1], point[2], point[3],ox,oy,robot_radius)
            timings[3] += time.time() - start
            if show_animation:
                plt.plot(rx, ry, "-r")
            points[3] += len(rx)
            #Calculate displacment
            x = np.array(rx)
            y = np.array(ry)
            dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
            dispalcment[3] += np.sum(np.sqrt(dist_array)) 

    graph(timings,'Time(sec)')
    graph(dispalcment,'Displacment(m)')
    


if __name__ == '__main__':
    main()