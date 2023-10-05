# It finds the optimal path for a car using Hybrid A* and bicycle model.

import time
import heapq as hq
import math
import matplotlib.pyplot as plt
import numpy as np
from autocar_nav.separation_axis_theorem import separating_axis_theorem, get_vertice_rect

# total cost f(n) = actual cost g(n) + heuristic cost h(n)
#                   노드간의 거리         노드에서 목적지까지 추정거리
class hybrid_a_star:
    def __init__(self, min_x, max_x, min_y, max_y, \
            obstacle=[], resolution=0.3, length=1.5, width = 0.8):
        self.min_x = min_x # / resolution
        self.max_x = max_x # / resolution
        self.min_y = min_y # / resolution
        self.max_y = max_y # / resolution
        # self.obstacle = [tuple(map(lambda x: round(x / resolution), tpl)) for tpl in obstacle]
        self.obstacle = obstacle
        # print(self.obstacle)
        self.resolution = resolution
        self.vehicle_length = length
        self.width = width

        ###
    def point_inside_obstacle_rectangle(self, point, obstacle):
        x = obstacle[0] #/self.resolution
        y = obstacle[1] #/self.resolution
        yaw = obstacle[2]
        length = obstacle[3] + self.vehicle_length #/self.resolution
        width = obstacle[4] + self.width #/self.resolution

        # 장애물의 네 꼭지점 좌표 계산
        half_length = length / 2
        half_width = width / 2
        vertices = [
            (x + half_length * math.cos(yaw) - half_width * math.sin(yaw), y + half_length * math.sin(yaw) + half_width * math.cos(yaw)),
            (x - half_length * math.cos(yaw) - half_width * math.sin(yaw), y - half_length * math.sin(yaw) + half_width * math.cos(yaw)),
            (x - half_length * math.cos(yaw) + half_width * math.sin(yaw), y - half_length * math.sin(yaw) - half_width * math.cos(yaw)),
            (x + half_length * math.cos(yaw) + half_width * math.sin(yaw), y + half_length * math.sin(yaw) - half_width * math.cos(yaw))
        ]

        lines = [(vertices[0][0], vertices[0][1], vertices[1][0], vertices[1][1]),
                 (vertices[1][0], vertices[1][1], vertices[2][0], vertices[2][1]),
                 (vertices[2][0], vertices[2][1], vertices[3][0], vertices[3][1]),
                 (vertices[3][0], vertices[3][1], vertices[0][0], vertices[0][1])]

        inside = True
        for line in lines:
            x1,y1,x2,y2 = line

            if (point[0] - x1) * (y2 - y1) - (point[1] - y1) * (x2 - x1) > 0:
                inside = False
                break

        return inside

    def euc_dist(self, position, target):
        output = np.sqrt(((position[0] - target[0]) ** 2) + ((position[1] - target[1]) ** 2)) # +(math.radians(position[2]) - math.radians(target[2])) ** 2
        return float(output)

    def costfunction(self, position, target):
        ratioDelta = 1
        output = ratioDelta * abs(position[2] - target[2])
        return float(output)

    def find_path(self, start, end, max_steer = 20):
        # max steering
        steering_inputs = [0, -max_steer, max_steer]
        cost_steering_inputs= [0, 0.3, 0.3]

        speed_inputs = [self.resolution,-self.resolution]
        cost_speed_inputs = [0,1]

        start = (float(start[0]), float(start[1]), float(start[2])) # /resolution
        end = (float(end[0]), float(end[1]), float(end[2]))
        # start = (round(start[0]),round(start[1]),round(start[2]))
        # end = (round(end[0]),round(end[1]),round(end[2]))
        # The above 2 are in discrete coordinates

        open_heap = [] # element of this list is like (cost,node_d) -->(cost,node_c)
        open_diction={} #  element of this is like node_d:(cost,node_c,(parent_d,parent_c)) --> (cost,node_c,parent_c)

        visited_diction={} #  element of this is like node_d:(cost,node_c,(parent_d,parent_c)) --> (cost,node_c,parent_c)
        cost_to_neighbour_from_start = 0

        hq.heappush(open_heap,(cost_to_neighbour_from_start + self.euc_dist(start, end), start))

        open_diction[start]=(cost_to_neighbour_from_start + self.euc_dist(start, end), start, start)

        start_time = time.time()
        while len(open_heap)>0:
            chosen_c_node =  open_heap[0][1]
            chosen_node_total_cost=open_heap[0][0]

            visited_diction[chosen_c_node]=open_diction[chosen_c_node]

            if self.euc_dist(chosen_c_node,end) < 2.0 * self.resolution:
                rev_final_path=[end] # reverse of final path
                node=chosen_c_node
                m = 1
                while m == 1:
                    visited_diction
                    open_node_contents=visited_diction[node] # (cost,node_c,(parent_d,parent_c))
                    parent_of_node=open_node_contents[2]

                    rev_final_path.append(parent_of_node)
                    node=open_node_contents[2]

                    if node == start:
                        rev_final_path.append(start)
                        break

                final_path = []
                for p in rev_final_path:
                    final_path.insert(0,(p[0], p[1], math.radians(p[2]))) # x self.resolution

                return final_path

            hq.heappop(open_heap)

            for i in range(0,3):
                for j in range(1):
                    delta=steering_inputs[i]
                    velocity=speed_inputs[j]

                    cost_to_neighbour_from_start =  chosen_node_total_cost-self.euc_dist(chosen_c_node, end)

                    neighbour_x_cts = chosen_c_node[0] + (velocity * math.cos(math.radians(chosen_c_node[2])))
                    neighbour_y_cts = chosen_c_node[1]  + (velocity * math.sin(math.radians(chosen_c_node[2])))
                    neighbour_theta_cts = math.radians(chosen_c_node[2]) + (velocity * math.tan(math.radians(delta))/(float(self.vehicle_length)))

                    neighbour_theta_cts=math.degrees(neighbour_theta_cts)
                    neighbour = ((neighbour_x_cts,neighbour_y_cts,neighbour_theta_cts))
                    car = [neighbour_x_cts, neighbour_y_cts, np.deg2rad(neighbour_theta_cts), self.vehicle_length, self.width]
                    # car_vertices = get_vertice_rect(car)
                    if (all(not separating_axis_theorem(get_vertice_rect(car), get_vertice_rect(obs)) for obs in self.obstacle) and \
                            (neighbour_x_cts >= self.min_x) and (neighbour_x_cts <= self.max_x) and \
                            (neighbour_y_cts >= self.min_y) and (neighbour_y_cts <= self.max_y)):

                            heurestic = self.euc_dist((neighbour_x_cts,neighbour_y_cts,neighbour_theta_cts),end)
                            cost_to_neighbour_from_start = abs(velocity)+ cost_to_neighbour_from_start +\
                                                                         cost_steering_inputs[i] + cost_speed_inputs[j]

                            total_cost = heurestic+cost_to_neighbour_from_start

                            skip=0
                            found_lower_cost_path_in_open = 0

                            if neighbour in open_diction:

                                if total_cost>open_diction[neighbour][0]:
                                    skip=1

                                elif neighbour in visited_diction:

                                    if total_cost>visited_diction[neighbour][0]:
                                        found_lower_cost_path_in_open=1


                            if skip==0 and found_lower_cost_path_in_open==0:
                                hq.heappush(open_heap, (total_cost,neighbour))
                                open_diction[neighbour]=(total_cost, neighbour, chosen_c_node)

            if time.time() - start_time > 1.5:
                return None
        # print("Did not find the goal - it's unattainable.")
        return None #[(start[0],start[1],math.radians(start[2])), (end[0],end[1],math.radians(end[2]))]

def main():
    print(__file__ + " start!!")

    # start and goal position
    #(x, y, theta) in meters, meters, degrees
    sx, sy, stheta = -5.0, 0.0, 0
    gx, gy, gtheta = 5.0, 0.0, 0 #2,4,0 almost exact

    #create obstacles
    # obstacle = [(-0.8,-0.4,math.pi/4,0.4,0.7),(0.8,0.4,math.pi/4,0.4,0.7)]
    obstacle = [(-1,0,0,0.5,0.5), (2.3,1.0,0,0.5,0.5),(0,-1,0,10.0,0.5)]#,(0,3,0,10.0,0.5)]

    ox, oy = [], []
    for obs in obstacle:
        ox.append(obs[0] + obs[3]/2 * math.cos(obs[2]) - obs[4]/2 * math.sin(obs[2]))
        ox.append(obs[0] - obs[3]/2 * math.cos(obs[2]) - obs[4]/2 * math.sin(obs[2]))
        ox.append(obs[0] - obs[3]/2 * math.cos(obs[2]) + obs[4]/2 * math.sin(obs[2]))
        ox.append(obs[0] + obs[3]/2 * math.cos(obs[2]) + obs[4]/2 * math.sin(obs[2]))
        oy.append(obs[1] + obs[3]/2 * math.sin(obs[2]) + obs[4]/2 * math.cos(obs[2]))
        oy.append(obs[1] - obs[3]/2 * math.sin(obs[2]) + obs[4]/2 * math.cos(obs[2]))
        oy.append(obs[1] - obs[3]/2 * math.sin(obs[2]) - obs[4]/2 * math.cos(obs[2]))
        oy.append(obs[1] + obs[3]/2 * math.sin(obs[2]) - obs[4]/2 * math.cos(obs[2]))

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")

    hy_a_star = hybrid_a_star(-5.0, 5.0,-5.0, 5.0, obstacle=obstacle, \
        resolution=1.0, length=1.5, width= 1.0)
    path = hy_a_star.find_path((sx,sy,stheta), (gx,gy,gtheta))

    rx, ry = [], []
    for node in path:
        rx.append(node[0])
        ry.append(node[1])

    print(path)

    plt.plot(rx, ry, "-r")
    plt.show()


if __name__ == '__main__':
    main()
