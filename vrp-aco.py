from distutils.log import info
import math
from multiprocessing.dummy import current_process
from xml.dom import minidom
import random
from numpy import average
random.seed()

adj_list = {}
vehicle_route = {}
matrix = []
pheromone = []

class ACO:

    def __init__(self, alpha, beta, rho, q, filename, iterations, ants):
        self.alpha = alpha
        self.beta = beta
        self.rho = rho
        self.q = q
        self.filename = filename
        self.iterations = iterations
        self.ants = ants

    def aco(self):
        trucks, locations, adj_list, vehicle_route, departure_node, arrival_node, capacity = self.make_adj_list()
        matrix, pheromone = self.matrix_construction(adj_list, locations)
        best_path = []
        overall_best = []
        t = []

        for _ in range(self.iterations):
            distance = 0
            path_nodes = []
            current_location = departure_node
            total_path_iteration = []

            for ant in range(self.ants):
                # every ant will visit every location ONCE
                total_distance_per_ant = 0
                locations_per_ant = []
                solution_per_ant = []
                unvisited = []
                for x in range(2, locations + 1):
                    unvisited.append(x)

                while len(unvisited) != 0:
                    unvisited, path_nodes, distance = self.create_route(matrix, pheromone, capacity, vehicle_route, current_location, unvisited)
                    locations_per_ant.append(path_nodes)
                    total_distance_per_ant = distance + total_distance_per_ant
                
                solution_per_ant = [locations_per_ant, total_distance_per_ant]
                pheromone = self.update_pheromone(solution_per_ant, pheromone)
                total_path_iteration.append(solution_per_ant)

            best_path = sorted(total_path_iteration, key = lambda x: x[1])[0]
            pheromone = self.update_pheromone(best_path, pheromone)
            t.append(best_path)

        overall_best = sorted(t, key = lambda x: x[1])[0]
        print("Best distance so far:", overall_best[-1])

    def make_adj_list(self):
        nodes = {}
        file = minidom.parse(self.filename)
        information = file.getElementsByTagName('name')[0].firstChild.nodeValue
        trucks = int(information[7:])
        locations = int(information[3:5])
        network = file.getElementsByTagName('node')

        for element in network:
            nodes[int(element.attributes['id'].value)] = []
            x_value = float((element.getElementsByTagName('cx')[0].firstChild.nodeValue))
            y_value = float((element.getElementsByTagName('cy')[0].firstChild.nodeValue))
            nodes[int(element.attributes['id'].value)].append(x_value)
            nodes[int(element.attributes['id'].value)].append(y_value)
            
        dict_length = len(nodes.keys())
        for x in range(1, dict_length + 1):
            adj_list[x] = []
            for y in range(1, dict_length + 1):
                if y != x:
                    temp = [y, round(math.sqrt((nodes[x][0] - nodes[y][0])**2 + (nodes[x][1] - nodes[y][1])**2))]
                    adj_list[x].append(temp)

        vehicle = file.getElementsByTagName('vehicle_profile')
        for element in vehicle:
            departure_node = int(element.getElementsByTagName('departure_node')[0].firstChild.nodeValue)
            arrival_node = int(element.getElementsByTagName('arrival_node')[0].firstChild.nodeValue)
            capacity = float(element.getElementsByTagName('capacity')[0].firstChild.nodeValue)
            
        requests = file.getElementsByTagName('request')
        for element in requests:
            vehicle_route[int(element.attributes['node'].value)] = []
            request_quantity = float((element.getElementsByTagName('quantity')[0].firstChild.nodeValue))
            vehicle_route[int(element.attributes['node'].value)].append(request_quantity)

        return trucks, locations, adj_list, vehicle_route, departure_node, arrival_node, capacity

    def matrix_construction(self, adj_list, locations):
        matrix = []
        pheromone = []
        for x in range(1,locations+1):
            matrix.append([0 for i in range(1,locations + 1)])
            pheromone.append([1 for i in range(1,locations + 1)]) #if pheromones are kept 0 initially for all, then we won't be able to calculate any pheromone values as 0 * anything is 0. 

        for element in range(1, len(adj_list) + 1):
            for value in range(len(adj_list[element])):
                index = adj_list[element][value][0]
                matrix[element-1][index-1] = adj_list[element][value][1]
        return matrix, pheromone

    def create_route(self, matrix, pheromone, capacity, vehicle_route, current_location, unvisited):
        path_nodes = []
        path_nodes.append(current_location)
        path = 0
        while (len(unvisited) != 0):
            
            probability = []
            prob = []
            p_sum = 0
                
            for i in unvisited:
                if matrix[current_location - 1][i - 1] == 0:
                    p = 0.0
                else:
                    # probabilistic function will be used to select the next unvisited location to visit by the ant
                    p = (((pheromone[current_location-1][i - 1])**self.alpha) * ((1/matrix[current_location-1][i - 1])**self.beta))
                # print(i, p)
                prob.append(p)
                p_sum = p + p_sum

            for p in prob:
                if p != 0:
                    probability.append(p / p_sum)
                else:
                    probability.append(p)

            next = random.choices(unvisited, weights = (x for x in probability))
            next_location = next[0]
            # capacity cannot be less than 0
            if capacity - vehicle_route[next_location][0] >= 0:
                capacity = capacity - vehicle_route[next_location][0]
                path = matrix[current_location-1][next_location - 1] + path
                unvisited.remove(next_location)
                path_nodes.append(next_location)
                current_location = next_location
            else:
                break

        path = matrix[current_location-1][0] + path
        return unvisited, path_nodes, path, 

    def update_pheromone(self,paths, pheromone):
        for p in paths[0]:
            for x in range(1, len(p)):
                current_location = p[x - 1]
                next_location = p[x]
                updated_pheromone = ((1 - self.rho) * pheromone[current_location - 1][next_location - 1]) + (self.rho * (1 / paths[-1]))
                pheromone[current_location - 1][next_location - 1] = updated_pheromone
                pheromone[next_location - 1][current_location - 1] = updated_pheromone
        return pheromone

if __name__ == '__main__':
    aco_algo = ACO(4,10, 0.6, 1, "Dataset/A-n80-k10.xml", 250, 10)
    aco_algo.aco()



# References:
# https://github.com/Lolik-Bolik/Vehicle-Routing-Problem
# https://sci-hub.se/10.1016/j.aei.2004.07.001
# https://www.academia.edu/37729334/Solving_Vehicle_Routing_Problem_using_Ant_Colony_Optimisation_ACO_Algorithm




