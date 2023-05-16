from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math

def create_data_model(N):
    a = []
    for i in range(N):
        row = input().split()
        for i in range(len(row)):
            row[i] = float(row[i])
        a.append(row)
    matrix0=[]
    for i in range(N):
        row=[]
        for j in range(N):
            row.append(int(round(1000000000*math.sqrt((a[i][0]-a[j][0])**2+(a[i][1]-a[j][1])**2))))
        matrix0.append(row)
    data = {}
    data['distance_matrix']=matrix0
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def print_solution(manager, routing, solution):
    print('{}'.format(solution.ObjectiveValue()*0.000000001))
    index = routing.Start(0)
    plan_output =''
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += '{} '.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    print(plan_output)

def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]


N = int(input()) 
data = create_data_model(N)
manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),data['num_vehicles'], data['depot'])
routing = pywrapcp.RoutingModel(manager)

transit_callback_index = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

solution = routing.SolveWithParameters(search_parameters)

if solution:
    print_solution(manager, routing, solution)
