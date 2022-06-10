from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def compute_inp_data():
    inp_data = {}
    inp_data['dist_mat'] = [[0, 29, 15, 35], [29, 0, 57, 42], [15, 57, 0, 61], [35, 42, 61, 0]]
    inp_data['total_vehicle'] = 1
    inp_data['iteration'] = 0
    return inp_data


def result_display(manage, route, finding):
    print('Total Travel: {} miles'.format(finding.ObjectiveValue()))
    first_index = route.Start(0)
    route_plan = 'Vehicle 0 routes:\n'
    route_plan_dist = 0
    while not route.IsEnd(first_index):
        route_plan += ' {} ->'.format(manage.IndexToNode(first_index))
        start_index = first_index
        first_index = finding.Value(route.NextVar(first_index))
        route_plan_dist += route.GetArcCostForVehicle(start_index, first_index, 0)
    route_plan += ' {}\n'.format(manage.IndexToNode(first_index))
    print(route_plan)
    route_plan += 'Total Travel: {}miles\n'.format(route_plan_dist)


def main():
    inp_data = compute_inp_data()
    manage = pywrapcp.RoutingIndexManager(len(inp_data['dist_mat']),
                                           inp_data['total_vehicle'], inp_data['iteration'])
    route = pywrapcp.RoutingModel(manage)


    def dist_call(from_index, to_index):
        node_start = manage.IndexToNode(from_index)
        node_end = manage.IndexToNode(to_index)
        return inp_data['dist_mat'][node_start][node_end]

    travel_index = route.RegisterTransitCallback(dist_call)
    route.SetArcCostEvaluatorOfAllVehicles(travel_index)
    find_params = pywrapcp.DefaultRoutingSearchParameters()
    find_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    finding = route.SolveWithParameters(find_params)
    if finding:
        result_display(manage, route, finding)


if __name__ == '__main__':
    main()
