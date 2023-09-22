import time
from search.algorithms import State
from search.map import Map
import getopt
import sys

import heapq

def dijkstra(start, goal, gridded_map):
    # Initialize the OPEN and CLOSED lists
    # OPEN uses heap, CLOSED uses dictionaries
    open_list = []
    closed_list = {}
    heapq.heappush(open_list, start)
    closed_list[start.state_hash()] = start
    print(closed_list)

    while len(open_list) != 0:
        n = heapq.heappop(open_list)
        if n == goal:
            return true
#While open list not empty
#   n = open_list.pop
#   if n = goal
#       return path
#   for child in n: (get children of n)
#       if child not in closed_list:
#           open_list.insert(child, n.get_g() (cost of parent) + (cost connecting child and parent) (not sure how to do this))
#           closed_list[child.state_hash()] = child
#       if child in closed_list and ( ((cost of parent) + (cost connecting child and parent)) < closed_list[child].get_g):
#           update cost/pointers in both open and closed lists
#           reheapify open list
#   return failures




    return None, None

def main():
    """
    Function for testing your A* and Dijkstra's implementation. 
    Run it with a -help option to see the options available. 
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, a in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances and generate plots: main.py --plots")
            exit()
        elif o in ("--plots"):
            plots = True

    test_instances = "test-instances/testinstances.txt"
    
    # Dijkstra's algorithm and A* should receive the following map object as input
    gridded_map = Map("dao-map/brc000d.map")
    
    nodes_expanded_dijkstra = []  
    nodes_expanded_astar = []

    time_dijkstra = []  
    time_astar = []

    start_states = []
    goal_states = []
    solution_costs = []
       
    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))
        
        solution_costs.append(float(list_instance[4]))
    file.close()
        
    for i in range(0, len(start_states)):    
        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()


        cost, expanded_diskstra = dijkstra(start, goal, gridded_map) # replace None, None with the call to your Dijkstra's implementation


        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()    

        start = start_states[i]
        goal = goal_states[i]
    
        time_start = time.time()
        cost, expanded_astar = None, None # replace None, None with the call to your A* implementation
        time_end = time.time()

        nodes_expanded_astar.append(expanded_astar)
        time_astar.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)", "Nodes Expanded (Dijkstra)", "nodes_expanded")
        plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")

if __name__ == "__main__":
    main()