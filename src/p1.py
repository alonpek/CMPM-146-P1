from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush
import os
from math import sqrt
import sys

def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    dist = {}
    prev = {}
    queue = []

    dist[initial_position] = 0
    heappush(queue, (0, initial_position))

    while len(queue) != 0:
        current_cost, current_cell = heappop(queue)
        if current_cell == destination:
            # construct the path
            path = []
            while current_cell in prev:
                path.insert(0, current_cell)
                current_cell = prev[current_cell]
            path.insert(0, initial_position)
            return path
        for neighbor_cell, neighbor_cost in adj(graph, current_cell):
            new_dist = neighbor_cost + dist[current_cell]
            if (neighbor_cell not in dist) or (new_dist < dist[neighbor_cell]):
                dist[neighbor_cell] = new_dist
                prev[neighbor_cell] = current_cell
                heappush(queue, (new_dist, neighbor_cell))

    return None


def dijkstras_shortest_path_to_all(initial_position, graph, adj):
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """
    dist = {}
    prev = {}
    queue = []

    dist[initial_position] = 0
    heappush(queue, (0, initial_position))

    while len(queue) != 0:
        current_cost, current_cell = heappop(queue)
        for neighbor_cell, neighbor_cost in adj(graph, current_cell):
            new_dist = neighbor_cost + dist[current_cell]
            if (neighbor_cell not in dist) or (new_dist < dist[neighbor_cell]):
                dist[neighbor_cell] = new_dist
                prev[neighbor_cell] = current_cell
                heappush(queue, (new_dist, neighbor_cell))

    return dist



def navigation_edges(level, cell):
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    adjacent = []
    walls = level['walls']
    spaces = level['spaces']

    test_cells = []
    test_cells.append((cell[0]-1, cell[1]-1))
    test_cells.append((cell[0], cell[1]-1))
    test_cells.append((cell[0]+1, cell[1]-1))
    test_cells.append((cell[0]-1, cell[1]))
    test_cells.append((cell[0]+1, cell[1]))
    test_cells.append((cell[0]-1, cell[1]+1))
    test_cells.append((cell[0], cell[1]+1))
    test_cells.append((cell[0]+1, cell[1]+1))

    for test_cell in test_cells:
        if test_cell[0] < 0 or test_cell[1] < 0:
            continue
        if test_cell in walls:
            continue
        if (test_cell[0] == cell[0] and test_cell[1] != cell[1]) or \
                (test_cell[0] != cell[0] and test_cell[1] == cell[1]):
            weight = (spaces[cell] + spaces[test_cell]) / 2
        else:
            weight = (spaces[cell] + spaces[test_cell]) * sqrt(2) / 2
        adjacent.append((test_cell, weight))

    return adjacent


def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from 
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """
    
    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]
    
    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'example.txt', 'a','e'
    print(filename)
    print(os.getcwd())

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    cost_to_all_cells(filename, src_waypoint, 'my_costs.csv')
