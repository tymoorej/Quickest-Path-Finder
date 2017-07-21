'''
---------------------------------------------------
| Assignment 1 part 2                             |
| Names: Nathan Douglas Klapstein, Tymoore Jamal  |
| IDs: 1449872 (Nathan), 1452978 (Tymoore)        |
| CMPUT 275 LBL EB2 (Nathan) / EB1 (Tymoore)      |
---------------------------------------------------
README:
    To run our code:
  	    Change the directory of your computer into the file where server.py is
        and then type the keyword: python3 server.py into the
        command terminal to run our code. Note this will only work after the
        C++ code is uploaded to the arduino.

    While running our code:
	    server.py will recieve input from the Arduino after the Arduino clicks on two points
        input will begin with an R followed by the lat and lon of the start point
        and then followed by the lat and lon of the end point. Then the server
        will wait for an A and once recieved it will print out N followed by
        an integer. This denotes how many waypoints are present. Then the server
        will wait for an A and once recieved it will print out W followed by the
        lat and lon of the waypoint, this cycle continues until it reached the
        end and then our program will print out E to denote the end. Then the
        user can begin the cycle again. If the Server ever gets unexpected input
        from the arduino it will reset to the state of waiting for an R.

'''
import sys
import adjacencygraph
import heapq
import serial
import argparse

# Setting up argparse.
parser = argparse.ArgumentParser(
    description='Client-server message test.',
    formatter_class=argparse.RawTextHelpFormatter,
)

parser.add_argument("-d0",
                    help="Debug off",
                    action="store_false",
                    dest="debug")

parser.add_argument("-s",
                    help="Set serial port for protocol",
                    nargs="?",
                    type=str,
                    dest="serial_port_name",
                    default="/dev/ttyACM0")

args = parser.parse_args()

debug = args.debug


def read_city_graph(filename):
    '''
    Function that takes in an args.infile to seperate
    and decode the file into sorted lines for function input. Note blank lines
    will be discarded. The Function then takes the sorted csv file defining a
    graph and converts into a AdjacencyGraph by reading and deciding
    on the sorted line values. It then outputs the created graph

    Args:
        filename: a input file that is a defined file that is noted by
        filename. The file should be in csv format and should describe a
        AdjacencyGraph in the same form as shown within Exercise 2's
        documentation.
    Returns:
        city_graph: A constructed AdjacencyGraph from csv created
        sorted values

    NOTE: filename should be listed as edmonton-roads-2.0.1.txt

    '''
    graph = adjacencygraph.AdjacencyGraph()
    with open(filename) as f:
        for line in f:
            line = line.split(',')
            if line[0] == 'V':  # if the line is designated as a vertex, add vertex
                graph.add_vertex(int(line[1]))

                vert_coords[int(line[1])] = []

                V_lat = int((float(line[2]) * 100000))
                V_lon = int((float(line[3]) * 100000))

                vert_coords[int(line[1])].append(V_lat)
                vert_coords[int(line[1])].append(V_lon)

            if line[0] == 'E':  # if the line is designated as an edge, add edge
                graph.add_edge((int(line[1]), int(line[2])))

    return graph


def get_cords(vertex):
    '''
    Grabs the coordinates of a vertex.

    Args:
        vertex: the vertex that you want the coordinates of
    Returns:
        (lat, lon): the position of the vertex.

    '''
    # pull the respective key items of vert_coords dict
    lat = vert_coords[vertex][0]
    lon = vert_coords[vertex][1]

    # return as tuple (lat, lon)
    return lat, lon


def cost_distance(vertex_0, vertex_1):
    '''
    Computes and returns the straight-line distance between the two
    vertices vertex_0 and vertex_1.

    Args:
        vertex_0, vertex_1: The ids of the vertices that we will be calculating
        the distance between.

    Returns:
        length: the distance between the two vertices.

    '''

    (lat0, lon0) = get_cords(vertex_0)
    (lat1, lon1) = get_cords(vertex_1)

    x = lat0 - lat1
    y = lon0 - lon1
    length = ((x ** 2) + (y ** 2)) ** (0.5)
    return length


def cost_distance_point(vertex, coordinates):
    '''
    Computes and returns the straight-line distance between a
    vertices v. and a input coordinate tuple cord_tup

    Args:
        cord_tup: a tuple containing input coordinates from the arduino
        v: a vertex to find its length compaired to input cord_tup

    Returns:
        length: the distance between the two coordinates.

    '''
    (lat0, lon0) = coordinates
    (lat1, lon1) = get_cords(vertex)

    x = lat0 - lat1
    y = lon0 - lon1
    length = ((x ** 2) + (y ** 2)) ** (0.5)
    return length


def get_closest_verticies(start_coords, end_coords):
    '''
    Given two coordinate pairs (start and end) this function finds the
    closest vertices to the start and end corrdinates and returns them in a tuple.

    Args:
        start_coords: The initial coordinate pair, that we need to find the
        closest vertex to.
        end_coords: The second coordinate pair, that we need to find the
        closest vertex to.

    Returns:
        (start_vertex,end_vertex): A tuple containing the closest vertex to
        start_coords and the closest vertex to end_coords.

    '''
    start_vertex = min(
        vert_coords, key=lambda v: cost_distance_point(v, start_coords))

    end_vertex = min(
        vert_coords, key=lambda v: cost_distance_point(v, end_coords))
    return (start_vertex, end_vertex)


def least_cost_path(graph, start, dest, cost):
    """
    Find and return a least cost path in graph from start vertex to dest vertex.

    Efficiency: If E is the number of edges, the run-time is O( E log(E) ).

    Args:
        graph (Graph): The digraph defining the edges between the vertices.
        start: The vertex where the path starts. It is assumed that start is a
        vertex of graph.
        dest: The vertex where the path ends. It is assumed that start is a vertex
        of graph.
        cost: A function, taking the two vertices of an edge as parameters and
        returning the cost of the edge. For its interface, see the definition of
        cost_distance.
    Returns:
        path: A potentially empty list (if no path can be found) of the vertices in
        the graph. If there was a path, the first vertex is always start, the last
        is always dest in the list. Any two consecutive vertices correspond to
        some edge in graph.

    """
    distance = 0
    reached = dict()
    runners = [(distance, start, start)]
    heapq.heapify(runners)

    while len(runners) != 0:
        current = heapq.heappop(runners)

        if current[1] in reached.keys():
            continue
        reached[current[1]] = current[2]
        for v in graph.neighbours(current[1]):
            if v in reached.keys():
                continue
            else:
                heapq.heappush(
                    runners, (current[0] + cost(current[1], v), v, current[1]))

    path = list()
    at = dest
    if dest not in reached.keys():
        return path
    else:
        while at != start:
            path.append(at)
            at = reached[at]
    path.append(reached[at])
    return path[::-1]


def get_waypoints(graph, lat0, lon0, lat1, lon1):
    '''
    Given a graph and two pairs of coordinates this function finds the path of
    vertices needed to go from lat0,lon0 to lat1,lon1.

    Args:
        graph (Graph): The digraph defining the edges between the vertices.
        lat0: The latitude of the first point.
        lon0: The longitude of the first point.
        lat1: The latitude of the first point.
        lon2: The longitude of the first point.

    Returns:
        path: A potentially empty list (if no path can be found) of the vertices in
        the graph. If there was a path, the first vertex is always the vertex
        cooresponding to lat0 and lon0, the last vertex is always the vertex
        cooresponding to lat1 and lon1 in the list. Any two consecutive vertices
        correspond to some edge in graph.

    '''

    start_input_coords = (lat0, lon0)
    end_input_coords = (lat1, lon1)

    closest_vertexs = get_closest_verticies(
        start_input_coords, end_input_coords)

    path = least_cost_path(graph, closest_vertexs[
                           0], closest_vertexs[1], cost_distance)

    return path


def server_listen():
    """
    This is the function that acts as the server. It will constantly attempt to
    communicate with the Arduino, once it gets the start and end coords from the
    arduino, it will calculate the quickest path between the corrdinates and send
    back the waypoints so that the arduino can draw a line from start to end.
    If the server ever recieves something that it does not expect it will reset
    back to the state of reqwait, which is the state of waiting for the initial
    request. We also made the decision to print all lines starting with
    R,N,W,A, and E, even with debugging turned off. This is because we believe
    that it is alot more useful than simply printing nothing.

    Args:
        N/A.
    Returns:
        NONE.

    """

    waypoints = []
    line = []

    # reset flag
    reqwait = True

    sp = serial.Serial()
    sp.port = args.serial_port_name
    sp.baudrate = 9600
    sp.timeout = 0.5
    sp.xonxoff = False
    sp.rtscts = False
    sp.dsrdtr = False

    sp.open()

    while True:

        value = ''

        if sp.inWaiting() > 0:  # Python version of  Serial.avialble()
            value = sp.readline()
            value = value.decode('ascii')
            value = value.strip()
            if len(value) > 0 and value[0] is 'D':
                if args.debug:
                    print("GOT NEW INPUT SERVER:")
                    print("input value:", end=" ")
                    print(value)
                continue
            else:
                if args.debug:
                    print("GOT NEW INPUT SERVER:")
                    print("input value:", end=" ")
                print(value)

        line = value.split()

        if line == []:  # diregard blank lines
            continue

        # arduino has sent out new request thats not blank
        if len(line) > 0:

            if line[0] == 'R' and reqwait == True:
                if args.debug:
                    print('server recieved "R"')

                reqwait = False

                inlat0 = int(line[1])
                inlon0 = int(line[2])
                inlat1 = int(line[3])
                inlon1 = int(line[4])

                waypoints = get_waypoints(
                    graph, inlat0, inlon0, inlat1, inlon1)

                # print out number of waypoints
                num_waypoints = len(waypoints)
                output = "".join(['N', " ", str(num_waypoints), '\n'])
                sp.write(output.encode('ascii'))

                if args.debug:
                    print('server outputting:', end=' ')

                print('N', num_waypoints, sep=' ')

                i = 0

                if num_waypoints == 0:
                    reqwait = True
                continue
            # the arduino has sent acknowledgement signal
            if args.debug:
                print('"About to recieve "A"')
                print("reqwait: ", reqwait)

            if line[0] == 'A' and not reqwait:

                if args.debug:
                    print('server recieved "A"')

                # print out i-th waypoint
                print_coords = vert_coords[waypoints[i]]
                if args.debug:
                    print('About to send a line')
                output = "".join(
                    ['W', " ", str(print_coords[0]), " ", str(print_coords[1]), '\n'])
                sp.write(output.encode('ascii'))

                if args.debug:
                    print('server outputting:', end=' ')

                print('W', print_coords[0], print_coords[1], sep=' ')

                i = i + 1

                if i == len(waypoints):
                    sp.write('E\n'.encode('ascii'))
                    if args.debug:
                        print('server outputting:', end=' ')

                    print('E')
                    reqwait = True
            else:
                if args.debug:
                    print('value not accepted: ', value)
                reqwait = True
                continue


# vertex coordinates global
vert_coords = dict()

graph = read_city_graph('edmonton-roads-2.0.1.txt')

if __name__ == "__main__":
    server_listen()


# end of file
