from heapq import *

def heuristic(a, b):
    """Returns the distance between two diferent points
    Returns:
        (int): distance between the two points
    """
    return abs(b[0] - a[0])**2 + abs(b[1] - a[1])**2

def astar(array, start, goal):
    """Returns a list of points in the path to be followed by the robot
    Arguments:
        array (numpy.array): map of obstacles
        start ((int, int)): cordinates of start point
        goal ((int, int)): cordinates of goal point
    Returns:
        path (list): list of cordinates in the path of robot
    Raises:
        "path not found" exception when no path exists between the 2 points
    """

    start = start[::-1]
    goal = goal[::-1]

    if array[goal] < 127:
        raise Exception("path not found")

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]

            data.reverse()
            path = []
            for i in range(0,len(data),4):
                path.append(data[i])   
            return path

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] < 127 :
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    raise Exception("path not found")