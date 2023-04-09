"""
# Title:    Project-3 (phase-2) : A* Algorithm Implementation
# Course:   ENPM661 - Planning for Autonomous Robots
"""

from numpy import cos, sin, deg2rad, round, sqrt

import math
import time
import heapq
import cv2
import numpy as np


__author__ = "Jay Prajapati, Akash Parmar"
__copyright__ = "Copyright 2023, Project-3 (phase-2)"
__credits__ = ["Jay Prajapati", "Akash Parmar"]
__license__ = "MIT"
__version__ = "1.0.1"
__email__ = "jayp@umd.edu", "akasparm@umd.edu"


class NODE:
    """class for data structure to store the node information
    """

    def __init__(self, coordinates, cost, preced=None, cost_goal=0, uleft=None, urght=None):
        """Constructor

        Args:
            coordinates (list): 
                [x, y, theta] for the node
            cost (float): 
                cost of the node
            preced (<class 'NODE'>, optional): 
                parent node for the node. Defaults to None.
            cost_goal (int, optional): 
                Cost to reach the goal for the current node. Defaults to 0.
            uleft (int, optional): 
                Angular velocity of the left wheel.
            urght (int, optional): 
                Angular velocity of the Right Wheel.
        """
        self.coordinates = coordinates
        self.x_cord = coordinates[0]
        self.y_cord = coordinates[1]
        self.theta = coordinates[2]
        self.cost = cost
        self.preced = preced
        self.cost_goal = cost_goal
        self.uleft = uleft
        self.urght = urght

    def __lt__(self, other):
        """Defining the comparision operator
        """
        comparision = self.cost + self.cost_goal < other.cost + other.cost_goal
        return comparision


def createCanvas():
    """Creates the obstacles

    Returns:
        np.ndarray: Binary image containing the obstacle data
    """
    # Create an empty frame
    map = np.zeros((canvas_height, canvas_width), dtype=np.uint8)

    # set the offset
    offset = clearance + robot_radius

    # Treversing through each pixel
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):

            # Representating robot radius offset

            if (j >= 150-offset and j <= 165+offset and i >= 0 and i <= 125+offset):
                map[i][j] = 1

            if (j >= 250-offset and j <= 265+offset and i >= 75-offset and i < 200):
                map[i][j] = 1

            if ((j-400)**2 + (i-90)**2 <= (50+offset)**2):
                map[i][j] = 1

            # Representating the Frame border

            if (j >= 0 and j < (offset)) or (i >= 0 and i < (offset)) or \
                    (i >= 200-(offset) and i < 200) or (j >= 600-(offset) and j < 600):
                map[i][j] = 1

    return map


def checkSolvable(x, y, map):
    """checks whether the given coordinates are on the obstacles or not

    Args:
        x (int): x-coordinate
        y (int): y-coordinate
        map (np.ndarray): canvas

    Returns:
        bool: Flag for the validity of the point
    """
    if (map[y][x] == 1).all():
        return False
    return True


def animationCanvas():
    """Creates the animation canvas

    Returns:
        np.ndarray: Animation canvas image
    """

    # Create an empty frame
    image = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

    # set the offset
    offset = clearance + robot_radius

    # colors for the objects
    clearance_color = (34, 34, 43)
    radius_color = (255, 255, 255)
    object_color = (0, 117, 213)

    # Treversing through each pixel
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):

            # Representating robot radius offset

            if (j >= 150-offset and j <= 165+offset and i >= 0 and i <= 125+offset):
                image[i][j] = radius_color

            if (j >= 250-offset and j <= 265+offset and i >= 75-offset and i < 200):
                image[i][j] = radius_color

            if ((j-400)**2 + (i-90)**2 <= (50+offset)**2):
                image[i][j] = radius_color

            # Representating Clearance

            if (j >= 150-clearance and j <= 165+clearance and i >= 0 and i <= 125+clearance):
                image[i][j] = clearance_color

            if (j >= 250-clearance and j <= 265+clearance and i >= 75-clearance and i < 200):
                image[i][j] = clearance_color

            if ((j-400)**2 + (i-90)**2 <= (50+clearance)**2):
                image[i][j] = clearance_color

            # Representating the Objects

            if (j >= 150 and j <= 165 and i >= 0 and i <= 125):
                image[i][j] = object_color

            if (j >= 250 and j <= 265 and i >= 75 and i < 200):
                image[i][j] = object_color

            if ((j-400)**2 + (i-90)**2 <= 50**2):
                image[i][j] = object_color

            # Representating the Frame border

            if (j >= 0 and j < (offset)) or (i >= 0 and i < (offset)) or \
                    (i >= 200-(offset) and i < 200) or (j >= 600-(offset) and j < 600):
                image[i][j] = radius_color

            if (j >= 0 and j < (robot_radius)) or (i >= 0 and i < (robot_radius)) or \
                    (i >= 200-(robot_radius) and i < 200) or (j >= 600-(robot_radius) and j < 600):
                image[i][j] = clearance_color

    return image


def roundVals(number):
    """Rounds the given number 

    Args:
        number (float): Number to be rounded

    Returns:
        float: Rounded value
    """
    return np.round(number)


def childValidity(child):
    """Checks whether the child is valid or not

    Args:
        child (list): [[x, y, theta], cost] child node info

    Returns:
        bool: Validity of child node
    """
    if child[0][0] >= 0 and child[0][0] <= canvas.shape[1]-1 and child[0][1] >= 0 and child[0][1] <= canvas.shape[0]-1:
        if canvas[int(round(child[0][1]))][int(round(child[0][0]))] == 0:
            return True
    return False


def calculateCost(parent_x, parent_y, parent_theta, ul, ur):
    """Generates the cost and odom info for child

    Args:
        parent_x (float): x-coordinate for parent
        parent_y (float): y-coordinate for the parent
        parent_theta (float): theta for parent
        ul (int): left wheel vel
        ur (int): right wheel vel

    Returns:
        list: [[x, y, theta], cost] for the child
    """
    t = 0
    robot_wheel_radius = 3.3
    wheel_distance = 16
    dt = 0.1
    xn = parent_x
    yn = parent_y
    thetan = 3.14 * parent_theta / 180

    dxn = 0
    dyn = 0

    cost = 0
    while t < 1:
        t = t + dt
        dxn += 0.5*robot_wheel_radius * (ul + ur) * math.cos(thetan) * dt
        dyn += 0.5*robot_wheel_radius * (ul + ur) * math.sin(thetan) * dt
        thetan += (robot_wheel_radius / wheel_distance) * (ur - ul) * dt
    cost = cost + sqrt((dxn)**2 + (dyn)**2)
    thetan = 180 * (thetan) / 3.14
    return [[xn+dxn, yn+dyn, thetan], cost]


def generateChildren(node):
    """Generates all the valid children nodes of a given node

    Args:
        node (<class 'NODE'>): Given node

    Returns:
        List: lsit containing all the children nodes
    """
    # actionset
    actionset = [
        [0, rpm1],
        [rpm1, 0],
        [rpm1, rpm1],
        [0, rpm2],
        [rpm2, 0],
        [rpm2, rpm2],
        [rpm1, rpm2],
        [rpm2, rpm2]
    ]

    # Get the x, y and theta of the node
    x_cord, y_cord, theta = node.coordinates

    # Get the cost of the node
    cost = node.cost

    # Creating list for the succeeding nodes
    succ_nodes = []

    # Generating and appending the succeeding
    # nodes using all the movement functions

    for action in actionset:
        ul, ur = action
        # print(x_cord, y_cord, theta, ul, ur)

        p60 = calculateCost(x_cord, y_cord, theta, ul, ur)
        # check validity
        if childValidity(p60):
            succ_nodes.append([
                [p60[0][0], p60[0][1], p60[0][2]],
                # [roundVals(p60[0][0]), roundVals(p60[0][1]), p60[0][2]],
                cost+p60[1],
                [ul, ur]
            ])

    return succ_nodes


def checkVisitedRegion(x, y):
    """check duplicate nodes

    Args:
        x (float): x-coordinate
        y (float): y-coordinate

    Returns:
        bool: Node duplicacy flag
    """
    # Update the value of x with the given threshold
    xn = int(roundVals(x)/visited_threshold)

    # Update the value of y with the given threshold
    yn = int(roundVals(y)/visited_threshold)

    # Update the value of theta with the given threshold
    # and handle the negative values of theta

    # Check duplicacy
    if visited_mat[yn][xn] == 0:
        visited_mat[yn][xn] = 1
        return True

    return False


def aStar(start_node, goal_node):
    """Node Exploration using a* algorithm

    Args:
        start_node (<class 'NODE'>): Start point node
        goal_node (<class 'NODE'>): Goal point node

    Returns:
        dict, np.ndarray, list: node_graph, animation_canvas, animation_frames
    """
    # Create a canvas for animation
    animation_canvas = animationCanvas()

    # Create a list to save all the naimation frames
    animation_frames = []

    # Create a dictionary to store the node graph
    node_graph = {}

    # Create dictionaries for the open_list and closed_list
    open_list = {}
    closed_list = {}

    # Create a queue
    queue = []

    # Add the start node in the openlist
    open_list[str([start_node.x_cord, start_node.y_cord])] = start_node

    # Add the initial node to the heap
    heapq.heappush(queue, [start_node.cost, start_node])

    # initialize the while loop
    i = 0
    while len(queue) != 0:
        # get the element from the heap
        fetched_ele = heapq.heappop(queue)

        # fetch the node from that element
        current_node = fetched_ele[1]

        # add the node to the node graph dictionary
        node_graph[str([current_node.x_cord, current_node.y_cord])
                   ] = current_node

        # mark the node on the canvas
        cv2.circle(
            img=animation_canvas,
            center=(int(current_node.x_cord), int(current_node.y_cord)),
            radius=1,
            color=(0, 0, 255),
            thickness=-1
        )

        # check if reached to goal node
        if sqrt((current_node.x_cord-goal_node.x_cord) ** 2 + (current_node.y_cord-goal_node.y_cord)**2) < goal_threshold:
            # assign the parent
            goal_node.preced = current_node.preced

            # assign the cost
            goal_node.cost = current_node.cost

            # print message
            print("#  ")
            print("#  Found the goal node")

            # break the loop
            break

        # check if the node is in closed list
        if str([current_node.x_cord, current_node.y_cord]) in closed_list:
            continue
        # else add it to the closed list
        else:
            closed_list[str(
                [current_node.x_cord, current_node.y_cord])] = current_node

        # delete the element from the open list
        del open_list[str([current_node.x_cord, current_node.y_cord])]

        # Generate the children nodes
        child_list = generateChildren(current_node)

        # Process each child node
        for child in child_list:
            # Get the x, y, theta
            child_x, child_y, child_theta = child[0]

            # get the cost
            child_cost = child[1]

            # get the rpm values
            ul, ur = child[2]

            # # mark the child node
            cv2.circle(
                img=animation_canvas,
                center=(int(child_x), int(child_y)),
                radius=1,
                color=(0, 255, 0),
                thickness=-1
            )

            # Save the animation frame at a rate of 1000 frames
            if i % 1000 == 0:
                animation_frames.append(animation_canvas.copy())

            # skip if the child is already in the closed list
            if str([child_x, child_y]) in closed_list:
                continue

            # calculate the cost to goal for the child
            child_cost_goal = sqrt((goal_node.x_cord-child_x)**2 +
                                   (goal_node.y_cord-child_y)**2)

            # generate a node for the child
            child_node = NODE(
                coordinates=[child_x, child_y, child_theta],
                cost=child_cost,
                preced=current_node,
                cost_goal=child_cost_goal,
                uleft=ul,
                urght=ur
            )

            # check duplicate nodes
            if checkVisitedRegion(child_x, child_y):
                # check if the node is in openlist
                if str([child_x, child_y]) in open_list:
                    # if yes compare the cost of the node
                    # if it is smaller then update the cost
                    if child_node.cost < open_list[str([child_x, child_y])].cost:
                        open_list[str([child_x, child_y])].cost = child_cost
                        open_list[str([child_x, child_y])
                                  ].preced = current_node
                # else add the node in the open list
                else:
                    open_list[str([child_x, child_y])] = child_node

                # add the node to the heap
                heapq.heappush(
                    queue, [(child_cost+child_cost_goal), child_node])
        # Increase the operator
        i += 1

    return node_graph, animation_canvas, animation_frames


def backTrack(node_graph, goal_node):
    """Track the path to the goal

    Args:
        node_graph (dict)
        goal_node (<class 'NODE'>): Goal point node

    Returns:
        list: path from the goal to the start node
    """
    # create list to save the path
    path = []

    # add the goal node coordinates
    path.append([
        int(goal_node.x_cord),
        int(goal_node.y_cord),
        goal_node.theta
    ])

    # update the parent to the goal node
    parent = list(node_graph.items())[-1][1].preced

    # Continue tracking the parent until reached the start node
    while parent is not None:
        # append the coordinates
        path.append([
            int(parent.x_cord),
            int(parent.y_cord),
            parent.theta
        ])
        # update the parent
        parent = parent.preced
    return path


def saveAnimation(animation_array):
    """Generates an Animation and Saves the video

    Args:
        animation_array (_type_): _description_
    """
    print("\n")
    print("#"*80)
    print("Generating the video file.")
    # create the video object
    video = cv2.VideoWriter(
        'shortest.mp4', cv2.VideoWriter_fourcc(*'MP4V'), 50, (600, 250))

    # write all the saved frames in the video
    for i in range(len(animation_array)):
        frame = cv2.flip(animation_array[i], 0)
        video.write(frame)
        cv2.imshow("Exploration", frame)
        cv2.waitKey(1)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    video.release()

    print("Video file generated succesfully.")
    print("#"*80)
    print("\n")


def main():
    """main function
    """
    # declare the goabl variables
    global canvas, canvas_height, canvas_width
    global rpm1, rpm2, clearance
    global robot_radius
    global goal_threshold, visited_mat, visited_threshold

    # declare the canvas height and canvas width
    canvas_height, canvas_width = 200, 600

    # threshold to check the duplicate nodes
    visited_threshold = 0.5

    robot_radius = 5

    # matrix to store the info of visited region
    visited_mat = np.zeros((400, 1200), dtype=np.uint8)

    # threshold for the goal periphery
    goal_threshold = 1.5

    # Print title on the terminal
    print("#"*80)
    print("#  Reach the goal (Path Planning using A*)")
    print("#  Project-3")
    print("#  ")
    print("#  by")
    print("#  Akash Parmar")
    print("#  Jay Prajapati")
    print("#"*80)

    # initialize a loop to get input from valid points
    loop = True
    while loop:
        print("\nEnter your choice for mode of operation,")
        print("\nType 1 for selecting the parameters manually")
        print("Type 2 for preset parameters")

        # get the choice from the user for selecting the points manually or default
        choice = int(input("\nYour Choice: "))

        # get input for manual extry
        if choice == 1:
            start_x = int(input("\nEnter x coordinate of Start Point: ")) + 50
            start_y = int(input("Enter y coordinate of Start Point: ")) + 100
            start_theta = int(input("Enter theta coordinate of Start Point: "))

            goal_x = int(input("Enter x coordinate of Goal Point: ")) + 50
            goal_y = int(input("Enter y coordinate of Goal Point: ")) + 100

            clearance = int(input("Enter the clearance: "))
            if clearance != 5:
                print("\n")
                print("#"*80)
                print("#  ERROR")
                print("#  The acceptable clearance is 5 as per the instructions")
                print("#"*80)
                continue

            rpm1 = int(input("Enter the rpm1: "))
            if rpm1 < 0:
                print("\n")
                print("#"*80)
                print("#  ERROR")
                print("#  The value should be positive")
                print("#"*80)
                continue

            rpm2 = int(input("Enter the rpm2: "))
            if rpm2 < 0:
                print("\n")
                print("#"*80)
                print("#  ERROR")
                print("#  The value should be positive")
                print("#"*80)
                continue

        # default setup points
        elif choice == 2:
            start_x = 0 + 50
            start_y = 0 + 100
            start_theta = 0

            goal_x = 500 + 50
            goal_y = 50 + 100

            rpm1 = 1
            rpm2 = 2

            clearance = 5

        # error message for the invalid choices
        else:
            print("Invalid Choice")

        # display the coordinates
        print("\nStart Point = [", start_x, ", ",
              start_y, ", ", start_theta, "]")
        print("Goal Point = [", goal_x, ", ", goal_y, ", ", 0, "]")

        # make the canvas
        canvas = createCanvas()

        # check the solvability
        if checkSolvable(start_x, start_y, canvas):
            if checkSolvable(goal_x, goal_y, canvas):

                # initiale the start and the goal node
                start_node = NODE([start_x, start_y, start_theta], 0)
                goal_node = NODE([goal_x, goal_y, 0], 0)

                print("\n")
                print("#"*80)
                print("#  Finding the goal node")

                # note the start time
                start_time = time.time()

                # Explore nodes
                node_graph, animation_canvas, animation_frames = aStar(
                    start_node, goal_node)

                # generate path
                return_path = backTrack(node_graph, goal_node)[::-1]

                # note the end time
                end_time = time.time()

                print("#  ")
                print("#  The output was processed in ",
                      end_time-start_time, " seconds.")
                print("#"*80)

                # Mark the path
                for a in return_path:
                    cv2.circle(animation_canvas,
                               (a[0], a[1]), 2, (255, 0, 0), -1)
                    animation_frames.append(animation_canvas.copy())
                break

        else:
            # print the error message
            print("\n")
            print("#"*80)
            print("#  ERROR")
            print("#  The start point or goal point is on the obstacle")
            print("#"*80)

    # Visualize the exploration and save the animation
    saveAnimation(animation_frames)


if __name__ == "__main__":
    main()
