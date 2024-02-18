#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import GridCells, Path
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetPlan
from nav_msgs.srv import GetPlanResponse
from nav_msgs.msg import OccupancyGrid
import map_helper as helper
import PriorityQueue


class A_Star:

    def __init__(self):

        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """

        rospy.init_node("a_star")  # start node
        rospy.sleep(10)
        self.paint = rospy.get_param("paint", True)  # show the grid cells
        rospy.Service('a_star', GetPlan, self.handle_a_star)  # service call
        self.path_pub = rospy.Publisher("/path", Path, queue_size=1)  # publish the path
        self.path_cell_pub = rospy.Publisher("/path_set", GridCells, queue_size=1)  # grid cells for path
        self.closed_cell_pub = rospy.Publisher("/closed_set", GridCells, queue_size=1)  # grid cells for closed set
        self.frontier_cell_pub = rospy.Publisher("/wavefront_set", GridCells, queue_size=1)  # grid cells for frontier
        self.goal_cell_pub = rospy.Publisher("/goal", GridCells, queue_size=1)  # grid cells for frontier

        rospy.spin()

    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """

        self.dynamic_map_client()
        start = (req.start.pose.position.x, req.start.pose.position.y)  # Get start Pose
        goal = (req.goal.pose.position.x, req.goal.pose.position.y)  # Get goal Pose
        cell = helper.to_cells([goal], self.my_map)
        self.goal_cell_pub.publish(cell)
        fix_start = helper.world_to_map(start[0],start[1],self.my_map)#self.world_to_map(*start)
        fix_goal = helper.world_to_map(goal[0],goal[1],self.my_map)#self.world_to_map(*goal)
        path = self.a_star(fix_start, fix_goal)  # run A*
        op_path = self.reconstruct_path(fix_start, fix_goal, path)  # get way points
        ros_path = Path()
        ros_path = self.publish_path(op_path)  # type: Path
        return GetPlanResponse(ros_path)

    def dynamic_map_client(self):

        """
            Service call to get map and set class variables
            This can be changed to call the expanded map
            :return:
        """

        rospy.wait_for_service('/expanded_map')
        try:
            s = rospy.ServiceProxy('/expanded_map', GetMap)
            resp = s()
            self.my_map = resp.map
            self._map = resp.map.data
            self._info = resp.map.info
            self._header = resp.map.header
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def a_star(self, start, goal):
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """

        # Set up variables
        open_set = PriorityQueue.PriorityQueue()
        frontier = []
        came_from = {}
        cost_so_far = {}
        open_set.put(start, 0)
        frontier.append(start)
        came_from[start] = start
        cost_so_far[start] = 0


        while not open_set.empty():

            current = open_set.get()
            frontier.remove(current)

            if current == goal:
                rospy.loginfo("end")
                break

            for next_node in helper.get_neighbors(current,self.my_map):
                new_cost = cost_so_far[current] + self.move_cost(current, next_node)
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.euclidean_heuristic(goal, next_node)
                    open_set.put(next_node, priority)
                    came_from[next_node] = current
                    frontier.append(next_node)

            # controls if grid cells are shown
            if self.paint:
                self.paint_cells(frontier, came_from.values())

        return came_from

    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
        return math.hypot(point1[0] - point2[0], point1[1] - point2[1])

    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        return math.hypot(current[0] - next[0], current[1] - next[1])


    def reconstruct_path(self, start, goal, came_from):
        """
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
       """
        current = goal
        my_path = []

        while current != start:
            my_path.append(current)
            current = came_from[current]
        my_path.append(start)
        my_path.reverse()
        op_path = self.optimize_path(my_path)

        return op_path

    def optimize_path(self, path):
        """
            remove redundant points in hte path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        op_path = []
        op_path.append(path[0])

        for ii in xrange(2, len(path) - 1):
            temp_dx, temp_dy = (path[ii][0] - path[ii - 1][0]), (path[ii][1] - path[ii - 1][1])
            temp_dx2, temp_dy2 = (path[ii][0] - path[ii + 1][0]), (path[ii][1] - path[ii + 1][1])

            if (temp_dx and temp_dx2) and (temp_dy and temp_dy2):
                rospy.loginfo(path[ii])
                op_path.append(path[ii - 1])
                op_path.append(path[ii])
                op_path.append(path[ii + 1])

        op_path.append(path[-1])

        return op_path

    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:
        """
        self.closed_cell_pub.publish(helper.to_cells(came_from,self.my_map))
        self.frontier_cell_pub.publish(helper.to_cells(frontier,self.my_map))


    def publish_path(self, points):
        """
            Create a Path() and publishes the cells if Paint is true
            :param points: list of tuples of the path
            :return: Path()
        """

        if self.paint:
            path_cells = helper.to_cells(points, self.my_map)
            self.path_cell_pub.publish(path_cells)

        path = Path()  # type: Path
        path.header = self._header
        for point in points:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.z = point[1]
            pose.pose.position.z = 0
            path.poses.append(pose)

        return path


if __name__ == '__main__':
    aStar = A_Star()
