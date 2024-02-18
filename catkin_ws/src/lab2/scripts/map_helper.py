#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose


def get_neighbors(loc, my_map):
    """
        returns the legal neighbors of loc
        :param loc: tuple of location
        :return: list of tuples
    """
    location = [(loc[0] + i, loc[1] + j) for i in xrange(-1, 2, 1) for j in xrange(-1, 2, 1)]
    legal = filter(lambda x: is_valid_loc(x, my_map), location)  # filter(self.is_valid_loc, location)
    return legal


def is_valid_loc(loc, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """

    return loc[0] >= 0 and loc[0] < my_map.info.width and \
           loc[1] >= 0 and loc[1] < my_map.info.height and \
           my_map.data[loc[0] + loc[1] * my_map.info.width] == 0


def convert_location(loc, my_map):
    """converts points to the grid"""
    x = (loc[0] + my_map.info.origin.position.x) / my_map.info.resolution
    y = (loc[1] - my_map.info.origin.position.y) / my_map.info.resolution
    return (int(x), int(y))


def world_to_map(x, y, my_map):
    """
        converts a point from the world to the map
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """
    fix_x = (x - my_map.info.origin.position.x) / my_map.info.resolution
    fix_y = (y - my_map.info.origin.position.y) / my_map.info.resolution
    return int(fix_x), int(fix_y)


def map_to_world(x, y, my_map):
    """
        converts a point from the map to the world
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """
    fix_x = (x + 0.5) * my_map.info.resolution + my_map.info.origin.position.x
    fix_y = (y + 0.5) * my_map.info.resolution + my_map.info.origin.position.y
    return fix_x, fix_y, 0


def to_cells(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """
    c = GridCells()
    c.header = my_map.header
    c.cell_width =  my_map.info.resolution
    c.cell_height = my_map.info.resolution
    cells = [Point(*map_to_world(l[0], l[1], my_map)) for l in points]
    c.cells = cells
    return c


def to_poses(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """

    pose_array = PoseArray()
    pose_array.header = my_map.header
    for pt in points:
        pose = Pose()
        x,y,z = map_to_world(pt[0], pt[1], my_map)
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose_array.poses.append(pose)
    return pose_array

def index_to_point(point, my_map):
    x = point % my_map.info.width
    y = point / my_map.info.width
    return x, y


def point_to_index(location, my_map):
    return location[0] + location[1] * my_map.info.width
