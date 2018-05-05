import abc
import collections
import random
import Queue

import numpy as np
import vis

# 1 test DijkstraStride
# 2 test common func

D_DOWN = 0
D_LEFT = 1
D_UP = 2
D_RIGHT = 3

COOR_MOVE = {D_UP: [0, 1], D_DOWN: [0, -1], D_LEFT: [-1, 0], D_RIGHT: [1, 0]}
# DIR_TO_ACTION = {D_UP: (0, 1), D_DOWN: (0, -1), D_LEFT: (-90, 1), D_RIGHT: (90, 1)}


def to_action(heading, new_heading, step=1):
    if heading == new_heading:
        return (0, step)
    elif (heading + 1) % 4 == new_heading:
        return (90, step)
    elif (new_heading + 1) % 4 == heading:
        return (-90, step)
    else:
        return (0, -1 * step)

def opposing_direction(direction):
    return (direction + 2) % 4

def turn_right(direction):
    return (direction + 1) % 4

def turn_left(direction):
    return (direction - 1) % 4

def manhattan_dist(loc_a, loc_b):
    return abs(loc_a[0]-loc_b[0]) + abs(loc_a[1]-loc_b[1])


class SearchingExploration(object):

    def __init__(self, explored_maze):
        self.explored_maze = explored_maze

    """
    Decides next move according to `loc` and its `heading`.

    Parameters
    ----------
    loc : tuple (int, int)
        Tuple of location, in format of (0, 0).
    heading : int
        Must be D_DOWN, D_LEFT, D_UP, D_RIGHT.

    Returns
    -------
    next_action : tuple of (rotation: int, movement: int), or tuple of ('Reset', 'Reset')
        For example: (90, 3).
    """

    @abc.abstractmethod
    def next_move(self, loc, heading):
        pass


class RunShortestPath(object):
    def __init__(self, explored_maze):
        self.explored_maze = explored_maze

    """
    Computes list of actions that would follow shortest path starting `loc_start` with `heading`.

    Parameters
    ----------
    loc_start : tuple (int, int)
        Tuple of location, in format of (0, 0).
    heading : int
        Must be D_DOWN, D_LEFT, D_UP, D_RIGHT.

    Returns
    -------
    action_list : list of tuple (rotation: int, movement: int)
        For example: [(90, 3), [0, 3)]
    """
    @abc.abstractmethod
    def compute_p2p_action(self, loc_start, heading, *args):
        pass

    """
    Returns next action computed previously by `compute_p2p_action`.

    Returns
    -------
    action : tuple of (rotation: int, movement: int)
    """
    @abc.abstractmethod
    def next_action(self):
        pass


class Robot(object):
    # run 1: searching_exploration
    phase_searching_exploration = 1
    # run 1: goal arrived and no continuing exploration, return ('Reset', 'Reset')
    phase_terminate_run1 = 2
    # run 1: continuing exploration
    phase_continuing_exploration = 3
    # run 2
    phase_run2 = 4

    def __init__(self, maze_dim):
        """
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        """

        self.location = (0, 0)
        self.heading = D_UP
        self.maze_dim = maze_dim
        self.explored_maze = ExploredMaze(maze_dim)

        self.searching_exploration = SearchingExploration_GoalOriented(self.explored_maze)
        self.continuing_exploration = None
        self.run_shortest_path = DijkstraStride(self.explored_maze, max_step=3)

        self.phase = Robot.phase_searching_exploration

        goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        self.goal_cells = []
        for r in goal_bounds:
            for c in goal_bounds:
                self.goal_cells.append((r, c))

    def next_move(self, sensors):
        """
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returning the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        """
        for i in range(3):
            self.explored_maze.sensor_update(self.location, (self.heading - 1 + i) % 4,  sensors[i])

        if self.phase == Robot.phase_searching_exploration:
            rotation, movement = self.searching_exploration.next_move(self.location, self.heading)
            print("run1: " + str(self.location) + " action: " + str(rotation) + ", " + str(movement))
            # update todo
            self.update_location(rotation, movement)
            # reached goal, continue if continuing exploration is set
            if self.location in self.goal_cells:
                if self.continuing_exploration is not None:
                    self.phase = Robot.phase_continuing_exploration
                else:
                    self.phase = Robot.phase_terminate_run1
            return rotation, movement

        elif self.phase == Robot.phase_terminate_run1:
            self.phase = Robot.phase_run2
            # vis.draw_explored_maze(self.explored_maze, self.location, self.heading)
            self.update_location('Reset', 'Reset')
            return 'Reset', 'Reset'

        elif self.phase == Robot.phase_continuing_exploration:
            rotation, movement = self.continuing_exploration.next_move(self.location, self.heading)
            self.update_location(rotation, movement)
            if (rotation, movement) == ('Reset', 'Reset'):
                self.phase = Robot.phase_run2
            return rotation, movement

        elif self.phase == Robot.phase_run2:
            # @todo update current
            self.run_shortest_path.compute_p2p_action((0, 0), D_UP, *self.goal_cells)
            rotation, movement = self.run_shortest_path.next_action()
            print("run2: " + str(self.location) + " action: " + str(rotation) + ", " + str(movement))
            self.update_location(rotation, movement)
            return rotation, movement

    """
    Updates `self.location` and `self.heading` according to `rotation` and `movement`.
    
    Parameters
    ----------
    rotation : int or 'Reset'
    movement : int or 'Reset'

    Raises
    ------
    Exception
        If (rotation, movement) cannot applied to current status, e.g. hit a wall.
    """
    def update_location(self, rotation, movement):
        if (rotation, movement) == ('Reset', 'Reset'):
            self.location = (0, 0)
            self.heading = D_UP
            return

        # perform rotation
        if rotation == -90:  # counterclockwise
            self.heading = turn_left(self.heading)
        elif rotation == 90:  # clockwise
            self.heading = turn_right(self.heading)
        elif rotation == 0:
            pass
        else:
            raise Exception("Invalid rotation in update_location: " + str(rotation))
        pass

        if abs(movement) > 3:
            raise Exception("Movement exceeds limit: " + str(movement))

        while movement:
            if movement > 0:
                if self.explored_maze.is_permissible(self.location, self.heading):
                    self.location = self.explored_maze.loc_of_neighbour(self.location, self.heading)
                    if self.location is None:
                        raise Exception("Movement stopped by outer wall")
                    movement -= 1
                else:
                    raise Exception("Movement stopped by wall")
            else:  # < 0
                if self.explored_maze.is_permissible(self.location, opposing_direction(self.heading)):
                    self.location = self.explored_maze.loc_of_neighbour(self.location, opposing_direction(self.heading))
                    if self.location is None:
                        raise Exception("Movement stopped by outer wall")
                    movement += 1
                else:
                    raise Exception("Movement stopped by wall")


class ExploredMaze(object):

    def __init__(self, maze_dim):
        self.maze_dim = maze_dim
        self.cell_map = {}
        for r in range(maze_dim):
            for c in range(maze_dim):
                cell = Cell((r, c))
                self.cell_map[(r, c)] = cell
                if r == 0:
                    cell.surroundings[D_LEFT] = Cell.WALL
                if r == maze_dim - 1:
                    cell.surroundings[D_RIGHT] = Cell.WALL
                if c == 0:
                    cell.surroundings[D_DOWN] = Cell.WALL
                if c == maze_dim - 1:
                    cell.surroundings[D_UP] = Cell.WALL

    """
    Updates maze connectivity status with sensor information.

    Parameters
    ----------
    loc : tuple (int, int)
        Tuple of location, in format of (0, 0).
    direction : int
        Must be D_DOWN, D_LEFT, D_UP, D_RIGHT.
    depth : int
        0 indicates wall;
        other positive number indicating how far from this `loc` mouse can move forward.
    
    """
    def sensor_update(self, loc, direction, depth):
        if depth == 0:
            self.cell_map[loc].set_wall(direction)
            neighbour_loc = self.loc_of_neighbour(loc, direction)
            if neighbour_loc is not None:
                self.cell_map[neighbour_loc].set_wall(opposing_direction(direction))
        else:  # depth > 0
            neighbour_loc = self.loc_of_neighbour(loc, direction)
            self.cell_map[loc].connect(direction, self.cell_map[neighbour_loc])
            self.sensor_update(neighbour_loc, direction, depth - 1)

    """
    Return location relative to `loc`. If the resulting location is out of maze, None is returned.
    
    Parameters
    ----------
    loc : tuple (int, int)
        Tuple of location, in format of (0, 0).
    direction : int
        Must be D_DOWN, D_LEFT, D_UP, D_RIGHT.
    step : int
        distance to `loc`, optional
    
    Returns
    -------
    new_location: tuple of (rotation: int, movement: int), or None
        None when out of maze.
    """
    def loc_of_neighbour(self, loc, direction, step=1):
        neighbour = (loc[0] + step * COOR_MOVE[direction][0], loc[1] + step * COOR_MOVE[direction][1])
        if neighbour[0] < 0 or neighbour[0] >= self.maze_dim or neighbour[1] < 0 or neighbour[1] >= self.maze_dim:
            return None
        else:
            return neighbour

    """
    Checks whether the mouse can move from a location along a direction according to current connectivity status.
    
    Parameters
    ----------
    loc : tuple (int, int)
        Tuple of location, in format of (0, 0).
    direction : int
        Must be D_DOWN, D_LEFT, D_UP, D_RIGHT.
    step : int
        distance to `loc`, optional
    
    Returns
    -------
    is_permissible: True, False, None
        None indicates there is not enough information along the way, i.e. not sure there is wall between two cells in the way.
    """
    def is_permissible(self, loc, direction, step=1):
        while step > 0:
            if not self.cell_map[loc].is_permissible(direction):
                return self.cell_map[loc].is_permissible(direction)

            step = step - 1
            loc = self.loc_of_neighbour(loc, direction)
        # reached destination loc
        return True


    def get_neighbours(self, loc):
        return self.cell_map[loc].get_neighbours()

    """
    Computes set of cells that can be reached from (0, 0).

    Parameters
    ----------

    Returns
    -------
    reached_set: set of location tuple (int, int)
    """
    def compute_reachable_cells(self):
        reached_set = set()
        q = Queue.Queue()
        q.put((0, 0))

        while not q.empty():
            loc = q.get()
            reached_set.add(loc)
            for neighbour_loc in self.get_neighbours(loc):
                if neighbour_loc not in reached_set:
                    q.put(neighbour_loc)
        return reached_set

    def get_unexplored(self, loc):
        return self.cell_map[loc].get_unexplored()

    # for test purpose
    def _set_neighbour(self, loc, direction, is_wall):
        if is_wall:
            self.cell_map[loc].set_wall(direction)
        else:
            neighbour_loc = self.loc_of_neighbour(loc, direction)
            if neighbour_loc is None:
                raise Exception("No neighbour of " + str(self) + ", direction = " + str(direction))
            self.cell_map[loc].connect(direction, self.cell_map[neighbour_loc])


    # def get_cell(self, loc):
    #     return self.cell_map[loc]


class Cell(object):
    UNEXPLORED = -1
    WALL = 1

    def __init__(self, loc):
        self.loc = loc
        self.surroundings = [Cell.UNEXPLORED for x in range(4)]

    """
    Connects this cell with a neighbouring cell and vice versa.

    Parameters
    ----------
    direction : int
        Must be D_DOWN, D_LEFT, D_UP, D_RIGHT.
    node : tuple (int, int)
        neighbouring location.
        
    Raises
    ------
    Exception
        When wall between these 2 nodes already established.
    """
    def connect(self, direction, node):
        node_direction = opposing_direction(direction)
        if self.surroundings[direction] == Cell.WALL and node.surroundings[node_direction] == Cell.WALL:
            raise Exception("Trying to connect nodes where WALL already exists, " + str(self) + " <-> " + str(node) )
        self.surroundings[direction] = node
        node.surroundings[node_direction] = self

    """
    Sets a wall to one edge of current cell.

    Parameters
    ----------
    direction : int
        Must be D_DOWN, D_LEFT, D_UP, D_RIGHT, the direction relative to current cell.

    Raises
    ------
    Exception
        When these 2 nodes are already connected.
    """
    def set_wall(self, direction):
        if self.surroundings[direction] == Cell.UNEXPLORED or self.surroundings[direction] == Cell.WALL:
            self.surroundings[direction] = Cell.WALL
        else:
            raise Exception("Already Connected for " + str(self) + ", " + str(direction))

    """
    Checks whether the mouse can move one step from current cell along the direction.

    Parameters
    ----------
    direction : int
        Must be D_DOWN, D_LEFT, D_UP, D_RIGHT.

    Returns
    -------
    is_permissible: True, False, None
        None indicates there is not enough information, i.e. not sure there is wall in that direction.
    """
    def is_permissible(self, direction):
        if self.surroundings[direction] == Cell.WALL:
            return False
        elif self.surroundings[direction] == Cell.UNEXPLORED:
            return None
        else:
            return True

    """
    Gets all connected neighbours currently explored.

    Parameters
    ----------

    Returns
    -------
    neighbour_list: list of location tuple (int, int)
    """
    def get_neighbours(self):
        return [x.loc for x in self.surroundings if x != Cell.WALL and x != Cell.UNEXPLORED ]

    def get_unexplored(self):
        return len([x for x in self.surroundings if x == Cell.UNEXPLORED ])

    def __str__(self):
        return self.node_name() + " => " + str(self.surroundings)

    def node_name(self):
        return str(self.loc)

    def __eq__(self, other):
        if not isinstance(other, Cell):
            return False
        return self.loc == other.loc


class SearchingExploration_WeightedRandom(SearchingExploration):
    def __init__(self, explored_maze):
        self.explored_maze = explored_maze

    def next_move(self, loc, heading):
        prob = [1, 2, 9, 5]  # backward, turn left, forward, turn right
        ranges = [[], [], [], []]
        total = 0
        for direction in range(4):
            has_prob = False
            if self.explored_maze.is_permissible(loc, direction):
                has_prob = True
            if has_prob:
                ranges[direction] = range(total, total + prob[direction])
                total += prob[direction]
        rnd = random.randint(0, total - 1)
        for direction in range(4):
            if rnd in ranges[direction]:
                return to_action(heading, direction)


class SearchingExploration_GoalOriented(SearchingExploration):

    def next_move(self, loc, heading):
        reachable_cells = self.explored_maze.compute_reachable_cells()
        # compute those cells unexplored in reachable_cells
        unexplored_cells = [cell for cell in reachable_cells if self.explored_maze.get_unexplored(cell) > 0]
        # compute overall cost = steps_to_cell + (4-unexplored_num) + manhattan
        CellWithDist = collections.namedtuple("CellWithDist",
                                              ["loc", "steps", "explored", "h_manhattan", "first_action"],
                                              verbose=False, rename=False)
        cells = []
        for cell in unexplored_cells:
            actions = dijkstra_shortest_path(self.explored_maze, loc, heading, 3, cell)
            cell_with_dist = CellWithDist(loc=cell,
                                          steps=len(actions),
                                          explored=4 - self.explored_maze.get_unexplored(cell),
                                          h_manhattan=1,
                                          first_action=actions[0])
            cells.append(cell_with_dist)

        sorted_cells = sorted(cells, key=lambda c: c.steps + c.explored + c.h_manhattan)
        return sorted_cells[0].first_action


class DijkstraStride(RunShortestPath):

    def __init__(self, explored_maze, max_step=1):
        super(DijkstraStride, self).__init__(explored_maze)
        self.max_step = max_step
        self.action_iter = None
        self.actions = None

    def next_action(self):
        return self.action_iter.next()

    def compute_p2p_action(self, loc_start, heading, *args):
        if self.action_iter is None:
            self.actions = dijkstra_shortest_path(self.explored_maze, loc_start, heading, self.max_step, *args)
            self.action_iter = iter(self.actions)
        else:
            return self.actions


def dijkstra_shortest_path(explored_maze, loc_start, heading, max_step=1, *args):
    reached_set = set()
    q = Queue.Queue()

    reached_set.add(loc_start)

    actions = []

    # tuple( loc, parent_rec, direction, action from parent )
    LocRecord = collections.namedtuple("LocRecord", ["loc", "parent_rec", "heading", "action"], verbose=False,
                                       rename=False)
    loc_rec = LocRecord(loc=loc_start, parent_rec=None, heading=heading, action=None)
    q.put(loc_rec)

    while True:
        loc_rec = q.get()
        loc = loc_rec.loc

        if loc in args:
            while loc_rec.action is not None:
                actions.append(loc_rec.action)
                loc_rec = loc_rec.parent_rec
            break

        for direction in range(4):
            for step in range(1, max_step + 1):
                neighbour = explored_maze.loc_of_neighbour(loc, direction, step)
                if neighbour is not None and neighbour not in reached_set and \
                        explored_maze.is_permissible(loc, direction, step) is True:
                    q.put(LocRecord(loc=neighbour, parent_rec=loc_rec, heading=direction,
                                    action=to_action(loc_rec.heading, direction, step)))
                    reached_set.add(neighbour)

    actions.reverse()
    return actions
