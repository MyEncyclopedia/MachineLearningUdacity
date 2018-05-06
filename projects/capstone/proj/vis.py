from maze import Maze
import turtle
import robot

sq_size = 54
origin = 0
window = None


def show_maze(maze_file):
    """
    Draw maze specified by `maze_file` input file. Quit app by clicking the window.

    Parameters
    ----------
    maze_file : string
    """
    maze = Maze(maze_file)
    draw_explored_maze(_init_explored_maze(maze), (0, 0), robot.D_UP, None)
    window.exitonclick()


def show_maze_shortest(maze_file, max_step=3):
    """
    Draw shortest path solution of the maze specified by `maze_file` input file. Quit app by clicking the window.

    Parameters
    ----------
    maze_file : string
    max_step : int, default 3
        max step allowed in each action.
    """
    maze = Maze(maze_file)
    explored_maze = _init_explored_maze(maze)
    actions = robot.dijkstra_shortest_path(explored_maze, (0, 0), robot.D_UP, max_step, *explored_maze.goal_cells)
    print("Total actions: " + str(len(actions)))
    print(actions)
    cell_seq = []
    heading = robot.D_UP
    cell = (0, 0)
    cell_seq.append((cell, heading))
    for action in actions:
        heading = robot.to_direction(heading, action)
        direction = heading
        if action[1] < 0:
            direction = robot.opposing_direction(heading)
        cell = explored_maze.loc_of_neighbour(cell, direction, step=abs(action[1]))
        cell_seq.append((cell, heading))

    draw_explored_maze(explored_maze, (0, 0), robot.D_UP, cell_seq)
    window.exitonclick()


def draw_explored_maze(explored_maze, cell, heading, cell_seq):
    """
    Draw explored maze, highlighting current mouse location and cell sequence it passes.

    Parameters
    ----------
    explored_maze : ExploredMaze
    cell : list of location tuple (int, int)
    heading : int
    cell_seq : None or list of (rotation: int, step: int)
    """
    global sq_size, origin
    global window
    origin = explored_maze.maze_dim * sq_size / -2

    window = turtle.Screen()
    wally = turtle.Turtle()
    window.clear()
    wally.speed(0)

    to_draw_set = set()
    to_draw_set.add((0, 0))
    done_set = set()
    while len(to_draw_set) > 0:
        loc = to_draw_set.pop()
        fill_color = "grey"
        if loc == cell:
            fill_color = "yellow"
        elif loc in explored_maze.goal_cells:
            fill_color = "green"
        elif explored_maze.get_unexplored(loc) > 0:
            fill_color = "pink"
        _draw_cell(wally, explored_maze, loc, fill_color)
        done_set.add(loc)
        for neighbour in explored_maze.get_neighbours(loc):
            if neighbour not in done_set:
                to_draw_set.add(neighbour)

    if cell_seq is not None:
        wally.pencolor("blue")
        first_point = True
        for loc, h in cell_seq:
            if first_point:
                wally.penup()
                wally.goto(*_square_center(loc))
                wally.pendown()
                first_point = False
                continue

            wally.pendown()
            wally.goto(*_square_center(loc))
            _draw_arrow(wally, h)

    _draw_heading(wally, cell, heading)


def _square_top_left(loc):
    global sq_size, origin
    return origin + sq_size * loc[0], origin + sq_size * (loc[1] + 1)


def _square_top_right(loc):
    global sq_size, origin
    return origin + sq_size * (loc[0] + 1), origin + sq_size * (loc[1] + 1)


def _square_bottom_left(loc):
    global sq_size, origin
    return origin + sq_size * loc[0], origin + sq_size * loc[1]


def _square_bottom_right(loc):
    global sq_size, origin
    return origin + sq_size * (loc[0] + 1), origin + sq_size * loc[1]


def _square_center(loc):
    global sq_size, origin
    return origin + sq_size * loc[0] + sq_size/2, origin + sq_size * loc[1] + sq_size/2


def _draw_cell(wally, explored_maze, loc, fill_color='yellow'):
    wally.penup()
    wally.goto(origin + sq_size * loc[0] + sq_size/2, origin + sq_size * (loc[1]+1) - sq_size/2 - sq_size/3)
    wally.setheading(0)
    wally.pendown()

    wally.color(fill_color)
    wally.fillcolor(fill_color)
    wally.begin_fill()
    wally.circle(sq_size/3)
    wally.end_fill()
    wally.penup()

    wally.pensize(2)
    wally.goto(origin + sq_size * loc[0] + sq_size/2 - 10, origin + sq_size * (loc[1]+1) - sq_size/2 - sq_size/3 + 10)
    wally.color("black")
    wally.write(str(loc), font=("Arial", 10, "normal"))

    edge_list = [(robot.D_UP,    _square_top_left(loc), 0),
                 (robot.D_RIGHT, _square_bottom_right(loc), 90),
                 (robot.D_DOWN,  _square_bottom_left(loc), 0),
                 (robot.D_LEFT,  _square_bottom_left(loc), 90)]
    for edge in edge_list:
        is_permissible = explored_maze.is_permissible(loc, edge[0])
        if not is_permissible:
            wally.goto(*edge[1])
            wally.setheading(edge[2])
            if is_permissible is None:
                wally.color("grey")
            else:
                wally.color("black")
            wally.pendown()
            wally.forward(sq_size)
            wally.penup()


def _draw_heading(wally, loc, heading):
    wally.goto(*_square_center(loc))
    wally.pensize(5)
    wally.color("red")
    wally.pendown()
    if heading == robot.D_UP:
        wally.setheading(90)
    elif heading == robot.D_RIGHT:
        wally.setheading(0)
    elif heading == robot.D_DOWN:
        wally.setheading(270)
    elif heading == robot.D_LEFT:
        wally.setheading(180)
    wally.forward(sq_size/3)
    _draw_arrow(wally, heading)
    wally.penup()


def _draw_arrow(wally, heading):
    wally.pendown()
    pos = wally.position()
    coor = {
        robot.D_UP: [(-1, -1), (1, -1)],
        robot.D_DOWN: [(-1, 1), (1, 1)],
        robot.D_LEFT: [(1, 1), (1, -1)],
        robot.D_RIGHT: [(-1, 1), (-1, -1)],
    }
    wally.goto(pos[0] + coor[heading][0][0] * sq_size/6, pos[1] + coor[heading][0][1] * sq_size/6)

    wally.penup()
    wally.goto(*pos)
    wally.pendown()
    wally.goto(pos[0] + coor[heading][1][0] * sq_size/6, pos[1] + coor[heading][1][1] * sq_size/6)

    wally.goto(*pos)
    wally.penup()


def _init_explored_maze(maze):
    explored_maze = robot.ExploredMaze(maze.dim)
    dir_map = {robot.D_DOWN: "down", robot.D_LEFT: "left", robot.D_RIGHT: "right", robot.D_UP: "up"}

    for r in range(maze.dim):
        for c in range(maze.dim):
            for direction in range(4):
                explored_maze._set_neighbour((r, c), direction, not maze.is_permissible((r, c), dir_map[direction]))

    return explored_maze


if __name__ == '__main__':
    show_maze_shortest("test_maze_01.txt")

