from maze import Maze
import turtle
import sys
import robot

sq_size = 50
origin = 0


def square_top_left(loc):
    global sq_size, origin
    return origin + sq_size * loc[0], origin + sq_size * (loc[1] + 1)


def square_top_right(loc):
    global sq_size, origin
    return origin + sq_size * (loc[0] + 1), origin + sq_size * (loc[1] + 1)


def square_bottom_left(loc):
    global sq_size, origin
    return origin + sq_size * loc[0], origin + sq_size * loc[1]


def square_bottom_right(loc):
    global sq_size, origin
    return origin + sq_size * (loc[0] + 1), origin + sq_size * loc[1]


def square_center(loc):
    global sq_size, origin
    return origin + sq_size * loc[0] + sq_size/2, origin + sq_size * loc[1] + sq_size/2


def draw_explored_maze(explored_maze, cell, heading):
    global sq_size, origin
    origin = explored_maze.maze_dim * sq_size / -2

    window = turtle.Screen()
    wally = turtle.Turtle()
    window.clear()
    # wally.clear()
    wally.speed(0)

    to_draw_set = set()
    to_draw_set.add((0, 0))
    done_set = set()
    while len(to_draw_set) > 0:
        loc = to_draw_set.pop()
        fill_color = "blue"
        if loc == cell:
            fill_color = "yellow"
        elif explored_maze.get_unexplored(loc) > 0:
            fill_color = "red"
        draw_cell(wally, explored_maze, loc, fill_color)
        if loc == cell:
            draw_heading(wally, cell, heading)
        done_set.add(loc)
        for neighbour in explored_maze.get_neighbours(loc):
            if neighbour not in done_set:
                to_draw_set.add(neighbour)


def draw_cell(wally, explored_maze, loc, fill_color='yellow'):
    wally.penup()
    # wally.down()
    # draw cell
    wally.goto(origin + sq_size * loc[0] + sq_size/2 , origin + sq_size * (loc[1]+1) - sq_size/2 - sq_size/3)
    wally.setheading(0)
    # wally.goto(origin + sq_size * (loc[0]+1), origin + sq_size * (loc[1]+1))
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

    edge_list = [(robot.D_UP,    square_top_left(loc), 0),
                 (robot.D_RIGHT, square_bottom_right(loc), 90),
                 (robot.D_DOWN,  square_bottom_left(loc), 0),
                 (robot.D_LEFT,  square_bottom_left(loc), 90)]
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


def draw_heading(wally, loc, heading):
    wally.goto(origin + sq_size * loc[0] + sq_size/2 , origin + sq_size * (loc[1]+1) - sq_size/2)
    wally.pensize(5)
    wally.color("black")
    wally.pendown()
    if heading == robot.D_UP:
        wally.setheading(90)
    elif heading == robot.D_RIGHT:
        wally.setheading(0)
    elif heading == robot.D_DOWN:
        wally.setheading(270)
    elif heading == robot.D_LEFT:
        wally.setheading(180)
    wally.forward(sq_size/2)
    wally.penup()


if __name__ == '__main__':
    '''
    This function uses Python's turtle library to draw a picture of the maze
    given as an argument when running the script.
    '''

    # Initialize the window and drawing turtle.
    window = turtle.Screen()
    wally = turtle.Turtle()
    wally.speed(10)
    wally.hideturtle()
    wally.penup()

    # maze centered on (0,0), squares are 20 units in length.
    sq_size = 20
    # origin = testmaze.dim * sq_size / -2

    turtle.begin_fill()  # Begin the fill process.
    turtle.down()  # "Pen" down?
    for i in range(4):  # For each edge of the shape
        turtle.forward(40)  # Move forward 40 units
        turtle.left(90)  # Turn ready for the next edge
    turtle.up()  # Pen up
    turtle.end_fill()  # End fill.


    window.exitonclick()