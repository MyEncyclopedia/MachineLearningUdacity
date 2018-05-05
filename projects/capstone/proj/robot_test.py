import unittest
import maze
import robot
import show_progress

def init_explored_maze(maze):
    explored_maze = robot.ExploredMaze(maze.dim)
    dir_map = {robot.D_DOWN: "down", robot.D_LEFT: "left", robot.D_RIGHT: "right", robot.D_UP: "up"}

    for r in range(maze.dim):
        for c in range(maze.dim):
            for direction in range(4):
                explored_maze._set_neighbour((r, c), direction, not maze.is_permissible((r, c), dir_map[direction]))

    return explored_maze



class TestShortestPath(unittest.TestCase):

    def test_shortest(self):
        testmaze = maze.Maze("test_maze_01.txt")
        explored_maze = init_explored_maze(testmaze)
        shortest_path = robot.DijkstraStride(explored_maze)
        actions = shortest_path.compute_p2p_action((0, 0), (0, 3))
        self.assertEqual([(0, 3)], actions)


if __name__ == '__main__':
    # testmaze = maze.Maze("test_maze_01.txt")


    # show_progress.draw_explored_maze(init_explored_maze(testmaze), (0, 0), robot.D_UP)
    unittest.main()
