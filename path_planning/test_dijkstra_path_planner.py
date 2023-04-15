import unittest
from dijkstra_path_planner import *


OCCUPANCY_GRID = [[0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0],
                  [0,0,0,0,0]]

class TestPathPlanner(unittest.TestCase):

    def test_center_cell_returns_four_neighbors(self):
        expected = list()
        parent_node = Node(grid_cost = 0, coordinate_pt = (2,2), parent = None)

        expected.append(Node(grid_cost = 1, coordinate_pt = (2,1), parent = parent_node)) # LEFT
        expected.append(Node(grid_cost = 1, coordinate_pt = (2,3), parent = parent_node)) # RIGHT
        expected.append(Node(grid_cost = 1, coordinate_pt = (1,2), parent = parent_node)) # UP
        expected.append(Node(grid_cost = 1, coordinate_pt = (3,2), parent = parent_node)) # DOWN

        actual = find_neighbours(node=parent_node, width=5, height=5, gridmap=OCCUPANCY_GRID, resolution=STEP_COST)
        
        # assert coordinates are as expected
        self.assertEqual(expected[0].get_coordinate_pt(), actual[0].get_coordinate_pt())
        self.assertEqual(expected[1].get_coordinate_pt(), actual[1].get_coordinate_pt())
        self.assertEqual(expected[2].get_coordinate_pt(), actual[2].get_coordinate_pt())
        self.assertEqual(expected[3].get_coordinate_pt(), actual[3].get_coordinate_pt())
        
        # assert grid cost is as expected
        self.assertEqual(expected[0].grid_cost, actual[0].grid_cost)
        self.assertEqual(expected[1].grid_cost, actual[1].grid_cost)
        self.assertEqual(expected[2].grid_cost, actual[2].grid_cost)
        self.assertEqual(expected[3].grid_cost, actual[3].grid_cost)
        

    def test_bottom_left_cell_returns_two_neighbors(self):
        pass

    def test_bottom_right_cell_returns_two_neighbors(self):
        pass

    def test_upper_left_cell_returns_two_neighbors(self):
        pass
    
    def test_upper_right_cell_returns_two_neighbors(self):
        pass

    def test_cell_ignores_occupied_neighbor(self):
        pass


if __name__ == '__main__':
    unittest.main()