# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque

# needed for distance calculation
import math
import numpy as np

# 1 = constant
# 2 = euclidian
# 3 = manhattan
# 4 = octile
# otherwise = 0
MODE = 4

# get the distance from a cell's parent to the cell given
def costFromParent(cell, parent):
	if parent != None:
		delta_x = cell.coords[0] - parent.coords[0]
		delta_y = cell.coords[1] - parent.coords[1]
		costFromParent = math.sqrt(delta_x**2 + delta_y**2)
	else:
		costFromParent = 0
	return costFromParent

# the actual heuristic that determines the behaviour of A*, which we need to change around
def variableHeuristic(cell, goal_cell):
	if MODE == 1:
		# this should perform A* with a non-negative constant
		return 20

	elif MODE == 2:
		# this should perform A* with euclidian distance heuristic
		delta_x = abs(cell.coords[0] - goal_cell.coords[0])
		delta_y = abs(cell.coords[1] - goal_cell.coords[1])
		distance = math.sqrt(delta_x**2 + delta_y**2)
		return distance

	elif MODE == 3:
		# this should perform A* with the manhattan distance as a heuristic
		abs_x = abs(cell.coords[0] - goal_cell.coords[0])
		abs_y = abs(cell.coords[1] - goal_cell.coords[1])
		distance = abs_x + abs_y
		return distance

	elif MODE == 4:
		# this should perform A* with the octile distance as a heuristic
		abs_x = abs(cell.coords[0] - goal_cell.coords[0])
		abs_y = abs(cell.coords[1] - goal_cell.coords[1])
		abs_arr = [abs_x, abs_y]
		temp_x = np.argmax(abs_arr)
		temp_n = np.argmin(abs_arr)
		distance = abs_arr[temp_x] + ((math.sqrt(2) - 1) * abs_arr[temp_n])
		return distance

	else:
		# this should perform dijkstra
		return 0

# get the total cost to get to a cell
def costToNode(cell):
	if cell.parent != None:
		costToNode = cell.parent.pathCost + costFromParent(cell, cell.parent)
	else:
		costToNode = costFromParent(cell, cell.parent)
	return costToNode

# same as greedy, reusing the breadth first search (FIFO) script
# as it is similar to djikstra, and so A* with 0 heuristic
class AStarPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
	self.aStarQueue = []

    # put the cell into the queue ordered by ascending cost function
    def pushCellOntoQueue(self, cell):
	cell.pathCost = costToNode(cell)

	# this outputs the expected cost to the goal, according to the heuristic
	heuristicCost = cell.pathCost + variableHeuristic(cell, self.goal)

	# iterate through the queue and append at the correct index
	index = 0
	# for element in self.aStarQueue:
	for i in range(len(self.aStarQueue)):
		element = self.aStarQueue[i]
		# calculate the cost to the node we are examining
		elementCost = costToNode(element)
		# predict the total cost of going from the examined node to the goal
		elementHeuristicCost = elementCost + variableHeuristic(element, self.goal)
		if heuristicCost < elementHeuristicCost:
			break
		index += 1

	self.aStarQueue.insert(index, cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.aStarQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
	cell = self.aStarQueue.pop(0)
        return cell

    # this happens if the cell to be visited has already been visited
    def resolveDuplicate(self, cell, parentCell):
	# if there is even a comparison to be made
	if(parentCell != None and cell.parent != None):
		existingCost = cell.pathCost
		newCost = parentCell.pathCost + costFromParent(cell, parentCell)
		if newCost < existingCost:
			if cell in self.aStarQueue:
				self.aStarQueue.remove(cell)

			cell.pathCost = newCost
			heuristicCost = cell.pathCost + variableHeuristic(cell, self.goal)
			index = 0
			for element in self.aStarQueue:
				elementHeuristicCost = element.pathCost + variableHeuristic(element, self.goal)
				if heuristicCost < elementHeuristicCost:
					break
				index += 1

			newCell = cell
			newCell.parent = parentCell			
			self.aStarQueue.insert(index, cell)
        pass
