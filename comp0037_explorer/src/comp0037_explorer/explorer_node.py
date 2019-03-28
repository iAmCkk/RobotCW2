import rospy

from explorer_node_base import ExplorerNodeBase
from geometry_msgs.msg  import Pose
from geometry_msgs.msg  import Pose2D

import random

# global functions, they do not necessarily have to be defined as part of the explorer node, I don't think
# cells given as tuples of xy coordinates, returns 1 if yes, 0 if no
def isAdjacent(cell_a, cell_b):

	"""
	cell_a_x = cell_a[0]
	cell_a_y = cell_a[1]

	cell_b_x = cell_b[0]
	cell_b_y = cell_b[1]
	"""

	# they are adjacent if their x values are within 1 and they y values are within 1 of each other
	delta_x = abs(cell_a[0] - cell_b[0])
	delta_y = abs(cell_a[1] - cell_b[1])

	# return 1 if this holds, and 0 otherwise, PERHAPS LIMIT ADJACENCY TO BE ONLY 4 DIRECTIONAL, SO DO NOT INCLUDE DIAGONAL
	# IS CURRENTLY SET TO EXACTLY THIS
	if (delta_x == 0 and delta_y == 0) or (delta_x <= 1 and delta_y <= 1):
		return 0
	elif (delta_x <= 1) or (delta_y <= 1):
		return 1
	else:	
		return 0


# find all frontier cells adjacent to the current frontier
def getAdjacents(frontier_cell_array, current_frontier_cells):

	# create an array of the cells in the frontier so far - the length of this is the return value
	current_frontier = current_frontier_cells

	# keep track of whether or not we need to run again to check for even more adjacents
	frontier_expanded = False

	# for all of the cells in the current frontier
	for j in range(0, len(current_frontier)):

		# check if there is something adjacent to that cell
		current_f_cell = current_frontier[j]
		for i in range(0, len(frontier_cell_array)):

			# if there is, AND WE HAVE NOT ALREADY GOT IT IN THE FRONTIER, add it to the current frontier
			if isAdjacent(frontier_cell_array[i], current_f_cell) and (frontier_cell_array[i] not in current_frontier):

				current_frontier.append(frontier_cell_array[i])
				frontier_expanded = True

	# if we added something, run again to check for new adjacencies
	if frontier_expanded:
		# print(current_frontier)
		current_frontier = getAdjacents(frontier_cell_array, current_frontier)

	return current_frontier

# for finding the largest frontier
def getKey(item):
	return item[0]

# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []
	self.searched = []

	self.frontier_information = []

    # NEED TO CREATE AN IMPLEMENTATION FOR THIS
    def updateFrontiers(self):
        pass

    # breadth first search function needs to be defined since it is recursive
    def breadthFirst(self, cell_x, cell_y, current_frontiers):

	# mark this cell as searched so that we do not visit it again
	self.searched.append((cell_x, cell_y))
	
	# if the current cell is an obstacle or unknown, ignore it
	cell_status = self.occupancyGrid.getCell(cell_x, cell_y)
	if cell_status != 0:
		return current_frontiers

	# otherwise, check if it is a frontier cell
	else:
		# and if it is a frontier cell, add it to the array
		if self.isFrontierCell(cell_x, cell_y) is True and (cell_x, cell_y) not in current_frontiers:
			current_frontiers.append( (cell_x, cell_y) )

		# then, search each of the cells neighbours that have not yet been searched
		for y in range(-1, 2):
			for x in range(-1, 2):
				if x == 0 and y == 0:
					continue
				else:
					to_search = (cell_x + x, cell_y + y)
					if to_search not in self.searched:
						resulting_frontiers = self. breadthFirst(cell_x + x, cell_y + y, current_frontiers)
					else:
						resulting_frontiers = current_frontiers

		# return the array containing all frontier cells
		return resulting_frontiers
		
    def chooseNewDestination(self):

	# the commented block below is the breadth first version of the implementation
	# this was not used because of the recursion limit in python (stack gets too big too quick)

	"""
	toBreak = 0
	for x in range(0, self.occupancyGrid.getWidthInCells()):
		for y in range(0, self.occupancyGrid.getHeightInCells()):
			cell = self.occupancyGrid.getCell(x, y)
			# if we have found a cell that has a known state
			if cell == 0:
				# break out of the double loop
				toBreak = 1
				break

		# exit the double loop
		if(toBreak):
			break

	"""

	# from the bottom left corner of the known map, perform a breadth first search
	# this array will be edited, it will store all of the tuples containing the coordinates of frontier cells
	# frontier_cells = []

	# reset the array storing the cells breadth first has searched through
	# self.searched = []
	
	# breadth first called - this appears to correctly return an array of frontier cells
	# frontier_cells = self.breadthFirst(x, y, frontier_cells)

	# ----- THE ACTUAL IMPLEMENTATION STARTS HERE -----

	baseLine = False
		
	if not baseLine:

		if len(self.frontier_information) == 0:
			# since there is a recursion limit that we hit with breadth first
			# iteratively search across the cells for frontiers
			frontier_cells = []
			for x in range(0, self.occupancyGrid.getWidthInCells()):
				for y in range(0, self.occupancyGrid.getHeightInCells()):
					# if we have found a frontier cell
					if self.isFrontierCell(x, y) is True and (x, y) not in self.blackList:
						# add it to the frontier cell array
						frontier_cells.append((x, y))

			# create an array of frontier information
			frontiers_w_sizes = []

			# check if there are even any frontiers
			if len(frontier_cells) == 0:
				return False, None

			# if there are, find the largest frontier
			else:
				# while there are still frontiers
				remaining_frontier_cells = frontier_cells
				while len(remaining_frontier_cells) != 0:
					# find a frontier
					adj = getAdjacents(frontier_cells, [ frontier_cells[0] ] )

					# add the frontier information to the array
					frontiers_w_sizes.append([len(adj), adj])

					# remove that frontier from the array, and repeat
					for i in range(0, len(adj)):
						remaining_frontier_cells.remove(adj[i])

			# sort the frontiers by length, in descending order - putting the largest frontier at index 0
			sorted_adj = sorted(frontiers_w_sizes, reverse=True, key=getKey)
			self.frontier_information = sorted_adj

		# get the largest frontier, and a coordinate on it to go to
		largest_frontier_coords = self.frontier_information[0][1]
		self.frontier_information.pop(0)

		if(len(largest_frontier_coords) > 1):
			index = random.randint(0, len(largest_frontier_coords) - 1)
		else:
			index = 0

		# pick a new coordinate if we have picked one that is in the blacklist - THIS SHOULD NOT HAPPEN (?)
		first_coord_largest = largest_frontier_coords[index]
		if first_coord_largest in self.blackList:
			index = random.randint(0, len(largest_frontier_coords))
			first_coord_largest = largest_frontier_coords[index]
	
		self.blackList.append(first_coord_largest)
		return True, first_coord_largest

	else:

		# the baseline approach as copied from the code
		for x in range(0, self.occupancyGrid.getWidthInCells()):
		    for y in range(0, self.occupancyGrid.getHeightInCells()):
		        candidate = (x, y)
		        if self.isFrontierCell(x, y) is True:
		            candidateGood = True
		            for k in range(0, len(self.blackList)):
		                if self.blackList[k] == candidate:
		                    candidateGood = False
		                    break

		            if candidateGood is True:
		                return True, candidate

		return False, None
	

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
	    # print '------- Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
