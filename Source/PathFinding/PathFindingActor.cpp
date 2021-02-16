// Fill out your copyright notice in the Description page of Project Settings.


#include "PathFindingActor.h"
#include "Kismet/KismetMathLibrary.h"

// This is the constructor for the class
APathFindingActor::APathFindingActor() 
{
	GenerateGrid();
}





// This function set a path for the actor to follow based on a new path
void APathFindingActor::SetPathToFollow(const TArray<FVector>& newPath)
{
	Path = newPath;
	NeedNewDestination = true;
}





// This function adds a path to follow based on a given path
void APathFindingActor::AddToPathFollow(const TArray<FVector>& newPath)
{
	for (const FVector& pos : newPath)
	{
		Path.Add(pos);
	}
}





// This function gets called every tick
void APathFindingActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// Does the actor need a new destination?
	if (NeedNewDestination)
	{	
		// If yes, then is the actors pending path pool greater than 0?
		if (Path.Num() > 0)
		{
			// If yes, then set it to active
			TargetPos = Path[0];
			Path.RemoveAt(0);
			NeedNewDestination = false;
		}
	}
	else
	{
		// If no, then check that the destination has not yet been reached
		float distanceToDestination = (Position - TargetPos).Size();
		if (distanceToDestination <= ReachRadius)
		{
			NeedNewDestination = true;
		}
	}
}





// This function generates a grid for the actor to follow
void APathFindingActor::GenerateGrid() 
{
	Grid = TArray<FRowStruct>();

	// Cycle through whole grid based on its proposed grid size
	for (int x = 0; x < gridSizeX; x++) 
	{
		// Initialize a new row struct and add it to the grid
		FRowStruct _rowStruct = FRowStruct();
		Grid.Add(_rowStruct);

		for (int y = 0; y < gridSizeY; y++) 
		{
			FGridNode newGridNode = FGridNode();

			// Initialize its evaluation value
			newGridNode.value = 10000;

			// Get the node's new position
			newGridNode.position = FVector(gridDisplacement * x, gridDisplacement * y, 0);
			
			// Get the node's position on the grid
			newGridNode.x = x;
			newGridNode.y = y;

			// Add to grid array
			Grid[x]._gridNode.Add(newGridNode);
		}
	}
}





// This accesor function gets the 3Dposition of a grid node based on its place in the grid (x and y) 
FVector APathFindingActor::GetGridNodePosition(int gridX, int gridY)
{
	return Grid[gridX]._gridNode[gridY].position;
}





// This function sets a grid node to a given value from a given position on the grid
void APathFindingActor::SetGridNodeValue(int gridX, int gridY, int newValue) 
{
	Grid[gridX]._gridNode[gridY].value = newValue;
}





// This function sets a grid node as a wall from a given position on the grid
void APathFindingActor::SetGridNodeAsWall(int gridX, int gridY) 
{
	Grid[gridX]._gridNode[gridY].isWall = true;
}



// This function finds a path between two given start and end positions
TArray<FVector> APathFindingActor::FindPath(FVector2D startPos, FVector2D endPos)
{
	// Reset the grid to its original value
	ResetGridValues();

	// Is the start position equal to the end position?
	if (startPos.X == endPos.X &&
		startPos.Y == endPos.Y)
	{
		// If yes, then return nothing
		return TArray<FVector>();
	}

	if (Grid[endPos.X]._gridNode[endPos.Y].isWall == true)
	{
		// If yes, then return nothing
		return TArray<FVector>();
	}

	// Turn the start and end positions into full-on grid nodes
	FGridNode startNode = Grid[FMath::RoundToInt(startPos.X)]._gridNode[FMath::RoundToInt(startPos.Y)];
	FGridNode endNode = Grid[FMath::RoundToInt(endPos.X)]._gridNode[FMath::RoundToInt(endPos.Y)];

	// Initialize the rest of the variables
	//int cellX = 0;
	//int cellY = 0;

	int bestIndex = 0;
	int smallestNum = 10000;

	int distanceToNeighbor = 0;

	//cellX = startNode.position.X;
	//cellY = startNode.position.Y;

	TArray<FGridNode> openList = TArray<FGridNode>();
	TArray<FGridNode> closeList = TArray<FGridNode>();
	TArray<FGridNode> neighbors = TArray<FGridNode>();

	FGridNode currentNode = FGridNode();

	//tmp.x = startNode.x;
	//tmp.y = startNode.y;

	//tmp.position = FVector(cellX, cellY, 0);

	//tmp.value = 0;

	startNode.isWall = Grid[startNode.x]._gridNode[startNode.y].isWall;

	startNode.x = startPos.X;
	startNode.y = startPos.Y;

	startNode.value = 0;

	startNode.position.X = Grid[startNode.x]._gridNode[startNode.y].position.X;
	startNode.position.Y = Grid[startNode.x]._gridNode[startNode.y].position.Y;

	Grid[startNode.x]._gridNode[startNode.y].value = 0;

	openList.Add(startNode);


	// This while loop will continue to run until the entire open list has been emptied
	while (openList.Num() > 0)
	{
		// Find the best index from the open list
		bestIndex = FindBestIndex(openList, endNode);

		// Set the current node to the best node and make sure
		// to delete it from the open list and add it to the closed list
		currentNode = openList[bestIndex];
		openList.RemoveAt(bestIndex);
		closeList.Add(currentNode);

		// Get all neighbors of the best node
		neighbors = FindNeighbors(currentNode);
		distanceToNeighbor = currentNode.value + 1;


		//Search grid but only set neighbor if the visited node is a neighbor of current node, based on positions
		for (int i = 0; i < neighbors.Num(); i++)
		{
			// Has the neighbor been evaluated yet and is it not a wall?
			if (neighbors[i].value > distanceToNeighbor && neighbors[i].isWall == false)
			{
				// If yes, then add it to the open list with its correct value
				neighbors[i].value = distanceToNeighbor;
				Grid[neighbors[i].x]._gridNode[neighbors[i].y].value = distanceToNeighbor;
				openList.Add(neighbors[i]);

			}
		}


		// Is the current node equal to the end node?
		if (currentNode.position == endNode.position)
		{
			// If yes, then start to back track

			TArray<FVector> path = TArray<FVector>();
			path.Add(currentNode.position);

			int countBeforeKick = 0;

			while (true)
			{
				// Find all neighbors of the current node
				neighbors = FindNeighbors(currentNode);

				//Search grid but only set neighbor if the visited node is a neighbor of current node, based on positions These searches the grid and makes sure the node is a neighbor of the currentNode
				for (int i = 0; i < neighbors.Num(); i++)
				{
					// is the current neighbor selected the smallest number and not a wall?
					if (neighbors[i].value < smallestNum && neighbors[i].isWall == false)
					{
						// If yes, then set it to the current node
						currentNode = neighbors[i];
						smallestNum = currentNode.value;
					}
				}

				if (path[path.Num() - 1] == currentNode.position)
				{

				}
				else if (path[path.Num() - 1] != currentNode.position && currentNode.isWall == false)
					// Add the current node to the path
					path.Add(currentNode.position);

				// Has the start node been reached?
				if (currentNode.x == startNode.x &&
					currentNode.y == startNode.y)
				{
					// If yes then invert the path
					TArray<FVector> invPath;
					for (int i = path.Num(); i > 0; i--)
					{
						invPath.Add(path[i - 1]);
					}
					// Return the path
					return invPath;
				}

				if (path.Num() == 0)
				{
					return path;
				}

				countBeforeKick++;

				if (countBeforeKick > 30000)
				{
					return TArray<FVector>();
				}
			}

		}

	}
	// If the open list has been emptied and
	// there is still no match, return nothing
	return TArray<FVector>();
}




// This function go through the entire grid resets all of the values of the grid
void APathFindingActor::ResetGridValues()
{
	for (int i = 0; i < Grid.Num(); i++)
	{
		for (int j = 0; j < Grid[i]._gridNode.Num(); j++)
		{
			Grid[i]._gridNode[j].value = 10000;
		}
	}

}





// This function finds the best index out of the current open list 
int APathFindingActor::FindBestIndex(TArray<FGridNode> openList, FGridNode endNode)
{
	float bestDist = 10000;
	int bestIndex = 1;

	// Cycle through the whole open list
	for (int i = 0; i < openList.Num(); i++)
	{
		// Are we using Djikstra?
		if (useDjikstra) 
		{
			// If yes, then use the Djikstra algorithm
			// Is the open list's value less than the best distance?
			if (openList[i].value < bestDist)
			{
				// If yes, then set that index to the best index
				bestDist = openList[i].value;
				bestIndex = i;
			}
		}
		else 
		{
			// If no, then use the A* algorithm
			// Is the open list's value less than the best distance?
			if (CalculateHeuristic(openList[i],endNode) < bestDist)
			{
				// If yes, then set that index to the best index
				bestDist = openList[i].value;
				bestIndex = i;
			}
		}
	}

	// Return result
	return bestIndex;
}





// This function calculates the heuristic for the A* algorithm
int APathFindingActor::CalculateHeuristic(FGridNode node, FGridNode endNode) 
{
	// This heuristic simply calculates the raw distance between itself and the end node
	int differenceX = FMath::Abs(node.x - endNode.x);
	int differenceY = FMath::Abs(node.y - endNode.y);
	return differenceX + differenceY;
}





//This function finds all of the neighbors of a given node
TArray<FGridNode> APathFindingActor:: FindNeighbors(FGridNode startingNode) 
{
	// Initialize values
	TArray<FVector2D> possibleNeighboors = TArray<FVector2D>();
	TArray<FGridNode> finalNeighboors = TArray<FGridNode>();

	// Find all posible neighbors
	possibleNeighboors.Add(FVector2D(startingNode.x + 1,startingNode.y));
	possibleNeighboors.Add(FVector2D(startingNode.x - 1,startingNode.y));
	possibleNeighboors.Add(FVector2D(startingNode.x, startingNode.y + 1));
	possibleNeighboors.Add(FVector2D(startingNode.x, startingNode.y - 1));


	// Cycle through possible neighbors list
	for (int i = 0; i < possibleNeighboors.Num(); i++) 
	{
		// Is this possible neighbor within range of the grid?
		if (possibleNeighboors[i].X >= 0 && possibleNeighboors[i].Y >= 0 && possibleNeighboors[i].X <= gridSizeX - 1 && possibleNeighboors[i].Y <= gridSizeY - 1) 
		{
			// If yes, then add it as an actual neighbor
			finalNeighboors.Add(Grid[possibleNeighboors[i].X]._gridNode[possibleNeighboors[i].Y]);
			finalNeighboors[finalNeighboors.Num() -1].value = Grid[possibleNeighboors[i].X]._gridNode[possibleNeighboors[i].Y].value;
		}
	}

	// Return result
	return finalNeighboors;
}





// This function adds all of the walls to the grid to represent them during path finding
void APathFindingActor::AddWalls(TArray<FVector2D> walls) 
{
	for (int i = 0; i < walls.Num(); i++) 
	{
		Grid[walls[i].X]._gridNode[walls[i].Y].isWall = true;
	}
}





// This function goes through all nodes and makes sure that all of them are not walls
void APathFindingActor::ResetWalls()
{
	for (int x = 0; x < gridSizeX; x++) 
	{
		for (int y = 0; y < gridSizeY; y++) 
		{
			Grid[x]._gridNode[y].isWall = false;
		}
	}
}