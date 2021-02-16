// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SteeringActor.h"
#include "Containers/Queue.h"
#include "PathFindingActor.generated.h"

// This struct represents an individual grid node
USTRUCT()
struct FGridNode
{
	GENERATED_USTRUCT_BODY()

	// Integers
	UPROPERTY() int value;
	UPROPERTY() int x = 0;
	UPROPERTY() int y = 0;

	// Booleans
	UPROPERTY() bool isWall = false;

	// FVectors
	UPROPERTY() FVector position;
};


// This struct represents a row of grid nodes
USTRUCT()
struct FRowStruct 
{
	GENERATED_USTRUCT_BODY()

	// TArray's
	UPROPERTY() TArray<FGridNode> _gridNode;
};

UCLASS()
class PATHFINDING_API  APathFindingActor : public ASteeringActor
{
	GENERATED_BODY()

public:	

	// Contructors
	APathFindingActor();

	// Built-in functions
	virtual void Tick(float DeltaTime) override;




	// Accesor Functions ----------------------------------------------- //

	int FindBestIndex(TArray<FGridNode> openList, FGridNode endNode);

	UFUNCTION(BlueprintCallable) TArray<FVector> FindPath(FVector2D startPos, FVector2D endPos);

	// ----------------------------------------------------------------- //




	// Mutator Functions ----------------------------------------------- //

	UFUNCTION(BlueprintCallable) void SetPathToFollow(const TArray<FVector>& newPath);
	UFUNCTION(BlueprintCallable) void AddToPathFollow(const TArray<FVector>& newPath);
	UFUNCTION(BlueprintCallable) void ResetWalls();

	UFUNCTION(BlueprintCallable) void AddWalls(TArray<FVector2D> walls);

	UFUNCTION(BlueprintCallable) void ResetGridValues();

	// ----------------------------------------------------------------- //

protected:




	// Vairables ------------------------------------------------------- //
	
	// TArray's
	TArray<FRowStruct> Grid;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) TArray<FVector> Path;

	// Booleans
	bool once = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) bool NeedNewDestination = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) bool useDjikstra = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) bool findNewPath = true;

	// Floats
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float ReachRadius = 100.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float gridDisplacement = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float gridSizeX = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite) float gridSizeY = 1.0f;

	// ----------------------------------------------------------------- //




	// Accesor Functions ----------------------------------------------- //

	int CalculateHeuristic(FGridNode node, FGridNode endNode);

	UFUNCTION(BlueprintCallable) FVector GetGridNodePosition(int gridX, int gridY);

	// ----------------------------------------------------------------- //




	// Mutator Functions ----------------------------------------------- //

	UFUNCTION(BlueprintCallable) void SetGridNodeValue(int gridX, int gridY, int newValue);
	UFUNCTION(BlueprintCallable) void SetGridNodeAsWall(int gridX, int gridY);
	
	UFUNCTION(BlueprintCallable) void GenerateGrid();

	TArray<FGridNode>FindNeighbors(FGridNode startingNode);

	// ----------------------------------------------------------------- //
};