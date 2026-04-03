#include "GAPathComponent.h"
#include "GameFramework/NavMovementComponent.h"
#include "Kismet/GameplayStatics.h"

UGAPathComponent::UGAPathComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	State = GAPS_None;
	bDestinationValid = false;
	ArrivalDistance = 100.0f;

	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;
}


const AGAGridActor* UGAPathComponent::GetGridActor() const
{
	if (GridActor.Get())
	{
		return GridActor.Get();
	}
	else
	{
		AGAGridActor* Result = NULL;
		AActor *GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is marked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}
}

APawn* UGAPathComponent::GetOwnerPawn()
{
	AActor* Owner = GetOwner();
	if (Owner)
	{
		APawn* Pawn = Cast<APawn>(Owner);
		if (Pawn)
		{
			return Pawn;
		}
		else
		{
			AController* Controller = Cast<AController>(Owner);
			if (Controller)
			{
				return Controller->GetPawn();
			}
		}
	}

	return NULL;
}


void UGAPathComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	if (GetOwnerPawn() == NULL)
	{
		return;
	}

	bool Valid = false;
	if (bDestinationValid)
	{
		State = RefreshPath();
		Valid = true;
	}
	else if (bDistanceMapPathValid)
	{
		Valid = true;
	}
	if (Valid)
	{
		if (State == GAPS_Active)
		{
			FollowPath();
		}
	}

	// Super important! Otherwise, unbelievably, the Tick event in Blueprint won't get called
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

EGAPathState UGAPathComponent::RefreshPath()
{
	AActor* Owner = GetOwnerPawn();
	if (Owner == NULL)
	{
		State = GAPS_Invalid;
		return State;
	}

	FVector StartPoint = Owner->GetActorLocation();

	check(bDestinationValid);

	float DistanceToDestination = FVector::Dist(StartPoint, Destination);

	// If we are close enough or if there's no more steps
	if (DistanceToDestination <= ArrivalDistance) //  || Steps.Num() <= 0
	{
		// Yay! We got there!
		State = GAPS_Finished;
	}
	else 
	{
		TArray<FPathStep> UnsmoothedSteps;
		Steps.Empty();
		State = AStar(StartPoint, Destination, UnsmoothedSteps);
		if (State == EGAPathState::GAPS_Active)
		{
			// Smooth the path!
			State = SmoothPath(StartPoint, UnsmoothedSteps, Steps);
		}
	}

	return State;
}

struct FCellRefAndFScore
{
	FCellRef Cell;
	float FScore;

	// Override < for heap
	bool operator<(const FCellRefAndFScore& CellRefAndFScore) const
	{
		return FScore < CellRefAndFScore.FScore;
	}

	// Override == for checking our list
	bool operator==(const FCellRefAndFScore& CellRefAndFScore) const
	{
		return FScore == CellRefAndFScore.FScore && Cell == CellRefAndFScore.Cell;
	}
};

static void ReconstructPath(const AGAGridActor& Grid, const TMap<FCellRef, FCellRef>& CameFrom, FCellRef Current, TArray<FPathStep>& StepsOut)
{
	// Clear Array to make sure we start fresh
	StepsOut.Empty();

	FPathStep TempPathStep;

	while (CameFrom.Contains(Current)) {
		TempPathStep.Set(Grid.GetCellPosition(Current), Current);
		StepsOut.Add(TempPathStep);
		Current = CameFrom[Current];
	}

	Algo::Reverse(StepsOut);
}

// Checks if a cell is in bounds and traversable
static bool ValidCell(const AGAGridActor& Grid, const FCellRef& Cell) {
	// Skip if out of bounds
	if (!Grid.IsCellRefInBounds(Cell)) {
		return false;
	}

	// Skip if not traversable
	ECellData Flags = Grid.GetCellData(Cell);
	if (!EnumHasAllFlags(Flags, ECellData::CellDataTraversable)) {
		return false;
	}

	return true;
}

// Get the 4 neighbors of a cell (in any order). Does not get invalid neighbors or untraversable cells
static void NeighborsOfCell(const AGAGridActor* Grid, const FCellRef& Cell, TArray<FCellRef>& OutNeighbors)
{
	OutNeighbors.Reset();

	FCellRef Neighbors[4] = { FCellRef(Cell.X + 1, Cell.Y), FCellRef(Cell.X - 1, Cell.Y), FCellRef(Cell.X, Cell.Y + 1), FCellRef(Cell.X, Cell.Y - 1) };

	for (FCellRef Neighbor : Neighbors) {
		if (!ValidCell(*Grid, Neighbor)) {
			continue;
		}

		OutNeighbors.Add(Neighbor);
	}
}

EGAPathState UGAPathComponent::AStar(const FVector& StartPoint, const FVector& EndPoint, TArray<FPathStep>& StepsOut) const
{
	const AGAGridActor* Grid = GetGridActor();

	// Assignment 2 Part3: replace this with an A* search!
	// HINT 1: you made need a heap structure. A TArray can be accessed as a heap -- just add/remove elements using
	// the TArray::HeapPush() and TArray::HeapPop() methods.
	// Note that whatever you push or pop needs to implement the 'less than' operator (operator<)
	// HINT 2: UE has some useful flag testing function. For example you can test for traversability by doing this:
	// ECellData Flags = Grid->GetCellData(CellRef);
	// bool bIsCellTraversable = EnumHasAllFlags(Flags, ECellData::CellDataTraversable)

	// Convert start and destination
	FCellRef StartFCellRef = Grid->GetCellRef(StartPoint, false);
	FCellRef DestinationFCellRef = Grid->GetCellRef(EndPoint, false);

	// Array treated as a set with heap structure used to keep track of which cells to explore next
	TArray<FCellRefAndFScore> OpenSet;

	// Add start to heap. Note, we use a custom struct (FCellRefAndFScore) to define how we compare in the heap.
	FCellRefAndFScore tempStart = { StartFCellRef, StartFCellRef.Distance(DestinationFCellRef) };
	OpenSet.HeapPush(tempStart);

	// Map to keep track of what a cell's prev is
	TMap<FCellRef, FCellRef> cameFrom;
	// Map to track g-scores (cost of the cheapest path from start to gScore[n]), initialize start with 0
	TMap<FCellRef, float> GScores;
	GScores.Add(StartFCellRef, 0.0f);
	// Map to track f-scores (current best guess of cost of the cheapest path from start to end going through fScore[n])
	TMap<FCellRef, float> FScores;

	// Allocate space for neighbors array (trying to prevent creating a new array many times)
	TArray<FCellRef> Neighbors;
	Neighbors.Reserve(4);

	// While we have more cells to explore...
	while (!OpenSet.IsEmpty()) {
		// Pop next off heap
		FCellRefAndFScore Current;
		OpenSet.HeapPop(Current);
		FCellRef CurrentCell = Current.Cell;

		// Check if we're at our destination
		if (CurrentCell == DestinationFCellRef) {
			ReconstructPath(*Grid, cameFrom, CurrentCell, StepsOut);
			return GAPS_Active;
		}

		// Get neighbors of the current cell
		Neighbors.Reset();
		NeighborsOfCell(Grid, CurrentCell, Neighbors);

		// Get current GScore. If not initialized yet, assume it is "infinity"
		float CurrentCellGScore = GScores.Contains(CurrentCell) ? GScores[CurrentCell] : FLT_MAX;
		for (FCellRef Neighbor : Neighbors) {
			// Cost to go from start to this neighbor
			float TentativeGScore = CurrentCellGScore + CurrentCell.Distance(Neighbor);

			// Get GScore for neighbor. If not initialized yet, assume it is "infinity"
			float NeighborGScore = GScores.Contains(Neighbor) ? GScores[Neighbor] : FLT_MAX;

			// If this tentative cost is better (faster) than our current best for that neighbor...
			if (TentativeGScore < NeighborGScore) {
				// This path to neighbor is better! Update cameFrom, GScore, and FScore
				cameFrom.Add(Neighbor, CurrentCell);
				GScores.Add(Neighbor, TentativeGScore);
				FScores.Add(Neighbor, TentativeGScore + Neighbor.Distance(DestinationFCellRef));

				// Add neighbor to the set if they're not there yet
				FCellRefAndFScore tempNeighbor = { Neighbor, FScores[Neighbor] };
				if (!OpenSet.Contains(tempNeighbor)) {
					OpenSet.HeapPush(tempNeighbor);
				}
			}
		}
	}

	// HINT 3: make sure you return the correct status, based on whether you succeeded to find a path or not.
	// See the comment in GAPathComponent above the EGAPathState enum

	// If we reached here, we did not find a valid path to the destination
	return GAPS_Invalid;
}

struct FCellRefAndDistance
{
	// The FCellRef this struct represents in the grid
	FCellRef Cell;
	// Distance from the start to this cell
	float Distance;

	// Override < for heap
	bool operator<(const FCellRefAndDistance& CellRefAndDistance) const
	{
		return Distance < CellRefAndDistance.Distance;
	}

	// Override == for checking our list
	bool operator==(const FCellRefAndDistance& CellRefAndDistance) const
	{
		return Distance == CellRefAndDistance.Distance && Cell == CellRefAndDistance.Cell;
	}
};


bool UGAPathComponent::Dijkstra(const FVector& StartPoint, FGAGridMap& DistanceMapOut, TMap<FCellRef, FCellRef>& Prev) const
{
	// Assignment 3 Part 3-1: implement Dijkstra's algorithm to fill out the distance map
	const AGAGridActor* Grid = GetGridActor();

	// Heap to keep track of all cells we're tracking
	TArray<FCellRefAndDistance> Heap;
	// Map to check if we visited a cell already
	TMap<FCellRef, bool> Visited;

	// Add start to Heap
	FCellRef StartFCellRef = Grid->GetCellRef(StartPoint, false);
	DistanceMapOut.SetValue(StartFCellRef, 0.f);
	FCellRefAndDistance CellAndDist = { StartFCellRef, 0.f };
	Heap.HeapPush(CellAndDist);

	// Allocate space for neighbors array (trying to prevent creating a new array many times)
	TArray<FCellRef> Neighbors;
	Neighbors.Reserve(4);

	// While we have more cells to explore...
	while (!Heap.IsEmpty()) {
		// Pop next off heap
		FCellRefAndDistance CurrentCellRefAndDistance;
		Heap.HeapPop(CurrentCellRefAndDistance);
		FCellRef CurrentCell = CurrentCellRefAndDistance.Cell;

		// Get distance for this cell
		float CurrentDistance;
		DistanceMapOut.GetValue(CurrentCell, CurrentDistance);

		// If the distance is outdated, skip it
		// NOTE: This is so that when we update a cell's distance, we don't remove it from the heap as this ruins the heap structure
		if (CurrentCellRefAndDistance.Distance > CurrentDistance)
		{
			continue;
		}

		// Mark this cell as visited
		Visited.Add(CurrentCell, true);

		// Get neighbors of the current cell
		NeighborsOfCell(Grid, CurrentCell, Neighbors);

		// For each unvisited neighbor
		for (FCellRef Neighbor : Neighbors) {
			// Check if cell is in the map, if not default to not visited
			if (Visited.Find(Neighbor) == nullptr) {
				Visited.Add(Neighbor, false);
			}

			if (Visited[Neighbor]) {
				continue;
			}

			// Get the neighbor's distance
			float NeighborDistance;
			DistanceMapOut.GetValue(Neighbor, NeighborDistance);

			// Cost to go from start to this neighbor
			float Alt = CurrentCellRefAndDistance.Distance + CurrentCell.Distance(Neighbor);

			// If this tentative cost is better (faster) than our current best for that neighbor...
			if (Alt < NeighborDistance) {
				// This path to neighbor is better! Update cameFrom and add to Heap with new distance.
				// NOTE: Cell might already be in the Heap with a worse distance. That's okay as we skip these! 
				Prev.Add(Neighbor, CurrentCell);
				FCellRefAndDistance NeighborFCellRefAndDistance = { Neighbor, Alt };
				DistanceMapOut.SetValue(Neighbor, Alt);
				Heap.HeapPush(NeighborFCellRefAndDistance);
			}
		}
	}

	return true;
}

bool UGAPathComponent::BuidPathFromDistanceMap(const FVector& EndPoint, const FCellRef& EndPointCellRef, const FGAGridMap& DistanceMap, TMap<FCellRef, FCellRef>& Prev)
{
	// Assignment 3 Part 3-2: reconstruct a path from the distance map
	bDistanceMapPathValid = false;
	
	const AGAGridActor* Grid = GetGridActor();

	// Clear Array to make sure we start fresh
	Steps.Empty();
	TArray<FPathStep> UnsmoothedSteps;

	FCellRef CurrentCellRef = EndPointCellRef;
	float CurrentDistance;
	DistanceMap.GetValue(CurrentCellRef, CurrentDistance);

	FPathStep TempPathStep;

	// While we're not at the start 
	while (CurrentDistance != 0) {
		// Add this step to the grid
		TempPathStep.Set(Grid->GetCellPosition(CurrentCellRef), CurrentCellRef);
		UnsmoothedSteps.Add(TempPathStep);

		// Update our vars to the prev step
		FCellRef* CurrentCellRefPtr = Prev.Find(CurrentCellRef);
		if (CurrentCellRefPtr == nullptr) {
			return false;
		}
		CurrentCellRef = *CurrentCellRefPtr;

		DistanceMap.GetValue(CurrentCellRef, CurrentDistance);
	}

	Algo::Reverse(UnsmoothedSteps);

	// Remember to smooth the path as well, using your existing smoothing code
	SmoothPath(Grid->GetCellPosition(CurrentCellRef), UnsmoothedSteps, Steps);

	// Set this to true when you've successfully built the path
	bDistanceMapPathValid = true;

	if (bDistanceMapPathValid)
	{
		// once you have built the path (i.e. filled in the Steps array in the GAPathComponent), set the path component's state to GAPS_Active
		State = GAPS_Active;
	}

	return bDistanceMapPathValid;
}


// Traceline using Bresenham's Algorithm https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm 
static bool Traceline(const AGAGridActor& Grid, FCellRef StartCell, FCellRef EndCell)
{
	int CellX = StartCell.X; // x0
	int CellY = StartCell.Y; // y0
	int EndCellX = EndCell.X; // x1
	int EndCellY = EndCell.Y; // y1

	int DistanceX = FMath::Abs(EndCellX - CellX); // dx
	int DirectionX = (CellX < EndCellX) ? 1 : -1; // sx
	int DistanceY = -1 * FMath::Abs(EndCellY - CellY); // dy
	int DirectionY = (CellY < EndCellY) ? 1 : -1; // sy
	int Err = DistanceX + DistanceY; // error

	while (true)
	{
		// Check current cell
		FCellRef Cell = FCellRef(CellX, CellY); // plot(x0, y0)
		if (!ValidCell(Grid, Cell)) {
			return false;
		}

		// End reached 
		// (Note: Checking here instead of in each direction to check each cell)
		if (CellX == EndCellX && CellY == EndCellY) {
			break;
		}

		int Err2 = 2 * Err; // e2

		// First, check the directions are valid (do one at a time for diagonal/corner cutting safety)

		// Step in X direction
		if (Err2 >= DistanceY) {
			FCellRef CellSideX = FCellRef(CellX + DirectionX, CellY);
			if (!ValidCell(Grid, CellSideX)) {
				return false;
			}
		}
		// Step in Y Direction
		if (Err2 <= DistanceX) {
			FCellRef CellSideY = FCellRef(CellX, CellY + DirectionY);
			if (!ValidCell(Grid, CellSideY)) {
				return false;
			}
		}

		// After checking valid directions, then update error and X/Y
		if (Err2 >= DistanceY) {
			Err += DistanceY;
			CellX += DirectionX;
		}
		if (Err2 <= DistanceX) {
			Err += DistanceX;
			CellY += DirectionY;
		}
	}

	return true;
}



EGAPathState UGAPathComponent::SmoothPath(const FVector& StartPoint, const TArray<FPathStep>& UnsmoothedSteps, TArray<FPathStep>& SmoothedStepsOut) const
{
	// Assignment 2 Part 4: smooth the path
	// High level description from the lecture:
	// * Trace to each subsequent step until you collide
	// * Back up one step (to the last clear one)
	// * Add that cell to your smoothed step
	// * Start again from there
	SmoothedStepsOut.Reset();
	int n = UnsmoothedSteps.Num();

	// There is no path
	if (n <= 0) {
		return GAPS_None;
	}

	const AGAGridActor* Grid = GetGridActor();

	FPathStep Step;
	Step.Set(StartPoint, Grid->GetCellRef(StartPoint));

	// Iterate through each step. Skip the first step since we know we can go from start to first step.
	for (int k = 1; k < n; k++) {
		if (!Traceline(*Grid, Step.CellRef, UnsmoothedSteps[k].CellRef)) {
			SmoothedStepsOut.Add(UnsmoothedSteps[k - 1]);
			Step = UnsmoothedSteps[k - 1];
		}
	}

	// Add the destination as the last step
	SmoothedStepsOut.Add(UnsmoothedSteps[n - 1]);

	// HINT: make sure you return the correct status, based on whether you succeeded to find a path or not.
	// See the comment in GAPathComponent above the EGAPathState enum
	return GAPS_Active;
}

void UGAPathComponent::FollowPath()
{
	AActor* Owner = GetOwnerPawn();
	if (Owner == NULL)
	{
		return;
	}

	FVector StartPoint = Owner->GetActorLocation();

	check(State == GAPS_Active);
	check(Steps.Num() > 0);

	// Always follow the first step, assuming that we are refreshing the whole path every tick
	FVector V = Steps[0].Point - StartPoint;
	V.Normalize();

	UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
	if (MovementComponent)
	{
			MovementComponent->RequestPathMove(V);
		}
	}


	EGAPathState UGAPathComponent::SetDestination(const FVector &DestinationPoint)
	{
		Destination = DestinationPoint;

		State = GAPS_Invalid;

		const AGAGridActor* Grid = GetGridActor();
		if (Grid)
		{
			FCellRef CellRef = Grid->GetCellRef(Destination);
			if (CellRef.IsValid())
			{
				DestinationCell = CellRef;
				bDestinationValid = true;

				RefreshPath();
		}
	}

	return State;
}