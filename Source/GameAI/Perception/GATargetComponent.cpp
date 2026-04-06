#include "GATargetComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameAI/Grid/GAGridActor.h"
#include "GAPerceptionSystem.h"
#include "ProceduralMeshComponent.h"



UGATargetComponent::UGATargetComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;

	SetTickGroup(ETickingGroup::TG_PostUpdateWork);
}


AGAGridActor* UGATargetComponent::GetGridActor() const
{
	AGAGridActor* Result = GridActor.Get();
	if (Result)
	{
		return Result;
	}
	else
	{
		AActor* GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
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

float UGATargetComponent::GetMaxAwarenessOfTarget()
{
	float MaxAwareness = 0.0f;

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGAPerceptionComponent>>& PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
		for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
		{
			const FTargetView* TargetView = PerceptionComponent->GetTargetView(TargetGuid);
			if (TargetView->Awareness > MaxAwareness)
			{
				MaxAwareness = TargetView->Awareness;
			}
		}
	}

	return MaxAwareness;
}


void UGATargetComponent::OnRegister()
{
	Super::OnRegister();

	// Generate a new guid
	TargetGuid = FGuid::NewGuid();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterTargetComponent(this);
	}

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		OccupancyMap = FGAGridMap(Grid, 0.0f);
	}
}

void UGATargetComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterTargetComponent(this);
	}
}



void UGATargetComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	bool isImmediate = false;

	// update my perception state FSM
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGAPerceptionComponent>> &PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
		for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
		{
			// If any of the perceptors are at 1.0 awareness of the target
			const FTargetView* TargetView = PerceptionComponent->GetTargetView(TargetGuid);
			//UE_LOG(LogTemp, Warning, TEXT("AWARENESS: %f"), TargetView->Awareness);
			if (TargetView && (FMath::Abs(TargetView->Awareness - 1.0f) <= 0.00001))
			{
				isImmediate = true;
				break;
			}
		}
	}

	// Update visible cells map
	FindVisibleCellsHelper();

	if (isImmediate)
	{
		AActor* Owner = GetOwner();
		LastKnownState.State = GATS_Immediate;

		// REFRESH MY STATE
		LastKnownState.Set(Owner->GetActorLocation(), Owner->GetVelocity());

		// Tell the omap to clear out and put all the probability in the observed location
		OccupancyMapSetPosition(LastKnownState.Position);
	}
	else if (IsKnown())
	{
		LastKnownState.State = GATS_Hidden;
	}

	if (LastKnownState.State == GATS_Hidden)
	{
		OccupancyMapUpdate();
	}

	// As long as I'm known, whether I'm immediate or not, diffuse the probability in the omap
	if (IsKnown())
	{
		OccupancyMapDiffuse();
	}

	if (bDebugOccupancyMap)
	{
		AGAGridActor* Grid = GetGridActor();
		Grid->DebugGridMap = OccupancyMap;
		GridActor->RefreshDebugTexture();
		GridActor->DebugMeshComponent->SetVisibility(true);
	}
}


void UGATargetComponent::OccupancyMapSetPosition(const FVector& Position)
{
	// TODO PART 4

	// We've been observed to be in a given position
	// Clear out all probability in the omap, and set the appropriate cell to P = 1.0
	OccupancyMap.ResetData(0.f);
	AGAGridActor* Grid = GetGridActor();
	FCellRef PositionFCellRef = Grid->GetCellRef(Position);
	OccupancyMap.SetValue(PositionFCellRef, 1.f);
}

void UGATargetComponent::FindVisibleCellsHelper()
{
	AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FGAGridMap VisibilityMap(Grid, 0.0f);

		// STEP 1: Build a visibility map, based on the perception components of the AIs in the world
		// The visibility map is a simple map where each cell is either 0 (not currently visible to ANY perceiver) or 1 (currently visible to one or more perceivers).
		UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
		if (PerceptionSystem)
		{
			TArray<TObjectPtr<UGAPerceptionComponent>>& PerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
			for (UGAPerceptionComponent* PerceptionComponent : PerceptionComponents)
			{
				// Find visible cells for this perceiver.
				// Reminder: Use the PerceptionComponent.VisionParameters when determining whether a cell is visible or not (in addition to a line trace).
				// Suggestion: you might find it useful to add a UGAPerceptionComponent::TestVisibility method to the perception component.

				// For every cell
				for (int32 Y = 0; Y < Grid->YCount; Y++)
				{
					for (int32 X = 0; X < Grid->XCount; X++)
					{
						// Get a CellRef
						FCellRef TempCell(X, Y);
						// Check if the cell is already visible from another perceptor to prevent excess ray tracing)
						float TempVal;
						VisibilityMap.GetValue(TempCell, TempVal);
						if (TempVal == 0)
						{
							// If Cell is not already visible, check if it is by this perception
							int32 Index = Grid->CellRefToIndex(TempCell);
							FVector TempCellCoord = Grid->GetCellPosition(TempCell);
							// Ignore all the student actors
							TArray<AActor*> TargetsToIgnore;
							UGameplayStatics::GetAllActorsOfClass(GetWorld(), PerceptionSystem->ActorClassToIgnoreTracelines, TargetsToIgnore);
							if (PerceptionComponent->InTargetViewCone(TempCellCoord, TargetsToIgnore))
							{
								VisibilityMap.SetValue(TempCell, 1);
							}
						}
					}
				}
			}
		}

		// Update GridActor's gridmap
		Grid->ProctorVisionGridMap = VisibilityMap;
	}
}


void UGATargetComponent::OccupancyMapUpdate()
{
	AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FGAGridMap VisibilityMap(Grid, 0.0f);

		// TODO PART 4
		float ProbabilityCulled = 0.0f;

		// STEP 1: visibility map completed before this function is called

		// STEP 2: Clear out the probability in the visible cells
		// Make sure to keep track of cells culled for every cell
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			for (int32 X = 0; X < Grid->XCount; X++)
			{
				// Get a CellRef and its value
				FCellRef TempCell(X, Y);
				float TempVal;
				VisibilityMap.GetValue(TempCell, TempVal);

				// If cell is visible
				if (TempVal == 1)
				{
					// Clear probability in the visible cells
					float TempCulledVal;
					OccupancyMap.GetValue(TempCell, TempCulledVal);
					ProbabilityCulled += TempCulledVal;
					OccupancyMap.SetValue(TempCell, 0);
				}
			}
		}


		// Get all the remaining probability that wasn't culled
		float TotalProbBeingUpdated = 0.f;
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			for (int32 X = 0; X < Grid->XCount; X++)
			{
				// Get a CellRef
				FCellRef TempCell(X, Y);

				float TempVisibility;
				VisibilityMap.GetValue(TempCell, TempVisibility);

				if (TempVisibility == 0)
				{
					// Get the probability of that cell
					float TempProbability;
					OccupancyMap.GetValue(TempCell, TempProbability);

					TotalProbBeingUpdated += TempProbability;
				}
			}
		}

		// STEP 3: Renormalize the OMap, so that it's still a valid probability distribution
		// Iterate through all cells that aren't visibile
		// and follow the equation (1 / (1-p_culled))*p(n)) so non-visibile cells with probability of 0 stay at 0.
		// Note that because of floating point error, we use the remaining probability rather than 1 - probability culled.
		// ALSO keep track of highest probability cell
		FCellRef HighestProbCell;
		float HighestProbability = -FLT_MAX;
		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			for (int32 X = 0; X < Grid->XCount; X++)
			{
				// Get a CellRef
				FCellRef TempCell(X, Y);

				float TempVisibility;
				VisibilityMap.GetValue(TempCell, TempVisibility);

				if (TempVisibility == 0)
				{
					// Get the probability of that cell
					float TempProbability;
					OccupancyMap.GetValue(TempCell, TempProbability);

					// Normalize 
					float TempNormalizedProbability = TempProbability * (1.f / TotalProbBeingUpdated);
					OccupancyMap.SetValue(TempCell, TempNormalizedProbability);

					// STEP 4.1: Extract the highest-likelihood cell on the omap
					if (TempNormalizedProbability > HighestProbability) {
						HighestProbability = TempNormalizedProbability;
						HighestProbCell = TempCell;
					}
				}
			}
		}

		// STEP 4.2: Refresh the LastKnownState
		LastKnownState.Position = Grid->GetCellPosition(HighestProbCell);
	}
}


void UGATargetComponent::OccupancyMapDiffuse()
{
	// TODO PART 4
	// Diffuse the probability in the OMAP
	// Implementation is based off notes from Lecture 6
	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		// Diffusion rate 
		float alpha = 0.127;

		// Buffer
		FGAGridMap Buffer(Grid, 0.0f);

		for (int32 Y = 0; Y < Grid->YCount; Y++)
		{
			for (int32 X = 0; X < Grid->XCount; X++)
			{
				// Get a CellRef
				FCellRef TempCell(X, Y);
				float P_Cell;
				OccupancyMap.GetValue(TempCell, P_Cell);
				float P_Left = P_Cell;

				// For each neighbor
				for (int32 DirX = -1; DirX <= 1; DirX++)
				{
					for (int32 DirY = -1; DirY <= 1; DirY++) {
						// Skip center
						if (DirX == 0 && DirY == 0) 
						{
							continue;
						}
						FCellRef TempNeighbor(X + DirX, Y + DirY);

						// Skip non-traversable)
						if (!Grid->IsCellRefInBounds(TempNeighbor) || !EnumHasAllFlags(Grid->GetCellData(TempNeighbor), ECellData::CellDataTraversable))
						{
							continue;
						}

						float d = alpha * P_Cell;
						// Cell is a neighbor if DirX is not 0 and DirY is not 0
						if (DirY != 0 && DirX != 0)
						{
							d *= 1.f/FMath::Sqrt(2.f);
						}

						// Add probability to buffer
						float OldBuffer;
						Buffer.GetValue(TempNeighbor, OldBuffer);
						Buffer.SetValue(TempNeighbor, OldBuffer + d);
						P_Left -= d;
					}
				}

				// After going through neighbors, add remaining probability
				float OldBuffer;
				Buffer.GetValue(TempCell, OldBuffer);
				Buffer.SetValue(TempCell, OldBuffer + P_Left);
			}
		}

		// Copy the buffer to the Occupancy Map
		OccupancyMap = MoveTemp(Buffer);
	}
}
