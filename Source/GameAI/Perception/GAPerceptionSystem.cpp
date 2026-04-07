#include "GAPerceptionSystem.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/GameModeBase.h"
#include "GameAI/Grid/GAGridActor.h"

UGAPerceptionSystem::UGAPerceptionSystem(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;

	SetTickGroup(ETickingGroup::TG_PostUpdateWork);
}

void UGAPerceptionSystem::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	FindVisibleCellsHelper();
}

void UGAPerceptionSystem::FindVisibleCellsHelper()
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
			TArray<TObjectPtr<UGAPerceptionComponent>>& TempPerceptionComponents = PerceptionSystem->GetAllPerceptionComponents();
			for (UGAPerceptionComponent* PerceptionComponent : TempPerceptionComponents)
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

AGAGridActor* UGAPerceptionSystem::GetGridActor() const
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


bool UGAPerceptionSystem::RegisterPerceptionComponent(UGAPerceptionComponent* PerceptionComponent)
{
	PerceptionComponents.AddUnique(PerceptionComponent);
	return true;
}

bool UGAPerceptionSystem::UnregisterPerceptionComponent(UGAPerceptionComponent* PerceptionComponent)
{
	return PerceptionComponents.Remove(PerceptionComponent) > 0;
}


bool UGAPerceptionSystem::RegisterTargetComponent(UGATargetComponent* TargetComponent)
{
	TargetComponents.AddUnique(TargetComponent);
	return true;
}

bool UGAPerceptionSystem::UnregisterTargetComponent(UGATargetComponent* TargetComponent)
{
	return TargetComponents.Remove(TargetComponent) > 0;
}


UGAPerceptionSystem* UGAPerceptionSystem::GetPerceptionSystem(const UObject* WorldContextObject)
{
	UGAPerceptionSystem* Result = NULL;
	AGameModeBase *GameMode = UGameplayStatics::GetGameMode(WorldContextObject);
	if (GameMode)
	{
		Result = GameMode->GetComponentByClass<UGAPerceptionSystem>();
	}

	return Result;
}
