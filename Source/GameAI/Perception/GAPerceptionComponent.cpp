#include "GAPerceptionComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GAPerceptionSystem.h"

UGAPerceptionComponent::UGAPerceptionComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;
}


void UGAPerceptionComponent::OnRegister()
{
	Super::OnRegister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->RegisterPerceptionComponent(this);
	}
}

void UGAPerceptionComponent::OnUnregister()
{
	Super::OnUnregister();

	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		PerceptionSystem->UnregisterPerceptionComponent(this);
	}
}


APawn* UGAPerceptionComponent::GetOwnerPawn() const
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



// Returns the Target this AI is attending to right now.

UGATargetComponent* UGAPerceptionComponent::GetCurrentTarget() const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);

	if (PerceptionSystem && PerceptionSystem->TargetComponents.Num() > 0)
	{
		UGATargetComponent* TargetComponent = PerceptionSystem->TargetComponents[0];
		if (TargetComponent->IsKnown())
		{
			return PerceptionSystem->TargetComponents[0];
		}
	}

	return NULL;
}

bool UGAPerceptionComponent::HasTarget() const
{
	return GetCurrentTarget() != NULL;
}


bool UGAPerceptionComponent::GetCurrentTargetState(FTargetState& TargetStateOut, FTargetView& TargetViewOut) const
{
	UGATargetComponent* Target = GetCurrentTarget();
	if (Target)
	{
		const FTargetView* TargetView = TargetMap.Find(Target->TargetGuid);
		if (TargetView)
		{
			TargetStateOut = Target->LastKnownState;
			TargetViewOut = *TargetView;
			return true;
		}

	}
	return false;
}


void UGAPerceptionComponent::GetAllTargetStates(bool OnlyKnown, TArray<FTargetState>& TargetStatesOut, TArray<FTargetView>& TargetViewsOut) const
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			const FTargetView* TargetView = TargetMap.Find(TargetComponent->TargetGuid);
			if (TargetView)
			{
				if (!OnlyKnown || TargetComponent->IsKnown())
				{
					TargetStatesOut.Add(TargetComponent->LastKnownState);
					TargetViewsOut.Add(*TargetView);
				}
			}
		}
	}
}


void UGAPerceptionComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateAllTargetViews(DeltaTime);
}


void UGAPerceptionComponent::UpdateAllTargetViews(float DeltaTime)
{
	UGAPerceptionSystem* PerceptionSystem = UGAPerceptionSystem::GetPerceptionSystem(this);
	if (PerceptionSystem)
	{
		TArray<TObjectPtr<UGATargetComponent>>& TargetComponents = PerceptionSystem->GetAllTargetComponents();
		for (UGATargetComponent* TargetComponent : TargetComponents)
		{
			UpdateTargetView(TargetComponent, DeltaTime);
		}
	}
}

void UGAPerceptionComponent::UpdateTargetView(UGATargetComponent* TargetComponent, float DeltaTime)
{
	if (TargetComponent == NULL) 
	{
		UE_LOG(LogTemp, Warning, TEXT("UGAPerceptionComponent::UpdateTargetView TargetComponent is NULL."));
		return;
	}

	// REMEMBER: the UGAPerceptionComponent is going to be attached to the controller, not the pawn. So we call this special accessor to 
	// get the pawn that our controller is controlling
	APawn* OwnerPawn = GetOwnerPawn();
	if (OwnerPawn == NULL)
	{
		UE_LOG(LogTemp, Warning, TEXT("UGAPerceptionComponent::UpdateTargetView OwnerPawn is NULL."));
		return;
	}

	// If we don't already have a target data for the given target component, add it
	FTargetView *TargetView = TargetMap.Find(TargetComponent->TargetGuid);
	if (TargetView == NULL)		
	{
		FTargetView NewTargetView;
		FGuid TargetGuid = TargetComponent->TargetGuid;
		TargetView = &TargetMap.Add(TargetGuid, NewTargetView);
	}

	if (TargetView)
	{
		// TODO PART 3
		// 
		// - Update TargetView->bClearLOS
		//		Use this.VisionParameters to determine whether the target is within the vision cone or not 
		//		(and ideally do so before you cast a ray towards it)
		// - Update TargetView->Awareness
		//		On ticks when the AI has a clear LOS, the Awareness should grow
		//		On ticks when the AI does not have a clear LOS, the Awareness should decay
		//
		// Awareness should be clamped to the range [0, 1]
		// You can add parameters to the UGAPerceptionComponent to control the speed at which awareness rises and falls

		// YOUR CODE HERE
		AActor* TargetActor = TargetComponent->GetOwner();
		if (TargetActor == NULL)
		{
			UE_LOG(LogTemp, Warning, TEXT("UGAPerceptionComponent::UpdateTargetView TargetComponent's Actor is NULL."));
			return;
		}

		// Update LOS
		TArray<AActor*> TargetsToIgnore;
		TargetsToIgnore.Add(TargetActor);
		TargetView->bClearLos = InTargetViewCone(TargetActor->GetActorLocation(), TargetsToIgnore);

		// Update Awareness
		if (TargetView->bClearLos)
		{
			TargetView->Awareness = FMath::Min(TargetView->Awareness + (AwarenessGrowthRate * DeltaTime), 1);
		}
		else
		{
			TargetView->Awareness = FMath::Max(TargetView->Awareness - (AwarenessDecayRate * DeltaTime), 0);
		}
		//UE_LOG(LogTemp, Warning, TEXT("GUID: %s, Awareness: %f"), *TargetComponent->TargetGuid.ToString(), TargetView->Awareness);
	}
}


const FTargetView* UGAPerceptionComponent::GetTargetView(FGuid TargetGuid) const
{
	return TargetMap.Find(TargetGuid);
}

// NOTE: I was looking at figure 3.20 in the textbook to detect whether something is in the cone
bool UGAPerceptionComponent::InTargetViewCone(FVector Point, const TArray<AActor*>& TargetsToIgnore) const
{
	APawn* OwnerPawn = GetOwnerPawn();
	if (OwnerPawn == NULL)
	{
		UE_LOG(LogTemp, Warning, TEXT("UGAPerceptionComponent::UpdateTargetView OwnerPawn is NULL."));
		return false;
	}

	FVector CurrentLocation = OwnerPawn->GetActorLocation();
	FVector Direction = Point - CurrentLocation;
	Direction = Direction.GetSafeNormal(); // Normalize
	float ConeThreshold = FMath::Cos(FMath::DegreesToRadians(VisionParameters.VisionAngle));

	bool InCone = FVector::DotProduct(OwnerPawn->GetActorForwardVector(), Direction) > ConeThreshold;
	bool ClearLOS = false;

	// LOS Code from Assignment 3
	if (InCone)
	{
		UWorld* World = GetWorld();
		FHitResult HitResult;
		FCollisionQueryParams Params;
		FVector Start = CurrentLocation;			// need a ray start
		FVector End = Point;						// need a ray end
		Start.Z += 50.0f;							// offset by 50uus so 
		// Add any actors that should be ignored by the raycast by calling
		Params.AddIgnoredActor(OwnerPawn);			// Ignore the AI themself
		Params.AddIgnoredActors(TargetsToIgnore);	// Ignore the passed in targets (ex: player pawn)
		bool bHitSomething = World->LineTraceSingleByChannel(HitResult, Start, End, ECollisionChannel::ECC_Visibility, Params);
		ClearLOS = !bHitSomething;
	}

	return ClearLOS;
}