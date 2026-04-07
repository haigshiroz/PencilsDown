#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "GAPerceptionComponent.h"
#include "GATargetComponent.h"
#include "GAPerceptionSystem.generated.h"



UCLASS(BlueprintType, Blueprintable, meta = (BlueprintSpawnableComponent))
class UGAPerceptionSystem : public UActorComponent
{
	GENERATED_UCLASS_BODY()


	UPROPERTY(BlueprintReadOnly)
	TArray<TObjectPtr<UGAPerceptionComponent>> PerceptionComponents;

	UPROPERTY(BlueprintReadOnly)
	TArray<TObjectPtr<UGATargetComponent>> TargetComponents;

	// Cached pointer to the grid actor
	UPROPERTY()
	mutable TSoftObjectPtr<AGAGridActor> GridActor;

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	// Finds visible cells from perceptors and updates the GridActor
	void FindVisibleCellsHelper();

	UFUNCTION(BlueprintCallable)
	AGAGridActor* GetGridActor() const;

	bool RegisterPerceptionComponent(UGAPerceptionComponent* PerceptionComponent);
	bool UnregisterPerceptionComponent(UGAPerceptionComponent* PerceptionComponent);

	bool RegisterTargetComponent(UGATargetComponent* TargetComponent);
	bool UnregisterTargetComponent(UGATargetComponent* TargetComponent);

	TArray<TObjectPtr<UGATargetComponent>>& GetAllTargetComponents() { return TargetComponents; }
	TArray<TObjectPtr<UGAPerceptionComponent>>& GetAllPerceptionComponents() { return PerceptionComponents; }

	static UGAPerceptionSystem* GetPerceptionSystem(const UObject* WorldContextObject);

	UPROPERTY(EditAnywhere)
	TSubclassOf<AActor> ActorClassToIgnoreTracelines;
};