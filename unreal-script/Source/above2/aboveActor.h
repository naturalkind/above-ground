// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "aboveActor.generated.h"

UCLASS()
class ABOVE2_API AaboveActor : public AActor
{
	GENERATED_BODY()
	
//protected:
//    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "SplineController")
//    USceneComponent *Root;

public:	
	// Sets default values for this actor's properties
    AaboveActor();
    class USplineComponent* MySpline;
    class APlayerController* MyPlayerController;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
    virtual void Tick(float DeltaTime) override;
    virtual void PostActorCreated() override;
    virtual void PostLoad() override;

    void SetPosition();

		
};
