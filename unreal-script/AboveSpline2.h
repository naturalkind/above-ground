// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AboveSpline2.generated.h"

UCLASS()
class ABOVE_API AAboveSpline2 : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AAboveSpline2();

    class USplineComponent* MySpline;
    class APlayerController* MyPlayerController;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
    virtual void Tick(float DeltaTime) override;

	
};
