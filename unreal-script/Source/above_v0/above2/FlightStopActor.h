// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"
#include "FlightStopActor.generated.h"

UCLASS()
class ABOVE2_API AFlightStopActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFlightStopActor();

    /** Returns the next flight curve */
    UCurveFloat* GetNextFlightCurve() { return NextFlightCurve; };

    /** Returns the next flight spline component */
    USplineComponent* GetNextFlightSplineComp() { return NextFlightStop; };


protected:
	// Called when the game starts or when spawned
//	virtual void BeginPlay() override;

    /** The FloatCurve corresponding to the next flight spline component */
    UPROPERTY(EditAnywhere)
    UCurveFloat* NextFlightCurve;

    /** A static mesh for our flight stop */
    UPROPERTY(VisibleAnywhere)
    UStaticMeshComponent* SM;

    /** The spline component that describes the flight path of the next flight */
    UPROPERTY(VisibleAnywhere)
    USplineComponent* NextFlightStop;

};
