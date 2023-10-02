// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"
#include "DrawDebugHelpers.h"
#include "FlightStopActor.generated.h"

UCLASS()
class ABOVE_API AFlightStopActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFlightStopActor();

    /** Returns the next flight curve */
    UCurveFloat* GetNextFlightCurve() { return ToyCurve; };

    /** Returns the next flight spline component */
    USplineComponent* GetNextFlightSplineComp() { return NextFlightStop; };

    UPROPERTY(EditAnywhere, Category="ToyCurve")
    UCurveFloat* ToyCurve;

    FORCEINLINE
    void DrawPoint (
            const FVector& Loc,
            const float Size = 7,
            const FColor& Color = FColor::Red,
            const float Duration=-1.f
        ) const {
            DrawDebugPoint(
                GetWorld(),
                Loc,
                Size, //thickness
                Color,
                false,
                Duration
            );
        }

    static UCurveFloat* LoadCurveFloatAsset( const TCHAR* AssetPath );
    
    virtual void PostActorCreated() override;
    void SetPosition();

protected:
	// Called when the game starts or when spawned

    /** A static mesh for our flight stop */
    UPROPERTY(VisibleAnywhere)
    UStaticMeshComponent* SM;

    /** The spline component that describes the flight path of the next flight */
    UPROPERTY(VisibleAnywhere)
    USplineComponent* NextFlightStop;

};
