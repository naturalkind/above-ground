// Fill out your copyright notice in the Description page of Project Settings.
#include "FlightStopActor.h"
#include "Logging/LogMacros.h"



// Sets default values
AFlightStopActor::AFlightStopActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

    SM = CreateDefaultSubobject<UStaticMeshComponent>(FName("SM"));
    SetRootComponent(SM);

    //Init splines
    NextFlightStop = CreateDefaultSubobject<USplineComponent>(FName("SplineComp"));

    //Attach them to root component
    NextFlightStop->SetupAttachment(SM);

    ToyCurve = NewObject<UCurveFloat>(this, TEXT("DynamicUCurveFloat"));
    FKeyHandle KeyHandle = ToyCurve->FloatCurve.AddKey(0.0f, 0.1f);
    ToyCurve->FloatCurve.SetKeyInterpMode(KeyHandle, ERichCurveInterpMode::RCIM_Cubic, true);
    FKeyHandle KeyHandle2 = ToyCurve->FloatCurve.AddKey(100.0f, 1300.0f);
    ToyCurve->FloatCurve.SetKeyInterpMode(KeyHandle2, ERichCurveInterpMode::RCIM_Cubic, true);

    FString test_path = ToyCurve->GetPathName();
    UE_LOG(LogTemp, Warning, TEXT("--------->%s"), *test_path); //Display

}

UCurveFloat* AFlightStopActor::LoadCurveFloatAsset( const TCHAR * AssetPath )
{
    ConstructorHelpers::FObjectFinder<UCurveFloat> CurveAsset( AssetPath );
    if ( CurveAsset.Succeeded() )
        {
        return CurveAsset.Object;
        }

    return nullptr;
}

void AFlightStopActor::PostActorCreated()
{
    Super::PostActorCreated();
    SetPosition();
}


void AFlightStopActor::SetPosition()
{
    FVector startPos = FVector(0.0f,0.0f,20.0f);
    this->SetActorLocation(startPos);
    UE_LOG(LogTemp, Warning, TEXT("START THIS !!!"));
}
