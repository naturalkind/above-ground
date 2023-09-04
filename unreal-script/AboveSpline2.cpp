// Fill out your copyright notice in the Description page of Project Settings.

#include "Logging/LogMacros.h"
#include "AboveSpline2.h"
#include "Components/SplineComponent.h"
#include "Kismet/GameplayStatics.h"

#include "Components/SplineMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "Containers/Array.h"


// Sets default values
AAboveSpline2::AAboveSpline2()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    // подключить к актеру spline
    //	USplineComponent* mySpline =  CreateDefaultSubobject<>("MySpline",true);
    this->MySpline = CreateDefaultSubobject<USplineComponent>("MySpline",true);
    //mySpline =

    FSplinePoint myPoint;

    myPoint.Position = FVector(0.0f,0.0f,0.0f);
    myPoint.Rotation = FRotator(0.0f,0.0f,0.0f);
    myPoint.Scale = FVector(0.0f,0.0f,0.0f);

    this->MySpline->AddPoint(myPoint,true);

    this->MyPlayerController = UGameplayStatics::GetPlayerController(this, 0);

//    USplineMeshComponent* NewMesh = NewObject<USplineMeshComponent>(this);


}

// Called when the game starts or when spawned
void AAboveSpline2::BeginPlay()
{
	Super::BeginPlay();
    float LenGth_s = this->MySpline->GetSplineLength();

    UE_LOG(LogTemp, Warning, TEXT("APlayerCameraManage-------->%f"), LenGth_s);

}

// Called every frame
void AAboveSpline2::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

