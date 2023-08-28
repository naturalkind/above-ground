// Fill out your copyright notice in the Description page of Project Settings.

#include "Logging/LogMacros.h"
#include "aboveActor.h"
#include "Components/SplineComponent.h"
#include "Kismet/GameplayStatics.h"

#include "Components/SplineMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/StaticMesh.h"
#include "Containers/Array.h"


// Sets default values
AaboveActor::AaboveActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

    this->MySpline = CreateDefaultSubobject<USplineComponent>("MySpline",true);
    SetRootComponent(this->MySpline);


    FVector startPos = FVector(0.0f,0.0f,20.0f);
    FRotator NewRotation = FRotator(-90.0f, 0.0f, 0.0f);
    SetActorLocation(startPos);

    SetActorRelativeLocation(startPos);
//    GetWorld()->SpawnActor<AaboveActor>()

//    MyCustomActor->SetActorLocation(startPos);
// ------------------------>
//    FSplinePoint myPoint;
//    myPoint.Position = FVector(0.0f,0.0f,0.0f);
//    myPoint.Rotation = FRotator(0.0f,0.0f,0.0f);
//    myPoint.Scale = FVector(0.0f,0.0f,0.0f);

//    this->MySpline->AddPoint(myPoint,true);
// ------------------------>
    this->MyPlayerController = UGameplayStatics::GetPlayerController(this, 0);

    int32 NumberPoints = MySpline->GetNumberOfSplinePoints();

    UE_LOG(LogTemp, Warning, TEXT("Test Log. Num Points %d"), NumberPoints); //Display

}

// Called when the game starts or when spawned
void AaboveActor::BeginPlay()
{
	Super::BeginPlay();
    float LenGth_s = this->MySpline->GetSplineLength();
    FVector actLocation = this->GetActorLocation();

    UE_LOG(LogTemp, Warning, TEXT("SplineLength-->%f, Location-->%s"), LenGth_s, *actLocation.ToString());
	
}

// Called every frame
void AaboveActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}


// This is called when actor is spawned (at runtime or when you drop it into the world in editor)
void AaboveActor::PostActorCreated()
{
    Super::PostActorCreated();
    SetPosition();
}

void AaboveActor::PostLoad()
{
    Super::PostLoad();
    UE_LOG(LogTemp, Warning, TEXT("START THIS !!!!!!!!!!!!!!!"));
//    SetPosition();
}

void AaboveActor::SetPosition()
{
    FVector startPos = FVector(0.0f,0.0f,20.0f);
    this->SetActorLocation(startPos);
    UE_LOG(LogTemp, Warning, TEXT("START THIS !!!"));
}

//while (true)
//  {
//      int32 bytes_read = 0;
//      if (Socket->Recv(data, sizeof(data), bytes_read)){
//          data[bytes_read] = 0;
//          ACharacter* myCharacter = UGameplayStatics::GetPlayerCharacter(World, 0);
//          myCharacter->AddMovementInput(FVector(10, 0, 0), 5.0);
//      }
//  }


//    for ( i = 0; i < 10; i++ ) { }
//    for(const FVector& EachLocation : StarLocations)
//    {
//    ClientMessage(EachLocation.ToString());
//    }
