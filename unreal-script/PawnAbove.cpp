// Fill out your copyright notice in the Description page of Project Settings.


#include "PawnAbove.h"
#include "Logging/LogMacros.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Kismet/GameplayStatics.h"


// Sets default values
APawnAbove::APawnAbove()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

//    CameraComp = CreateDefaultSubobject<UCameraComponent>(TEXT("CameraComponent"));
//    CameraComp->SetupAttachment(SpringArmComp,USpringArmComponent::SocketName);

}



// Called when the game starts or when spawned
void APawnAbove::BeginPlay()
{

	Super::BeginPlay();

    FRotator NewRotation = FRotator(-90.0f, 0.0f, 0.0f);
    FVector NewLocation = FVector(0.0f, 0.0f, 1200.0f);
    //get player controller at index 0
    APlayerController* MyPlayerController = UGameplayStatics::GetPlayerController(this, 0);
//    APlayerController* MyPlayerController = UGameplayStatics::GetPlayerController(this->GetWorld(), 0);
//    APlayerController* MyPlayerController = GetWorld()->GetFirstPlayerController();

    //Get this actors rotation
//    FRotator rot = this->GetActorRotation();

    //Set control rotation of controller
    MyPlayerController->SetControlRotation(NewRotation);
    MyPlayerController->GetPawn()->SetActorLocation(NewLocation);


//    APawn* Pawn = this;
//    MyPlayerController->SetPawn(Pawn);

//    GetWorld()->GetFirstPlayerController()->SetViewTarget(this);

//    APlayerCameraManager *camManager = GetWorld()->GetFirstPlayerController()->PlayerCameraManager;
//    camManager->SetActorRotation(NewRotation);
//    this->SetActorRotation(NewRotation);


//    MyPlayerController->GetPawn()->TeleportTo(NewLocation, NewRotation);


}

// Called every frame
void APawnAbove::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    // Standard way to log to console.
    APlayerCameraManager *camManager = GetWorld()->GetFirstPlayerController()->PlayerCameraManager;
    FVector camLocation = camManager->GetCameraLocation();
    FVector camForward  = camManager->GetCameraRotation().Vector();

    UE_LOG(LogTemp, Warning, TEXT("APlayerCameraManage-------->%s"), *camForward.ToString());

    // possition pawn vector
    FVector MyCharacterPosition = GetWorld()->GetFirstPlayerController()->GetPawn()->GetActorLocation();
    GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Orange, FString::Printf(TEXT("My Location is: %s"), *MyCharacterPosition.ToString()));

}

// Called to bind functionality to input
void APawnAbove::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

}

