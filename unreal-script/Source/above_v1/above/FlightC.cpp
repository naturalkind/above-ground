// Fill out your copyright notice in the Description page of Project Settings.
#include "FlightC.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "HeadMountedDisplayFunctionLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "CaptureManager.h"
#include "Misc/FileHelper.h"
//#include "HAL/PlatformFilemanager.h"
//#include "Kismet/HeadMountedDisplayFunctionLibrary.h"

#include "Logging/LogMacros.h"


// Sets default values
AFlightC::AFlightC()
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;

    // Set size for collision capsule
    GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);

    // set our turn rates for input
    BaseTurnRate = 45.f;
    BaseLookUpRate = 45.f;

    // Don't rotate when the controller rotates. Let that just affect the camera.
    bUseControllerRotationPitch = false;
    bUseControllerRotationYaw = false;
    bUseControllerRotationRoll = false;

    // Configure character movement
//    GetCharacterMovement()->bOrientRotationToMovement = true; // Character moves in the direction of input...
//    GetCharacterMovement()->RotationRate = FRotator(0.0f, 540.0f, 0.0f); // ...at this rotation rate
//    GetCharacterMovement()->JumpZVelocity = 600.f;
//    GetCharacterMovement()->AirControl = 0.2f;

    // Create a camera boom (pulls in towards the player if there is a collision)
    CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
    CameraBoom->SetupAttachment(RootComponent);
    CameraBoom->TargetArmLength = 300.0f; // The camera follows at this distance behind the character
    CameraBoom->bUsePawnControlRotation = true; // Rotate the arm based on the controller

//    // Create a follow camera
    FollowCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FollowCamera"));
    FollowCamera->SetupAttachment(CameraBoom, USpringArmComponent::SocketName); // Attach the camera to the end of the boom and let the boom adjust to match the controller orientation
    FollowCamera->bUsePawnControlRotation = false; // Camera does not rotate relative to arm

//    // Note: The skeletal mesh and anim blueprint references on the Mesh component (inherited from Character)
//    // are set in the derived blueprint asset named MyCharacter (to avoid direct content references in C++)

    FlightBoxCollider = CreateDefaultSubobject<UBoxComponent>(FName("FlightBoxCollider"));

    FlightBoxCollider->SetBoxExtent(FVector(150.f));

    FlightBoxCollider->SetupAttachment(GetRootComponent());
 
}


// Called when the game starts or when spawned
void AFlightC::BeginPlay()
{
    Super::BeginPlay();
    //Register a function that gets called when the box overlaps with a component
    FlightBoxCollider->OnComponentBeginOverlap.AddDynamic(this, &AFlightC::OnFlightBoxColliderOverlap);

    // work https://code911.top/howto/unreal-how-to-delete-link-code-example
    AActor* FoundActor = UGameplayStatics::GetActorOfClass(GetWorld(), ACaptureManager::StaticClass());
    CaptureActor = Cast<ACaptureManager>(FoundActor);
    //    if (CaptureActor->IsA<ACaptureManager>())
    if (CaptureActor)
    {
        UE_LOG(LogTemp, Warning, TEXT("CaptureActor....>>> %s"), *CaptureActor->GetFName().ToString());
    }

}

// Called every frame
void AFlightC::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    //If the timeline has started, advance it by DeltaSeconds
    if (FlightTimeline.IsPlaying()) FlightTimeline.TickTimeline(DeltaTime);

}

// Called to bind functionality to input
void AFlightC::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
//    Super::SetupPlayerInputComponent(PlayerInputComponent);
    // Set up gameplay key bindings
    check(PlayerInputComponent);
    PlayerInputComponent->BindAction("Jump", IE_Pressed, this, &ACharacter::Jump);
    PlayerInputComponent->BindAction("Jump", IE_Released, this, &ACharacter::StopJumping);

    PlayerInputComponent->BindAxis("MoveForward", this, &AFlightC::MoveForward);
    PlayerInputComponent->BindAxis("MoveRight", this, &AFlightC::MoveRight);

    //Bind the functions that execute on key press
    PlayerInputComponent->BindAction("NextFlightPath", IE_Pressed, this, &AFlightC::NextFlightPathSelected);
//    PlayerInputComponent->BindAction("PreviousFlightPath", IE_Pressed, this, &AFlightC::PreviousFlightPathSelected);

    // We have 2 versions of the rotation bindings to handle different kinds of devices differently
    // "turn" handles devices that provide an absolute delta, such as a mouse.
    // "turnrate" is for devices that we choose to treat as a rate of change, such as an analog joystick
    PlayerInputComponent->BindAxis("Turn", this, &APawn::AddControllerYawInput);
    PlayerInputComponent->BindAxis("TurnRate", this, &AFlightC::TurnAtRate);
    PlayerInputComponent->BindAxis("LookUp", this, &APawn::AddControllerPitchInput);
    PlayerInputComponent->BindAxis("LookUpRate", this, &AFlightC::LookUpAtRate);

    // handle touch devices
    PlayerInputComponent->BindTouch(IE_Pressed, this, &AFlightC::TouchStarted);
    PlayerInputComponent->BindTouch(IE_Released, this, &AFlightC::TouchStopped);



}

float AFlightC::GetDistanceAlongSplineAtWorldLocation(const USplineComponent* InSpline, const FVector InWorldLocation)
{
    if (!InSpline)
        return 0.f;
    auto InputKeyFloat = InSpline->FindInputKeyClosestToWorldLocation(InWorldLocation);
    auto InputKey = FMath::TruncToInt(InputKeyFloat);
    auto A = InSpline->GetDistanceAlongSplineAtSplinePoint(InputKey);
    auto B = InSpline->GetDistanceAlongSplineAtSplinePoint(InputKey + 1);
    return A + ((B - A) * (InputKeyFloat - InputKey));
}


void AFlightC::TickTimeline(float Value)
{

    float SplineLength = ActiveSplineComponent->GetSplineLength();
//    float SplineLength = this->ActiveSplineComponent->GetSplineLength();

//    UE_LOG(LogTemp, Warning, TEXT("APlayerCameraManage-------->%f"), SplineLength);
//    UE_LOG(LogTemp, Warning, TEXT("TickTimeline..............")); //Display
    //Get the new location based on the provided values from the timeline.
    //The reason we're multiplying Value with SplineLength is because all our designed curves in the UE4 editor have a time range of 0 - X.
    //Where X is the total flight time
    FVector NewLocation = ActiveSplineComponent->GetLocationAtDistanceAlongSpline(Value * SplineLength, ESplineCoordinateSpace::World);

    SetActorLocation(NewLocation);

    FRotator NewRotation = ActiveSplineComponent->GetRotationAtDistanceAlongSpline(Value * SplineLength, ESplineCoordinateSpace::World);

    //We're not interested in the pitch value of the above rotation so we make sure to set it to zero
    NewRotation.Pitch = 0;

    SetActorRotation(NewRotation);

    CaptureActor->TestFunc(NewLocation);

    UE_LOG(LogTemp, Warning, TEXT("POSITION IN WORLD-------->%s"), *NewLocation.ToString());

    TStr += FString::Printf(TEXT("%s\n"), *NewLocation.ToString());


//    this->GetOwner()->SetActorLocation(NewLocation);
//    CaptureActor->GetOwner()->SetActorLocation(NewLocation);
//    CaptureActor->GetOwner()->SetActorLocation(NewLocation);

    //GameManager->CaptureNonBlocking();

    
    float endT = this->GetDistanceAlongSplineAtWorldLocation(ActiveSplineComponent, NewLocation);
    UE_LOG(LogTemp, Warning, TEXT("POSITION IN WORLD-------->%s"), *NewLocation.ToString());
    if (SplineLength==endT) {
        if (isFinishedSaveData) {
            // Save file
            FString SaveDirectoryCoord = FString("/media/sadko/unrealdir/above/Saved");
            FString FileNameCoord = FString("coord_scren.txt");
            bool AllowOverwritingCoord = false;
            IPlatformFile& FileManagerCoord = FPlatformFileManager::Get().GetPlatformFile();
            if (FileManagerCoord.CreateDirectoryTree(*SaveDirectoryCoord))
            {
                // Get absolute file path
                FString AbsoluteFilePath = SaveDirectoryCoord + "/" + FileNameCoord;

                // Allow overwriting or file doesn't already exist
                if (AllowOverwritingCoord || !FileManagerCoord.FileExists(*AbsoluteFilePath))
                {
                    FFileHelper::SaveStringToFile(TStr, *AbsoluteFilePath);
                    isFinishedSaveData = false;
                }
            }
        }
    }
    
}


FVector GetSplinePointLocationInConstructionScript(USplineComponent* SplineComponent, int32 PointIndex)
{
// Check if the SplineComponent is valid
if (!SplineComponent)
{
    UE_LOG(LogTemp, Error, TEXT("Invalid SplineComponent"));
    return FVector::ZeroVector;
}

// Check if the PointIndex is valid
if (PointIndex < 0 || PointIndex >= SplineComponent->GetNumberOfSplinePoints())
{
    UE_LOG(LogTemp, Error, TEXT("Invalid PointIndex"));
    return FVector::ZeroVector;
}
// Get the location of the spline point
FVector SplinePointLocation = SplineComponent->GetLocationAtSplinePoint(PointIndex, ESplineCoordinateSpace::World);

return SplinePointLocation;
}


void AFlightC::OnFlightBoxColliderOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult)
{
    UE_LOG(LogTemp, Warning, TEXT("START 1")); //Display
    if (OtherActor->IsA<AFlightStopActor>())
    {
        UE_LOG(LogTemp, Warning, TEXT("START 2")); //Display
        //Store a reference of the nearby flight stop actor
        ActiveFlightStopActor = Cast<AFlightStopActor>(OtherActor);
    }
}
void AFlightC::NextFlightPathSelected()
{
    GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Orange, TEXT("KLICK BUTTON"));


//    // get actor in scene by class
    AActor* FoundActor = UGameplayStatics::GetActorOfClass(GetWorld(), AFlightStopActor::StaticClass());
    AFlightStopActor* GameManager = Cast<AFlightStopActor>(FoundActor);

//    const int32 NumPointSpline = GameManager->GetNextFlightSplineComp()->GetNumberOfSplinePoints();
//    // location first point in spline
//    FVector SplinePointLocation = GameManager->GetNextFlightSplineComp()->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::World);
//    UE_LOG(LogTemp, Warning, TEXT("NextFlightPathSelected-------->%s"), *SplinePointLocation.ToString());

    ActiveFlightStopActor = Cast<AFlightStopActor>(GameManager);


    if (ActiveFlightStopActor)
    {
        //Get the next flight path's spline component and update the flight timeline with the corresponding curve
        ActiveSplineComponent = ActiveFlightStopActor->GetNextFlightSplineComp();
        UpdateFlightTimeline(ActiveFlightStopActor->GetNextFlightCurve());
    }
}

void AFlightC::UpdateFlightTimeline(UCurveFloat* CurveFloatToBind)
{
    UE_LOG(LogTemp, Warning, TEXT("...UpdateFlightTimeline"));
    //Initialize a timeline
    FlightTimeline = FTimeline();

    FOnTimelineFloat ProgressFunction;

    //Bind the function that ticks the timeline
    ProgressFunction.BindUFunction(this, FName("TickTimeline"));

    //Assign the provided curve and progress function for our timeline
    FlightTimeline.AddInterpFloat(CurveFloatToBind, ProgressFunction);
    FlightTimeline.SetLooping(false);
    FlightTimeline.PlayFromStart();

    //Set the timeline's length to match the last key frame based on the given curve
    FlightTimeline.SetTimelineLengthMode(TL_LastKeyFrame);

    //The ResetActiveFlightStopActor executes when the timeline finishes.
    //By calling ResetActiveFlightStopActor at the end of the timeline we make sure to reset any invalid references on ActiveFlightStopActor
    FOnTimelineEvent TimelineEvent;
    TimelineEvent.BindUFunction(this, FName("ResetActiveFlightStopActor"));
    FlightTimeline.SetTimelineFinishedFunc(TimelineEvent);
}

void AFlightC::ResetActiveFlightStopActor()
{
    UE_LOG(LogTemp, Warning, TEXT("...ResetActiveFlightStopActoru")); //Display
    ActiveFlightStopActor = nullptr;
}

void AFlightC::TouchStarted(ETouchIndex::Type FingerIndex, FVector Location)
{
    // jump, but only on the first touch
    if (FingerIndex == ETouchIndex::Touch1)
    {
        Jump();
    }
}

void AFlightC::TouchStopped(ETouchIndex::Type FingerIndex, FVector Location)
{
    if (FingerIndex == ETouchIndex::Touch1)
    {
        StopJumping();
    }
}

void AFlightC::TurnAtRate(float Rate)
{
    // calculate delta for this frame from the rate information
    AddControllerYawInput(Rate * BaseTurnRate * GetWorld()->GetDeltaSeconds());
}

void AFlightC::LookUpAtRate(float Rate)
{
    // calculate delta for this frame from the rate information
    AddControllerPitchInput(Rate * BaseLookUpRate * GetWorld()->GetDeltaSeconds());
}

void AFlightC::MoveForward(float Value)
{
    if ((Controller != NULL) && (Value != 0.0f))
    {
        // find out which way is forward
        const FRotator Rotation = Controller->GetControlRotation();
        const FRotator YawRotation(0, Rotation.Yaw, 0);

        // get forward vector
        const FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
        AddMovementInput(Direction, Value);
    }
}

void AFlightC::MoveRight(float Value)
{
    if ( (Controller != NULL) && (Value != 0.0f) )
    {
        // find out which way is right
        const FRotator Rotation = Controller->GetControlRotation();
        const FRotator YawRotation(0, Rotation.Yaw, 0);

        // get right vector
        const FVector Direction = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y);
        // add movement in that direction
        AddMovementInput(Direction, Value);
    }
}
