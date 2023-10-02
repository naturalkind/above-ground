// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/TimelineComponent.h"
#include "Components/BoxComponent.h"
#include "GameFramework/Character.h"
#include "FlightStopActor.h"
#include "CaptureManager.h"
#include "FlightC.generated.h"

UCLASS(config=Game)
class ABOVE2_API AFlightC : public ACharacter
{
	GENERATED_BODY()

   UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
   class USpringArmComponent* CameraBoom;

   UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
   class UCameraComponent* FollowCamera;

public:
	// Sets default values for this character's properties
	AFlightC();

    FTimeline FlightTimeline;

    UFUNCTION()
    void TickTimeline(float Value);


    /** Base turn rate, in deg/sec. Other scaling may affect final turn rate. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category=Camera)
    float BaseTurnRate;

    /** Base look up/down rate, in deg/sec. Other scaling may affect final rate. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category=Camera)
    float BaseLookUpRate;

    // ????????????????

    UPROPERTY(EditAnywhere)
    class ACaptureManager* CaptureActor;  //Create a pointer to your another class
//    class AActor* CaptureActor;

//    UPROPERTY(EditAnywhere)
//    class USceneCaptureComponent2D* CaptureActor;
//    class ASceneCapture2D* CaptureActor;
    // ????????????????

    /** The active spline component, meaning the flight path that the character is currently following */
    USplineComponent* ActiveSplineComponent;

    /** The selected flight stop actor */
    AFlightStopActor* ActiveFlightStopActor;
    /** Box overlap function */
    UFUNCTION()
    void OnFlightBoxColliderOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult);

    /** Executes when we're pressing the NextFlightPath key bind */
    void NextFlightPathSelected();

    /** Updates the flight timeline with a new curve and starts the flight */
    void UpdateFlightTimeline(UCurveFloat* CurveFloatToBind);

    UFUNCTION()
    void ResetActiveFlightStopActor();


protected:
	// Called when the game starts or when spawned
    virtual void BeginPlay() override;

    /*The Box component that detects any nearby flight stops*/
    UPROPERTY(VisibleAnywhere)
    UBoxComponent* FlightBoxCollider;

    /** Called for forwards/backward input */
    void MoveForward(float Value);

    /** Called for side to side input */
    void MoveRight(float Value);

    /**
     * Called via input to turn at a given rate.
     * @param Rate	This is a normalized rate, i.e. 1.0 means 100% of desired turn rate
     */
    void TurnAtRate(float Rate);

    /**
     * Called via input to turn look up/down at a given rate.
     * @param Rate	This is a normalized rate, i.e. 1.0 means 100% of desired turn rate
     */
    void LookUpAtRate(float Rate);

    /** Handler for when a touch input begins. */
    void TouchStarted(ETouchIndex::Type FingerIndex, FVector Location);

    /** Handler for when a touch input stops. */
    void TouchStopped(ETouchIndex::Type FingerIndex, FVector Location);


public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    /** Returns CameraBoom subobject **/
    FORCEINLINE class USpringArmComponent* GetCameraBoom() const { return CameraBoom; }
    /** Returns FollowCamera subobject **/
    FORCEINLINE class UCameraComponent* GetFollowCamera() const { return FollowCamera; }
};
