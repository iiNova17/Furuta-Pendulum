#include "RipTwinGameMode.h"

#include "Components/LightComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/StaticMeshActor.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/DefaultPawn.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "RipLabActor.h"
#include "RipTwinActor.h"
#include "RipTwinHUD.h"

ARipTwinGameMode::ARipTwinGameMode()
{
	DefaultPawnClass = ADefaultPawn::StaticClass();
	HUDClass = ARipTwinHUD::StaticClass();
}

void ARipTwinGameMode::StartPlay()
{
	// The engine template map provides sky, sun and fog. Swap its checker floor for the lab, and turn the sun
	// so it comes over the camera's shoulder and throws the rig's shadow towards the whiteboard.
	for (TActorIterator<AStaticMeshActor> It(GetWorld()); It; ++It)
	{
		if (It->GetName().Contains(TEXT("Floor")))
		{
			It->Destroy();
		}
	}
	for (TActorIterator<ADirectionalLight> It(GetWorld()); It; ++It)
	{
		It->GetLightComponent()->SetMobility(EComponentMobility::Movable);
		It->SetActorRotation(FRotator(-48.0, -150.0, 0.0));
	}
	GetWorld()->SpawnActor<ARipLabActor>(FVector::ZeroVector, FRotator::ZeroRotator);
	GetWorld()->SpawnActor<ARipTwinActor>(FVector::ZeroVector, FRotator::ZeroRotator);
	Super::StartPlay();
}

void ARipTwinGameMode::RestartPlayer(AController* NewPlayer)
{
	// The twin is a 10x model (1 mm = 1 unit), about 130 units tall, centred on the motor axis.
	const FVector Eye(300.0, 215.0, 150.0), Target(-60.0, 5.0, 125.0);
	RestartPlayerAtTransform(NewPlayer, FTransform((Target - Eye).Rotation(), Eye));
	if (ADefaultPawn* Pawn = Cast<ADefaultPawn>(NewPlayer->GetPawn()))
	{
		if (UFloatingPawnMovement* Move = Cast<UFloatingPawnMovement>(Pawn->GetMovementComponent()))
		{
			Move->MaxSpeed = 300.f;
		}
	}
}
