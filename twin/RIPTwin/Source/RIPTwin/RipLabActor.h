#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RipLabActor.generated.h"

class UMaterialInterface;
class UStaticMesh;

// A simple lab corner around the rig: desk (top surface at z = 0, where the rig stands), floor, two walls
// and a whiteboard with the project title. Built from scaled cubes; 1 unit = 1 mm like the rig.
UCLASS()
class RIPTWIN_API ARipLabActor : public AActor
{
	GENERATED_BODY()

public:
	ARipLabActor();

protected:
	virtual void BeginPlay() override;

private:
	void Box(const FVector& Center, const FVector& Size, const FLinearColor& Color, float Roughness, float Metallic = 0.f);
	void Label(const FString& Text, const FVector& Location, float Size, const FLinearColor& Color);

	UPROPERTY()
	TObjectPtr<UStaticMesh> Cube;
	UPROPERTY()
	TObjectPtr<UMaterialInterface> Material;
};
