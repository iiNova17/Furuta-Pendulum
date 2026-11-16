#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "RipTwinActor.generated.h"

class FSocket;
class UMaterialInterface;
class UProceduralMeshComponent;

// Surface finish of a twin body (parameters of M_Twin / M_Ghost).
struct FFinish
{
	FLinearColor Color;
	float Roughness = 0.5f;
	float Metallic = 0.f;
};

// Latest angles from one sender. Packet source byte: 0 = real rig, 1 = model.
struct FRipSource
{
	float QArm = 0.f;       // rad, Simscape/firmware convention
	float QPend = 0.f;      // rad, 0 hanging, pi upright
	double LastTime = -1e9; // FPlatformTime::Seconds() of the last packet
	int32 WindowCount = 0;
	float RateHz = 0.f;

	bool IsLive(double Now) const { return Now - LastTime < 1.0; }
};

// The rig, posed from UDP packets on Port. Packet (16 bytes, little-endian):
//   char[4] "RIP1" | uint8 source | uint8[3] pad | float32 q_arm | float32 q_pend
// The solid rig follows the real rig; the translucent ghost follows the model. With no rig data the
// solid rig follows the model instead, so the twin also works as a pure model viewer.
UCLASS()
class RIPTWIN_API ARipTwinActor : public AActor
{
	GENERATED_BODY()

public:
	ARipTwinActor();
	virtual void Tick(float DeltaSeconds) override;

	UPROPERTY(EditAnywhere, Category = "Twin")
	int32 Port = 7400;

	FRipSource Real, Model;
	bool bModelDrivesSolid = false;
	FString Status;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	struct FBodySet
	{
		USceneComponent* ArmJoint = nullptr;
		USceneComponent* PendJoint = nullptr;
		UProceduralMeshComponent* Arm = nullptr;
		UProceduralMeshComponent* Pend = nullptr;
	};

	UProceduralMeshComponent* MakeMesh(const FString& Name, USceneComponent* Parent);
	FBodySet MakeSet(const FString& Prefix);
	bool LoadMesh(UProceduralMeshComponent* Mesh, const FString& File, UMaterialInterface* Base, const FFinish& Finish);
	bool LoadGeometry(const FString& File);
	void Receive(double Now);
	void Pose(const FBodySet& Set, float QArm, float QPend) const;
	void Automation();

	UProceduralMeshComponent* Housing = nullptr;
	FBodySet Solid, Ghost;

	UPROPERTY()
	TObjectPtr<UMaterialInterface> TwinMaterial;
	UPROPERTY()
	TObjectPtr<UMaterialInterface> GhostMaterial;

	FVector Pivot = FVector::ZeroVector;      // pendulum pivot in the arm-joint frame (zero pose), UE units
	FVector PendAxis = FVector::ForwardVector; // pendulum joint axis, UE frame
	FSocket* Socket = nullptr;
	double RateWindowStart = 0.0;

	// Scripted checks: -ripshot=<s> saves Saved/Screenshots/twin.png, -ripquit=<s> exits, s seconds after
	// start - or after the first packet if -ripafterdata is also given.
	float ShotAt = -1.f, QuitAt = -1.f;
	bool bShotTaken = false, bAfterData = false;
	double FirstPacketTime = -1.0;
};
