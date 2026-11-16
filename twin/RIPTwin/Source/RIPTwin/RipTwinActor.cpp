#include "RipTwinActor.h"

#include "Common/UdpSocketBuilder.h"
#include "Dom/JsonObject.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "ProceduralMeshComponent.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "UObject/ConstructorHelpers.h"
#include "UnrealClient.h"

namespace
{
	// Graphite housing, orange arm, blue pendulum (sRGB)
	const FFinish HousingFinish{FLinearColor::FromSRGBColor(FColor(52, 56, 62)), 0.55f, 0.f}; // graphite PLA
	const FFinish ArmFinish{FLinearColor::FromSRGBColor(FColor(255, 128, 20)), 0.3f, 0.f};    // orange
	const FFinish PendFinish{FLinearColor::FromSRGBColor(FColor(30, 90, 220)), 0.25f, 0.f};   // blue
	const FFinish GhostFinish{FLinearColor::FromSRGBColor(FColor(80, 230, 255)), 0.2f, 0.f};

	FString ContentFile(const TCHAR* Name) { return FPaths::ProjectContentDir() / TEXT("RIP") / Name; }
}

ARipTwinActor::ARipTwinActor()
{
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	Housing = MakeMesh(TEXT("Housing"), RootComponent);
	Solid = MakeSet(TEXT("Solid"));
	Ghost = MakeSet(TEXT("Ghost"));

	static ConstructorHelpers::FObjectFinder<UMaterialInterface> TwinMat(TEXT("/Game/RIP/M_Twin.M_Twin"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> GhostMat(TEXT("/Game/RIP/M_Ghost.M_Ghost"));
	TwinMaterial = TwinMat.Object;
	GhostMaterial = GhostMat.Object;
}

UProceduralMeshComponent* ARipTwinActor::MakeMesh(const FString& Name, USceneComponent* Parent)
{
	UProceduralMeshComponent* Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(*Name);
	Mesh->SetupAttachment(Parent);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	return Mesh;
}

ARipTwinActor::FBodySet ARipTwinActor::MakeSet(const FString& Prefix)
{
	// Meshes are stored in the zero pose, so each joint is a scene component at the joint origin and its
	// mesh is offset back by that origin: rotating the joint component then rotates the body about the axis.
	FBodySet Set;
	Set.ArmJoint = CreateDefaultSubobject<USceneComponent>(*(Prefix + TEXT("ArmJoint")));
	Set.ArmJoint->SetupAttachment(RootComponent);
	Set.Arm = MakeMesh(Prefix + TEXT("Arm"), Set.ArmJoint);
	Set.PendJoint = CreateDefaultSubobject<USceneComponent>(*(Prefix + TEXT("PendJoint")));
	Set.PendJoint->SetupAttachment(Set.ArmJoint);
	Set.Pend = MakeMesh(Prefix + TEXT("Pendulum"), Set.PendJoint);
	return Set;
}

void ARipTwinActor::BeginPlay()
{
	Super::BeginPlay();

	bool bOk = LoadGeometry(ContentFile(TEXT("twin_geometry.json")));
	bOk &= LoadMesh(Housing, ContentFile(TEXT("housing.ripmesh")), TwinMaterial, HousingFinish);
	bOk &= LoadMesh(Solid.Arm, ContentFile(TEXT("arm.ripmesh")), TwinMaterial, ArmFinish);
	bOk &= LoadMesh(Solid.Pend, ContentFile(TEXT("pendulum.ripmesh")), TwinMaterial, PendFinish);
	bOk &= LoadMesh(Ghost.Arm, ContentFile(TEXT("arm.ripmesh")), GhostMaterial, GhostFinish);
	bOk &= LoadMesh(Ghost.Pend, ContentFile(TEXT("pendulum.ripmesh")), GhostMaterial, GhostFinish);
	for (const FBodySet* Set : {&Solid, &Ghost})
	{
		Set->PendJoint->SetRelativeLocation(Pivot);
		Set->Pend->SetRelativeLocation(-Pivot);
	}
	Ghost.ArmJoint->SetVisibility(false, true);

	Socket = FUdpSocketBuilder(TEXT("RIPTwin"))
		.AsNonBlocking()
		.AsReusable()
		.BoundToPort(Port)
		.WithReceiveBufferSize(1 << 16)
		.Build();
	if (!Socket)
	{
		Status = FString::Printf(TEXT("could not open UDP port %d"), Port);
	}
	else if (!bOk && Status.IsEmpty())
	{
		Status = TEXT("could not load the rig meshes - run twin/matlab/export_twin_meshes.m");
	}

	FParse::Value(FCommandLine::Get(), TEXT("ripshot="), ShotAt);
	FParse::Value(FCommandLine::Get(), TEXT("ripquit="), QuitAt);
	bAfterData = FParse::Param(FCommandLine::Get(), TEXT("ripafterdata"));
	RateWindowStart = FPlatformTime::Seconds();
}

void ARipTwinActor::EndPlay(const EEndPlayReason::Type Reason)
{
	if (Socket)
	{
		Socket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
		Socket = nullptr;
	}
	Super::EndPlay(Reason);
}

bool ARipTwinActor::LoadGeometry(const FString& File)
{
	FString Text;
	TSharedPtr<FJsonObject> Json;
	if (!FFileHelper::LoadFileToString(Text, *File) ||
		!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Json) || !Json.IsValid())
	{
		Status = TEXT("missing Content/RIP/twin_geometry.json - run twin/matlab/export_twin_meshes.m");
		return false;
	}
	auto Vec = [&Json](const TCHAR* Key, FVector& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		if (Json->TryGetArrayField(Key, A) && A->Num() == 3)
		{
			Out = FVector((*A)[0]->AsNumber(), (*A)[1]->AsNumber(), (*A)[2]->AsNumber());
		}
	};
	Vec(TEXT("pivot"), Pivot);
	Vec(TEXT("pend_axis"), PendAxis);
	PendAxis = PendAxis.GetSafeNormal();
	return true;
}

bool ARipTwinActor::LoadMesh(UProceduralMeshComponent* Mesh, const FString& File, UMaterialInterface* Base, const FFinish& Finish)
{
	TArray<uint8> Bytes;
	if (!FFileHelper::LoadFileToArray(Bytes, *File) || Bytes.Num() < 8 || FMemory::Memcmp(Bytes.GetData(), "RIPM", 4) != 0)
	{
		return false;
	}
	uint32 NumTri = 0;
	FMemory::Memcpy(&NumTri, Bytes.GetData() + 4, 4);
	if (Bytes.Num() < 8 + int64(NumTri) * 36)
	{
		return false;
	}
	const float* P = reinterpret_cast<const float*>(Bytes.GetData() + 8);

	// Unshared vertices with one normal per triangle: flat shading, like the CAD faces.
	TArray<FVector> Vertices, Normals;
	TArray<int32> Triangles;
	Vertices.SetNumUninitialized(NumTri * 3);
	Normals.SetNumUninitialized(NumTri * 3);
	Triangles.SetNumUninitialized(NumTri * 3);
	for (uint32 T = 0; T < NumTri; ++T)
	{
		const float* Q = P + 9 * T;
		const FVector A(Q[0], Q[1], Q[2]), B(Q[3], Q[4], Q[5]), C(Q[6], Q[7], Q[8]);
		const FVector N = FVector::CrossProduct(B - A, C - A).GetSafeNormal();
		for (int32 K = 0; K < 3; ++K)
		{
			Normals[3 * T + K] = N;
		}
		Vertices[3 * T] = A;
		Vertices[3 * T + 1] = B;
		Vertices[3 * T + 2] = C;
		Triangles[3 * T] = 3 * T;
		Triangles[3 * T + 1] = 3 * T + 2; // Unreal's front faces wind the other way round
		Triangles[3 * T + 2] = 3 * T + 1;
	}
	const TArray<FVector2D> NoUV;
	const TArray<FLinearColor> NoColors;
	const TArray<FProcMeshTangent> NoTangents;
	Mesh->CreateMeshSection_LinearColor(0, Vertices, Triangles, Normals, NoUV, NoColors, NoTangents, false);

	if (Base)
	{
		UMaterialInstanceDynamic* Material = UMaterialInstanceDynamic::Create(Base, this);
		Material->SetVectorParameterValue(TEXT("Color"), Finish.Color);
		Material->SetScalarParameterValue(TEXT("Roughness"), Finish.Roughness);
		Material->SetScalarParameterValue(TEXT("Metallic"), Finish.Metallic);
		Mesh->SetMaterial(0, Material);
	}
	return true;
}

void ARipTwinActor::Receive(double Now)
{
	if (!Socket)
	{
		return;
	}
	uint8 Buffer[64];
	uint32 Pending = 0;
	while (Socket->HasPendingData(Pending))
	{
		int32 Read = 0;
		if (!Socket->Recv(Buffer, sizeof(Buffer), Read))
		{
			break;
		}
		if (Read < 16 || FMemory::Memcmp(Buffer, "RIP1", 4) != 0 || Buffer[4] > 1)
		{
			continue;
		}
		FRipSource& Source = Buffer[4] == 0 ? Real : Model;
		if (Source.LastTime < 0.0)
		{
			UE_LOG(LogTemp, Display, TEXT("RIPTwin: first %s packet"), Buffer[4] == 0 ? TEXT("real") : TEXT("model"));
		}
		if (FirstPacketTime < 0.0)
		{
			FirstPacketTime = Now;
		}
		FMemory::Memcpy(&Source.QArm, Buffer + 8, 4);
		FMemory::Memcpy(&Source.QPend, Buffer + 12, 4);
		Source.LastTime = Now;
		++Source.WindowCount;
	}
}

void ARipTwinActor::Pose(const FBodySet& Set, float QArm, float QPend) const
{
	// Unreal is left-handed (y mirrored), so a right-handed rotation by q is a rotation by -q here.
	Set.ArmJoint->SetRelativeRotation(FQuat(FVector::UpVector, -QArm));
	Set.PendJoint->SetRelativeRotation(FQuat(PendAxis, -QPend));
}

void ARipTwinActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	const double Now = FPlatformTime::Seconds();
	Receive(Now);

	if (Now - RateWindowStart >= 1.0)
	{
		for (FRipSource* Source : {&Real, &Model})
		{
			Source->RateHz = Source->WindowCount / float(Now - RateWindowStart);
			Source->WindowCount = 0;
		}
		RateWindowStart = Now;
	}

	const bool bReal = Real.IsLive(Now), bModel = Model.IsLive(Now);
	bModelDrivesSolid = !bReal && bModel;
	const FRipSource& SolidSource = bModelDrivesSolid ? Model : Real;
	Pose(Solid, SolidSource.QArm, SolidSource.QPend);

	const bool bGhost = bReal && bModel;
	if (Ghost.ArmJoint->IsVisible() != bGhost)
	{
		Ghost.ArmJoint->SetVisibility(bGhost, true);
	}
	if (bGhost)
	{
		Pose(Ghost, Model.QArm, Model.QPend);
	}
	Automation();
}

void ARipTwinActor::Automation()
{
	double T = GetWorld()->GetTimeSeconds();
	if (bAfterData)
	{
		if (FirstPacketTime < 0.0)
		{
			return;
		}
		T = FPlatformTime::Seconds() - FirstPacketTime;
	}
	if (ShotAt >= 0.f && !bShotTaken && T >= ShotAt)
	{
		FScreenshotRequest::RequestScreenshot(FPaths::ProjectSavedDir() / TEXT("Screenshots") / TEXT("twin.png"), true, false);
		bShotTaken = true;
	}
	if (QuitAt >= 0.f && T >= QuitAt)
	{
		FPlatformMisc::RequestExit(false);
	}
}
