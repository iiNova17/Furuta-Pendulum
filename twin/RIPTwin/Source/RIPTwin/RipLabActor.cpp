#include "RipLabActor.h"

#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	FLinearColor SRGB(uint8 R, uint8 G, uint8 B) { return FLinearColor::FromSRGBColor(FColor(R, G, B)); }

	// Layout (mm): desk top at z = 0, floor 750 below, walls behind the desk on the -X and -Y sides.
	constexpr double DeskThick = 30.0, FloorZ = -750.0;
	constexpr double WallX = -520.0, WallY = -420.0, WallTop = 1300.0;
}

ARipLabActor::ARipLabActor()
{
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UMaterialInterface> TwinMat(TEXT("/Game/RIP/M_Twin.M_Twin"));
	Cube = CubeMesh.Object;
	Material = TwinMat.Object;
}

void ARipLabActor::Box(const FVector& Center, const FVector& Size, const FLinearColor& Color, float Roughness, float Metallic)
{
	UStaticMeshComponent* Mesh = NewObject<UStaticMeshComponent>(this);
	Mesh->SetupAttachment(RootComponent);
	Mesh->SetStaticMesh(Cube);
	Mesh->SetRelativeLocation(Center);
	Mesh->SetRelativeScale3D(Size / 100.0); // the engine cube is 100 units
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	UMaterialInstanceDynamic* Mat = UMaterialInstanceDynamic::Create(Material, this);
	Mat->SetVectorParameterValue(TEXT("Color"), Color);
	Mat->SetScalarParameterValue(TEXT("Roughness"), Roughness);
	Mat->SetScalarParameterValue(TEXT("Metallic"), Metallic);
	Mesh->SetMaterial(0, Mat);
	Mesh->RegisterComponent();
}

void ARipLabActor::Label(const FString& Text, const FVector& Location, float Size, const FLinearColor& Color)
{
	UTextRenderComponent* T = NewObject<UTextRenderComponent>(this);
	T->SetupAttachment(RootComponent);
	T->SetText(FText::FromString(Text));
	T->SetRelativeLocation(Location); // text faces +X, i.e. out of the -X wall
	T->SetWorldSize(Size);
	T->SetTextRenderColor(Color.ToFColor(true));
	T->SetHorizontalAlignment(EHTA_Center);
	T->SetVerticalAlignment(EVRTA_TextCenter);
	T->RegisterComponent();
}

void ARipLabActor::BeginPlay()
{
	Super::BeginPlay();
	const FLinearColor Wood = SRGB(104, 72, 50), Steel = SRGB(60, 63, 68), Wall = SRGB(222, 226, 230);
	const FLinearColor Floor = SRGB(92, 96, 102), Board = SRGB(246, 247, 248), Frame = SRGB(170, 174, 180);
	const FLinearColor Ink = SRGB(30, 50, 95), Accent = SRGB(220, 110, 30);

	// desk: 750 x 1200 top, steel legs
	const FVector Top(-60.0, -20.0, -DeskThick / 2);
	Box(Top, FVector(750, 1200, DeskThick), Wood, 0.45f);
	for (const double SX : {-1.0, 1.0})
	{
		for (const double SY : {-1.0, 1.0})
		{
			const double LegLength = -DeskThick - FloorZ;
			Box(FVector(Top.X + SX * 330.0, Top.Y + SY * 555.0, FloorZ + LegLength / 2), FVector(40, 40, LegLength), Steel, 0.35f, 0.8f);
		}
	}

	// room corner
	Box(FVector(0, 0, FloorZ - 10), FVector(6000, 6000, 20), Floor, 0.8f);
	Box(FVector(WallX - 10, 0, (FloorZ + WallTop) / 2), FVector(20, 6000, WallTop - FloorZ), Wall, 0.9f);
	Box(FVector(0, WallY - 10, (FloorZ + WallTop) / 2), FVector(6000, 20, WallTop - FloorZ), Wall, 0.9f);

	// whiteboard on the -X wall, behind the rig
	const FVector BoardCenter(WallX + 8, 40, 300);
	Box(BoardCenter, FVector(10, 900, 440), Frame, 0.3f, 0.9f);
	Box(BoardCenter + FVector(6, 0, 0), FVector(6, 860, 400), Board, 0.15f);
	Label(TEXT("CONTROL II"), BoardCenter + FVector(12, 0, 110), 90.f, Ink);
	Label(TEXT("Furuta Pendulum"), BoardCenter + FVector(12, 0, 15), 52.f, Ink);
	Label(TEXT("digital twin"), BoardCenter + FVector(12, 0, -60), 40.f, Accent);
	Label(TEXT("TEAM 8   |   NEDAL   |   HAJAR   |   AHMAD"), BoardCenter + FVector(12, 0, -140), 34.f, Ink);
}
