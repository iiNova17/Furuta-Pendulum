#include "RipTwinHUD.h"

#include "Engine/Canvas.h"
#include "EngineUtils.h"
#include "HAL/PlatformTime.h"
#include "RipTwinActor.h"

void ARipTwinHUD::DrawHUD()
{
	Super::DrawHUD();
	TActorIterator<ARipTwinActor> It(GetWorld());
	if (!It || !Canvas)
	{
		return;
	}
	const ARipTwinActor* Twin = *It;
	const double Now = FPlatformTime::Seconds();

	// panel in the bottom-left corner, clear of the whiteboard
	const float PanelH = 96.f + (Twin->bModelDrivesSolid ? 22.f : 0.f) + (Twin->Status.IsEmpty() ? 0.f : 24.f);
	const float X = 24.f, Top = Canvas->ClipY - PanelH - 14.f;
	DrawRect(FLinearColor(0.f, 0.f, 0.f, 0.55f), X - 14.f, Top, 640.f, PanelH);

	auto Line = [&](const FRipSource& S, const TCHAR* Name, const FLinearColor& Color, float Y)
	{
		const FString Text = S.IsLive(Now)
			? FString::Printf(TEXT("%s   q_arm %7.1f deg    q_pend %7.1f deg    %3.0f Hz"), Name,
				FMath::RadiansToDegrees(S.QArm), FMath::RadiansToDegrees(S.QPend), S.RateHz)
			: FString::Printf(TEXT("%s   no data"), Name);
		DrawText(Text, Color, X, Y, nullptr, 1.3f);
	};
	DrawText(FString::Printf(TEXT("Furuta pendulum - digital twin   (UDP port %d)"), Twin->Port),
		FLinearColor::White, X, Top + 10.f, nullptr, 1.5f);
	Line(Twin->Real, TEXT("REAL "), FLinearColor(1.f, 0.65f, 0.2f), Top + 42.f);
	Line(Twin->Model, TEXT("MODEL"), FLinearColor(0.35f, 0.9f, 1.f), Top + 66.f);
	float Y = Top + 90.f;
	if (Twin->bModelDrivesSolid)
	{
		DrawText(TEXT("no rig data - the solid rig shows the model"), FLinearColor(0.8f, 0.8f, 0.8f), X, Y, nullptr, 1.1f);
		Y += 22.f;
	}
	if (!Twin->Status.IsEmpty())
	{
		DrawText(Twin->Status, FLinearColor::Red, X, Y, nullptr, 1.2f);
	}
}
