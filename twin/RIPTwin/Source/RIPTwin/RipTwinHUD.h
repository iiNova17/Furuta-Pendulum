#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "RipTwinHUD.generated.h"

// Text overlay: UDP port, latest angles and packet rate of each source.
UCLASS()
class RIPTWIN_API ARipTwinHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;
};
