#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "RipTwinGameMode.generated.h"

// Spawns the twin at the origin and starts a free-fly camera (WASD + mouse) looking at it.
UCLASS()
class RIPTWIN_API ARipTwinGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ARipTwinGameMode();
	virtual void StartPlay() override;
	virtual void RestartPlayer(AController* NewPlayer) override;
};
