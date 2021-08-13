#pragma once

#include "ForceFeedback/Effects/ForceFeedbackEffectBase.h"
#include "ForceFeedback/Data/ForceFeedbackEffectConditionData.h"
#include "ForceFeedbackEffectCondition.generated.h"

UCLASS(Blueprintable)
class UForceFeedbackEffectCondition : public UForceFeedbackEffectBase
{
    GENERATED_BODY()
public:

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
		FForceFeedbackEffectConditionData EffectData;

protected:

    SDL_HapticEffect ToSDLEffect() override;
};