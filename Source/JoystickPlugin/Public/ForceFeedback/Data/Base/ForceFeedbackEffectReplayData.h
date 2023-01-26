#pragma once

#include "ForceFeedbackEffectReplayData.Generated.h"

USTRUCT(BlueprintType)
struct JOYSTICKPLUGIN_API FForceFeedbackEffectReplayData
{
	GENERATED_BODY()

	FForceFeedbackEffectReplayData()
		: Delay(0.0f)
		  , RetriggerDelay(0.0f)
	{
	}

	UPROPERTY(BlueprintReadWrite, EditAnywhere, meta = (ShortToolTip = "Delay before starting", ToolTip = "Delay before starting the effect.", UIMin="0", ClampMin="0"), Category = "Force Feedback|Replay|Data")
	float Delay;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, meta = (ShortToolTip = "Retrigger Delay", ToolTip = "Delay before the effect can be retriggerd.", UIMin="0", ClampMin="0"), Category = "Force Feedback|Replay|Data")
	float RetriggerDelay;
};
