// JoystickPlugin is licensed under the MIT License.
// Copyright Jayden Maalouf. All Rights Reserved.

#pragma once

#include "ForceFeedbackPeriodicEffectType.Generated.h"

UENUM(BlueprintType)
enum class EForceFeedbackPeriodicEffectType : uint8
{
	Sine,
	Triangle,
	SawtoothUp,
	SawtoothDown,
	LeftRight
};
