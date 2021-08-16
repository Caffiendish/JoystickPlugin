#include "ForceFeedback/Effects/ForceFeedbackEffectCustom.h"

SDL_HapticEffect UForceFeedbackEffectCustom::ToSDLEffect()
{
	SDL_HapticEffect effect = Super::ToSDLEffect();

	effect.type = SDL_HAPTIC_CUSTOM;
	//effect.custom.type = SDL_HAPTIC_CUSTOM;

	effect.custom.direction = EffectData.DirectionData.ToSDLDirection();
	effect.custom.length = EffectData.DurationData.InfiniteDuration ? SDL_HAPTIC_INFINITY : FMath::Clamp(Uint32(EffectData.DurationData.Duration * 1000.0f), Uint32(0), Uint32(UINT32_MAX));
	effect.custom.delay = FMath::Clamp(Uint16(EffectData.ReplayData.Delay * 1000.0f), Uint16(0), Uint16(UINT16_MAX));
	effect.custom.interval = FMath::Clamp(Uint16(EffectData.ReplayData.RetriggerDelay * 1000.0f), Uint16(0), Uint16(UINT16_MAX));

	effect.custom.channels = FMath::Clamp(Uint8(EffectData.Channels), Uint8(0), Uint8(UINT8_MAX));
	effect.custom.period = FMath::Clamp(Uint16(EffectData.Period), Uint16(0), Uint16(UINT16_MAX));
	effect.custom.samples = FMath::Clamp(Uint16(EffectData.Samples), Uint16(0), Uint16(UINT16_MAX));

	Uint16* dataArray = new Uint16[EffectData.Data.Num()]();
	for (int32 i = 0; i < EffectData.Data.Num(); i++)
	{
		dataArray[i] = FMath::Clamp(Uint16(EffectData.Data[i]), Uint16(0), Uint16(UINT16_MAX));
	}
	effect.custom.data = dataArray;

	effect.custom.attack_length = FMath::Clamp(Uint16(EffectData.EnvelopeData.AttackDuration * 1000.0f), Uint16(0), Uint16(UINT16_MAX));
	effect.custom.attack_level = FMath::Clamp(Uint16(EffectData.EnvelopeData.AttackLevel * float(UINT16_MAX)), Uint16(0), Uint16(UINT16_MAX));
	effect.custom.fade_length = FMath::Clamp(Uint16(EffectData.EnvelopeData.FadeDuration * 1000.0f), Uint16(0), Uint16(UINT16_MAX));
	effect.custom.fade_level = FMath::Clamp(Uint16(EffectData.EnvelopeData.FadeLevel * float(UINT16_MAX)), Uint16(0), Uint16(UINT16_MAX));

	return effect;
}