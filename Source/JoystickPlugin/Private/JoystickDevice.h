#pragma once

#include "IInputDevice.h"
#include "GenericPlatform/GenericApplicationMessageHandler.h"
#include "Interfaces/JoystickEventInterface.h"

#include "Data/DeviceId.h"
#include "Data/JoystickInfo.h"
#include "Data/JoystickState.h"

struct FDeviceInfoSDL;
class FDeviceSDL;

class FJoystickDevice : public IInputDevice, public IJoystickEventInterface
{
public:
	
	FJoystickDevice(const TSharedRef<FGenericApplicationMessageHandler>& InMessageHandler);
	~FJoystickDevice();

	void Tick(float DeltaTime) override;
	void SendControllerEvents() override;
	void SetMessageHandler(const TSharedRef<FGenericApplicationMessageHandler>& InMessageHandler) override;
	bool Exec(UWorld* InWorld, const TCHAR* Cmd, FOutputDevice& Ar) override;
	void SetChannelValue(int32 ControllerId, FForceFeedbackChannelType ChannelType, float Value) override;
	void SetChannelValues(int32 ControllerId, const FForceFeedbackValues& Values) override;

	bool AddEventListener(UObject* Listener);
	void IgnoreGameControllers(bool bIgnore);

	virtual void JoystickPluggedIn(const FDeviceInfoSDL &Device) override;
	virtual void JoystickUnplugged(FDeviceId DeviceId) override;
	virtual void JoystickButton(FDeviceId DeviceId, int32 Button, bool Pressed) override;
	virtual void JoystickAxis(FDeviceId DeviceId, int32 Axis, float Value) override;
	virtual void JoystickHat(FDeviceId DeviceId, int32 Hat, EJoystickPOVDirection Value) override;
	virtual void JoystickBall(FDeviceId DeviceId, int32 Ball, FVector2D Delta) override;

	TMap<FDeviceId, FJoystickState> CurrentState;
	TMap<FDeviceId, FJoystickState> PreviousState;

	TMap<FDeviceId, FJoystickInfo> InputDevices;	
	TSharedPtr<FDeviceSDL> DeviceSDL;

private:
	void InitInputDevice(const FDeviceInfoSDL &Device);

	TArray<TWeakObjectPtr<UObject>> EventListeners;

	TMap<FDeviceId, TArray<FKey>> DeviceButtonKeys;
	TMap<FDeviceId, TArray<FKey>> DeviceAxisKeys;
	TMap<FDeviceId, TArray<FKey>> DeviceHatKeys[2];
	TMap<FDeviceId, TArray<FKey>> DeviceBallKeys[2];

	TSharedRef<FGenericApplicationMessageHandler> MessageHandler;
};
