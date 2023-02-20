﻿// JoystickPlugin is licensed under the MIT License.
// Copyright Jayden Maalouf. All Rights Reserved.

#include "JoystickSubsystem.h"

#include "JoystickFunctionLibrary.h"
#include "JoystickInputDevice.h"
#include "JoystickInputSettings.h"

DEFINE_LOG_CATEGORY(LogJoystickPlugin);

UJoystickSubsystem::UJoystickSubsystem()
	: OwnsSDL(false)
	  , IsInitialised(false)
{
}

void UJoystickSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	UJoystickInputSettings* JoystickInputSettings = GetMutableDefault<UJoystickInputSettings>();
	if (IsValid(JoystickInputSettings))
	{
		JoystickInputSettings->ResetDevices();
	}

	Log(TEXT("DeviceSDL Starting"));

	if (SDL_WasInit(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) != 0)
	{
		Log(TEXT("SDL already loaded"));
		OwnsSDL = false;
	}
	else
	{
		Log(TEXT("DeviceSDL::InitSDL() SDL init 0"));
		SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER  | SDL_INIT_HAPTIC);
		OwnsSDL = true;
	}

	if (JoystickSubsystemReady.IsBound())
	{
		JoystickSubsystemReady.Broadcast();
	}

	IsInitialised = true;
}

void UJoystickSubsystem::Deinitialize()
{
	Super::Deinitialize();

	Log(TEXT("DeviceSDL Closing"));

	for (const auto& Device : Devices)
	{
		RemoveDevice(Device.Key);
	}

	SDL_DelEventWatch(HandleSDLEvent, this);

	if (OwnsSDL)
	{
		SDL_Quit();
	}

	IsInitialised = false;
}

void UJoystickSubsystem::InitialiseInputDevice(const TSharedPtr<FJoystickInputDevice> NewInputDevice)
{
	if (NewInputDevice == nullptr || !NewInputDevice.IsValid())
	{
		return;
	}

	InputDevicePtr = NewInputDevice;

	const int Result = SDL_WasInit(SDL_INIT_JOYSTICK);
	if (Result == 0)
	{
		return;
	}

	const int JoystickCount = GetJoystickCount();
	for (int i = 0; i < JoystickCount; i++)
	{
		AddDevice(i);
	}

	SDL_AddEventWatch(HandleSDLEvent, this);
}

int UJoystickSubsystem::GetJoystickCount() const
{
	return SDL_NumJoysticks();
}

int UJoystickSubsystem::GetRegisteredDeviceCount() const
{
	const FJoystickInputDevice* InputDevice = GetInputDevice();
	if (InputDevice == nullptr)
	{
		return -1;
	}

	return InputDevice->GetDeviceCount();
}

bool UJoystickSubsystem::GetJoystickData(const int DeviceId, FJoystickDeviceData& JoystickDeviceData) const
{
	FJoystickInputDevice* InputDevice = GetInputDevice();
	if (InputDevice == nullptr)
	{
		return false;
	}


	if (const FJoystickDeviceData* DeviceData = InputDevice->GetDeviceData(DeviceId))
	{
		JoystickDeviceData = *DeviceData;
		return true;
	}
	
	return false;
}

bool UJoystickSubsystem::GetJoystickInfo(const int DeviceId, FJoystickInfo& JoystickInfo) const
{
	FJoystickInputDevice* InputDevice = GetInputDevice();
	if (InputDevice == nullptr)
	{
		return false;
	}

	if (const FJoystickInfo* Info = InputDevice->GetDeviceInfo(DeviceId))
	{
		JoystickInfo = *Info;
		return true;
	}

	return false;
}

void UJoystickSubsystem::MapJoystickDeviceToPlayer(const int DeviceId, const int PlayerId)
{
	FJoystickInputDevice* InputDevice = GetInputDevice();
	if (InputDevice == nullptr)
	{
		return;
	}

	InputDevice->SetPlayerOwnership(DeviceId, PlayerId);
}

void UJoystickSubsystem::SetIgnoreGameControllers(const bool IgnoreControllers)
{
	UJoystickInputSettings* JoystickInputSettings = GetMutableDefault<UJoystickInputSettings>();
	if (!IsValid(JoystickInputSettings))
	{
		return;
	}
	
	const bool ChangedValue = JoystickInputSettings->SetIgnoreGameControllers(IgnoreControllers);
	if (ChangedValue && IgnoreControllers)
	{
		for (const auto& Device : Devices)
		{
			if (DeviceMapping.Contains(Device.Value.InstanceId) && SDL_IsGameController(Device.Value.DeviceIndex))
			{
				RemoveDevice(Device.Key);
			}
		}
	}
	else if (ChangedValue && !IgnoreControllers)
	{
		const int JoystickCount = GetJoystickCount();
		for (int i = 0; i < JoystickCount; i++)
		{
			if (SDL_IsGameController(i))
			{
				AddDevice(i);
			}
		}
	}
}

void UJoystickSubsystem::GetDeviceIds(TArray<int>& DeviceIds) const
{
	const FJoystickInputDevice* InputDevice = GetInputDevice();
	if (InputDevice == nullptr)
	{
		return;
	}

	InputDevice->GetDeviceIds(DeviceIds);
}

void UJoystickSubsystem::AddHapticDevice(FDeviceInfoSDL& Device) const
{
	Device.Haptic = SDL_HapticOpenFromJoystick(Device.Joystick);
	if (Device.Haptic != nullptr)
	{
		Log(TEXT("--- Haptic device detected"));

		Log(TEXT("Number of Haptic Axis: %i"), SDL_HapticNumAxes(Device.Haptic));
		Log(TEXT("Rumble Support: %i"), SDL_HapticRumbleSupported(Device.Haptic));

		
		Log(TEXT("SDL_HAPTIC_CONSTANT support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_CONSTANT));
		Log(TEXT("SDL_HAPTIC_SINE support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_SINE));
		Log(TEXT("SDL_HAPTIC_TRIANGLE support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_TRIANGLE));
		Log(TEXT("SDL_HAPTIC_SAWTOOTHUP support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_SAWTOOTHUP));
		Log(TEXT("SDL_HAPTIC_SAWTOOTHDOWN support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_SAWTOOTHDOWN));
		Log(TEXT("SDL_HAPTIC_RAMP support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_RAMP));
		Log(TEXT("SDL_HAPTIC_SPRING support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_SPRING));
		Log(TEXT("SDL_HAPTIC_DAMPER support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_DAMPER));
		Log(TEXT("SDL_HAPTIC_INERTIA support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_INERTIA));
		Log(TEXT("SDL_HAPTIC_FRICTION support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_FRICTION));
		Log(TEXT("SDL_HAPTIC_CUSTOM support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_CUSTOM));
		Log(TEXT("SDL_HAPTIC_GAIN support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_GAIN));
		Log(TEXT("SDL_HAPTIC_AUTOCENTER support: %i"), (SDL_HapticQuery(Device.Haptic) & SDL_HAPTIC_AUTOCENTER));

		if (SDL_HapticRumbleInit(Device.Haptic) != -1)
		{
			Log(TEXT("--- init Rumble device SUCCESSFUL"));
		}
		else
		{
			Log(TEXT("ERROR HapticRumbleInit FAILED"));
		}
	}
}

bool UJoystickSubsystem::AddDevice(const int DeviceIndex)
{
	const UJoystickInputSettings* JoystickInputSettings = GetMutableDefault<UJoystickInputSettings>();
	if (!IsValid(JoystickInputSettings))
	{
		return false;
	}

	auto a = SDL_IsGameController(DeviceIndex);
	if (a == SDL_TRUE && JoystickInputSettings->GetIgnoreGameControllers())
	{
		// Let UE handle it
		return false;
	}

	FDeviceInfoSDL Device;
	Device.DeviceIndex = DeviceIndex;

	Device.Joystick = SDL_JoystickOpen(DeviceIndex);
	if (Device.Joystick == nullptr)
	{
		return false;
	}

	Device.InstanceId = SDL_JoystickInstanceID(Device.Joystick);
	GetDeviceIndexGuid(DeviceIndex, Device.ProductId);

	// DEBUG
	Device.DeviceName = FString(ANSI_TO_TCHAR(SDL_JoystickName(Device.Joystick)));
	Log(TEXT("--- Instance ID: %d"), Device.InstanceId);
	Log(TEXT("--- Device Index: %d"), Device.DeviceIndex);
	Log(TEXT("--- %s"), *Device.DeviceName);
	Log(TEXT("--- Number of Axis %i"), SDL_JoystickNumAxes(Device.Joystick));
	Log(TEXT("--- Number of Balls %i"), SDL_JoystickNumBalls(Device.Joystick));
	Log(TEXT("--- Number of Buttons %i"), SDL_JoystickNumButtons(Device.Joystick));
	Log(TEXT("--- Number of Hats %i"), SDL_JoystickNumHats(Device.Joystick));

	if (SDL_JoystickIsHaptic(Device.Joystick))
	{
		AddHapticDevice(Device);
	}

	for (auto& ExistingDevice : Devices)
	{
		if (ExistingDevice.Value.Joystick == nullptr && ExistingDevice.Value.DeviceName == Device.DeviceName)
		{
			Device.DeviceId = ExistingDevice.Key;
			Devices[Device.DeviceId] = Device;

			DeviceMapping.Add(Device.InstanceId, Device.DeviceId);
			JoystickPluggedIn(Device);
			return true;
		}
	}

	Device.DeviceId = Devices.Num();
	Devices.Add(Device.DeviceId, Device);

	DeviceMapping.Add(Device.InstanceId, Device.DeviceId);
	JoystickPluggedIn(Device);
	return true;
}

bool UJoystickSubsystem::RemoveDevice(const int DeviceId)
{
	JoystickUnplugged(DeviceId);

	FDeviceInfoSDL* DeviceInfo = GetDeviceInfo(DeviceId);
	if (DeviceInfo == nullptr)
	{
		return nullptr;
	}

	if (DeviceInfo->Haptic != nullptr)
	{
		Log(TEXT("Closing Haptic Device for %d"), DeviceId);
		SDL_HapticClose(DeviceInfo->Haptic);
		DeviceInfo->Haptic = nullptr;
	}
	if (DeviceInfo->Joystick != nullptr)
	{
		Log(TEXT("Closing Joystick Device for %d"), DeviceId);
		SDL_JoystickClose(DeviceInfo->Joystick);
		DeviceInfo->Joystick = nullptr;
	}

	Log(TEXT("Device Removed %d"), DeviceId);
	DeviceMapping.Remove(DeviceInfo->InstanceId);
	return true;
}

void UJoystickSubsystem::Update() const
{
	if (OwnsSDL)
	{
		SDL_Event Event;
		while (SDL_PollEvent(&Event))
		{
			// The event watcher handles it
		}
	}
}

int UJoystickSubsystem::HandleSDLEvent(void* UserData, SDL_Event* Event)
{
	UJoystickSubsystem& JoystickSubsystem = *static_cast<UJoystickSubsystem*>(UserData);
	FJoystickInputDevice* InputDevice = JoystickSubsystem.GetInputDevice();
	if (InputDevice == nullptr)
	{
		return -1;
	}

	switch (Event->type)
	{
		case SDL_JOYDEVICEADDED:
			JoystickSubsystem.AddDevice(Event->cdevice.which);
			break;
		/*case SDL_CONTROLLERDEVICEADDED:
		{
			if (Self.bIgnoreGameControllers)
			{
				// Since JOYSTICK is inited before GAMECONTROLLER (by GAMECONTROLLER), 
				// a controller can be added as a joystick before we can check that it is a controller.
				// Remove it again and let UE handle it.
				FDeviceIndex DeviceIndex = FDeviceIndex(Event->cdevice.which);
				for (auto &Device : Self.Devices)
				{
					if (Device.Value.DeviceIndex == DeviceIndex && Self.DeviceMapping.Contains(Device.Value.InstanceId))
					{
						Self.DeviceMapping.Remove(Device.Value.InstanceId);
						Self.EventInterface->JoystickUnplugged(Device.Value.DeviceId);
					}
				}
			}
			break;
		}*/
		case SDL_JOYDEVICEREMOVED:
			{
				const int InstanceId = Event->cdevice.which;
				if (JoystickSubsystem.DeviceMapping.Contains(InstanceId))
				{
					const int DeviceId = JoystickSubsystem.DeviceMapping[InstanceId];
					JoystickSubsystem.RemoveDevice(DeviceId);
				}
				break;
			}
		case SDL_JOYBUTTONDOWN:
		case SDL_JOYBUTTONUP:
			if (JoystickSubsystem.DeviceMapping.Contains(Event->jbutton.which))
			{
				const int DeviceId = JoystickSubsystem.DeviceMapping[Event->jbutton.which];
				InputDevice->JoystickButton(DeviceId, Event->jbutton.button, Event->jbutton.state == SDL_PRESSED);

				Log(TEXT("Event JoystickButton Device=%d Button=%d State=%d"), DeviceId, Event->jbutton.button, Event->jbutton.state);
			}
			break;
		case SDL_JOYAXISMOTION:
			if (JoystickSubsystem.DeviceMapping.Contains(Event->jaxis.which))
			{
				const int DeviceId = JoystickSubsystem.DeviceMapping[Event->jaxis.which];
				InputDevice->JoystickAxis(DeviceId, Event->jaxis.axis, Event->jaxis.value / (Event->jaxis.value < 0 ? 32768.0f : 32767.0f));
			}
			break;
		case SDL_JOYHATMOTION:
			if (JoystickSubsystem.DeviceMapping.Contains(Event->jhat.which))
			{
				const int DeviceId = JoystickSubsystem.DeviceMapping[Event->jhat.which];
				InputDevice->JoystickHat(DeviceId, Event->jhat.hat, UJoystickFunctionLibrary::HatValueToDirection(Event->jhat.value));
			}
			break;
		case SDL_JOYBALLMOTION:
			if (JoystickSubsystem.DeviceMapping.Contains(Event->jball.which))
			{
				const int DeviceId = JoystickSubsystem.DeviceMapping[Event->jball.which];
				InputDevice->JoystickBall(DeviceId, Event->jball.ball, FVector2D(Event->jball.xrel, Event->jball.yrel));
			}
			break;
		default:
			break;
	}

	return 0;
}

FJoystickDeviceData UJoystickSubsystem::CreateInitialDeviceState(const int DeviceId)
{
	const FDeviceInfoSDL* DeviceInfo = GetDeviceInfo(DeviceId);
	if (DeviceInfo == nullptr || DeviceInfo->Joystick == nullptr)
	{
		return FJoystickDeviceData();
	}

	FJoystickDeviceData DeviceState = FJoystickDeviceData();
	const int AxesCount = SDL_JoystickNumAxes(DeviceInfo->Joystick);
	const int ButtonCount = SDL_JoystickNumButtons(DeviceInfo->Joystick);
	const int HatsCount = SDL_JoystickNumHats(DeviceInfo->Joystick);
	const int BallsCount = SDL_JoystickNumBalls(DeviceInfo->Joystick);

	DeviceState.Axes.SetNumZeroed(AxesCount);
	DeviceState.Buttons.SetNumZeroed(ButtonCount);
	DeviceState.Hats.SetNumZeroed(HatsCount);
	DeviceState.Balls.SetNumZeroed(BallsCount);
	return DeviceState;
}

FJoystickInputDevice* UJoystickSubsystem::GetInputDevice() const
{
	if (!InputDevicePtr.IsValid())
	{
		return nullptr;
	}

	return InputDevicePtr.Get();
}

bool UJoystickSubsystem::IsReady() const
{
	return IsInitialised;
}

FString UJoystickSubsystem::GetDeviceIndexGuidString(const int DeviceIndex) const
{
	char Buffer[32];
	constexpr int8 SizeBuffer = sizeof(Buffer);

	const SDL_JoystickGUID GUID = SDL_JoystickGetDeviceGUID(DeviceIndex);
	SDL_JoystickGetGUIDString(GUID, Buffer, SizeBuffer);
	return ANSI_TO_TCHAR(Buffer);
}

void UJoystickSubsystem::GetDeviceIndexGuid(const int DeviceIndex, FGuid& Guid) const
{
	const SDL_JoystickGUID SDLGuid = SDL_JoystickGetDeviceGUID(DeviceIndex);
	memcpy(&Guid, &SDLGuid, sizeof(FGuid));
}

FDeviceInfoSDL* UJoystickSubsystem::GetDeviceInfo(const int DeviceId)
{
	const FJoystickInputDevice* InputDevice = GetInputDevice();
	if (InputDevice == nullptr)
	{
		return nullptr;
	}

	if (Devices.Num() == 0 || !Devices.Contains(DeviceId))
	{
		return nullptr;
	}

	return Devices.Find(DeviceId);
}

void UJoystickSubsystem::JoystickPluggedIn(const FDeviceInfoSDL& Device) const
{
	FJoystickInputDevice* InputDevice = GetInputDevice();
	if (InputDevice == nullptr)
	{
		return;
	}

	InputDevice->JoystickPluggedIn(Device);
	if (JoystickPluggedInDelegate.IsBound())
	{
		JoystickPluggedInDelegate.Broadcast(Device.DeviceId);
	}
}

void UJoystickSubsystem::JoystickUnplugged(const int DeviceId) const
{
	FJoystickInputDevice* InputDevice = GetInputDevice();
	if (InputDevice == nullptr)
	{
		return;
	}

	InputDevice->JoystickUnplugged(DeviceId);
	if (JoystickUnpluggedDelegate.IsBound())
	{
		JoystickUnpluggedDelegate.Broadcast(DeviceId);
	}
}

template <typename FmtType, typename... Types>
void UJoystickSubsystem::Log(const FmtType& Fmt, Types... Args)
{
	const UJoystickInputSettings* JoystickInputSettings = GetMutableDefault<UJoystickInputSettings>();
	if (!IsValid(JoystickInputSettings))
	{
		return;
	}

	if (JoystickInputSettings->DebugLogs)
	{
		UE_LOG(LogJoystickPlugin, Log, TEXT("%s"), *FString::Printf(Fmt, Args...));		
	}
}