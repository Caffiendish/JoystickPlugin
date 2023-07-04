// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "MotionSensorFilter.h"

#include "MotionData.generated.h"
/**
 * 
 */
USTRUCT(BlueprintType)
struct JOYSTICKPLUGIN_API FMotionData
{
	GENERATED_BODY()

	FMotionData()
		: Deadzone(1.f)
		, Sensitivity(100)
		, Accelerometer(0.f)
		, Gyroscope(0.f)
		, bHasAccel(false)
		, bHasGyro(false)
		, Accel(0)
		, Gyro(0)
		, SensorFlags(0)
		, AvailableSensors(0)
	{
	}

	FMotionData(bool HasAccel, bool HasGyro, int InSens)
		: Sensitivity(InSens)
		, Accelerometer(0)
		, Gyroscope(0)
		, bHasAccel(HasAccel)
		, bHasGyro(HasGyro)
		, SensorFlags(0)
	{
		AvailableSensors = 0;
		if (HasAccel)
		{
			AvailableSensors |= 1;
		}
		if (HasGyro)
		{
			AvailableSensors |= 2;
		}
	}

	FVector DegreeToRad(FVector degree)
	{
		return degree * (PI / 180);
	}

	void Update(uint32 timeStamp, uint8 type, FVector vector)
	{
		if ((SensorFlags & type) == type)
			return;

		SensorFlags |= type;

		switch (type)
		{
		case 1:
		{
			Accel = vector;
		}break;

		case 2:
		{
			Gyro = vector;
		}break;

		default:
			break;
		};

		if (SensorFlags == AvailableSensors)
		{
			Update(timeStamp, Accel, Gyro);
		}
	}

	void Update(uint32 timeStamp, FVector accel, FVector gyro)
	{
		if (timeStamp != 0)
		{
			Accelerometer = -accel;

			if (gyro.Length() < Deadzone)
			{
				gyro = FVector::Zero();
			}

			gyro *= (Sensitivity / 100.0f);

			Gyroscope = gyro;

			float deltaTime = FMath::Abs((timeStamp - TimeStamp) / 1000.0f);

			FVector deltaGyro = gyro * deltaTime;

			Rotation += deltaGyro;

			Motion.Update(accel,DegreeToRad(gyro), deltaTime);
			SensorFlags = 0;
		}

		TimeStamp = timeStamp;
	}

	UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
	float Deadzone;

	UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
	int Sensitivity;

	UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
	FVector Accelerometer;

	UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
	FVector Gyroscope;

	UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
	FVector Rotation;

	/* Timestamp */
	/*UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")*/
	uint32 TimeStamp;

	UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
	bool bHasAccel;

	UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
	bool bHasGyro;

	UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
	FMotionSensorFilter Motion;

protected:

	UPROPERTY()
	FVector Accel;
	UPROPERTY()
	FVector Gyro;
	UPROPERTY()
	uint8 SensorFlags;
	UPROPERTY()
	uint8 AvailableSensors;
};
