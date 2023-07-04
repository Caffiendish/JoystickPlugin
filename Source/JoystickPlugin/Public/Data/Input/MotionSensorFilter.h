// Fill out your copyright notice in the Description page of Project Settings.

#pragma once


#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include "MotionSensorFilter.generated.h"
/**
 * 
 */
USTRUCT(BlueprintType)
struct JOYSTICKPLUGIN_API FMotionSensorFilter
{
	GENERATED_BODY()

    /// <summary>
        /// Sample rate coefficient.
        /// </summary>
    UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
    float SampleRateCoefficient = 0.45f;

    /// <summary>
    /// Gets or sets the sample period.
    /// </summary>
    UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
    float SamplePeriod;

        /// <summary>
        /// Gets or sets the algorithm proportional gain.
        /// </summary>
    UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
    float Kp;

        /// <summary>
        /// Gets or sets the algorithm integral gain.
        /// </summary>
    UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
    float Ki;

        /// <summary>
        /// Gets the Quaternion output.
        /// </summary>
    UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
    FQuat Quaternion;

        /// <summary>
        /// Integral error.
        /// </summary>
    UPROPERTY(VisibleAnywhere, BlueprintReadonly, Category = "Joystick|Data")
    FVector _intergralError;


    FMotionSensorFilter() :
		FMotionSensorFilter(0.f)
        {

        }

    /// <summary>
    /// Initializes a new instance of the <see cref="MotionSensorFilter"/> class.
    /// </summary>
    /// <param name="samplePeriod">
    /// Sample period.
    /// </param>
    FMotionSensorFilter(float samplePeriod) : FMotionSensorFilter(samplePeriod, 1.f, 0.f)
    { }

    /// <summary>
    /// Initializes a new instance of the <see cref="MotionSensorFilter"/> class.
    /// </summary>
    /// <param name="samplePeriod">
    /// Sample period.
    /// </param>
    /// <param name="kp">
    /// Algorithm proportional gain.
    /// </param>
    FMotionSensorFilter(float samplePeriod, float kp) : FMotionSensorFilter(samplePeriod, kp, 0.f) { }

    /// <summary>
    /// Initializes a new instance of the <see cref="MotionSensorFilter"/> class.
    /// </summary>
    /// <param name="samplePeriod">
    /// Sample period.
    /// </param>
    /// <param name="kp">
    /// Algorithm proportional gain.
    /// </param>
    /// <param name="ki">
    /// Algorithm integral gain.
    /// </param>
    FMotionSensorFilter(float samplePeriod, float kp, float ki)
    {
        SamplePeriod = samplePeriod;
        Kp = kp;
        Ki = ki;

        Reset();

        _intergralError = FVector();
    }


    void Update(FVector accel, FVector gyro)
    {
        Update(accel, gyro, SamplePeriod);
    }

    /// <summary>
    /// Algorithm IMU update method. Requires only gyroscope and accelerometer data.
    /// </summary>
    /// <param name="accel">
    /// Accelerometer measurement in any calibrated units.
    /// </param>
    /// <param name="gyro">
    /// Gyroscope measurement in radians.
    /// </param>
    void Update(FVector accel, FVector gyro, float timeDelta)
    {
        SamplePeriod = timeDelta;
        // Normalise accelerometer measurement.
        float norm = 1.f / accel.Length();

        if (!FMath::IsFinite(norm))
        {
            return;
        }

        accel *= norm;

        float q2 = Quaternion.X;
        float q3 = Quaternion.Y;
        float q4 = Quaternion.Z;
        float q1 = Quaternion.W;

        // Estimated direction of gravity.
        FVector gravity;

        gravity.X = 2.f * (q2 * q4 - q1 * q3);
        gravity.Y = 2.f * (q1 * q2 + q3 * q4);
        gravity.Z = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4;


        // Error is cross product between estimated direction and measured direction of gravity.
        FVector error = FVector();

        error.X = accel.Y * gravity.Z - accel.Z * gravity.Y;
        error.Y = accel.Z * gravity.X - accel.X * gravity.Z;
        error.Z = accel.X * gravity.Y - accel.Y * gravity.X;

        if (Ki > 0.f)
        {
            _intergralError += error; // Accumulate integral error.
        }
        else
        {
            _intergralError = FVector::Zero(); // Prevent integral wind up.
        }

        // Apply feedback terms.
        gyro += (Kp * error) + (Ki * _intergralError);

        // Integrate rate of change of quaternion.
        FVector delta =  FVector(q2, q3, q4);

        q1 += (-q2 * gyro.X - q3 * gyro.Y - q4 * gyro.Z) * (SampleRateCoefficient * SamplePeriod);
        q2 += (q1 * gyro.X + delta.Y * gyro.Z - delta.Z * gyro.Y) * (SampleRateCoefficient * SamplePeriod);
        q3 += (q1 * gyro.Y - delta.X * gyro.Z + delta.Z * gyro.X) * (SampleRateCoefficient * SamplePeriod);
        q4 += (q1 * gyro.Z + delta.X * gyro.Y - delta.Y * gyro.X) * (SampleRateCoefficient * SamplePeriod);

        // Normalise quaternion.
        FQuat quaternion(q2, q3, q4, q1);

        norm = 1.f / quaternion.Size();

        if (!FMath::IsFinite(norm))
        {
            return;
        }

        Quaternion = quaternion * norm;
    }

    void Reset()
    {
        Quaternion = FQuat::Identity;
    }
};