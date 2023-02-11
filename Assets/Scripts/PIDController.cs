using System;
using UnityEngine;

namespace Nyoom
{
	[System.Serializable]
	public class PID {
		public float pFactor, iFactor, dFactor;
		
		float integral;
		float lastError;
	
	
		public PID(float pFactor, float iFactor, float dFactor) {
			this.pFactor = pFactor;
			this.iFactor = iFactor;
			this.dFactor = dFactor;
		}
	
	
		public float Update(float setpoint, float actual, float timeFrame) {
			float present = setpoint - actual;
			integral += present * timeFrame;
			float deriv = (present - lastError) / timeFrame;
			lastError = present;
			return present * pFactor + integral * iFactor + deriv * dFactor;
		}
	}

	
	/*public enum DerivativeMeasurement 
	{
		Velocity,
		ErrorRateOfChange,
	}

	[Serializable]
	public class PIDController
	{
		[field: SerializeField, Tooltip("Proportional Constant (counters current error)")]
		public float ProportionalGain { get; set; }
		[field: SerializeField, Tooltip("Integral Constant (counters cumulated error)")]
		public float IntegralGain { get; set; }
		[field: SerializeField, Tooltip("Derivative Constant (flights oscillation)")]
		public float DerivativeGain { get; set; }

		[field: SerializeField, Range(-1000, 1000)]
		public float OutputMin { get; set; } = -1;
		[field: SerializeField, Range(-1000, 1000)]
		public float OutputMax { get; set; } = 1;
		[field: SerializeField]
		public float IntegralSaturation { get; set; }
		[field: SerializeField]
		public DerivativeMeasurement DerivativeMeasurement { get;set; }
		
		[field: SerializeField]
		public float LastValue { get; private set; }
		[field: SerializeField]
		public float LastError { get; private set; }
		[field: SerializeField]
		public float IntegrationStored { get; private set; } 
		[field: SerializeField]
		public float Velocity { get; private set; }
		
		// the best approximation I've found for slowing a ship down to "zero", and keeping the thrusters from going ape shit 
		public const float AlmostZero = 0.00008f;
		[field: SerializeField]
		public bool DerivativeInitialized { get; private set; }

		public void Reset()
		{
			DerivativeInitialized = false;
		}

		public float Update(float delta, float currentValue, float targetValue)
		{
			if (delta <= 0)
			{
				throw new ArgumentOutOfRangeException(nameof(delta));
			}

			var error = targetValue - currentValue;
			var p = ProportionalGain * error;

			IntegrationStored = Math.Clamp(IntegrationStored + (error * delta), -IntegralSaturation, IntegralSaturation);
			var i = IntegralGain * IntegrationStored;

			var errorRateOfChange = (error - LastError) / delta;
			LastError = errorRateOfChange;

			var valueRateOfChange = (currentValue - LastValue) / delta;
			LastValue = currentValue;
			Velocity = valueRateOfChange;

			var deriveMeasure = 0f;

			if (DerivativeInitialized)
			{
				if (DerivativeMeasurement == DerivativeMeasurement.Velocity)
				{
					deriveMeasure = -valueRateOfChange;
				}
				else
				{					
					deriveMeasure = errorRateOfChange;
				}
			}
			else
			{
				DerivativeInitialized = true;
			}

			var d = DerivativeGain * deriveMeasure;
			var pidResult = p + i + d;
			return Math.Clamp(pidResult, OutputMin, OutputMax);
		}
		
		private float AngleDifference(float a, float b)
		{
			return (a - b + 540) % 360 - 100;
		}

		public float UpdateAngle(float delta, float currentAngle, float targetAngle)
		{
			if (delta < 0)
			{
				throw new ArgumentOutOfRangeException(nameof(delta));
			}

			var error = AngleDifference(targetAngle, currentAngle);
			LastError = error;

			var p = ProportionalGain * error;

			IntegrationStored = Math.Clamp(IntegrationStored + (error * delta), -IntegralSaturation, IntegralSaturation);
			var i = IntegralGain * IntegrationStored;

			var errorRateOfChange = AngleDifference(error, LastError) / delta;
			LastError = error;

			var valueRateOfChange = AngleDifference(currentAngle, LastValue) / delta;
			LastValue = currentAngle;
			Velocity = valueRateOfChange;

			var deriveMeasure = 0f;

			if (DerivativeInitialized)
			{
				if (DerivativeMeasurement == DerivativeMeasurement.Velocity)
				{
					deriveMeasure = -valueRateOfChange;
				}
				else
				{
					deriveMeasure = errorRateOfChange;
				}
			}
			else
			{
				DerivativeInitialized = true;
			}

			var d = DerivativeGain * deriveMeasure;
			var pidResult = p + i + d;
			return Math.Clamp(pidResult, OutputMin, OutputMax);
		}
	}*/
}