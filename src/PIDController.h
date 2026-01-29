#pragma once

#include <cstdint>

namespace YOBA {
	class PIDController {
		public:
			float tick(
				const float measuredValue,
				const float targetValue,

				const float p,
				const float i,
				const float d,

				const float outputMin,
				const float outputMax,

				const float deltaTime,

				const float derivativeLPFTau = 0.1f
			) {
				const auto error = targetValue - measuredValue;

				// ----------------------------- Integral -----------------------------

				_integral += error * deltaTime;

				// Applying anti-windup
				if (_integral > outputMax) {
					_integral = outputMax;
				}
				else if (_integral < outputMin) {
					_integral = outputMin;
				}

				// ----------------------------- Derivative -----------------------------

				auto derivative = (measuredValue - _derivativePreviousMeasuredValue) / deltaTime;
				_derivativePreviousMeasuredValue = measuredValue;

				// Applying low-pass filter
				const auto derivativeLPFAlpha = deltaTime / (derivativeLPFTau + deltaTime);
				derivative = derivativeLPFAlpha * derivative + (1.f - derivativeLPFAlpha) * _derivativePreviousValue;
				_derivativePreviousValue = derivative;

				// ----------------------------- Output -----------------------------

				auto output =
					p * error
					+ i * _integral
					+ d * derivative;

				// Clamping output
				if (output > outputMax) {
					output = outputMax;
				}
				else if (output < outputMin) {
					output = outputMin;
				}

				return output;
			}

		private:
			float _integral = 0;

			float _derivativePreviousMeasuredValue = 0;
			float _derivativePreviousValue = 0;
		};
}
