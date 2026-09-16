#pragma once

#include <limits>
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

				const float deltaTime,

				const float outputMin = -std::numeric_limits<float>::infinity(),
				const float outputMax = std::numeric_limits<float>::infinity(),

				const float derivativeEMATau = 0.1f
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

				// Applying EMA filter
				const auto derivativeEMAAlpha = deltaTime / (derivativeEMATau + deltaTime);
				derivative = derivativeEMAAlpha * derivative + (1.f - derivativeEMAAlpha) * _derivativePreviousValue;
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

			void reset() {
				_integral = 0;
				_derivativePreviousMeasuredValue = 0;
				_derivativePreviousValue = 0;
			}

		private:
			float _integral = 0;

			float _derivativePreviousMeasuredValue = 0;
			float _derivativePreviousValue = 0;
		};
}
