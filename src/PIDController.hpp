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

				const float derivativeEMAFilterTau = 0.1f
			) {
				const auto error = targetValue - measuredValue;

				// ----------------------------- Derivative -----------------------------

				float derivative;

				// On first tick() call derivative part can't be computed, because we need at least
				// one measured value to work with
				if (_inResetState) {
					_inResetState = false;

					derivative = 0;
				}
				else {
					derivative = (_derivativePrevMeasuredValue - measuredValue) / deltaTime;

					// Applying EMA filter
					const auto derivativeEMAAlpha = deltaTime / (derivativeEMAFilterTau + deltaTime);
					derivative = derivativeEMAAlpha * derivative + (1.f - derivativeEMAAlpha) * _derivativePrevValue;
				}

				_derivativePrevMeasuredValue = measuredValue;
				_derivativePrevValue = derivative;

				// ----------------------------- Integral -----------------------------

				const float integral = _integralPrevValue + error * deltaTime;

				// Anti-windup protection
				float output = p * error + d * derivative;
				const float outputWithIntegral = output + i * integral;

				// Output is undersaturated or oversaturated, using old integral value
				if ((outputWithIntegral < outputMin && error < 0.f) || (outputWithIntegral > outputMax && error > 0.f)) {
					output += i * _integralPrevValue;
				}
				// Output is in normal range, using new integral value
				else {
					_integralPrevValue = integral;
					output = outputWithIntegral;
				}

				// ----------------------------- Output -----------------------------

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
				_inResetState = true;

				_integralPrevValue = 0;

				_derivativePrevMeasuredValue = 0;
				_derivativePrevValue = 0;
			}

		private:
			bool _inResetState = true;

			float _integralPrevValue = 0;

			float _derivativePrevMeasuredValue = 0;
			float _derivativePrevValue = 0;
		};
}
