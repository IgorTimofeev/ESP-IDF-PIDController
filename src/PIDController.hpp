#pragma once

#include <limits>
#include <cstdint>
#include <algorithm>

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

				const float derivativeEMAFilterAlpha = 0.1f
			) {
				const auto error = targetValue - measuredValue;

				// ----------------------------- Derivative -----------------------------

				float derivative;

				// On first tick() call derivative part can't be computed, because we need at least
				// one measured value to work with
				if (_isReset) {
					_isReset = false;

					derivative = 0;
				}
				else {
					derivative = (measuredValue - _prevMeasuredValue) / deltaTime;

					// Applying EMA filter
					derivative = derivativeEMAFilterAlpha * derivative + (1.f - derivativeEMAFilterAlpha) * _prevDerivative;
				}

				_prevDerivative = derivative;
				_prevMeasuredValue = measuredValue;

				// ----------------------------- Integral -----------------------------
				
				const float newIntegral = _prevIntegral + error * deltaTime;
				const float newOutput = p * error + i * newIntegral + d * derivative;

				// Anti-windup, allowing integral to update only if...
				if (
					// Output is NOT saturated
					!(newOutput > outputMax || newOutput < outputMin)
					// or error sign allows to leave the saturation state
					|| (newOutput > outputMax && error < 0.0f)
					|| (newOutput < outputMin && error > 0.0f)
				) {
					_prevIntegral = newIntegral;
				}

				return std::clamp(p * error + i * _prevIntegral + d * derivative, outputMin, outputMax);
			}

			void reset() {
				_isReset = true;

				_prevIntegral = 0;

				_prevMeasuredValue = 0;
				_prevDerivative = 0;
			}

		private:
			bool _isReset = true;

			float _prevIntegral = 0;

			float _prevMeasuredValue = 0;
			float _prevDerivative = 0;
		};
}
