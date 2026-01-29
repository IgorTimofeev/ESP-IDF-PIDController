#pragma once

#include <cstdint>

namespace YOBA {
	class PIDController {
		public:
			float tick(
				const float measuredValue,
				const float setpoint,

				const float p,
				const float i,
				const float d,

				const float outputMin,
				const float outputMax,

				const float deltaTime
			) {
				const auto error = setpoint - measuredValue;

				_integral += error * deltaTime;

				// Output
				auto output =
					// Proportional
					p * error
					// Integral
					+ i * _integral
					// Derivative
					+ d * ((measuredValue - _measuredValuePrev) / deltaTime);

				if (output > outputMax) {
					output = outputMax;
				}
				else if (output < outputMin) {
					output = outputMin;
				}

				_measuredValuePrev = measuredValue;

				return output;
			}

		private:
			float _integral = 0;
			float _measuredValuePrev = 0;
		};
}
