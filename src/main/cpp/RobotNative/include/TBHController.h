//
// Created by siryellsalot on 2/10/26.
//

#ifndef INC_2026_TBHCONTROLLER_H
#define INC_2026_TBHCONTROLLER_H

#include <units/angular_velocity.h>
#include <units/dimensionless.h>

using units::angular_velocity::revolutions_per_minute_t;
using namespace units;

using WebersPerRad = compound_unit<volt, second, inverse<radian>>;
using webers_per_rad_t = unit_t<WebersPerRad>;

struct TBHController {
    radians_per_second_t target, lastError;
    webers_per_rad_t kI;
    volt_t output, tbh;

    TBHController(const webers_per_rad_t kI) :
        target(0),
        lastError(0),
        kI(kI),
        output(0),
        tbh(0) {}

    volt_t Calculate(radians_per_second_t currentVelocity) {
        auto error = target - currentVelocity;
        output += kI * error;

        if (std::signbit(error.value()) != std::signbit(lastError.value())) {
            output = 0.5 * (output + tbh);
            tbh = output;
        }

        lastError = error;
        return output;
    }

    void setTarget(radians_per_second_t target) { this->target = target; }
};

#endif // INC_2026_TBHCONTROLLER_H