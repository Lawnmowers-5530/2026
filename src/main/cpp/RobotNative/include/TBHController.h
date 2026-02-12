//
// Created by siryellsalot on 2/10/26.
//

#ifndef INC_2026_TBHCONTROLLER_H
#define INC_2026_TBHCONTROLLER_H

#include <units/dimensionless.h>
#include <units/angular_velocity.h>

using units::angular_velocity::revolutions_per_minute_t;
using namespace units;

using CoulombsPerRad = compound_unit<
    ampere,
    second,
    inverse<radian>
    >;
using coulombs_per_rad_t = unit_t<CoulombsPerRad>;

struct TBHController {
    radians_per_second_t target, lastError;
    coulombs_per_rad_t kI;
    ampere_t output, tbh;


    TBHController(const coulombs_per_rad_t kI):
        target(0),
        lastError(0),
        kI(kI),
        output(0),
        tbh(0)
    {}

    ampere_t Calculate(radians_per_second_t currentVelocity) {
        auto error = target - currentVelocity;
        output += kI * error;

        if (std::signbit(error.value()) != std::signbit(lastError.value())) {
            output = 0.5 * (output + tbh);
            tbh = output;
        }

        lastError = error;
        return output;
    }

    void setTarget(radians_per_second_t target) {
        this->target = target;
    }
};

#endif //INC_2026_TBHCONTROLLER_H