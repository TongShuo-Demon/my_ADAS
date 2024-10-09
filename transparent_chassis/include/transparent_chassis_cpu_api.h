
#pragma once

#include "transparent_chassis_cpu.h"

namespace ParkingPerception
{
    namespace TransparentChassisCpu
    {
        std::shared_ptr<TransparentChassis> CreateTransparentChassis(TransparentChassisData config_params);
    } // namespace TransparentChassisCpu
} // namespace ParkingPerception