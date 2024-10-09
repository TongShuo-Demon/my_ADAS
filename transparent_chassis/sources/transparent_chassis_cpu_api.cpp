#include "transparent_chassis_cpu_api.h"

namespace ParkingPerception
{
    namespace TransparentChassisCpu
    {
        std::shared_ptr<TransparentChassis> CreateTransparentChassis(TransparentChassisData config_params)
        {
            return std::make_shared<TransparentChassis>(config_params);
        }
    } // namespace TransparentChassisCpu
} // namespace ParkingPerception