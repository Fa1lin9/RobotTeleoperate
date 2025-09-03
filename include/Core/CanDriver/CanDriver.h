#ifndef CAN_DRIVER_H_
#define CAN_DRIVER_H_

#include <cstdint>
#include <optional>
#include <tuple>
#include <vector>

namespace CanDriver {

    bool InitCan();
    void CloseCan();

    void ConfigMaxVelocity(int can_device, uint8_t can_id, double max_velocity);
    void ConfigMaxAcceleration(int can_device, uint8_t can_id, double max_acceleration);

    void RecvMaxVelocity(int can_device, uint8_t can_id, double* velocity);
    void RecvMaxAcceleration(int can_device, uint8_t can_id, double* accel);

    void SendPosition(int can_device, uint8_t can_id, double position);
    std::optional<std::tuple<int32_t, double>> RecvPosition(int can_device, uint8_t can_id, double* position);
    std::optional<std::tuple<int32_t, double>> SendRecvPosition(int can_device, uint8_t can_id, double* position);
    std::vector<std::tuple<int32_t, double>> SendRecvMultiPosition(
        int can_device, uint8_t can_id, double* position, bool is_recv);


}

#endif  // CAN_DRIVER_H_
