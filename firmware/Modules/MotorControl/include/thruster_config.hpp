#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace thruster_config {

// Body-fixed right-handed coordinate system:
//   +x = forward, +y = right, +z = down.
struct Vector3 {
    float x;
    float y;
    float z;
};

enum class BodyDirection : uint8_t {
    Forward,
    Right,
    Down,
};

struct CoordinateSystem {
    BodyDirection x_positive;
    BodyDirection y_positive;
    BodyDirection z_positive;
};

inline constexpr CoordinateSystem kCoordinateSystem{
    BodyDirection::Forward,
    BodyDirection::Right,
    BodyDirection::Down,
};

enum class Position : uint8_t {
    RearRight,
    FrontRight,
    FrontLeft,
    RearLeft,
};

enum class Orientation : uint8_t {
    HorizontalDiagonal,
    Vertical,
};

enum class PropellerHand : uint8_t {
    Normal,
    Reverse,
};

enum class RotationDirection : uint8_t {
    Counterclockwise,
    Clockwise,
};

enum class ControlAxis : uint8_t {
    SurgeX,
    SwayY,
    HeaveZ,
    RollX,
    PitchY,
    YawZ,
};

struct ThrusterConfig {
    uint8_t channel;
    const char *name;
    Position position;
    Orientation orientation;
    PropellerHand propeller_hand;
    RotationDirection rotation_above_neutral;

    // Body-frame force produced when pwm_us is above neutral. This is the
    // authoritative PWM polarity; propeller hand and rotation are metadata.
    Vector3 positive_force;

    // Optional per-channel calibration, bound to the physical channel.
    int16_t neutral_trim_us;
    uint16_t deadzone_compensation_us;
};

inline constexpr std::size_t kThrusterCount = 8U;

// Single source of truth for channel number, installation and PWM polarity.
inline constexpr std::array<ThrusterConfig, kThrusterCount> kThrusters{{
    {0U, "horizontal_rear_right",
     Position::RearRight, Orientation::HorizontalDiagonal,
     PropellerHand::Normal, RotationDirection::Counterclockwise,
     {-0.707F, -0.707F, 0.0F}, 0, 50U},
    {1U, "vertical_rear_left",
     Position::RearLeft, Orientation::Vertical,
     PropellerHand::Normal, RotationDirection::Counterclockwise,
     {0.0F, 0.0F, 1.0F}, 0, 50U},
    {2U, "vertical_front_right",
     Position::FrontRight, Orientation::Vertical,
     PropellerHand::Normal, RotationDirection::Counterclockwise,
     {0.0F, 0.0F, 1.0F}, 0, 50U},
    {3U, "horizontal_front_left",
     Position::FrontLeft, Orientation::HorizontalDiagonal,
     PropellerHand::Normal, RotationDirection::Counterclockwise,
     {-0.707F, 0.707F, 0.0F}, 0, 50U},
    {4U, "horizontal_front_right",
     Position::FrontRight, Orientation::HorizontalDiagonal,
     PropellerHand::Reverse, RotationDirection::Clockwise,
     {0.707F, -0.707F, 0.0F}, 0, 50U},
    {5U, "vertical_front_left",
     Position::FrontLeft, Orientation::Vertical,
     PropellerHand::Reverse, RotationDirection::Clockwise,
     {0.0F, 0.0F, -1.0F}, 0, 50U},
    {6U, "vertical_rear_right",
     Position::RearRight, Orientation::Vertical,
     PropellerHand::Reverse, RotationDirection::Clockwise,
     {0.0F, 0.0F, -1.0F}, 0, 50U},
    {7U, "horizontal_rear_left",
     Position::RearLeft, Orientation::HorizontalDiagonal,
     PropellerHand::Reverse, RotationDirection::Clockwise,
     {0.707F, 0.707F, 0.0F}, 0, 50U},
}};

constexpr Vector3 position_vector(Position position)
{
    switch (position) {
    case Position::RearRight:
        return {-1.0F, 1.0F, 0.0F};
    case Position::FrontRight:
        return {1.0F, 1.0F, 0.0F};
    case Position::FrontLeft:
        return {1.0F, -1.0F, 0.0F};
    case Position::RearLeft:
        return {-1.0F, -1.0F, 0.0F};
    }
    return {0.0F, 0.0F, 0.0F};
}

constexpr Vector3 cross(Vector3 lhs, Vector3 rhs)
{
    return {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
    };
}

constexpr Vector3 torque_vector(const ThrusterConfig &thruster)
{
    return cross(position_vector(thruster.position), thruster.positive_force);
}

constexpr int8_t axis_direction(float value)
{
    return value > 0.0F ? 1 : (value < 0.0F ? -1 : 0);
}

constexpr int8_t axis_direction(const ThrusterConfig &thruster,
                                ControlAxis axis)
{
    const Vector3 torque = torque_vector(thruster);
    switch (axis) {
    case ControlAxis::SurgeX:
        return axis_direction(thruster.positive_force.x);
    case ControlAxis::SwayY:
        return axis_direction(thruster.positive_force.y);
    case ControlAxis::HeaveZ:
        return axis_direction(thruster.positive_force.z);
    case ControlAxis::RollX:
        return axis_direction(torque.x);
    case ControlAxis::PitchY:
        return axis_direction(torque.y);
    case ControlAxis::YawZ:
        return axis_direction(torque.z);
    }
    return 0;
}

constexpr bool is_valid_configuration()
{
    uint16_t channel_mask = 0U;
    std::size_t horizontal_count = 0U;
    std::size_t vertical_count = 0U;

    for (const auto &thruster : kThrusters) {
        if (thruster.channel >= kThrusterCount || thruster.name == nullptr) {
            return false;
        }

        const uint16_t channel_bit =
            static_cast<uint16_t>(1U << thruster.channel);
        if ((channel_mask & channel_bit) != 0U) {
            return false;
        }
        channel_mask = static_cast<uint16_t>(channel_mask | channel_bit);

        if (thruster.orientation == Orientation::HorizontalDiagonal) {
            ++horizontal_count;
            if (thruster.positive_force.z != 0.0F ||
                thruster.positive_force.x == 0.0F ||
                thruster.positive_force.y == 0.0F) {
                return false;
            }
        } else {
            ++vertical_count;
            if (thruster.positive_force.x != 0.0F ||
                thruster.positive_force.y != 0.0F ||
                thruster.positive_force.z == 0.0F) {
                return false;
            }
        }
    }

    const uint16_t expected_mask =
        static_cast<uint16_t>((1U << kThrusterCount) - 1U);
    return channel_mask == expected_mask &&
           horizontal_count == 4U &&
           vertical_count == 4U;
}

static_assert(is_valid_configuration(),
              "Thruster configuration must cover eight unique physical channels");

} // namespace thruster_config
