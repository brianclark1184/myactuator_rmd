#include "myactuator_rmd/protocol/requests.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>

#include "myactuator_rmd/actuator_state/acceleration_type.hpp"
#include "myactuator_rmd/actuator_state/can_baud_rate.hpp"
#include "myactuator_rmd/protocol/single_motor_message.hpp"
#include "myactuator_rmd/exceptions.hpp"


namespace myactuator_rmd {

  bool CanIdRequest::isWrite() const noexcept {
    return (getAs<std::uint8_t>(2) == 0) ? true : false;
  }

  GetCanIdRequest::GetCanIdRequest()
  : CanIdRequest{}  {
    setAt(static_cast<std::uint8_t>(1), 2);
    return;
  }

  SetCanIdRequest::SetCanIdRequest(std::uint16_t const can_id)
  : CanIdRequest{}  {
    setAt(static_cast<std::uint8_t>(0), 2);
    setAt(static_cast<std::uint8_t>(can_id), 6);
    return;
  }

  std::uint16_t SetCanIdRequest::getCanId() const noexcept {
    return static_cast<std::uint16_t>(getAs<std::uint8_t>(7));
  }

  SetAccelerationRequest::SetAccelerationRequest(std::uint32_t const acceleration, AccelerationType const mode)
  : SingleMotorRequest{} {
    if ((acceleration < 100) || (acceleration > 60000)) {
      throw ValueRangeException("Acceleration value '" + std::to_string(acceleration) + "' out of range [100, 60000]");
    }
    auto const acceleration_type {static_cast<std::uint8_t>(mode)};
    setAt(acceleration_type, 1);
    setAt(acceleration, 4);
    return;
  }

  std::uint32_t SetAccelerationRequest::getAcceleration() const noexcept {
    return getAs<std::uint32_t>(4);
  }

  AccelerationType SetAccelerationRequest::getMode() const noexcept {
    return static_cast<AccelerationType>(getAs<std::uint8_t>(1));
  }

  SetCanBaudRateRequest::SetCanBaudRateRequest(CanBaudRate const baud_rate)
  : SingleMotorRequest{} {
    setAt(static_cast<std::uint8_t>(baud_rate), 7);
    return;
  }

  CanBaudRate SetCanBaudRateRequest::getBaudRate() const noexcept {
    return static_cast<CanBaudRate>(getAs<std::uint8_t>(7));
  }

  SetEncoderZeroRequest::SetEncoderZeroRequest(std::int32_t const encoder_offset)
  : SingleMotorRequest{} {
    setAt(encoder_offset, 4);
    return;
  }

  std::int32_t SetEncoderZeroRequest::getEncoderZero() const noexcept {
    return getAs<std::int32_t>(4);
  }

  SetPositionAbsoluteRequest::SetPositionAbsoluteRequest(float const position, float const max_speed)
  : SingleMotorRequest{} {
    auto const v {static_cast<std::uint16_t>(max_speed)};
    auto const pos {static_cast<std::int32_t>(position*100.0f)};
    setAt(v, 2);
    setAt(pos, 4);
    return;
  }

  float SetPositionAbsoluteRequest::getMaxSpeed() const noexcept {
    return static_cast<float>(getAs<std::uint16_t>(2));
  }

  float SetPositionAbsoluteRequest::getPosition() const noexcept {
    return static_cast<float>(getAs<std::int32_t>(4)/100.0f);
  }

  SetTorqueRequest::SetTorqueRequest(float const current)
  : SingleMotorRequest{} {
    auto const c {static_cast<std::int16_t>(current/0.01f)};
    setAt(c, 4);
    return;
  }

  float SetTorqueRequest::getTorqueCurrent() const noexcept {
    return static_cast<float>(getAs<std::int16_t>(4))*0.01f;
  }

  SetTimeoutRequest::SetTimeoutRequest(std::chrono::milliseconds const& timeout) {
    setAt(static_cast<std::uint32_t>(timeout.count()), 4);
    return;
  }

  std::chrono::milliseconds SetTimeoutRequest::getTimeout() const noexcept {
    std::chrono::milliseconds const timeout {getAs<std::uint32_t>(4)};
    return timeout;
  }

  SetVelocityRequest::SetVelocityRequest(float const speed)
  : SingleMotorRequest{} {
    auto const s {static_cast<std::int32_t>(speed*100.0f)};
    setAt(s, 4);
    return;
  }

  float SetVelocityRequest::getSpeed() const noexcept {
    return static_cast<float>(getAs<std::int32_t>(4))/100.0f;
  }

  SetMotionModeRequest::SetMotionModeRequest(std::uint16_t const motor_id, std::int16_t p_des, std::int16_t v_des,
                         std::uint16_t kp, std::uint16_t kd, std::int16_t t_ff)
    : SingleMotorRequest{} {
        auto const can_id = 0x400 + motor_id;
        setAt(can_id, 0);  // Set CAN ID
        
        // Bit manipulation to set p_des, v_des, kp, kd, t_ff according to the detailed breakdown
        setAt(static_cast<std::uint8_t>((p_des >> 8) & 0xFF), 1); // p_des upper 8 bits
        setAt(static_cast<std::uint8_t>(p_des & 0xFF), 2);        // p_des lower 8 bits
        setAt(static_cast<std::uint8_t>((v_des >> 4) & 0xFF), 3); // v_des upper 8 bits
        setAt(static_cast<std::uint8_t>(((v_des & 0xF) << 4) | ((kp >> 8) & 0xF)), 4); // v_des lower 4 bits + kp upper 4 bits
        setAt(static_cast<std::uint8_t>(kp & 0xFF), 5);           // kp lower 8 bits
        setAt(static_cast<std::uint8_t>((kd >> 4) & 0xFF), 6);    // kd upper 8 bits
        setAt(static_cast<std::uint8_t>(((kd & 0xF) << 4) | ((t_ff >> 8) & 0xF)), 7); // kd lower 4 bits + t_ff upper 4 bits
        setAt(static_cast<std::uint8_t>(t_ff & 0xFF), 8);         // t_ff lower 8 bits
    }

}
