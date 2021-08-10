#ifndef CARMA_DELPHI_SRR2_DRIVER__CARMA_DELPHI_SRR2_DRIVER_HPP_
#define CARMA_DELPHI_SRR2_DRIVER__CARMA_DELPHI_SRR2_DRIVER_HPP_

#include "carma_utils/carma_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace carma_delphi_srr2_driver
{

class CarmaDelphiSrr2Driver : public carma_utils::CarmaNode
{
public:
  CarmaDelphiSrr2Driver();
  ~CarmaDelphiSrr2Driver();

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
};

}  // namespace carma_delphi_srr2_driver

#endif  //  CARMA_DELPHI_SRR2_DRIVER__CARMA_DELPHI_SRR2_DRIVER_HPP_
