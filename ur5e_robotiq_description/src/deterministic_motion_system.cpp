#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <chrono>
#include <cmath>

namespace ur5e_robotiq_sim
{

class DeterministicMotion
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  DeterministicMotion() = default;
  ~DeterministicMotion() override = default;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/) override
  {
    this->model_ = gz::sim::Model(_entity);
    if (!this->model_.Valid(_ecm))
    {
      gzerr << "[DeterministicMotion] Plugin must be attached to a Model entity." << std::endl;
      return;
    }

    if (_sdf->HasElement("center_x"))
      this->x_ = _sdf->Get<double>("center_x");
    if (_sdf->HasElement("center_z"))
      this->z_ = _sdf->Get<double>("center_z");
    if (_sdf->HasElement("y_min"))
      this->y_min_ = _sdf->Get<double>("y_min");
    if (_sdf->HasElement("y_max"))
      this->y_max_ = _sdf->Get<double>("y_max");
    if (_sdf->HasElement("period"))
      this->period_ = _sdf->Get<double>("period");

    gzmsg << "[DeterministicMotion] Configured for model '" << this->model_.Name(_ecm)
          << "' with X=" << this->x_ << ", Z=" << this->z_
          << ", Y=[" << this->y_min_ << ", " << this->y_max_ << "]"
          << ", period=" << this->period_ << "s" << std::endl;
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    // Simulation time in seconds
    double t_sim = std::chrono::duration<double>(_info.simTime).count();

    // Periodic linear motion (triangular ping-pong) between y_min and y_max
    double phase = std::fmod(t_sim, this->period_);
    if (phase < 0.0)
      phase += this->period_;

    double half_period = 0.5 * this->period_;
    double fraction = 0.0;
    if (phase <= half_period)
    {
      fraction = phase / half_period;
    }
    else
    {
      fraction = (this->period_ - phase) / half_period;
    }

    double y = this->y_min_ + fraction * (this->y_max_ - this->y_min_);

    gz::math::Pose3d target_pose(this->x_, y, this->z_, 0.0, 0.0, 0.0);
    this->model_.SetWorldPoseCmd(_ecm, target_pose);
  }

private:
  gz::sim::Model model_{gz::sim::kNullEntity};
  double x_{0.70};
  double z_{0.85};
  double y_min_{0.25};
  double y_max_{0.45};
  double period_{4.0};
};

}  // namespace ur5e_robotiq_sim

GZ_ADD_PLUGIN(
    ur5e_robotiq_sim::DeterministicMotion,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    ur5e_robotiq_sim::DeterministicMotion,
    "ur5e_robotiq_sim::DeterministicMotion")
