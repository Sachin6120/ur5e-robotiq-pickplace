// JointCmdProbe.cc -- read JointVelocityCmd straight out of the ECM.
//
// WHY A SYSTEM PLUGIN AND NOT ANOTHER TOPIC LOGGER
//
//   Three rounds of reconstructing the mimic loop's command from /joint_states
//   have each closed a hypothesis without landing the mechanism, and the last
//   one hit the method's ceiling: once a joint is pinned at its velocity clamp,
//   "stuck at a stale command" and "obeying a live command computed from the
//   wrong reference" produce identical position and velocity traces. No amount
//   of reconstruction separates them, because the thing that differs is the
//   command itself, and the command is an ECM component that is never
//   published on any topic (confirmed live: /world/empty/state does not carry
//   JointVelocityCmd, see docs/m3_check_state_topic.sh).
//
//   So read the component. That is what a system plugin is for.
//
// WHAT IT ANSWERS, AND HOW
//
//   The probe reads JointVelocityCmd TWICE per iteration:
//
//     PreUpdate  -- after gz_ros2_control has written it this step
//     PostUpdate -- after the physics system has run
//
//   Three outcomes, and they are mutually exclusive:
//
//     1. pre_cmd changes every iteration and matches
//        -500 * (pos_mimic - pos_mimicked * multiplier) clamped to the joint's
//        velocity limit
//        -> the mimic loop is writing correct live values. The command is
//           right and the divergence is downstream, in how dartsim applies it.
//
//     2. pre_cmd holds one unchanging value across many iterations while
//        positions keep moving
//        -> the value is stale. Nothing is refreshing it, and the joint is
//           coasting on a command written once. Look at whether
//           gz_ros2_control's write() is reaching this joint at all.
//
//     3. pre_cmd is live and correct, but post_cmd is absent or zeroed
//        -> the component is CONSUMED by the physics system each step. At an
//           update_rate of 500 against a 1 ms physics step, the joint is then
//           velocity-servoed on every other step and free on the rest, which
//           would give exactly the loose tracking and low effort measured.
//
//   Outcome 3 is the one no topic-level tool can see, and it is why the probe
//   reads on both sides of physics rather than once.
//
// PLUGIN ORDER MATTERS -- READ THIS BEFORE TRUSTING THE OUTPUT
//
//   gz-sim runs every system's PreUpdate before any system's Update. Within
//   the PreUpdate phase, systems run in the order they are declared in the
//   SDF. gz_ros2_control writes JointVelocityCmd during ITS PreUpdate.
//
//   So this plugin MUST be declared AFTER gz_ros2_control in the model or
//   world SDF. Declared before it, pre_cmd shows the PREVIOUS iteration's
//   value and outcome 1 will masquerade as outcome 2. Wired into
//   ur5e_robotiq.urdf.xacro immediately after the gz_ros2_control <plugin>
//   block for exactly this reason -- verify the rendered SDF by eye if this
//   file is ever reordered.
//
// USAGE
//
//   Build (see CMakeLists.txt alongside this file), then set
//   GZ_SIM_SYSTEM_PLUGIN_PATH to the build directory before launching the sim.
//
// OUTPUT
//   sim_t, iteration, joint, pre_cmd, post_cmd, pre_present, post_present,
//   position, velocity
//
//   pre_present / post_present distinguish "component exists holding 0.0" from
//   "component does not exist" -- which is precisely the distinction outcome 3
//   turns on, and one a bare numeric column would erase.

#include <gz/plugin/Register.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>

#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace joint_cmd_probe
{

class JointCmdProbe
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    if (!model.Valid(_ecm)) {
      gzerr << "[JointCmdProbe] not attached to a model; nothing to probe\n";
      return;
    }

    std::string out = "/tmp/joint_cmd_probe.csv";
    if (_sdf->HasElement("output")) {
      out = _sdf->Get<std::string>("output");
    }

    // sdf::Element is const here; clone to walk repeated <joint> elements.
    auto sdfClone = _sdf->Clone();
    for (auto el = sdfClone->GetElement("joint"); el;
         el = el->GetNextElement("joint"))
    {
      const std::string name = el->Get<std::string>();
      // Match by suffix: the real names carry a xacro prefix.
      gz::sim::Entity je = gz::sim::kNullEntity;
      for (const auto & cand : model.Joints(_ecm)) {
        auto n = _ecm.Component<gz::sim::components::Name>(cand);
        if (n && n->Data().size() >= name.size() &&
            n->Data().compare(n->Data().size() - name.size(), name.size(), name) == 0)
        {
          je = cand;
          break;
        }
      }
      if (je == gz::sim::kNullEntity) {
        // Loud, not silent: a typo here would produce a clean empty CSV that
        // reads exactly like "the command was never written".
        gzerr << "[JointCmdProbe] joint not found in model: " << name << "\n";
        continue;
      }
      this->joints_[je] = name;

      // JointPosition and JointVelocity are only populated if something has
      // created the components. gz_ros2_control creates them, but create them
      // here too so the probe does not depend on load order for its own
      // context columns.
      if (!_ecm.EntityHasComponentType(
          je, gz::sim::components::JointPosition().TypeId()))
      {
        _ecm.CreateComponent(je, gz::sim::components::JointPosition());
      }
      if (!_ecm.EntityHasComponentType(
          je, gz::sim::components::JointVelocity().TypeId()))
      {
        _ecm.CreateComponent(je, gz::sim::components::JointVelocity());
      }
    }

    this->csv_.open(out, std::ios::out);
    if (!this->csv_) {
      gzerr << "[JointCmdProbe] cannot open " << out << " for writing\n";
      return;
    }
    this->csv_ << "sim_t,iteration,joint,pre_cmd,post_cmd,"
               << "pre_present,post_present,position,velocity\n";
    this->csv_.flush();

    gzmsg << "[JointCmdProbe] probing " << this->joints_.size()
          << " joints -> " << out << "\n";
    gzmsg << "[JointCmdProbe] THIS PLUGIN MUST BE DECLARED AFTER "
          << "gz_ros2_control in the SDF, or pre_cmd lags by one iteration\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.paused) { return; }
    this->pre_.clear();
    for (const auto & [entity, name] : this->joints_) {
      auto c = _ecm.Component<gz::sim::components::JointVelocityCmd>(entity);
      if (c && !c->Data().empty()) {
        this->pre_[entity] = {true, c->Data()[0]};
      } else {
        this->pre_[entity] = {false, 0.0};
      }
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.paused || !this->csv_) { return; }

    const double t =
      std::chrono::duration<double>(_info.simTime).count();

    for (const auto & [entity, name] : this->joints_) {
      auto c = _ecm.Component<gz::sim::components::JointVelocityCmd>(entity);
      const bool post_present = (c && !c->Data().empty());
      const double post_cmd = post_present ? c->Data()[0] : 0.0;

      auto p = _ecm.Component<gz::sim::components::JointPosition>(entity);
      auto v = _ecm.Component<gz::sim::components::JointVelocity>(entity);
      const double pos = (p && !p->Data().empty()) ? p->Data()[0] : 0.0;
      const double vel = (v && !v->Data().empty()) ? v->Data()[0] : 0.0;

      auto it = this->pre_.find(entity);
      const bool pre_present = (it != this->pre_.end()) && it->second.first;
      const double pre_cmd = pre_present ? it->second.second : 0.0;

      this->csv_ << t << ',' << _info.iterations << ',' << name << ','
                 << pre_cmd << ',' << post_cmd << ','
                 << (pre_present ? 1 : 0) << ',' << (post_present ? 1 : 0) << ','
                 << pos << ',' << vel << '\n';
    }
    // Line-buffered on purpose: a killed sim must not take the tail with it,
    // and the tail is where the grasp fails.
    this->csv_.flush();
  }

private:
  std::map<gz::sim::Entity, std::string> joints_;
  std::map<gz::sim::Entity, std::pair<bool, double>> pre_;
  std::ofstream csv_;
};

}  // namespace joint_cmd_probe

GZ_ADD_PLUGIN(
  joint_cmd_probe::JointCmdProbe,
  gz::sim::System,
  joint_cmd_probe::JointCmdProbe::ISystemConfigure,
  joint_cmd_probe::JointCmdProbe::ISystemPreUpdate,
  joint_cmd_probe::JointCmdProbe::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  joint_cmd_probe::JointCmdProbe, "gz::sim::systems::JointCmdProbe")
