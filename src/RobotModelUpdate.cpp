#include "RobotModelUpdate.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/Schema.h>
#include <mc_rbdyn/RobotLoader.h>

namespace mc_plugin
{

RobotModelUpdate::~RobotModelUpdate() = default;

void RobotModelUpdate::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("RobotModelUpdate::init called with configuration:\n{}", config.dump(true, true));
  robot_ = config("robot", controller.controller().robot().name());
  reset(controller);
}

void RobotModelUpdate::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RobotModelUpdate::reset called");

  auto & ctl = controller.controller();
  auto & gui = *ctl.gui();
  auto & robot = ctl.robot(robot_);

  std::vector<RobotUpdateBody> joints;
  joints.emplace_back(RobotUpdateBody{"waist", Eigen::Vector3d{1, 1, 1}});
  joints.push_back(RobotUpdateBody{"WAIST_Y", Eigen::Vector3d{1, 1, 1}});
  mc_rtc::Configuration conf;
  conf.add("joints", joints);
  mc_rtc::log::info("Conf is:\n{}", conf.dump(true, true));

  robotUpdate.load(conf);

  mc_rtc::log::info("Robot Update is:\n{}", robotUpdate.dump(true, true));

  gui.removeElements(this);
  gui.addElement(this,
      {"RobotModelUpdate", robot_},
      mc_rtc::gui::Button("Reset to default",
        [this, &ctl]()
        {
          resetToDefault(ctl.robot(robot_));
        })
  );

  robotUpdate.addToGUI(gui, {"RobotModelUpdate", robot_}, "Update Robot Model", 
       [this, &ctl]() {
        mc_rtc::log::info("Updated robot schema");
        updateRobotModel(ctl);
       });
}

void RobotModelUpdate::resetToDefault(mc_rbdyn::Robot & robot)
{
  auto robots = mc_rbdyn::loadRobot(robot.module());
  auto & defaultRobot = robots->robot();
  robot.mb() = defaultRobot.mb();
  robot.mbg() = defaultRobot.mbg();
  robot.forwardKinematics();
  robot.forwardVelocity();
  robot.forwardAcceleration();
}

void RobotModelUpdate::updateRobotModel(mc_control::MCController & ctl)
{
  auto & robot = ctl.robot(robot_);
  auto & outputRobot = ctl.outputRobot(robot_);

  auto updateJoints = [&](mc_rbdyn::Robot & robot, const RobotUpdate& robotUpdate)
  {
    for(const auto & joint : robotUpdate.joints)
    {
      mc_rtc::log::warning("HAS UPDATE FOR {}", joint.name);
      if(robot.hasJoint(joint.name))
      {
        auto jIdx = robot.jointIndexByName(joint.name);
        auto originalTransform = robot.mb().transform(jIdx);
        auto newTransform = originalTransform;
        newTransform.translation() = originalTransform.translation().cwiseProduct(joint.scale);
        robot.mb().transform(jIdx, newTransform);
        mc_rtc::log::info("Updated transform for joint {} from {} to {} (scale: {})", joint.name, originalTransform.translation().transpose(), newTransform.translation().transpose(), joint.scale);
      }
    }
    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();
  };

  robotUpdate.toConfiguration().save("/tmp/robotUpdate.json");

  updateJoints(robot, robotUpdate);
  updateJoints(outputRobot, robotUpdate);
}

void RobotModelUpdate::before(mc_control::MCGlobalController &controller)
{
  auto & ctl = controller.controller();
  auto & robot = ctl.robot(robot_);
}

void RobotModelUpdate::after(mc_control::MCGlobalController & controller)
{
}

mc_control::GlobalPlugin::GlobalPluginConfiguration RobotModelUpdate::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("RobotModelUpdate", mc_plugin::RobotModelUpdate)
