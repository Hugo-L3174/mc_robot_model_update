#include "RobotModelUpdate.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/Schema.h>
#include <mc_rbdyn/RobotLoader.h>

namespace mc_plugin
{

RobotModelUpdate::~RobotModelUpdate() = default;

void RobotModelUpdate::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto& ctl = controller.controller();
  mc_rtc::log::info("RobotModelUpdate::init called with configuration:\n{}", config.dump(true, true));
  if (!ctl.config().has("RobotModelUpdate"))
  {
    robot_ = config("robot", controller.controller().robot().name());
  }else
  {
    auto ctlConf = ctl.config()("RobotModelUpdate");
    ctlConf("robot", robot_);
  }


  config_.load(config);
  
  reset(controller);
}

void RobotModelUpdate::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RobotModelUpdate::reset called");

  auto & ctl = controller.controller();
  auto & gui = *ctl.gui();
  auto & robot = ctl.robot(robot_);

  std::vector<RobotUpdateBody> joints;
  
  auto bodyDim = config_("human")("Celia");
  double BodyHeight = bodyDim("BodyHeight");
  double HipHeight = bodyDim("HipHeight");
  double LegHeight = HipHeight - 0.1;
  double HipWidth = bodyDim("HipWidth");
  double LegsWidth = HipWidth - 0.03;
  double ShoulderHeight = bodyDim("ShoulderHeight");
  double ArmHeight = ShoulderHeight - 0.15;
  double ShoulderWidth = bodyDim("ShoulderWidth");
  double ArmsWidth = ShoulderWidth - 0.06;
  double ElbowSpan = bodyDim("ElbowSpan");
  double WristSpan = bodyDim("WristSpan");
  double KneeHeight = bodyDim("KneeHeight");
  double AnkleHeight = bodyDim("AnkleHeight");

  joints.push_back(RobotUpdateBody{"Head_0", Eigen::Vector3d{-0.03, 0, ShoulderHeight - HipHeight}});

  joints.push_back(RobotUpdateBody{"Torso_0", Eigen::Vector3d{0, 0, LegHeight - HipHeight}});

  joints.push_back(RobotUpdateBody{"LArm_0", Eigen::Vector3d{-0.03, ArmsWidth/2, ArmHeight - HipHeight}});
  joints.push_back(RobotUpdateBody{"LElbow", Eigen::Vector3d{0, ((ElbowSpan-ArmsWidth)/2)*0.66, 0}});
  joints.push_back(RobotUpdateBody{"LForearm", Eigen::Vector3d{0, ((ElbowSpan-ArmsWidth)/2)*0.33, 0}});
  joints.push_back(RobotUpdateBody{"LWrist_0", Eigen::Vector3d{0, (WristSpan-ElbowSpan)/2, 0}});
  joints.push_back(RobotUpdateBody{"RArm_0", Eigen::Vector3d{-0.03, -ArmsWidth/2, ArmHeight - HipHeight}});
  joints.push_back(RobotUpdateBody{"RElbow", Eigen::Vector3d{0, -((ElbowSpan-ArmsWidth)/2)*0.66, 0}});
  joints.push_back(RobotUpdateBody{"RForearm", Eigen::Vector3d{0, -((ElbowSpan-ArmsWidth)/2)*0.33, 0}});
  joints.push_back(RobotUpdateBody{"RWrist_0", Eigen::Vector3d{0, -(WristSpan-ElbowSpan)/2, 0}});

  joints.push_back(RobotUpdateBody{"LLeg_0", Eigen::Vector3d{0, LegsWidth/2, LegHeight - HipHeight }});
  joints.push_back(RobotUpdateBody{"LShin_0", Eigen::Vector3d{0, 0, KneeHeight - LegHeight}});
  joints.push_back(RobotUpdateBody{"LAnkle_0", Eigen::Vector3d{0, 0, AnkleHeight - KneeHeight}});
  joints.push_back(RobotUpdateBody{"RLeg_0", Eigen::Vector3d{0, -LegsWidth/2, LegHeight - HipHeight}});
  joints.push_back(RobotUpdateBody{"RShin_0", Eigen::Vector3d{0, 0, KneeHeight - LegHeight}});
  joints.push_back(RobotUpdateBody{"RAnkle_0", Eigen::Vector3d{0, 0, AnkleHeight - KneeHeight}});

  mc_rtc::Configuration conf;
  conf.add("joints", joints);
  mc_rtc::log::info("Conf is:\n{}", conf.dump(true, true));

  robotUpdate.load(conf);

  mc_rtc::log::info("Robot Update is:\n{}", robotUpdate.dump(true, true));

  gui.removeElements(this);
  gui.addElement(this,
      {"Plugin", "RobotModelUpdate", robot_},
      mc_rtc::gui::Button("Reset to default",
        [this, &ctl]()
        {
          resetToDefault(ctl.robot(robot_));
          resetToDefault(ctl.outputRobot(robot_));
        })
  );

  robotUpdate.addToGUI(gui, {"Plugin", "RobotModelUpdate", robot_}, "Update Robot Model", 
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
        newTransform.translation() = joint.relTranslation;
        robot.mb().transform(jIdx, newTransform);
        mc_rtc::log::info("Updated transform for joint {} from {} to {}", joint.name, originalTransform.translation().transpose(), newTransform.translation().transpose());
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
