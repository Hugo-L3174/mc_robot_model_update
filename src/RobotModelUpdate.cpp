#include "RobotModelUpdate.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/Schema.h>

namespace mc_plugin
{

RobotModelUpdate::~RobotModelUpdate() = default;

void RobotModelUpdate::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = controller.controller();
  mc_rtc::log::info("RobotModelUpdate::init called with configuration:\n{}", config.dump(true, true));
  if(!ctl.config().has("RobotModelUpdate"))
  {
    robot_ = config("robot", controller.controller().robot().name());
  }
  else
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

  mc_rtc::Configuration conf;
  conf.add("joints", joints);
  mc_rtc::log::info("Conf is:\n{}", conf.dump(true, true));

  robotUpdate.load(conf);

  mc_rtc::log::info("Robot Update is:\n{}", robotUpdate.dump(true, true));

  gui.removeElements(this);
  gui.addElement(this, {"Plugin", "RobotModelUpdate", robot_},
                 mc_rtc::gui::Button("Reset to default",
                                     [this, &ctl]()
                                     {
                                       resetToDefault(ctl.robot(robot_));
                                       resetToDefault(ctl.outputRobot(robot_));
                                     }),
                 mc_rtc::gui::Button("Load Xsens config", [this, &ctl]() { configFromXsens(ctl); }));

  robotUpdate.addToGUI(gui, {"Plugin", "RobotModelUpdate", robot_}, "Update Robot Model",
                       [this, &ctl]()
                       {
                         mc_rtc::log::info("Updated robot schema");
                         updateRobotModel(ctl);
                       });
}

void RobotModelUpdate::configFromXsens(mc_control::MCController & ctl)
{
  // New version: using data directly from xsens to scale
  // step 1: get poses (world frame)
  auto Hips_W_0 = ctl.datastore()
                      .call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("Pelvis"))
                      .inv();
  Hips_W_0.translation().z() += -0.09; // origin of hips is at the top

  auto Head_0_W =
      ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("Head"));
  auto Torso_0_W =
      ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("T8"));

  auto LArm_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                       static_cast<const std::string &>("Left Upper Arm"));
  auto LForearm_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Left Forearm"));
  auto LWrist_0_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Left Hand"));

  auto RArm_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                       static_cast<const std::string &>("Right Upper Arm"));
  auto RForearm_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Right Forearm"));
  auto RWrist_0_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Right Hand"));

  auto LLeg_0_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Left Upper Leg"));
  auto LShin_0_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                          static_cast<const std::string &>("Left Lower Leg"));
  auto LAnkle_0_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Left Foot"));

  auto RLeg_0_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Right Upper Leg"));
  auto RShin_0_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                          static_cast<const std::string &>("Right Lower Leg"));
  auto RAnkle_0_W = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Right Foot"));

  // step 2: convert world poses to local frames (kinematic tree from hips link)
  auto Head_Hips = Head_0_W * Hips_W_0;
  Head_Hips.translation().x() += -0.04; // offset from middle of body to actual joint
  Head_Hips.translation().y() = 0; // center

  auto LArm_Hips = LArm_W * Hips_W_0;
  // Upper arm model is ~2/3 shirt and ~1/3 upper arm to the elbow:
  auto LForearm_Arm = LForearm_W * LArm_W.inv();
  LForearm_Arm.translation().x() += -0.03; // sensor is front of the arm
  Eigen::Vector3d LElbow_Arm = 0.66 * LForearm_Arm.translation();
  Eigen::Vector3d LForearm_Elbow = 0.33 * LForearm_Arm.translation();
  auto LWrist_Forearm = LWrist_0_W * LForearm_W.inv();

  auto RArm_Hips = RArm_W * Hips_W_0;
  // Upper arm model is ~2/3 shirt and ~1/3 upper arm to the elbow:
  auto RForearm_Arm = RForearm_W * RArm_W.inv();
  RForearm_Arm.translation().x() += -0.03; // sensor is front of the arm
  Eigen::Vector3d RElbow_Arm = 0.66 * RForearm_Arm.translation();
  Eigen::Vector3d RForearm_Elbow = 0.33 * RForearm_Arm.translation();
  auto RWrist_Forearm = RWrist_0_W * RForearm_W.inv();

  auto LLeg_Hips = LLeg_0_W * Hips_W_0;
  auto LShin_Leg = LShin_0_W * LLeg_0_W.inv();
  auto LAnkle_Shin = LAnkle_0_W * LShin_0_W.inv();

  auto RLeg_Hips = RLeg_0_W * Hips_W_0;
  auto RShin_Leg = RShin_0_W * RLeg_0_W.inv();
  auto RAnkle_Shin = RAnkle_0_W * RShin_0_W.inv();

  // step 3: update joint configs using local translations

  std::vector<RobotUpdateBody> joints;

  joints.push_back(RobotUpdateBody{"Head_0", Head_Hips.translation()});

  // joints.push_back(RobotUpdateBody{"Torso_0", Hips_W_0.inv().translation()});

  joints.push_back(RobotUpdateBody{"LArm_0", LArm_Hips.translation()});
  joints.push_back(RobotUpdateBody{"LElbow", LElbow_Arm});
  joints.push_back(RobotUpdateBody{"LForearm", LForearm_Elbow});
  joints.push_back(RobotUpdateBody{"LWrist_0", LWrist_Forearm.translation()});
  joints.push_back(RobotUpdateBody{"RArm_0", RArm_Hips.translation()});
  joints.push_back(RobotUpdateBody{"RElbow", RElbow_Arm});
  joints.push_back(RobotUpdateBody{"RForearm", RForearm_Elbow});
  joints.push_back(RobotUpdateBody{"RWrist_0", RWrist_Forearm.translation()});

  joints.push_back(RobotUpdateBody{"LLeg_0", LLeg_Hips.translation()});
  joints.push_back(RobotUpdateBody{"LShin_0", LShin_Leg.translation()});
  joints.push_back(RobotUpdateBody{"LAnkle_0", LAnkle_Shin.translation()});
  joints.push_back(RobotUpdateBody{"RLeg_0", RLeg_Hips.translation()});
  joints.push_back(RobotUpdateBody{"RShin_0", RShin_Leg.translation()});
  joints.push_back(RobotUpdateBody{"RAnkle_0", RAnkle_Shin.translation()});

  // step 4: load configuration
  mc_rtc::Configuration conf;
  conf.add("joints", joints);
  robotUpdate.load(conf);
  mc_rtc::log::info("Robot Update is:\n{}", robotUpdate.dump(true, true));
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

  auto updateJoints = [&](mc_rbdyn::Robot & robot, const RobotUpdate & robotUpdate)
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
        mc_rtc::log::info("Updated transform for joint {} from {} to {}", joint.name,
                          originalTransform.translation().transpose(), newTransform.translation().transpose());
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

void RobotModelUpdate::before(mc_control::MCGlobalController & controller)
{
  auto & ctl = controller.controller();
  auto & robot = ctl.robot(robot_);
}

void RobotModelUpdate::after(mc_control::MCGlobalController & controller) {}

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
