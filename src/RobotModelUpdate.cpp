#include "RobotModelUpdate.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rbdyn/RobotLoader.h>

namespace mc_plugin
{

RobotModelUpdate::~RobotModelUpdate() = default;

void RobotModelUpdate::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = controller.controller();
  mc_rtc::log::info("RobotModelUpdate::init called with configuration:\n{}", config.dump(true, true));
  config_ = config;
  if(auto ctlConfig = ctl.config().find("RobotModelUpdate"))
  {
    config_.load(*ctlConfig);
  }
  robot_ = config_("robot", ctl.robot().name());

  ctl.datastore().make_call("RobotModelUpdate::LoadConfig", [this, &ctl]() { configFromXsens(ctl);});
  ctl.datastore().make_call("RobotModelUpdate::UpdateModel", [this, &ctl]() { updateRobotModel(ctl);});

  reset(controller);
}

void RobotModelUpdate::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RobotModelUpdate::reset called");

  auto & ctl = controller.controller();
  auto & gui = *ctl.gui();
  auto & robot = ctl.robot(robot_);

  std::vector<RobotUpdateJoint> joints;

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

  gui.addElement(this, {},
                 mc_rtc::gui::Button("Rescale human model", [this, &ctl]() { configFromXsens(ctl);
                                                                             updateRobotModel(ctl);}));

  robotUpdate.addToGUI(gui, {"Plugin", "RobotModelUpdate", robot_}, "Update robot model from loaded config",
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
  auto X_Hips_0 = ctl.datastore()
                      .call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("Pelvis"))
                      .inv();
  X_Hips_0.translation().z() += -0.09; // origin of hips is at the top

  auto X_0_Head =
      ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("Head"));
  auto X_0_Torso =
      ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose", static_cast<const std::string &>("T8"));

  auto X_0_LArm = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Left Upper Arm"));
  auto X_0_LForearm = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                             static_cast<const std::string &>("Left Forearm"));
  auto X_0_LWrist = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Left Hand"));

  auto X_0_RArm = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Right Upper Arm"));
  auto X_0_RForearm = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                             static_cast<const std::string &>("Right Forearm"));
  auto X_0_RWrist = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Right Hand"));

  auto X_0_LLeg = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Left Upper Leg"));
  auto X_0_LShin = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                          static_cast<const std::string &>("Left Lower Leg"));
  auto X_0_LAnkle = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Left Foot"));

  auto X_0_RLeg = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                         static_cast<const std::string &>("Right Upper Leg"));
  auto X_0_RShin = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                          static_cast<const std::string &>("Right Lower Leg"));
  auto X_0_RAnkle = ctl.datastore().call<sva::PTransformd>("XsensPlugin::GetSegmentPose",
                                                           static_cast<const std::string &>("Right Foot"));

  // step 2: convert world poses to local frames (kinematic tree from hips link)
  auto X_Hips_Head = X_0_Head * X_Hips_0;
  X_Hips_Head.translation().y() = 0; // center

  auto X_Hips_LArm = X_0_LArm * X_Hips_0;
  // Upper arm model is ~2/3 shirt and ~1/3 upper arm to the elbow:
  auto X_LArm_LForearm = X_0_LForearm * X_0_LArm.inv();
  // X_LArm_LForearm.translation().x() += -0.03; // sensor is front of the arm
  Eigen::Vector3d X_LArm_LElbow = 0.66 * X_LArm_LForearm.translation();
  Eigen::Vector3d X_LElbow_LForearm = 0.33 * X_LArm_LForearm.translation();
  auto X_LForearm_LWrist = X_0_LWrist * X_0_LForearm.inv();

  auto X_Hips_RArm = X_0_RArm * X_Hips_0;
  // Upper arm model is ~2/3 shirt and ~1/3 upper arm to the elbow:
  auto X_RArm_RForearm = X_0_RForearm * X_0_RArm.inv();
  // X_RArm_RForearm.translation().x() += -0.03; // sensor is front of the arm
  Eigen::Vector3d X_RArm_RElbow = 0.66 * X_RArm_RForearm.translation();
  Eigen::Vector3d X_RElbow_RForearm = 0.33 * X_RArm_RForearm.translation();
  auto X_RForearm_RWrist = X_0_RWrist * X_0_RForearm.inv();

  auto X_Hips_LLeg = X_0_LLeg * X_Hips_0;
  auto X_LLeg_LShin = X_0_LShin * X_0_LLeg.inv();
  auto X_LShin_LAnkle = X_0_LAnkle * X_0_LShin.inv();

  auto X_Hips_RLeg = X_0_RLeg * X_Hips_0;
  auto X_RLeg_RShin = X_0_RShin * X_0_RLeg.inv();
  auto X_RShin_RAnkle = X_0_RAnkle * X_0_RShin.inv();

  // step 3: update joint configs using local translations

  std::vector<RobotUpdateJoint> joints;

  joints.push_back(RobotUpdateJoint{"Head_0", X_Hips_Head.translation()});

  // torso link doesn't need an update as it has the same origin as the hips
  // joints.push_back(RobotUpdateJoint{"Torso_0", Hips_W_0.inv().translation()});

  joints.push_back(RobotUpdateJoint{"LArm_0", X_Hips_LArm.translation()});
  joints.push_back(RobotUpdateJoint{"LElbow", X_LArm_LElbow});
  joints.push_back(RobotUpdateJoint{"LForearm", X_LElbow_LForearm});
  joints.push_back(RobotUpdateJoint{"LWrist_0", X_LForearm_LWrist.translation()});
  joints.push_back(RobotUpdateJoint{"RArm_0", X_Hips_RArm.translation()});
  joints.push_back(RobotUpdateJoint{"RElbow", X_RArm_RElbow});
  joints.push_back(RobotUpdateJoint{"RForearm", X_RElbow_RForearm});
  joints.push_back(RobotUpdateJoint{"RWrist_0", X_RForearm_RWrist.translation()});

  joints.push_back(RobotUpdateJoint{"LLeg_0", X_Hips_LLeg.translation()});
  joints.push_back(RobotUpdateJoint{"LShin_0", X_LLeg_LShin.translation()});
  joints.push_back(RobotUpdateJoint{"LAnkle_0", X_LShin_LAnkle.translation()});
  joints.push_back(RobotUpdateJoint{"RLeg_0", X_Hips_RLeg.translation()});
  joints.push_back(RobotUpdateJoint{"RShin_0", X_RLeg_RShin.translation()});
  joints.push_back(RobotUpdateJoint{"RAnkle_0", X_RShin_RAnkle.translation()});


  // step 4: save joint configurations
  robotUpdate.joints = joints;

  // step 5: update body scales
  // logic: scale ref is distance between original body origin and original next body origin
  // new scale is new distance/ old distance

  // torso and hips must be scaled in Z AND Y using arms transforms and legs transforms

  std::vector<RobotUpdateBody> bodies;
  Eigen::Vector3d scale(1,1,1);
  auto & robot = ctl.robot(robot_);

  // getting original relative transform of joint
  auto originalTransform1 = robot.mb().transform(robot.jointIndexByName("Head_0")).translation();
  auto newTransform1 = X_Hips_Head.translation();
  scale.z() = newTransform1.z() / originalTransform1.z();

  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LArm_0")).translation();
  auto originalTransform2 = robot.mb().transform(robot.jointIndexByName("RArm_0")).translation();
  newTransform1 = X_Hips_LArm.translation();
  auto newTransform2 = X_Hips_RArm.translation();
  scale.y() = (newTransform1.y() - newTransform2.y()) / (originalTransform1.y() - originalTransform2.y());

  bodies.push_back(RobotUpdateBody{"TorsoLink", scale});



  // step 6: save scale configurations
  robotUpdate.bodies = bodies;
  robotUpdate.toConfiguration().save("/tmp/robotUpdate.json");

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

  auto updateRobot = [&](mc_rbdyn::Robot & robot, const RobotUpdate & robotUpdate)
  {
    for(const auto & joint : robotUpdate.joints)
    {
      mc_rtc::log::warning("HAS UPDATE FOR {}", joint.name);
      if(robot.hasJoint(joint.name))
      {
        auto jIdx = robot.jointIndexByName(joint.name);
        // getting original relative transform of joint
        auto originalTransform = robot.mb().transform(jIdx);
        auto newTransform = originalTransform;
        // updating with new translation while keeping same orientation
        newTransform.translation() = joint.relTranslation;
        robot.mb().transform(jIdx, newTransform);
        mc_rtc::log::info("Updated transform for joint {} from {} to {}", joint.name,
                          originalTransform.translation().transpose(), newTransform.translation().transpose());
      }
    }
    
    for (const auto & surface : robot.availableSurfaces())
    {
      // for all surfaces on the robot, find the parent body
      auto parentName = robot.surface(surface).bodyName();
      auto newScale = Eigen::Vector3d(1, 1, 1);
      for (const auto & body : robotUpdate.bodies)
      {
        // take new scale of the parent body of the surface
        if (body.name == parentName)
        {
          newScale = body.scale;
        }
      }
      // apply the new scale of the body to the translation of the surface transform
      auto originalTransform = robot.surface(surface).X_b_s();
      auto newTransform = originalTransform;
      newTransform.translation() = newScale.array() * newTransform.translation().array();
      robot.surface(surface).X_b_s(newTransform);
      mc_rtc::log::info("Updated transform for surface {} from {} to {}", surface,
                        originalTransform.translation().transpose(), newTransform.translation().transpose());
    }
    
    for (const auto & convex : robot.convexes())
    {
      for (const auto & body : robotUpdate.bodies)
      {
        if (convex.first == body.name)
        {
          auto originalConvex = convex.second.second;
          auto originalConvexTransform = robot.collisionTransform(convex.first);
          auto newConvex = originalConvex;
          newConvex->addScale(body.scale.x(), body.scale.y(), 0); //body.scale.z());
          robot.removeConvex(convex.first);
          // careful: in our case body name and convex name are the same because there is only one convex per body
          robot.addConvex(body.name, body.name, newConvex, originalConvexTransform);
          mc_rtc::log::info("Updated convex {} with scale {}", convex.first, body.scale.transpose());
        }
      }
      
    }
    
    
    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();
  };

  updateRobot(robot, robotUpdate);
  updateRobot(outputRobot, robotUpdate);
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
