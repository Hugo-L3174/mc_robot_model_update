#include "RobotModelUpdate.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rbdyn/RobotLoader.h>
#include <sch-core/S_Polyhedron.h>
#include "Convex.h"

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
  auto & robot = ctl.robot(robot_);
  ctl.datastore().make_call("RobotModelUpdate::LoadConfig", [this, &ctl]() { configFromXsens(ctl); });
  ctl.datastore().make_call("RobotModelUpdate::UpdateModel", [this, &ctl]() { updateRobotModel(ctl); });

  mc_rtc::gui::PolyhedronConfig polyConfig;
  polyConfig.triangle_color = {0, 0.9, 0, 0.5};
  polyConfig.vertices_config.color = {0, 1, 0, 0.5};
  polyConfig.show_vertices = false;
  polyConfig.edge_config.color = {0, 1, 0, 0.5};
  polyConfig.show_edges = false;

  for(const auto & [convexName, convexPair] : robot.convexes())
  {
    const auto & bodyName = convexPair.first;
    const auto & object = convexPair.second;

    if(auto poly = std::dynamic_pointer_cast<sch::S_Polyhedron>(object))
    {
      const auto & convexName_ = convexName;
      ctl.gui()->addElement({"Human", "Convex"}, mc_rtc::gui::Convex(
                                                     convexName, polyConfig, [poly]() { return poly; },
                                                     [bodyName, convexName_, &robot]()
                                                     {
                                                       const sva::PTransformd X_0_b = robot.frame(bodyName).position();
                                                       const sva::PTransformd & X_b_c =
                                                           robot.collisionTransform(convexName_);
                                                       sva::PTransformd X_0_c = X_b_c * X_0_b;
                                                       return X_0_c;
                                                     }));
    }
  }

  for(const auto & visual : robot.module()._visual)
  {
    for(size_t i = 0; i < visual.second.size(); i++)
    {
      const auto & v = visual.second[i];
      ctl.gui()->addElement({"Human", "Visuals"}, mc_rtc::gui::Visual(
                                                      fmt::format("{}_{}", visual.first, i), [&v]() { return v; },
                                                      [&robot, &v, &visual]()
                                                      {
                                                        return v.origin * robot.frame(visual.first).position();
                                                      }));
    }
  }

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
                 mc_rtc::gui::Button("Rescale human model",
                                     [this, &ctl]()
                                     {
                                       configFromXsens(ctl);
                                       updateRobotModel(ctl);
                                     }));

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

  X_Hips_0.translation().z() -= 0.09; // origin of hips is at the top

  // step 2: convert world poses to local frames (kinematic tree from hips link)
  auto X_Hips_Head = X_0_Head * X_Hips_0;
  X_Hips_Head.translation().y() = 0; // center
  X_Hips_Head.translation().x() -= 0.05;

  auto X_Hips_LArm = X_0_LArm * X_Hips_0;
  X_Hips_LArm.translation().x() -= 0.05; // origin of arms is middle of the model whereas measure xsens is top shoulder
  X_Hips_LArm.translation().y() -= 0.03;
  X_Hips_LArm.translation().z() -= 0.03;
  // Upper arm model is ~2/3 shirt and ~1/3 upper arm to the elbow:
  auto X_LArm_LForearm = X_0_LForearm * X_0_LArm.inv();
  Eigen::Vector3d X_LArm_LElbow = 0.66 * X_LArm_LForearm.translation();
  Eigen::Vector3d X_LElbow_LForearm = 0.34 * X_LArm_LForearm.translation();
  auto X_LForearm_LWrist = X_0_LWrist * X_0_LForearm.inv();
  X_LForearm_LWrist.translation().x() += 0.01;
  X_LForearm_LWrist.translation().z() -= 0.005;

  auto X_Hips_RArm = X_0_RArm * X_Hips_0;
  X_Hips_RArm.translation().x() -= 0.05; // origin of arms is middle of the model whereas measure xsens is top shoulder
  X_Hips_LArm.translation().y() += 0.03;
  X_Hips_RArm.translation().z() -= 0.03;
  // Upper arm model is ~2/3 shirt and ~1/3 upper arm to the elbow:
  auto X_RArm_RForearm = X_0_RForearm * X_0_RArm.inv();
  X_RArm_RForearm.translation().x() -= 0.01;
  Eigen::Vector3d X_RArm_RElbow = 0.66 * X_RArm_RForearm.translation();
  Eigen::Vector3d X_RElbow_RForearm = 0.34 * X_RArm_RForearm.translation();
  X_RElbow_RForearm.x() -= 0.01;
  auto X_RForearm_RWrist = X_0_RWrist * X_0_RForearm.inv();
  X_RForearm_RWrist.translation().x() += 0.01;
  X_RForearm_RWrist.translation().z() -= 0.005;

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
  Eigen::Vector3d scaleTorso = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleRArm = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleLArm = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleRForearm = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleLForearm = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleHips = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleRUpperLeg = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleLUpperLeg = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleRLowerLeg = Eigen::Vector3d{1, 1, 1};
  Eigen::Vector3d scaleLLowerLeg = Eigen::Vector3d{1, 1, 1};

  auto & robot = ctl.robot(robot_);

  // getting original relative transform of joints

  // torso
  auto originalTransform1 = robot.mb().transform(robot.jointIndexByName("Head_0")).translation();
  auto newTransform1 = X_Hips_Head.translation();
  scaleTorso.z() = newTransform1.z() / originalTransform1.z();
  if(firstScale_)
  {
    scaleTorso.z() *= 0.9;
  }

  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LArm_0")).translation();
  auto originalTransform2 = robot.mb().transform(robot.jointIndexByName("RArm_0")).translation();
  newTransform1 = X_Hips_LArm.translation();
  auto newTransform2 = X_Hips_RArm.translation();
  scaleTorso.y() = (newTransform1.y() - newTransform2.y()) / (originalTransform1.y() - originalTransform2.y());
  if(firstScale_)
  {
    scaleTorso.y() *= 0.9;
    scaleTorso.x() *= 0.9;
  }

  // r upper arm
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("RElbow")).translation();
  mc_rtc::log::info("Relbow translation: {}",
                    robot.mb().transform(robot.jointIndexByName("RElbow")).translation().transpose());
  mc_rtc::log::info("Rwrist translation: {}",
                    robot.mb().transform(robot.jointIndexByName("RWrist_0")).translation().transpose());
  mc_rtc::log::info("Rarm elbow new translation: {}", X_RArm_RElbow.transpose());
  mc_rtc::log::info("Relbow rwrist new translation: {}", X_RForearm_RWrist.translation().transpose());
  newTransform1 = X_RArm_RElbow;
  scaleRArm.y() = newTransform1.y() / originalTransform1.y();

  // r forearm
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("RWrist_0")).translation();
  newTransform1 = X_RForearm_RWrist.translation();
  scaleRForearm.y() = newTransform1.y() / originalTransform1.y();

  // l upper arm
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LElbow")).translation();
  newTransform1 = X_LArm_LElbow;
  scaleLArm.y() = newTransform1.y() / originalTransform1.y();

  // l forearm
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LWrist_0")).translation();
  newTransform1 = X_LForearm_LWrist.translation();
  scaleLForearm.y() = newTransform1.y() / originalTransform1.y();

  // hips
  originalTransform1 = (robot.mb().transform(robot.jointIndexByName("LLeg_0")).translation()
                        + robot.mb().transform(robot.jointIndexByName("RLeg_0")).translation())
                       / 2;
  newTransform1 = (X_Hips_LLeg.translation() + X_Hips_RLeg.translation()) / 2;
  scaleHips.z() = newTransform1.z() / originalTransform1.z();
  if(firstScale_)
  {
    scaleHips.x() *= 0.9;
  }

  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LLeg_0")).translation();
  originalTransform2 = robot.mb().transform(robot.jointIndexByName("RLeg_0")).translation();
  newTransform1 = X_Hips_LLeg.translation();
  newTransform2 = X_Hips_RLeg.translation();
  scaleHips.y() = (newTransform1.y() - newTransform2.y()) / (originalTransform1.y() - originalTransform2.y());
  auto bottomScale = 1; // scaleHips.y();

  // l upper leg
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LShin_0")).translation();
  newTransform1 = X_LLeg_LShin.translation();
  scaleLUpperLeg.z() = newTransform1.z() / originalTransform1.z();
  scaleLUpperLeg.y() = bottomScale;
  scaleLUpperLeg.x() = bottomScale;

  // r upper leg
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("RShin_0")).translation();
  newTransform1 = X_RLeg_RShin.translation();
  scaleRUpperLeg.z() = newTransform1.z() / originalTransform1.z();
  scaleRUpperLeg.y() = bottomScale;
  scaleRUpperLeg.x() = bottomScale;

  // l lower leg
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("LAnkle_0")).translation();
  newTransform1 = X_LShin_LAnkle.translation();
  scaleLLowerLeg.z() = newTransform1.z() / originalTransform1.z();
  scaleLLowerLeg.y() = bottomScale;
  scaleLLowerLeg.x() = bottomScale;

  // r lower leg
  originalTransform1 = robot.mb().transform(robot.jointIndexByName("RAnkle_0")).translation();
  newTransform1 = X_RShin_RAnkle.translation();
  scaleRLowerLeg.z() = newTransform1.z() / originalTransform1.z();
  scaleRLowerLeg.y() = bottomScale;
  scaleRLowerLeg.x() = bottomScale;

  bodies.push_back(RobotUpdateBody{"TorsoLink", scaleTorso});
  bodies.push_back(RobotUpdateBody{"RArmLink", scaleRArm});
  bodies.push_back(RobotUpdateBody{"RElbowLink", scaleRArm});
  bodies.push_back(RobotUpdateBody{"RForearmLink", scaleRForearm});
  bodies.push_back(RobotUpdateBody{"LArmLink", scaleLArm});
  bodies.push_back(RobotUpdateBody{"LElbowLink", scaleLArm});
  bodies.push_back(RobotUpdateBody{"LForearmLink", scaleLForearm});
  bodies.push_back(RobotUpdateBody{"HipsLink", scaleHips});
  bodies.push_back(RobotUpdateBody{"RLegLink", scaleRUpperLeg});
  bodies.push_back(RobotUpdateBody{"RShinLink", scaleRLowerLeg});
  bodies.push_back(RobotUpdateBody{"LLegLink", scaleLUpperLeg});
  bodies.push_back(RobotUpdateBody{"LShinLink", scaleLLowerLeg});

  // step 6: save scale configurations
  robotUpdate.bodies = bodies;
  robotUpdate.toConfiguration().save("/tmp/robotUpdate.json");
  if(firstScale_)
  {
    firstScale_ = false;
  }
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

    for(const auto & surface : robot.availableSurfaces())
    {
      // for all surfaces on the robot, find the parent body
      auto parentName = robot.surface(surface).bodyName();
      auto newScale = Eigen::Vector3d(1, 1, 1);
      for(const auto & body : robotUpdate.bodies)
      {
        // take new scale of the parent body of the surface
        if(body.name == parentName)
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

    for(const auto & convex : robot.convexes())
    {
      for(const auto & body : robotUpdate.bodies)
      {
        if(convex.first == body.name)
        {
          auto originalConvex = convex.second.second;
          // check we can actually cast the convex to a polyhedron
          if(auto poly = std::dynamic_pointer_cast<sch::S_Polyhedron>(originalConvex))
          {
            auto vertices = poly->getPolyhedronAlgorithm()->vertexes_;
            for(auto vertex : vertices)
            {
              auto origVertexCoord = vertex->getCoordinates();
              vertex->setCoordinates(origVertexCoord.m_x * body.scale.x(), origVertexCoord.m_y * body.scale.y(),
                                     origVertexCoord.m_z * body.scale.z());
            }
          }
          // careful: if there were existing collision constraints on the convex, they need to be removed and readded
          mc_rtc::log::info("Updated convex {} with scale {}", convex.first, body.scale.transpose());
        }
      }
    }

    auto & visuals = const_cast<mc_rbdyn::RobotModule &>(robot.module())._visual;
    for(const auto & body : robotUpdate.bodies)
    {
      for(auto & visual : visuals[body.name])
      {
        if(visual.geometry.type == rbd::parsers::Geometry::MESH)
        {
          auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(visual.geometry.data);
          mesh.scaleV = mesh.scaleV.cwiseProduct(body.scale);
          mc_rtc::log::info("Updated visual {} with scale {}", body.name, body.scale.transpose());
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
