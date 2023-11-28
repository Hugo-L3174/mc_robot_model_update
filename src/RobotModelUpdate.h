/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

namespace mc_plugin
{

struct RobotUpdateBody
{
  MC_RTC_NEW_SCHEMA(RobotUpdateBody)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(RobotUpdateBody, __VA_ARGS__))
  MEMBER(std::string, name, "Name of the body")
#undef MEMBER
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(RobotUpdateBody, __VA_ARGS__))
  MEMBER(Eigen::Vector3d, scale, "Scale")
#undef MEMBER
};

struct RobotUpdate
{
  MC_RTC_NEW_SCHEMA(RobotUpdate)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(RobotUpdate, __VA_ARGS__))
  MEMBER(std::vector<RobotUpdateBody>, joints, "joints to update")
#undef MEMBER
};


struct RobotModelUpdate : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~RobotModelUpdate() override;

protected:
  void updateRobotModel(mc_control::MCController & ctl);
  void resetToDefault(mc_rbdyn::Robot & robot);

protected:
  std::string robot_;
  RobotUpdate robotUpdate;
};

} // namespace mc_plugin