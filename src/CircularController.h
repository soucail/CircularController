#pragma once

#include <RBDyn/Jacobian.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>

#include <memory>

#include "api.h"

struct CircularController_DLLAPI CircularController : public mc_control::MCController
{
  CircularController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::EndEffectorTask> circularTask;  
  std::shared_ptr<mc_tasks::PostureTask> postureTask;


private:
  mc_rtc::Configuration config_;
  rbd::Jacobian jacobian;
  bool start_moving_;
  bool init_;
  double ctlTime_;
  double omega_;
  double R_;
};
