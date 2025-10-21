/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file mjcf.h
 * @brief Reading from MuJoCo XML (MJCF) files, exposed functions.
 * @author GTDynamics Contributors
 */

#pragma once

#include <gtdynamics/universal_robot/Robot.h>

#include <string>

namespace gtdynamics {

/**
 * @fn Construct Robot from a MuJoCo XML (MJCF) file.
 * @param[in] file_path path to the MJCF file.
 * @param[in] model_name name of the robot model (optional, extracted from file if not specified).
 * @return Robot object created from the MJCF file
 */
Robot CreateRobotFromMJCF(const std::string &file_path,
                          const std::string &model_name = "");

}  // namespace gtdynamics
