/*
 * copyright (c) 2021 Jaemin Yoon from Interactive & Networked Robotics
 * Laboratory in Seoul National University, South Korea.
 * All rights reserved.
 */
#pragma once

#include <string>

namespace inrol_sim {
  void initialize(std::string asset_root_directory = "./");
  void sim_function(double pd[3], double Rd[3][3],
                    double xk_sim[3], double qk_sim[4],
                    double ak_sim[8], double Fc_sim[6]);
  
  void vis_initialize(std::string asset_root_directory);
	void vis_function(double xk_sim[3], double qk_sim[4],
					double ak_sim[8]);
}