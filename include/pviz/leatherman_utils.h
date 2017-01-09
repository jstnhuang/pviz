#ifndef _PVIZ_LEATHERMAN_UTILS_H_
#define _PVIZ_LEATHERMAN_UTILS_H_

#include <vector>

#include "geometry_msgs/Pose.h"
#include "kdl/frames.hpp"

// Utils copied from bcohen's leatherman library, so all the pviz code is in one
// repo.

namespace leatherman {
void poseVectorToKDL(const std::vector<double> &pose, KDL::Frame &k);
void kdlToPoseVector(const KDL::Frame &k, std::vector<double> &pose);
void multiply(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b,
	      geometry_msgs::Pose &c);
void multiply(const std::vector<double> &a, const std::vector<double> &b,
	      std::vector<double> &c);
void HSVtoRGB(double *r, double *g, double *b, double h, double s, double v);
}  // namespace leatherman

#endif  // _PVIZ_LEATHERMAN_UTILS_H_
