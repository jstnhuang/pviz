#include "pviz/leatherman_utils.h"

#include <cmath>

#include "geometry_msgs/Pose.h"
#include "kdl/frames.hpp"
#include "tf/transform_datatypes.h"

namespace leatherman {
void poseVectorToKDL(const std::vector<double> &pose, KDL::Frame &k) {
  k.p.x(pose[0]);
  k.p.y(pose[1]);
  k.p.z(pose[2]);

  // RPY
  if (pose.size() == 6) k.M = KDL::Rotation::RPY(pose[3], pose[4], pose[5]);
  // quaternion
  else if (pose.size() > 6)
    k.M = KDL::Rotation::Quaternion(pose[3], pose[4], pose[5], pose[6]);
  else
    ROS_ERROR("Not enough elements to define the pose.");
}

void kdlToPoseVector(const KDL::Frame &k, std::vector<double> &pose) {
  pose.resize(6, 0.0);
  pose[0] = k.p[0];
  pose[1] = k.p[1];
  pose[2] = k.p[2];
  k.M.GetRPY(pose[3], pose[4], pose[5]);
}

void multiply(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b,
              geometry_msgs::Pose &c) {
  tf::Transform bta, btb, btc;
  tf::poseMsgToTF(a, bta);
  tf::poseMsgToTF(b, btb);
  btc = bta * btb;
  tf::poseTFToMsg(btc, c);
}

void multiply(const std::vector<double> &a, const std::vector<double> &b,
              std::vector<double> &c) {
  KDL::Frame fa, fb, fc;
  poseVectorToKDL(a, fa);
  poseVectorToKDL(b, fb);
  fc = fa * fb;
  kdlToPoseVector(fc, c);
}

void HSVtoRGB(double *r, double *g, double *b, double h, double s, double v) {
  int i;
  double f, p, q, t;
  if (s == 0) {
    // achromatic (grey)
    *r = *g = *b = v;
    return;
  }
  h /= 60;  // sector 0 to 5
  i = floor(h);
  f = h - i;  // factorial part of h
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));
  switch (i) {
    case 0:
      *r = v;
      *g = t;
      *b = p;
      break;
    case 1:
      *r = q;
      *g = v;
      *b = p;
      break;
    case 2:
      *r = p;
      *g = v;
      *b = t;
      break;
    case 3:
      *r = p;
      *g = q;
      *b = v;
      break;
    case 4:
      *r = t;
      *g = p;
      *b = v;
      break;
    default:
      *r = v;
      *g = p;
      *b = q;
      break;
  }
}
}  // namespace leatherman
