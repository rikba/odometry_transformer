#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "odometry_transformer/odometry_transformer.h"

namespace odometry_transformer {

class OdometryTransformerNodelet : public nodelet::Nodelet {
 private:
  inline virtual void onInit() {
    try {
      tf_ = std::make_shared<OdometryTransformer>(getNodeHandle(),
                                                  getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<OdometryTransformer> tf_;
};

}

PLUGINLIB_EXPORT_CLASS(odometry_transformer::OdometryTransformerNodelet,
    nodelet::Nodelet
)