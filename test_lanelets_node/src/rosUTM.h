#pragma once
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Projection.h>



namespace lanelet {
namespace projection {


  class RosUtmProjector : public Projector {
 public:
    explicit RosUtmProjector(lanelet::Origin origin);

  BasicPoint3d forward(const GPSPoint& gps) const override;

  GPSPoint reverse(const BasicPoint3d& utm) const override;

 private:

  /*
  int zone_{};
  bool isInNorthernHemisphere_{true}, useOffset_{}, throwInPaddingArea_{};
  double xOffset_{}, yOffset_{};
  */
};

}
}
