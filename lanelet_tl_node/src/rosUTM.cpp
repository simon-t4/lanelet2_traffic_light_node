
#include "rosUTM.h"

#include <geodesy/utm.h>

namespace lanelet {
namespace projection {

RosUtmProjector::RosUtmProjector(Origin origin)
    : Projector(origin) {

  }

BasicPoint3d RosUtmProjector::forward(const GPSPoint& gps) const {
  BasicPoint3d utm{0., 0., gps.ele};

  geographic_msgs::GeoPoint wgs_point;
  wgs_point.latitude = gps.lat;
  wgs_point.longitude = gps.lon;
  //  wgs_point.altitude = point.z;

  geodesy::UTMPoint utm_point;
  geodesy::fromMsg(wgs_point,utm_point);
  utm.x() = fmod(utm_point.easting,1e5);
  utm.y() = fmod(utm_point.northing,1e5);


  //  utm.x() = 1.1;
  // utm.y() = 2.2;

  return utm;
}

GPSPoint RosUtmProjector::reverse(const BasicPoint3d& utm) const {
  GPSPoint gps{0., 0., utm.z()};
  
  gps.lat = 100.1, gps.lon = 200.0;
  return gps;
}

}  // namespace projection
}  // namespace lanelet
