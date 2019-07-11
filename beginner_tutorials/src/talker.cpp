#include "ros/ros.h"
#include "std_msgs/String.h"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>

#include <lanelet2_core/primitives/Lanelet.h>

#include <cstdio>

#include <sstream>


//using namespace lanelet;





//std::string example_map_path = "/home/simon/work/catkin_ws/src/lanelet2/lanelet2_maps/res/mapping_example.osm";
std::string example_map_path = "/home/simon/work/peoria_data/map/Lanelet2/Peoria_0301_2019_fixed.osm";


void print_line_string_points(lanelet::LineString3d ls)
{
  for (auto pi = ls.begin(); pi != ls.end(); pi++) {
    //   lanelet::Point3d p = *pi;
    std::cout << "\t " << *pi << "\n"; 
  }
}


int main (int argc, char **argv)
{

  lanelet::Origin origin({49, 8.4});
  lanelet::Origin origin2({0.0, 0.0});
    lanelet::projection::UtmProjector projector(origin2);

  lanelet::ErrorMessages errors;
  // lanelet::LaneletMapPtr map = load(example_map_path, projector, &errors);
    lanelet::LaneletMapPtr map = load(example_map_path, origin2, &errors);
  
  //  assert(errors.empty());

  lanelet::PointLayer& points = map->pointLayer;


  if (1 == 0) {
    for (auto i = points.begin(); i != points.end(); i++) {
      lanelet::Point3d pt = *i;
      std::cout << " pt = " << pt << "\n";
      
    }
  }

  // loop through lanelets to find all traffic lights related to given lanelets, print out traffic light, point positions for line strings.
  
  lanelet::LaneletLayer& lanelets = map->laneletLayer;
  for (auto i = lanelets.begin(); i != lanelets.end(); i++) {

    lanelet::Lanelet ll = *i;
    std::vector<lanelet::TrafficLight::Ptr> traffic_light_re = ll.regulatoryElementsAs<lanelet::TrafficLight>();
    //  std::cout << "num. of traffic lights = " << traffic_light_re.size() << "\n";
    for (auto tli = traffic_light_re.begin(); tli != traffic_light_re.end(); tli++) {

      std::cout << "lanelet = " << ll << "\n";

      std::cout << "traffic light = " << *tli << "\n";

      lanelet::LineString3d ls;
      lanelet::LineStringOrPolygon3d lights = (*tli)->trafficLights().front();
      std::cout << "light = " << lights << "\n";
      bool is_ls = lights.isLineString(); 
      if (is_ls) {
	std::cout << "light is a line_string\n";
	ls = static_cast<lanelet::LineString3d>(lights);

	for (auto pi = ls.begin(); pi != ls.end(); pi++) {
	  lanelet::Point3d p = *pi;
	  
	  std::cout << "\t Point " << p << "\n"; 
	}
	
      }
      bool is_poly = lights.isPolygon(); 
      if (is_poly) std::cout << "light is a polygon\n";
    }
  }


  lanelet::BasicPoint3d current_position(3, 5, 7);
  lanelet::BasicPoint2d current_position_2d(3, 5);
  
  std::cout << "current position = " << current_position_2d << "\n";
  
  lanelet::Lanelets nearest_lanelets = map->laneletLayer.nearest(current_position_2d, 1);
  
  
  for (auto i = nearest_lanelets.begin(); i != nearest_lanelets.end(); i++) {
    
     lanelet::Lanelet ll = *i;
    
    std::cout << "Nearest lanelet = " << ll << "\n";
    lanelet::LineString3d left_ls = ll.leftBound();
    lanelet::LineString3d right_ls = ll.rightBound();

    std::cout << "Left bound "<< left_ls << "\n";
    print_line_string_points(left_ls);
    std::cout << "Right bound "<< right_ls << "\n";
    print_line_string_points(right_ls);

    std::vector<lanelet::TrafficLight::Ptr> nearest_tl_re = ll.regulatoryElementsAs<lanelet::TrafficLight>();

    if (nearest_tl_re.empty())
      {
	std::cout << "No traffic lights...." << "\n";
      }
    else
      {
	for (auto tli = nearest_tl_re.begin(); tli != nearest_tl_re.end(); tli++) {
	  
	  lanelet::LineString3d ls;
	  lanelet::LineStringOrPolygon3d lights = (*tli)->trafficLights().front();
	  //  std::cout << "light = " << lights << "\n";
	  bool is_ls = lights.isLineString(); 
	  if (lights.isLineString()) {
	    //	std::cout << "light is a line_string\n";
	    ls = static_cast<lanelet::LineString3d>(lights);
	    for (auto pi = ls.begin(); pi != ls.end(); pi++) {
	      lanelet::Point3d p = *pi;
	      std::cout << "\t Point " << p << "\n"; 
	    }	
	  }
	}
	
      }
  }
  

  
  exit(1);
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while(ros::ok())
    {
      std_msgs::String msg;
      std::stringstream ss;
      ss << "Hello world " << count;
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());

      chatter_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }

  return 0;
}
