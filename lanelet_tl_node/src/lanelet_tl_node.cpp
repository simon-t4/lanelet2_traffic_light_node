#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
 
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_msgs/MapBin.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>

#include <lanelet2_extension/io/message_conversion.hpp>
#include <lanelet2_extension/query/autoware_query.h>
#include <lanelet2_extension/visualization/autoware_visualization.h>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>


#include <sensor_msgs/CameraInfo.h>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/AdjustXY.h>

#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <sstream>


#include <lanelet2_core/geometry/Point.h>


//std::string example_map_path = "/home/simon/work/catkin_ws/src/lanelet2/lanelet2_maps/res/mapping_example.osm";
//std::string example_map_path = "/home/simon/work/peoria_data/map/Lanelet2/Peoria_0301_2019_fixed_edit.osm";
std::string example_map_path = "/home/simon/work/peoria_data/map/Autoware_lanelet2/Peoria_autoware_lanelet_20190702.osm";
/*

#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


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
#include <Eigen/Eigen>

#include <autoware_msgs/Signals.h>
#include <boost/timer/timer.hpp>
#include <cstdio>

#include <sstream>
#include "rosUTM.h"


*/
//using namespace lanelet;

#define TL_BY_LANELET 0
#define TL_BY_NEAREST 1

#define signalLampRadius 0.3

static std::string camera_id_str;


static bool loaded_lanelet_map = false;
static lanelet::LaneletMapPtr lanelet_map;

static int adjust_proj_x = -60;
static int adjust_proj_y = 10;

typedef struct
{
  double thiX;
  double thiY;
  double thiZ;
} Angle;


static Eigen::Vector3f position;
static Eigen::Quaternionf orientation;
static float fx,
  fy,
  imageWidth,
  imageHeight,
  cx,
  cy;

static  tf::StampedTransform camera_to_map_tf;
static tf::StampedTransform map_to_camera_tf;

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

/* Callback function to shift projection result */

void adjust_xyCallback(const autoware_msgs::AdjustXY::ConstPtr &config_msg)
{
  adjust_proj_x = config_msg->x;
  adjust_proj_y = config_msg->y;
}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr camInfoMsg)
{


  //  std::cout << "recieved camera information\n";
  fx = static_cast<float>(camInfoMsg->P[0]);
  fy = static_cast<float>(camInfoMsg->P[5]);
  imageWidth = camInfoMsg->width;
  imageHeight = camInfoMsg->height;
  cx = static_cast<float>(camInfoMsg->P[2]);
  cy = static_cast<float>(camInfoMsg->P[6]);
}


//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------


void get_transform(std::string from_frame, std::string to_frame, Eigen::Quaternionf &ori, Eigen::Vector3f& pos, tf::StampedTransform & tf)
{

  static tf::TransformListener listener;
  // tf::StampedTransform tf;
 
  // target_frame    source_frame
  ros::Time now = ros::Time();
  listener.waitForTransform(from_frame, to_frame, now, ros::Duration(10.0));
  listener.lookupTransform(from_frame, to_frame, now, tf);

 
  tf::Vector3 &p = tf.getOrigin();
  tf::Quaternion o = tf.getRotation();

  pos.x() = p.x();
  pos.y() = p.y();
  pos.z() = p.z();
  ori.w() = o.w();
  ori.x() = o.x();
  ori.y() = o.y();
  ori.z() = o.z();

}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------




Eigen::Vector3f transform(const Eigen::Vector3f &psrc, tf::StampedTransform &tfsource)
{
  tf::Vector3 pt3(psrc.x(), psrc.y(), psrc.z());
  tf::Vector3 pt3s = tfsource * pt3;
  Eigen::Vector3f tf_v(pt3s.x(), pt3s.y(), pt3s.z());
  return tf_v;
}




//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------


void print_line_string_points(lanelet::LineString3d ls)
{
  for (auto pi = ls.begin(); pi != ls.end(); pi++) {
    //   lanelet::Point3d p = *pi;
    std::cerr << "\t " << *pi << "\n"; 
  }
}




//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

void make_light_marker(lanelet::Point3d p, visualization_msgs::Marker& marker, std::string frame_id, std::string ns, int tl_count, int point_count){
  
  uint32_t box_shape = visualization_msgs::Marker::CUBE; 
  
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = tl_count*10+point_count;
  marker.type = box_shape;
  marker.pose.position.x = p.x();
  marker.pose.position.y = p.y();
  marker.pose.position.z = p.z();
  
  float s = 0.5;

  marker.scale.x = s;
  marker.scale.y = s;
  marker.scale.z = s;
  
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  
  if (point_count == 0) marker.color.r = 1.0f;
  else if (point_count == 1) marker.color.g = 1.0f;
  if (point_count == 2) marker.color.b = 1.0f;
  
  marker.lifetime = ros::Duration();
}




//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------


void visualize_lanelet_linestring(int lane_id, lanelet::ConstLineString3d ls, visualization_msgs::Marker& points,
				  visualization_msgs::Marker& line_strip,
				  std::string frame_id, std::string ns, float lr, float lg, float lb, float lss= 0.15)
{

  
  points.header.frame_id = line_strip.header.frame_id = frame_id;
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = ns;
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  points.id = lane_id*10+0;
  line_strip.id = lane_id*10+1;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  line_strip.scale.x = lss; 


  points.color.g = 1.0f;
  points.color.a = 1.0f;
  line_strip.color.r = lr;
  line_strip.color.g = lg;
  line_strip.color.b = lb;
  line_strip.color.a = 1.0f;

  // fill out lane line
  for (auto i = ls.begin(); i != ls.end(); i++){
    geometry_msgs::Point p;
    p.x = (*i).x();
    p.y = (*i).y();
    p.z = 100.0; //(*i).z();
    points.points.push_back(p);
    line_strip.points.push_back(p);
  }
}

//--------------------------------------------------------------------
//
// project into image place (from AUtoware feat_proj.cpp)
// Point3 type not found -> from Vector Map maths?
//
//--------------------------------------------------------------------


bool project2(const Eigen::Vector3f &pt, int &u, int &v, bool useOpenGLCoord = false)
{
  float nearPlane = 1.0;
  float farPlane = 200.0;

  Eigen::Vector3f _pt = transform(pt, camera_to_map_tf);

  float _u = _pt.x() * fx / _pt.z() + cx;
  float _v = _pt.y() * fy / _pt.z() + cy;
  
  u = static_cast<int>(_u);
  v = static_cast<int>(_v);
  if (u < 0 || imageWidth < u || v < 0 || imageHeight < v || _pt.z() < nearPlane || farPlane < _pt.z())
    {
      u = -1, v = -1;
      return false;
    }

  if (useOpenGLCoord)
    {
      v = imageHeight - v;
    }

  return true;
}



void visualize_lanelet_map(ros::Publisher& pub, lanelet::LaneletMap& lanelet_map){
  

  visualization_msgs::MarkerArray left_ls_array;
  visualization_msgs::MarkerArray right_ls_array;
  visualization_msgs::MarkerArray center_ls_array;
  left_ls_array.markers.resize(lanelet_map.laneletLayer.size());
  right_ls_array.markers.resize(lanelet_map.laneletLayer.size());
  center_ls_array.markers.resize(lanelet_map.laneletLayer.size());
  int lanelet_count = 0;
  for (auto i = lanelet_map.laneletLayer.begin(); i != lanelet_map.laneletLayer.end(); i++) {
    lanelet::Lanelet ll = *i;
    
    //std::cerr << "Nearest lanelet = " << ll << "\n";
    lanelet::ConstLineString3d left_ls = ll.leftBound();
    lanelet::ConstLineString3d right_ls = ll.rightBound();
    lanelet::ConstLineString3d center_ls = ll.centerline();
    visualization_msgs::Marker left_line_strip, left_points, right_line_strip, right_points, center_line_strip, center_points;
    
    visualize_lanelet_linestring(lanelet_count, left_ls, left_points, left_ls_array.markers[lanelet_count], "map", "aleft_lane_bound", 1.0f, 0.0f, 0.0f);
    visualize_lanelet_linestring(lanelet_count,right_ls, right_points, right_ls_array.markers[lanelet_count], "map", "aright_lane_bound",0.0f, 0.0f, 1.0f);
    visualize_lanelet_linestring(lanelet_count,center_ls, center_points, center_ls_array.markers[lanelet_count], "map", "acenter_lane_bound",0.0f, 1.0f, 1.0f);
    // left_ls_array.markers[lanelet_count] = left_ls;
    lanelet_count++;
  }
  //  std::cerr << "laneclet count = " << lanelet_count << "\n";
  //  std::cerr << "marker array size = " << left_ls_array.markers.size() << "\n";


  //  ros::spinOnce();
  pub.publish(left_ls_array);
  pub.publish(right_ls_array);
  pub.publish(center_ls_array);
  
}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------
 
void binMapCallback(lanelet2_msgs::MapBin msg)
{
  lanelet_utils::Map::fromBinMsg(msg, lanelet_map);
  loaded_lanelet_map = true;
}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------
double getAbsoluteDiff2Angles(const double x, const double y, const double c)
{
  // c can be PI or 180;
  return c - fabs(fmod(fabs(x - y), 2*c) - c);
}

double normalise( const double value, const double start, const double end )
{
  const double width       = end - start   ;   //
  const double offsetValue = value - start ;   // value relative to 0

  return ( offsetValue - ( floor( offsetValue / width ) * width ) ) + start ;
  // + start to reset back to start of original range
}


bool inRange(lanelet::BasicPoint2d p, lanelet::BasicPoint2d cam, double max_r)
{
  double d = lanelet::geometry::distance(p, cam);
  return (d < max_r);
}
bool inView(lanelet::BasicPoint2d p, lanelet::BasicPoint2d cam, double heading, double max_a, double max_r)
{
  double d = lanelet::geometry::distance(p, cam);
  double a = getAbsoluteDiff2Angles(heading, atan2(p.y() -cam.y(), p.x()-cam.x()), M_PI);
  // std::cerr << "angle from cam to point = " << atan2(p.y()-cam.y(), p.x()-cam.x()) << "\n";
  //  if (d < max_r) std::cerr << "distance = " << d << " diff heading = " << a << "\n";


  
  return (d < max_r && a < max_a);
}


bool isAttributeValue(lanelet::ConstPoint3d p, std::string attr_str, std::string value_str)
{
  lanelet::Attribute attr = p.attribute(attr_str);
  if (attr.value().compare(value_str) == 0) 
    return true;
  return false;
}

int main (int argc, char **argv)
{

  
  loaded_lanelet_map = false;
  bool load_from_file =false;
  
  ros::init(argc, argv, "lanelet_tl_node");  
  ros::NodeHandle rosnode;
  ros::Subscriber bin_map_sub;


  ros::Publisher visible_traffic_light_polygon_pub = rosnode.advertise<jsk_recognition_msgs::PolygonArray>("visible_traffic_lights_poly", 1);

  
  if (load_from_file==true) {

    lanelet::ErrorMessages errors;
    lanelet::projection::MGRSProjector projector;
    lanelet_map = load(example_map_path, projector, &errors);
    
    loaded_lanelet_map = true;
  }
  else {  
    bin_map_sub = rosnode.subscribe("/lanelet_map_bin", 10000, binMapCallback);
  }


  while(ros::ok() && !loaded_lanelet_map)
    {
      ros::spinOnce();
      // loop_rate.sleep();
    }
  
  
  std::cerr << "loaded lanelet map\n";
  
  ros::NodeHandle private_nh("~");
  std::string cameraInfo_topic_name;
  private_nh.param<std::string>("camera_info_topic", cameraInfo_topic_name, "/camera_info");
  
  /* get camera ID */
  camera_id_str = cameraInfo_topic_name;
  camera_id_str.erase(camera_id_str.find("/camera_info"));
  if (camera_id_str == "/")
    {
      camera_id_str = "camera";
    }
  


  //  std::cerr << "got camera info\n";
  
  // subcribe to camera info & image etc
  ros::Subscriber cameraInfoSubscriber = rosnode.subscribe(cameraInfo_topic_name, 100, cameraInfoCallback);
  ros::Subscriber cameraImage = rosnode.subscribe(cameraInfo_topic_name, 100, cameraInfoCallback);
  //  ros::Subscriber adjust_xySubscriber = rosnode.subscribe("/config/adjust_xy", 100, adjust_xyCallback);
  

  // publisher to pub regions of interest: ie areas in image where traffic lights should be
  ros::Publisher roi_sign_pub = rosnode.advertise<autoware_msgs::Signals>("roi_signal", 100);
  
  ros::Rate loop_rate(10);
  Eigen::Vector3f pos;
  Eigen::Quaternionf ori;
  Eigen::Vector3f prev_position(0,0,0);
  Eigen::Quaternionf prev_orientation(0,0,0,0);
  
  ros::spinOnce();
  
  // main loop
  int count = 0;
  
  int last_lane_count = 0;


  lanelet::Lanelets all_lanelets = lanelet_utils::Query::laneletLayer(lanelet_map);
  std::vector<lanelet::TrafficLight::Ptr> tl_reg_elems = lanelet_utils::Query::trafficLights(all_lanelets);
  std::vector<lanelet::autoware::AutowareTrafficLight::Ptr> aw_tl_reg_elems = lanelet_utils::Query::autowareTrafficLights(all_lanelets);

  
  std::vector<lanelet::TrafficLight::Ptr> visible_tl;
  std::vector<lanelet::autoware::AutowareTrafficLight::Ptr> visible_aw_tl;
  while(ros::ok())
    {

       ros::spinOnce();


       try {

	// need both trasnform directions - cam to map for projecting into camera space
	//    and map to camera for searching in lanelet map coordinates (use as camera position)


	 get_transform(camera_id_str, "map", orientation, position, camera_to_map_tf);
	 get_transform("map", camera_id_str, orientation, position, map_to_camera_tf);
	
      }

      catch (tf::TransformException &exc) {}
      
       if (prev_orientation.vec() != orientation.vec()  &&
	  prev_position != position)
	{
	  
	  lanelet::BasicPoint2d camera_position_2d(position.x(), position.y());

	  // for each traffic light in map check if in range and in view angle of camera
	  
	  double cam_yaw =  tf::getYaw(map_to_camera_tf.getRotation())+M_PI/2;

	  for (auto tli = tl_reg_elems.begin(); tli != tl_reg_elems.end(); tli++) {



	    lanelet::TrafficLight::Ptr tl = *tli;
	    lanelet::LineString3d ls;
	    
	    //ls = static_cast<lanelet::LineString3d>(lights);
      
	    auto lights = tl->trafficLights();
	    for (auto lsp: lights){
	      
	      
	      if (lsp.isLineString()) { // traffic ligths can either polygons or linestrings
	
		lanelet::ConstLineString3d ls = static_cast<lanelet::LineString3d>(lsp);
		
	
		lanelet::BasicPoint2d tl_base_0 = lanelet::utils::to2D(ls.front()).basicPoint();
		lanelet::BasicPoint2d tl_base_1 = lanelet::utils::to2D(ls.back()).basicPoint();

		double max_r = 100.0;

		if (inRange(tl_base_0, camera_position_2d, max_r) &&
		    inRange(tl_base_1, camera_position_2d, max_r)) {
		  
		  double dx = tl_base_1.x() - tl_base_0.x();
		  double dy = tl_base_1.y() - tl_base_0.y();
		  double nx = -dy;  // 90 rotation for cos sin = 1's and 0's -> normal is -dy, dx
		  double ny = dx;
		  double dir = normalise(atan2(ny,nx)+M_PI, -M_PI, M_PI);
		  
		  double diff = getAbsoluteDiff2Angles(dir, cam_yaw, M_PI);
		  
		  if (fabs(diff-M_PI) < 50.0/180.0*M_PI) {
		    // testing range twice (above inRange) for each base point at the moment
		    if (inView(tl_base_0, camera_position_2d, cam_yaw, 50.0/180.0*M_PI, max_r) &&
			inView(tl_base_1, camera_position_2d, cam_yaw, 50.0/180.0*M_PI, max_r)) {
		      //std::cerr << "traffic light possibly in view of camera\n";
		      auto tl_id = tl->id();
		      visible_tl.push_back(tl);
		      // find equivalent autoware traffic light element (is this necessary?)
		      // can we get base line string information from reg. elem as autoware traffic light?
		      
		      for (auto aw_tli = aw_tl_reg_elems.begin(); aw_tli != aw_tl_reg_elems.end(); aw_tli++) {

			if ((*aw_tli)->id() == tl_id) {
			  //  std::cerr << "found equiv reg elem trafficlight\n";
			  visible_aw_tl.push_back(*aw_tli);
			  
			}
		      }
		    }
		    
		  }
		}
	      }
	    }
	    //	  int n_lanelets = 1;
	    // lanelet::Lanelets nearest_lanelets = lanelet_map->laneletLayer.nearest(camera_position_2d, n_lanelets);
	  
	  
	  }
	  int lane_count = 0;
	  
	  // for found traffic lights
	  //  - extract points of individual lights
	  //  - project into image space
	  //  - check in range
	  //  - create roi sign
	  //  - publish
	  //  - visualize in rviz by marker
	  
	  
	  int tl_count = 0;
	  autoware_msgs::Signals signalsInFrame;
	  bool viz_tl_signs  = true;
 	  for (auto tli = visible_aw_tl.begin(); tli != visible_aw_tl.end(); tli++) {

	    lanelet::autoware::AutowareTrafficLight::Ptr tl = *tli;
	    lanelet::ConstLineStrings3d lights;
	    
	    lights = tl->lightBulbs();
	    for (auto lsi: lights){
	      lanelet::ConstLineString3d ls = static_cast<lanelet::LineString3d>(lsi);

	      for (auto p: ls) {



		Eigen::Vector3f signal_center(p.x(), p.y(), p.z());
		Eigen::Vector3f signal_centerx(p.x(), p.y(), p.z()+signalLampRadius); /// ????
		int u,v;
		
		if (project2(signal_center, u,v, false) == true) {
		  int radius, ux,vx;
		  project2(signal_centerx, ux, vx, false);
		  radius = (int)sqrt((Eigen::Vector2f(ux-u,vx-v)).squaredNorm());
		  
		  autoware_msgs::ExtractedPosition sign;
		  sign.signalId = p.id();
		  sign.u = u + adjust_proj_x;
		  sign.v = v + adjust_proj_y;
	      
		  sign.radius = radius;
		  sign.x = signal_center.x(), sign.y = signal_center.y(), sign.z = signal_center.z();
		  sign.hang = 0.0;// angle [0, 360] degrees: not used in setContexts for region_tlr
		  
		  sign.type = 0; // type should be int for color/form of bulb 1: RED, 2: GREEN 3: YELLOW // to do left arrorws etc 21, 22, 23
		   if (isAttributeValue(p,"color","red")) sign.type = 1;
		  else if (isAttributeValue(p,"color","yellow")) sign.type = 3;
		  else if (isAttributeValue(p,"color","green")) sign.type = 2;

		   // only left arrow defined in tgrafficlight detection contexts
		  if (p.hasAttribute("arrow")){
		    if (isAttributeValue(p, "arrow", "left")) {
		      sign.type+=20;
		    }
		  }
		  
		    
		  sign.linkId = tl->id(); // this should be lane id? -> traffic light reg elem id ok because unique to multiple physical traffic ligths
		  sign.plId = p.id();
		  std::cerr << "sending signal roi at " << sign.u << " " << sign.v << "\n";

		  // nexts get relative signal angle (to camera coord system)
		  // limit +- 50 degrees
		  
		  //perhaps use related lanelet to determine if in frame 
		  // if in frame.....
		  signalsInFrame.Signals.push_back(sign);
		}
		
		//	point_count++;
		
	      } // for each point pi
	    } // if linestring
	    
	    //tl_count++;
	  }

	  //	  timer.stop();
	  //  std::cerr << " timer = " << timer.format() << "\n";
	  
	  // publish signals detected in camera frame
	  signalsInFrame.header.stamp = ros::Time::now();
	  roi_sign_pub.publish(signalsInFrame);
	  	 
	}
      prev_orientation = orientation;
      prev_position = position;
      

      if (visible_tl.size() > 0) {
	std::cerr << "publishing visible tl\n";
	lanelet_utils::Visualization::publishTrafficLightsAsPolygonArray(visible_tl, visible_traffic_light_polygon_pub, 40001, 1.5);

      }
      
      // ros loop stuff
      loop_rate.sleep();
      ++count;
       
    }
  
  return 0;
}
