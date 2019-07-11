#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
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

#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>



#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>





#include <lanelet2_core/primitives/Lanelet.h>
#include <Eigen/Eigen>

#include <autoware_msgs/Signals.h>

#include <cstdio>

#include <sstream>
#include "rosUTM.h"



#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>
//using namespace lanelet;

#define TL_BY_LANELET 0
#define TL_BY_NEAREST 1

#define signalLampRadius 0.3

static std::string camera_id_str;


//std::string example_map_path = "/home/simon/work/catkin_ws/src/lanelet2/lanelet2_maps/res/mapping_example.osm";
std::string example_map_path = "/home/simon/work/peoria_data/map/Lanelet2/Peoria_0301_2019_fixed.osm";

//static int adjust_proj_x = 0;
//static int adjust_proj_y = 0;

typedef struct
{
  double thiX;
  double thiY;
  double thiZ;
} Angle;

//static VectorMap vmap;
//static Angle cameraOrientation; // camera orientation = car's orientation

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




/////////////////////////////////////////////////////////////////////////////
//
// process_mem_usage(double &, double &) - takes two doubles by reference,
// attempts to read the system-dependent data for a process' virtual memory
// size and resident set size, and return the results in KB.
//
// On failure, returns 0.0, 0.0

void process_mem_usage(double& vm_usage, double& resident_set)
{
  using std::ios_base;
  using std::ifstream;
  using std::string;

  vm_usage     = 0.0;
  resident_set = 0.0;

  // 'file' stat seems to give the most reliable results
  //
  ifstream stat_stream("/proc/self/stat",ios_base::in);

  // dummy vars for leading entries in stat that we don't care about
  //
  string pid, comm, state, ppid, pgrp, session, tty_nr;
  string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  string utime, stime, cutime, cstime, priority, nice;
  string O, itrealvalue, starttime;

  // the two fields we want
  //
  unsigned long vsize;
  long rss;

  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
	      >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
	      >> utime >> stime >> cutime >> cstime >> priority >> nice
	      >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

  stat_stream.close();

  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
  vm_usage     = vsize / 1024.0;
  resident_set = rss * page_size_kb;
}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

/* Callback function to shift projection result */
/*
void adjust_xyCallback(const autoware_msgs::AdjustXY::ConstPtr &config_msg)
{
  adjust_proj_x = config_msg->x;
  adjust_proj_y = config_msg->y;
}
*/
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


  //std::cerr << " get transform from " << from_frame << " to " << to_frame << "\n"; 
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


  // std::cerr << "cam pos = " << pos.x() << " " << pos.y() << "\n"; 
  
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


void print_line_string_points(lanelet::ConstLineString3d ls)
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


void delete_linestring_marker(visualization_msgs::Marker& points, visualization_msgs::Marker& line_strip , std::string ns, int lane_id){
  
  points.header.frame_id = line_strip.header.frame_id = "map";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();

  points.ns = line_strip.ns;
  points.id = lane_id*10+0;
  line_strip.id = lane_id*10+1;
  points.action = line_strip.action = visualization_msgs::Marker::DELETE;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0; //(*i).z();

  for (int i = 0; i < 100; i++) {points.points.push_back(p);
  line_strip.points.push_back(p);
  }
  
}

void build_location_marker(int id, lanelet::BasicPoint2d location, visualization_msgs::Marker& m,
			   std::string frame_id, std::string ns, float r, float g, float b, float s= 0.15)
{
  
  
  m.header.frame_id = frame_id;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.action = visualization_msgs::Marker::ADD;

  m.type = visualization_msgs::Marker::POINTS;
  
  m.pose.orientation.w = 1.0;
  m.id = id;
  m.scale.x = s;
  m.scale.y = s;

  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0f;
  geometry_msgs::Point p;
  p.x = location.x();
  p.y = location.y();
  p.z = 118.0; //(*i).z();
  m.points.push_back(p);
}



void visualize_lanelet_linestring(int lane_id, lanelet::ConstLineString3d ls, visualization_msgs::Marker& points,
				  visualization_msgs::Marker& line_strip,
				  std::string frame_id, std::string ns, float lr, float lg, float lb, float lss= 0.1)
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
    p.z = 118.0; //(*i).z();
    points.points.push_back(p);
    line_strip.points.push_back(p);
  }
}



void delete_lanelet(ros::Publisher &pub,
		    std::string ns, int id){


  visualization_msgs::Marker del_marker;

  del_marker.action = visualization_msgs::Marker::DELETE;
  del_marker.id = (id*10)*10+1;
  del_marker.ns = ns;
  pub.publish(del_marker);
  del_marker.id = (id*10+1)*10+1;
  
  pub.publish(del_marker);
}

  
void visualize_lanelet(lanelet::ConstLanelet & lanelet, ros::Publisher &pub,
		       float *lline, float *rline, std::string ns, int id){
  visualization_msgs::Marker left_line_strip, left_points, right_line_strip, right_points;

  lanelet::ConstLineString3d left_ls = lanelet.leftBound();
  lanelet::ConstLineString3d right_ls = lanelet.rightBound();
	  

  visualize_lanelet_linestring(id*10+0, left_ls, left_points, left_line_strip, "map", "left_lane_bound", lline[0], lline[1], lline[2], lline[3]);
  visualize_lanelet_linestring(id*10+1,right_ls, right_points, right_line_strip, "map", "right_lane_bound", rline[0], rline[1], rline[2], rline[3]);
  
  pub.publish(left_points);
  pub.publish(left_line_strip);
  pub.publish(right_points);
  pub.publish(right_line_strip);
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



void bc_transform()
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(10, 0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));

}
//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

int main (int argc, char **argv)
{

  // UTM 1KM BLOCK
  lanelet::Origin origin({49, 8.4});

  // origin not used in projector but projector template requires one
  lanelet::projection::RosUtmProjector ros_projector(origin);
  lanelet::ErrorMessages errors;
  

  double vm, rss, vm1, rss1;

  //  process_mem_usage(vm, rss);
  // std::cerr << "before VM: " << vm << "; RSS: " << rss << std::endl;


  lanelet::LaneletMapPtr map = load(example_map_path, ros_projector, &errors);

  //  process_mem_usage(vm1, rss1);
  //  std::cerr << "after VM: " << vm1 << "; RSS: " << rss1 << std::endl;
  // std::cerr << "duff VM: " << vm1-vm << "; RSS: " << rss1-rss << std::endl;
  // std::cerr << " sizeof map = " << sizeof(*map) << "\n";
  //  lanelet::PointLayer& pl = map->pointLayer;
  // std::cerr << " sizeof map = " << sizeof(*map) << " << pl = " << sizeof(pl) << "\n";

  // assert(errors.empty());  // error when loading Peoria OSM map - but seems to load

  ros::init(argc, argv, "test_lanelet_node");

  ros::NodeHandle rosnode;
  ros::NodeHandle private_nh("~");

  // publisher to visualise lanelet elements within rviz
  ros::Publisher marker_pub = rosnode.advertise<visualization_msgs::Marker>("lanelet_reachable_marker", 100);
  ros::Publisher marker_array_pub = rosnode.advertise<visualization_msgs::MarkerArray>("lanelet_left_ls_marker_array", 100);

  std::string cameraInfo_topic_name;

  private_nh.param<std::string>("camera_info_topic", cameraInfo_topic_name, "/camera_info");

  /* get camera ID */
  camera_id_str = cameraInfo_topic_name;
  camera_id_str.erase(camera_id_str.find("/camera_info"));
  // if (camera_id_str == "/")
  //   {
      camera_id_str = "camera";
      //  }
  
  
  // subcribe to camera info & image etc
    ros::Subscriber cameraInfoSubscriber = rosnode.subscribe(cameraInfo_topic_name, 100, cameraInfoCallback);
  ros::Subscriber cameraImage = rosnode.subscribe(cameraInfo_topic_name, 100, cameraInfoCallback);

  
  // publisher to pub regions of interest: ie areas in image where traffic lights should be
  
  ros::spinOnce();
  
  // main loop
  int count = 0;

  ros::Rate loop_rate(10);
  
  int last_lane_count = 0;



  lanelet::traffic_rules::TrafficRulesPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,
													      lanelet::Participants::Vehicle);

  lanelet::routing::RoutingGraphUPtr routing_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
  int loop_count = 0;


  int size_prev_reachable_set = 0;
  
  while(ros::ok())
    {

      //      bc_transform();
      // std::cerr << ".";
      ros::spinOnce();

      visualization_msgs::Marker del_marker;
      del_marker.action = visualization_msgs::Marker::DELETEALL;
    
      visualize_lanelet_map(marker_array_pub, *map);


      
      //      lanelet::BasicPoint2d camera_position_2d(80529.8, 7244.68); // original
      //     lanelet::BasicPoint2d camera_position_2d(80529.8, 7244.68);


      // good position on four lane (2 each way, with an intersection ahead

      
           try {

	// need both trasnform directions - cam to map for projecting into camera space
	//    and map to camera for searching in lanelet map coordinates (use as camera position)
	get_transform(camera_id_str, "map", orientation, position, camera_to_map_tf);
       	get_transform("map", camera_id_str, orientation, position, map_to_camera_tf);
	
      }
      catch (tf::TransformException &exc) {}
	   
	   lanelet::BasicPoint2d camera_position_2d(position.x(), position.y());
	   
      
      // get n nearest lanelets to camera pose. Which lanelets are actually being used (which are reachable from current lane - is nearest even the same as the current lane? need to discard unneeded lanelets - ie not on the current way path 
      int n_lanelets = 1;
      lanelet::Lanelets nearest_lanelets = map->laneletLayer.nearest(camera_position_2d, n_lanelets);
      
      
	  
      int viz_near_lanelets = true;
      
      //	  std::cerr << "number opf lanelets = " << nearest_lanelets.size();
      //publish nearest lanelets as markers for rviz
      
      int lane_count = 0;
      
      visualization_msgs::Marker current_location_marker;
      build_location_marker(1, camera_position_2d, current_location_marker,
      			    "map", "current_location", 1.0f, 1.0f, 1.0f, 1.0f);



      std::vector < std::pair<double, lanelet::Lanelet>> actual_nearest_lanelets;
      actual_nearest_lanelets.clear();
      actual_nearest_lanelets = lanelet::geometry::findNearest(map->laneletLayer, camera_position_2d, 1);
      
      std::cerr << "found " << actual_nearest_lanelets.size() << " nearest\n";

      int n_ll = (int) actual_nearest_lanelets.size();
      
      if (viz_near_lanelets) {

	//	for (auto i = actual_nearest_lanelets.begin(); i != actual_nearest_lanelets.end(); i++) {
	for (int i = 0; i < n_ll; i++) {
	  
	  lanelet::ConstLanelet ll = actual_nearest_lanelets[i].second;
	  
	  std::cerr << "Nearest lanelet = " << ll <<"  id = " << ll.id() << "\n";
	  
	  // print out attributes of nearest lanelet



	  lanelet::AttributeMap attr_map = ll.attributes();

	 
	  for (const auto& elem : attr_map)
	    {
	      
	      std::cerr << "Attr: " << elem.first << "\t\t " << elem.second.value() << "\n";
	    }

	  bool road_found = false;
 	  if (ll.hasAttribute(lanelet::AttributeName::Subtype)){
	      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
	      std::cerr <<" extraced attribute: " << attr.value() << "\n";
	      if (attr.value() == lanelet::AttributeValueString::Road) {
		road_found = true;
	      }
	    }



	  if (road_found) {
	    lanelet::ConstLanelet cll = map->laneletLayer.get(ll.id());
	 
	    lanelet::ConstLineString3d left_ls = ll.leftBound();
	    lanelet::ConstLineString3d right_ls = ll.rightBound();
	    
	  //print_line_string_points(left_ls);
	    
	    
	    
	  //if (traffic_rules->canPass(ll)) { std::cerr << "can pass lane\n"; }

	  lanelet::ConstLanelets following_ll = routing_graph->following(ll);
	  //  std::cerr << "size of following ll = " << following_ll.size() << "\n";
	  

	  //	  std::cerr << "road width in lanes  = " << routing_graph->besides(ll).size() << "\n";
	  
	  if (!!routing_graph->left(ll)) {
	    std::cerr << "left\n";
	  }

	  
	  lanelet::ConstLanelets alefts_ll = routing_graph->adjacentLefts(ll);
	  // std::cerr << "size of alefts ll = " << alefts_ll.size() << "\n";

	  
	  // when query may be empty need to use optional type
	  lanelet::Optional<lanelet::ConstLanelet> right_ll = routing_graph->right(ll);

	  visualization_msgs::Marker left_line_strip, left_points, right_line_strip, right_points;

	  //marker_pub.publish(current_location_marker);

	  float lline[4] = {1.0f, 1.0f, 0.0f, 0.8f}; float rline[4] =  { 1.0f, 1.0f, 0.0f, 0.8f};

	  //marker_pub.publish(del_marker);
	  ros::spinOnce();

	  
	  visualization_msgs::Marker fleft_line_strip, fleft_points, fright_line_strip, fright_points;

	  lline[3] = rline[3] = 0.3f;
	   int lcount = 0;
	 
	   //	  visualize_lanelet(*(following_ll.begin()), marker_pub, lline, rline, "foll_lane", 1);
	  //int lcount = 0;
	  lline[0] = 0.0;  rline[1] = 1.0; rline[2] = 0.0;lline[3] = 0.2;
	  rline[0] = 0.0; rline[1] = 1.0; rline[2] = 0.0; lline[3] = 0.2;
	  for (auto li = alefts_ll.begin(); li != alefts_ll.end(); li++) {
	    lanelet::ConstLanelet lll = *li;
	    //  visualize_lanelet(lll, marker_pub, lline, rline, "left_lanes", 100+lcount);
	    lcount++;
	  }
	  
	  //	  std::cerr << "<2>";
	  lanelet::ConstLanelets reachable_ll = routing_graph->reachableSet(ll, 5, 1);
	  lcount = 0;
	  lline[0] = 0.5;  lline[1] = 1.0; lline[2] = 0.0;lline[3] = 0.7;
	  rline[0] = 0.5; rline[1] = 1.0; rline[2] = 0.0; rline[3] = 0.7;


	  	  int max_size = 100;

	  //if (lcount < max_size) {

		  marker_pub.publish(del_marker);
		  /*
		  for (int di = 0; di < 10; di++) {
		    for (int mi = 0; mi < max_size; mi++) {
		      //	      std::cerr << "need to delete markers " << mi << "\n";
		      delete_lanelet(marker_pub, "left_lanes", 200+mi);
	 
	    }
		  }
		  */
	    // }
 	
	  for (auto li = reachable_ll.begin(); li != reachable_ll.end(); li++) {
	    lanelet::ConstLanelet lll = *li;

	    if (lcount < max_size) visualize_lanelet(lll, marker_pub, lline, rline, "left_lanes", 200+lcount);
	    lcount++;
	  }
	  //	  int size_rs = lcount;
	  
	  //if (lcount < size_prev_reachable_set) {
	    size_prev_reachable_set = lcount; 
	
	  
	    visualize_lanelet(ll, marker_pub, lline, rline, "ll_lane", 1);
	    
	    lane_count++;

	    i = n_ll; // only want first lanelet of subtype road
	  } // if round_found
	}
      }
      
      /*
      
      // actual nearest!
      std::vector < std::pair<double, lanelet::Lanelet>> actual_nearest_lanelets;
      actual_nearest_lanelets.clear();
      actual_nearest_lanelets = lanelet::geometry::findNearest(map->laneletLayer, camera_position_2d, 1);
	  
      std::cerr << "found " << actual_nearest_lanelets.size() << " nearest\n";


      
      //last_lane_count = 20;
      for (int ii =0; ii < 100;  ii++) {
	visualization_msgs::Marker pt, ls;
	delete_linestring_marker(pt, ls, "anleft_lane_bound", ii);
	marker_pub.publish(pt);
	marker_pub.publish(ls);
	
	delete_linestring_marker(pt, ls, "anright_lane_bound", ii);
	marker_pub.publish(pt);
	    marker_pub.publish(ls);
	    //  std::cerr << "deleted marker " << i << "\n";
	  }
	  //last_lane_count = lane_count;

	  
	  
	  for (auto i = actual_nearest_lanelets.begin(); i != actual_nearest_lanelets.end(); i++) {
	    lanelet::Lanelet ll = i->second;
	      
	    std::cerr << "Nearest lanelet = " << ll << "\n";
	      lanelet::ConstLineString3d left_ls = ll.leftBound();
	      lanelet::ConstLineString3d right_ls = ll.rightBound();
	      
	      visualization_msgs::Marker left_line_strip, left_points, right_line_strip, right_points;
	      
	      visualize_lanelet_linestring(lane_count, left_ls, left_points, left_line_strip, "map", "anleft_lane_bound", 1.0f, 1.0f, 0.0f, 0.25);
	      visualize_lanelet_linestring(lane_count,right_ls, right_points, right_line_strip, "map", "anright_lane_bound",1.0f, 0.0f, 1.0f, 0.25);
	      
	      marker_pub.publish(left_points);
	      marker_pub.publish(left_line_strip);
	      marker_pub.publish(right_points);
	      marker_pub.publish(right_line_strip);
isuali	      
	      lane_count++;
	   
	  }
	  
	  // find traffic lights by either:
	  
	  //    - lanelet: use closest lanelets related regulatory elements
	  //    - geomtry: get regulatory elements closes to current position
	  //    - all reg elems: set n_reg_elems high


	  //      int find_tl_mode = TL_BY_LANELET;
	  int find_tl_mode = TL_BY_NEAREST;
	  
	  std::vector<lanelet::TrafficLight::Ptr> nearest_tl_re;
	  
	  if (find_tl_mode == TL_BY_LANELET) {
	    
	    
	    int lane_count = 0;
	    for (auto i = nearest_lanelets.begin(); i != nearest_lanelets.end(); i++) {
	      
	      lanelet::Lanelet ll = *i;
	      std::vector<lanelet::TrafficLight::Ptr> ll_tl_re = ll.regulatoryElementsAs<lanelet::TrafficLight>();
	      nearest_tl_re.insert(nearest_tl_re.end(), ll_tl_re.begin(), ll_tl_re.end());
	      lane_count++;
	    }
	  }
	  else if (find_tl_mode == TL_BY_NEAREST) {
	    int n_reg_elems = 5;	    
	    std::vector<lanelet::RegulatoryElementPtr> nearest_re = map->regulatoryElementLayer.nearest(camera_position_2d, n_reg_elems);
	    nearest_tl_re = lanelet::utils::transformSharedPtr<lanelet::TrafficLight>(nearest_re);
	    
	  }
	  
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
	  for (auto tli = nearest_tl_re.begin(); tli != nearest_tl_re.end(); tli++) {
	    
	    lanelet::LineString3d ls;
	    lanelet::LineStringOrPolygon3d lights = (*tli)->trafficLights().front();
	    //  std::cout << "light = " << lights << "\n";
	    //bool is_ls = lights.isLineString(); 
	    if (lights.isLineString()) { // traffic ligths can either polygons or linestrings
	      ls = static_cast<lanelet::LineString3d>(lights);
	      
	      int point_count = 0;
	      for (auto pi = ls.begin(); pi != ls.end(); pi++) {
		lanelet::Point3d p = *pi;
		Eigen::Vector3f signal_center(p.x(), p.y(), p.z());
		Eigen::Vector3f signal_centerx(p.x(), p.y(), p.z()+signalLampRadius); /// ????
		int u,v;
		
		if (viz_tl_signs) {
		  // visualise traffic lights
		  visualization_msgs::Marker marker;
		  make_light_marker(p, marker, "map", "lanelet_tl", tl_count, point_count);
		  
		  if (marker_pub.getNumSubscribers() >0)
		    marker_pub.publish(marker);
		}
		
		point_count++;
		
	      } // for each point pi
	    } // if linestring
	    
	    tl_count++;
	  }
      */
      
    
      
      // ros loop stuff
      loop_rate.sleep();
      ++count;
      loop_count++;
    }
  
  return 0;
}
