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
#include <autoware_msgs/DTLane.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_msgs/Lane.h>

#include <autoware_msgs/LaneArray.h>

#include <cstdio>

#include <sstream>
#include "rosUTM.h"



#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>

#include <boost/timer/timer.hpp>



// use of routing graph in detecting lanelets for each waypoint
// folowing routing graph may be computationally cheaper if very large map
// (computation cost may be in unique insert used to store connected lanelets)
// for smaller maps - faster to just find nearest lanelet for each waypoint.
#define LANE_RULES_USE_ROUTING_GRAPH false


double config_acceleration = 1;            // m/s^2
double config_stopline_search_radius = 1;  // meter
int config_number_of_zeros_ahead = 0;
int config_number_of_zeros_behind = 0;
int config_number_of_smoothing_count = 0;

int waypoint_max;
double search_radius;  // meter
double curve_weight;
double crossroad_weight;
double clothoid_weight;
std::string frame_id;

ros::Publisher traffic_pub;
ros::Publisher red_pub;
ros::Publisher green_pub;
double curve_radius_min;
double crossroad_radius_min;
double clothoid_radius_min;
autoware_msgs::LaneArray cached_waypoint;

//std::string example_map_path = "/home/simon/work/catkin_ws/src/lanelet2/lanelet2_maps/res/mapping_example.osm";
std::string example_map_path = "/home/simon/work/peoria_data/map/Lanelet2/Peoria_0301_2019_fixed.osm";

static lanelet::LaneletMapPtr lanelet_map;
static lanelet::routing::RoutingGraphUPtr routing_graph;
static ros::Publisher stop_pub;
static ros::Publisher wp_marker_pub;
static ros::Publisher lanelet_pub;

 
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




//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

autoware_msgs::Lane create_new_lane(const autoware_msgs::Lane& lane, const std_msgs::Header& header)
{
  autoware_msgs::Lane l = lane;
  l.header = header;

  for (autoware_msgs::Waypoint& w : l.waypoints)
    {
      w.pose.header = header;
      w.twist.header = header;
    }

  return l;
}



//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------


void  create_waypoint_array(const autoware_msgs::Lane& lane, std::vector<Eigen::Vector3d>& waypoints)
{

  waypoints.clear();
  //std::vector<Eigen::Vector3d> waypoints;
    for (const autoware_msgs::Waypoint& w : lane.waypoints){
    
    // careful: vector map X-Y axis are reversed from geometry point
    // check
      geometry_msgs::Point gp = w.pose.pose.position;
      Eigen::Vector3d p(gp.x, gp.y, gp.z);
    
      waypoints.push_back(p);
      // std::cerr << p << "\n";
    }
    std::cerr << "number of waypoints = " << waypoints.size() << "\n";
    // return waypoints;
    
}


//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

lanelet::Lanelets find_nearest_lanelets(Eigen::Vector3d p, int n)
{

  return(lanelet_map->laneletLayer.nearest(lanelet::BasicPoint2d(p.x(), p.y()), n));
  
}


//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

lanelet::Lanelet find_actual_nearest_lanelet(Eigen::Vector3d p)
{
  std::vector<std::pair<double, lanelet::Lanelet>> actual_nearest_lls = lanelet::geometry::findNearest(lanelet_map->laneletLayer, lanelet::BasicPoint2d(p.x(), p.y()), 1);
  return actual_nearest_lls.front().second;
}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------



void insert_unique_lanelet(lanelet::ConstLanelet ll, lanelet::ConstLanelets & lls)
{
  if (std::find(lls.begin(), lls.end(), ll) == lls.end())
    lls.push_back(ll);

}
//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------


void add_connecting_lanelets(lanelet::ConstLanelet lanelet, lanelet::ConstLanelets & candidate_lanelets)
{
  lanelet::Optional<lanelet::ConstLanelet> ll_opt = routing_graph->right(lanelet);
  lanelet::ConstLanelet ll;
  
  if (!!ll_opt) insert_unique_lanelet(ll_opt.get(), candidate_lanelets);
  ll_opt = routing_graph->left(lanelet);
  if (!!ll_opt) insert_unique_lanelet(ll_opt.get(), candidate_lanelets);
  
  lanelet::ConstLanelets following_ll = routing_graph->following(lanelet);
  for (auto fll_i =following_ll.begin(); fll_i < following_ll.end(); fll_i++)
    insert_unique_lanelet((*fll_i), candidate_lanelets);  
}
 

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------



std::vector<lanelet::ConstLineString3d> check_lanelet_for_stoplines(lanelet::ConstLanelet ll)
{ 
  std::vector<lanelet::ConstLineString3d> stoplines;

  // find stop lines referened by right ofway reg. elems.

  std::vector<std::shared_ptr<const lanelet::RegulatoryElement>> regelem = ll.regulatoryElements();
  // std::cerr << "size of reg elemes = " << regelem.size() << "\n";

  for (auto j = regelem.begin(); j != regelem.end(); j++) {
    lanelet::AttributeMap attr_map = (*j)->attributes();
    std::vector<std::string> rroles = (*j)->roles();


    //  std::cerr << "ref elem id = " << (*j)->id() << "\n";
 
    lanelet::ConstRuleParameterMap param_map = (*j)->getParameters();
    //   std::cerr << "reg elem has " <<  param_map.size() << " rule params \n";


    //    if ((*j)->empty()) std::cerr << "reg elem has no parameters\n";
    
    //    std::cerr << "reg elem has " <<  rroles.size() << "roles \n";
    //  for (auto s : rroles)
    //   std::cerr << "roles = " << s << "\n";

    //    std::cerr << "reg elem has " <<  attr_map.size() << "attributes \n";

    for (const auto& elem : attr_map)    {
      auto key_comp = elem.first;
      auto value_comp = elem.second;
      //    std::cerr << "Attr: " << key_comp << "\t\t" << value_comp << "\n";
      //   msg.attributes.push_back(attr);
      //   std::cerr << "Attr: " << key_comp << "\t\t" <<value_comp.value() << "\n";

    }
    
  }
  
  std::vector<std::shared_ptr<const lanelet::RightOfWay>> right_of_way_reg_elems = ll.regulatoryElementsAs<const lanelet::RightOfWay>();
  
  if (right_of_way_reg_elems.size() > 0)
    {
      ///      std::cerr << "lanelet has right of way " << right_of_way_reg_elems.size() << "\n";

      // lanelet has a right of way elem elemetn
      for (auto j = right_of_way_reg_elems.begin(); j < right_of_way_reg_elems.end(); j++)
	{
	  if ((*j)->getManeuver(ll) == lanelet::ManeuverType::Yield)
	    {
	      // lanelet has a yield reg. elem.
	      lanelet::Optional<lanelet::ConstLineString3d> row_stopline_opt = (*j)->stopLine();
	      if (!!row_stopline_opt) stoplines.push_back(row_stopline_opt.get());
	  }
	  
	}
    }
  
  // find stop lines referenced by traffic lights
  std::vector<std::shared_ptr<const lanelet::TrafficLight>> traffic_light_reg_elems = ll.regulatoryElementsAs<const lanelet::TrafficLight>();
  
  if (traffic_light_reg_elems.size() > 0)
    {
      //      std::cerr << "lanelet has traffic lights " << traffic_light_reg_elems.size() << "\n";
      // lanelet has a traffic light elem elemetn
      for (auto j = traffic_light_reg_elems.begin(); j < traffic_light_reg_elems.end(); j++)
	{
	  
	  lanelet::Optional<lanelet::ConstLineString3d> traffic_light_stopline_opt = (*j)->stopLine();
	  if (!!traffic_light_stopline_opt) stoplines.push_back(traffic_light_stopline_opt.get());
	  
	}
      
    }
  // find stop lines referenced by traffic signs
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> traffic_sign_reg_elems = ll.regulatoryElementsAs<const lanelet::TrafficSign>();
  
  if (traffic_sign_reg_elems.size() > 0)
    {
      //      std::cerr << "lanelet has traffic sign " << traffic_sign_reg_elems.size() << "\n";

      // lanelet has a traffic sign reg elem - can have multiple ref lines (but stop sign shod have 1
      for (auto j = traffic_sign_reg_elems.begin(); j < traffic_sign_reg_elems.end(); j++)
	{
	  
	  lanelet::ConstLineStrings3d traffic_sign_stoplines = (*j)->refLines();
	  if (traffic_sign_stoplines.size() > 0) stoplines.push_back(traffic_sign_stoplines.front());
	  
	}
      
    }
  return stoplines;
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

//--------------------------------------------------------------------------------
//
//
//
//--------------------------------------------------------------------------------

  namespace bg = boost::geometry;
  typedef bg::model::point<double, 2, bg::cs::cartesian> point_t;
  typedef bg::model::segment<point_t> segment_t;

std::vector<size_t>  check_waypoints_for_stoplines(std::vector<Eigen::Vector3d> waypoints)
{
		  
  
  lanelet::ConstLanelet current_lanelet = find_actual_nearest_lanelet(waypoints.front());
  lanelet::ConstLanelets current_lanelets;
  current_lanelets.push_back(current_lanelet);
  std::vector<size_t> waypoint_stopline_indexes;


  
  //float lline[4] = {1.0f, 1.0f, 0.0f, 0.8f}; float rline[4] =  { 1.0f, 0.0f, 1.0f, 0.8f};
  // visualize_lanelet(current_lanelet, lanelet_pub, lline, rline, "ll_lane", 11);

  
  size_t wp_index = 0;

  // TODO:
  // perhaps need to test boundary condition when waypoint is last in lanelet?

  for (auto wp_i = waypoints.begin(); wp_i < waypoints.end()-1; wp_i++)
    {

      lanelet::BasicPoint3d wp_p0((*wp_i).x(),(*wp_i).y(),(*wp_i).z());
      lanelet::BasicPoint2d wp_p02((*wp_i).x(),(*wp_i).y());
      lanelet::BasicPoint3d wp_p1((*(wp_i+1)).x(),(*(wp_i+1)).y(),(*(wp_i+1)).z());
      lanelet::ConstLanelets candidate_lanelets;

      visualization_msgs::Marker m;
      build_location_marker(((int)wp_index), wp_p02, m,
			    "map", "wp", 1.0f, 1.0f, 0.0f, 0.5f);
      // if (wp_index == 0) 
	wp_marker_pub.publish(m);
      
      // check if waypoint is in current laneleyts - if not discard
      int ll_count = 0;
      
      // if no current lanelets - search again
      if (current_lanelets.size() == 0) {
	//current_lanelet = find_actual_nearest_lanelet(*wp_i);
	current_lanelets.push_back(find_actual_nearest_lanelet(*wp_i));
      }
      float lline[4] = {1.0f, 0.0f, 0.0f, 0.4f}; float rline[4] =  { 0.0f, 0.0f, 1.0f, 0.4f};
      visualize_lanelet(current_lanelet, lanelet_pub, lline, rline, "ll_lane", wp_index);

      for (auto curr_i = current_lanelets.begin(); curr_i < current_lanelets.end(); )
	{
	  
	  current_lanelet = (*curr_i);
	  ll_count++;
	  
	  if (!lanelet::geometry::within(wp_p02, lanelet::utils::toHybrid(current_lanelet.polygon2d())))
	    current_lanelets.erase(curr_i);
	  else
	    {
	      
	      std::vector<lanelet::ConstLineString3d> current_lanelet_stoplines = check_lanelet_for_stoplines(current_lanelet);

	      //std::cerr << "current lanelet has " << current_lanelet_stoplines.size() << "stoplines\n";
	      // check if waypoint segment intersects with stoplines
	      // kida awkward to force use of boost::geometry intersect.
	      // 3d line segment intersection not implememented
	      segment_t wp_segment(point_t(wp_p0.x(), wp_p0.y()),
				   point_t(wp_p1.x(), wp_p1.y()));
	      int sl_count = 0;
	      for (auto sl_i = current_lanelet_stoplines.begin(); sl_i < current_lanelet_stoplines.end(); sl_i++)
		{
		  
		  lanelet::ConstHybridLineString3d hsl = lanelet::utils::toHybrid(*sl_i);
		  bool intersection = false;
		  for (size_t sls_i = 0; sls_i < hsl.numSegments(); sls_i++) {
		    lanelet::BasicSegment3d sls = hsl.segment(sls_i);
		    segment_t sl_segment( point_t(sls.first.x(), sls.first.y()),
					  point_t(sls.second.x(), sls.second.y()));
		    intersection = bg::intersects(wp_segment, sl_segment);
		    if (intersection){
		      waypoint_stopline_indexes.push_back(wp_index);
		      break;
		    }
		  }


		  if (intersection) { 
		    visualization_msgs::Marker line_strip, ls_points;
		    std::cerr << "found stopline interesection\n";
		    visualize_lanelet_linestring(sl_count+wp_index*10, *sl_i, ls_points, line_strip, "map", "stop_line", 1.0, 0.0, 0.0, 1.0);
		    stop_pub.publish(ls_points);
		    stop_pub.publish(line_strip);
		    sl_count++;
		  }

		}
	      
	      // build up a list of possible connecting lanelets that might contain current waypoint 
	      if (LANE_RULES_USE_ROUTING_GRAPH) add_connecting_lanelets(current_lanelet, candidate_lanelets);
	      
	      // increment iterator here, erase call increments if wp not in current lanelet 
	      curr_i++; 
	    }
	}
      
      // add candidate lanelets to current for next waypoint (in case path moves to next lanelets)

      if (LANE_RULES_USE_ROUTING_GRAPH) {
	for (auto cand_i = candidate_lanelets.begin(); cand_i < candidate_lanelets.end(); cand_i++)
	  {
	    insert_unique_lanelet((*cand_i), current_lanelets);  
	}
      }
      
      wp_index++;
    } // for each waypoint
  
  
  return waypoint_stopline_indexes;
}

//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------





autoware_msgs::Lane apply_acceleration(const autoware_msgs::Lane& lane, double acceleration, size_t start_index,
				       size_t fixed_cnt, double fixed_vel)
{
  autoware_msgs::Lane l = lane;

  if (fixed_cnt == 0)
    return l;

  double square_vel = fixed_vel * fixed_vel;
  double distance = 0;
  //  std::cerr << "STart index = " << start_index << "\n";
  for (size_t i = start_index; i < l.waypoints.size(); ++i)
    {
      if (i - start_index < fixed_cnt)
	{
	  l.waypoints[i].twist.twist.linear.x = fixed_vel;
	  continue;
	}
      geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
      geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
      distance += hypot(b.x - a.x, b.y - a.y);

      double v = sqrt(square_vel + 2 * acceleration * distance);
      if (v < l.waypoints[i].twist.twist.linear.x)
	{
	  //	  if (i%5==0) std::cerr << "["<< i << "](" << l.waypoints[i].twist.twist.linear.x << ")->" << v << "\t";
	  
	  l.waypoints[i].twist.twist.linear.x = v;
	  
	  
	}
      else
	break;
    }

  return l;
}
//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------






autoware_msgs::Lane apply_stopline_acceleration(const autoware_msgs::Lane& lane,
						std::vector<size_t> stopline_indexes, 
						double acceleration,
						size_t ahead_cnt,
						size_t behind_cnt)
{
  autoware_msgs::Lane l = lane;
  if (stopline_indexes.empty())
    return l;

	  
  
  for (const size_t i : stopline_indexes)
    l = apply_acceleration(l, acceleration, i, behind_cnt + 1, 0);


  std::reverse(l.waypoints.begin(), l.waypoints.end());

  std::vector<size_t> reverse_indexes;
  for (const size_t i : stopline_indexes)
    reverse_indexes.push_back(l.waypoints.size() - i - 1);
  std::reverse(reverse_indexes.begin(), reverse_indexes.end());


  
  
  for (const size_t i : reverse_indexes)
    l = apply_acceleration(l, acceleration, i, ahead_cnt + 1, 0);

  std::reverse(l.waypoints.begin(), l.waypoints.end());

  return l;
}



//-------------------------------------------------------------------------
//
//
//
//-------------------------------------------------------------------------

void create_waypoint(const autoware_msgs::LaneArray& msg)
{

  //  std::cerr << "> create waypoint callback\n";

  boost::timer::cpu_timer lane_rule_timer;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id;
  cached_waypoint.lanes.clear();
  cached_waypoint.lanes.shrink_to_fit();
  cached_waypoint.id = msg.id;


  
  
  for (const autoware_msgs::Lane& l : msg.lanes)
    cached_waypoint.lanes.push_back(create_new_lane(l, header));

  if (lanelet_map->laneletLayer.empty() || lanelet_map->pointLayer.empty() || lanelet_map->regulatoryElementLayer.empty()) {
    
    traffic_pub.publish(cached_waypoint);
    return;
  }
  
  

  
  autoware_msgs::LaneArray traffic_waypoint;
  autoware_msgs::LaneArray red_waypoint;
  autoware_msgs::LaneArray green_waypoint;
  traffic_waypoint.id = red_waypoint.id = green_waypoint.id = msg.id;


  //  std::cerr << "Number of Lanes = " << msg.lanes.size() << "\n";
  
  // for each lane (lane is a collection of waypoints
  for (size_t i = 0; i < msg.lanes.size(); ++i)
    {

      autoware_msgs::Lane waypoint_lane = create_new_lane(msg.lanes[i], header);


      // choose to us eigne vectors -> perhaps a lanelet data type better?
      std::vector<Eigen::Vector3d> waypoints;
      create_waypoint_array(waypoint_lane, waypoints);
      if (waypoints.size() < 2)
	{
	
	  traffic_waypoint.lanes.push_back(waypoint_lane);
	  continue;
	}
      
      
      
      lanelet::Lanelets nearest_lanelets = find_nearest_lanelets(waypoints.front(),1);
      if (nearest_lanelets.size() <= 0)
	{
	  traffic_waypoint.lanes.push_back(waypoint_lane);
	  continue;
	}
      
      

      //velocity smoothing
      for (int k = 0; k < config_number_of_smoothing_count; ++k)
	{
	  autoware_msgs::Lane temp_lane = waypoint_lane;
	  if (waypoint_lane.waypoints.size() >= 3)
	    {
	      for (size_t j = 1; j < waypoint_lane.waypoints.size() - 1; ++j)
		{
		  if (waypoint_lane.waypoints.at(j).twist.twist.linear.x != 0)
		    {
		      waypoint_lane.waypoints[j].twist.twist.linear.x =
			(temp_lane.waypoints.at(j - 1).twist.twist.linear.x + temp_lane.waypoints.at(j).twist.twist.linear.x +
			 temp_lane.waypoints.at(j + 1).twist.twist.linear.x) /
			3;
		    }
		}
	    }
	}
      
      


      // check if any waypoint segments are intersected by stoplines
      std::vector<size_t> waypoint_stopline_indexes =  check_waypoints_for_stoplines(waypoints);
      
      traffic_waypoint.lanes.push_back(waypoint_lane);
      green_waypoint.lanes.push_back(waypoint_lane);

      // apply acceleration to waypoint velocities to consider stoplines
      waypoint_lane = apply_stopline_acceleration(waypoint_lane, waypoint_stopline_indexes, config_acceleration, config_number_of_zeros_ahead, config_number_of_zeros_behind);
      
      red_waypoint.lanes.push_back(waypoint_lane);
     
    }
  
  // publish traffic waypoint
  traffic_pub.publish(traffic_waypoint);
  red_pub.publish(red_waypoint);
  green_pub.publish(green_waypoint);
  
  std::cerr << "lane rule timer = " << lane_rule_timer.format() << "\n";
    

  
}


//-------------------------------------------------------------------------
//
//
  // follows logic of vector map based implmenentation
  // but ignores acceleration adjustment for cross roads
  // and does not reduce velocity for curvature
  // assumed that this is implemented in local planner before
  // path is published
  // -> do only stopline acceleration control

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

  process_mem_usage(vm, rss);
  std::cerr << "before VM: " << vm << "; RSS: " << rss << std::endl;
  boost::timer::cpu_timer load_timer;
  lanelet_map = load(example_map_path, ros_projector, &errors);

  std::cerr << "load timer = " << load_timer.format() << "\n";

  
  process_mem_usage(vm1, rss1);
  
  std::cerr << "after VM: " << vm1 << "; RSS: " << rss1 << std::endl;
  std::cerr << "duff VM: " << vm1-vm << "; RSS: " << rss1-rss << std::endl;

  // assert(errors.empty());  // error when loading Peoria OSM map - but seems to load

  ros::init(argc, argv, "lanelet_lr_node");
  ros::NodeHandle rosnode;
  // parameters from ros param

  int sub_vmap_queue_size;
  rosnode.param<int>("/lane_rule/sub_vmap_queue_size", sub_vmap_queue_size, 1);
  int sub_waypoint_queue_size;
  rosnode.param<int>("/lane_rule/sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
  int sub_config_queue_size;
  rosnode.param<int>("/lane_rule/sub_config_queue_size", sub_config_queue_size, 1);
  int pub_waypoint_queue_size;
  rosnode.param<int>("/lane_rule/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
  bool pub_waypoint_latch;
  rosnode.param<bool>("/lane_rule/pub_waypoint_latch", pub_waypoint_latch, true);



  traffic_pub =
    rosnode.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
  red_pub = rosnode.advertise<autoware_msgs::LaneArray>("/red_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
  green_pub =
    rosnode.advertise<autoware_msgs::LaneArray>("/green_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
   
  ros::Subscriber waypoint_sub = rosnode.subscribe("/lane_waypoints_array", sub_waypoint_queue_size, create_waypoint);
  
  // publisher to visualise lanelet elements within rviz

  stop_pub = rosnode.advertise<visualization_msgs::Marker>("/lanelet_stop_line_marker", 100);
   wp_marker_pub = rosnode.advertise<visualization_msgs::Marker>("/test_wp_marker", 2000);
   lanelet_pub = rosnode.advertise<visualization_msgs::Marker>("lanelet_current_marker", 100);

  ros::spinOnce();
  
  // main loop
  ros::Rate loop_rate(10);


  Eigen::Vector3d p0(81012.5, 7316.45, 106.518);
  Eigen::Vector3d p1(81013, 7315.53, 106.51);
  std::vector<Eigen::Vector3d> test_wp;
  test_wp.push_back(p0);
  test_wp.push_back(p1);
  //  std::vector<size_t> waypoint_stopline_indexes =  check_waypoints_for_stoplines(test_wp);
  //  exit(1);

  if (LANE_RULES_USE_ROUTING_GRAPH)
    {

      lanelet::traffic_rules::TrafficRulesPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,
														  lanelet::Participants::Vehicle);
      
      boost::timer::cpu_timer timer;
      routing_graph = lanelet::routing::RoutingGraph::build(*lanelet_map, *traffic_rules);
      
      timer.stop();
      std::cerr << " timer = " << timer.format() << "\n";
    }

  int loop_count = 0;
  while(ros::ok())
    {
      //    std::vector<size_t> waypoint_stopline_indexes =  check_waypoints_for_stoplines(test_wp);

      ros::spinOnce();
      loop_count++;
    }
  
  return 0;
}
