/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    } 
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;
std::vector<std::vector<PathPoint>> backup_spirals;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;
  
  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];	
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;
  	
  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if(behavior == STOPPED){

  	int max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  	//cout << "Using backup spirals." << endl;
  	//spirals = backup_spirals;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);  
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];
  	
  //if(spirals.size() == 7 && best_spirals.size() == 7) backup_spirals = spirals; //ADDED

  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag){

	for( int i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);  
	}
	obst_flag = true;
}

///////////////////////////////////////////////////////////////////////////////

double correct_angle(double angle) {
    while(abs(angle) > M_PI) {
        if(angle < -M_PI) angle += 2 * M_PI;
        if(angle > M_PI) angle -= 2 * M_PI;
    }
    return angle;
}

void print_vector(char *name, vector<double> v) {
    printf("%s: ", name);
    for (auto i = v.begin(); i != v.end(); ++i) printf("%f ", *i);
    printf("\n");
}

#define ALMOST_ZERO 0.000001
#define FULL_STOP -0.5
//#define FULL_STOP -1

class Vector2D {

    public:
    
    double x, y;

    Vector2D(double x, double y) {
        this->x = x;
        this->y = y;
    }
    
    Vector2D *sum(Vector2D *v) {
        return new Vector2D(this->x + v->x, this->y + v->y);
    }
    
    Vector2D *subtract(Vector2D *v) {
        return new Vector2D(this->x - v->x, this->y - v->y);
    }
    
    Vector2D *multiply(double k) {
        return new Vector2D(this->x * k, this->y * k);
    }
    
    double dot_product(Vector2D *v) {
        return this->x * v->x + this->y * v->y;
    }
    
    double magnitude() {
        return sqrt(this->x * this->x + this->y * this->y);
    }
    
    double angle() {
        return atan2(this->y, this->x);
    }
    
    double distance(Vector2D *v) {
        return v->subtract(this)->magnitude();
    }
    
    Vector2D *unitary() {
        double m = magnitude();
        if (abs(m) < ALMOST_ZERO) return new Vector2D(0, 0);
        return new Vector2D(this->x / m, this->y / m);
    }

};

Vector2D *polar_to_vector(double magnitude, double angle) {
    return new Vector2D(magnitude * cos(angle), magnitude * sin(angle));
    //return new Vector2D(magnitude * sin(angle), magnitude * cos(angle));
}

double min(double n1, double n2) {
    return n1 < n2 ? n1 : n2;
}

struct Recommendation {
    double steering, speed;
};

class WayPoints {

  public: 
  
  int n_points, all_waypoints_stopped, any_waypoint_stopped;
  Vector2D *location, *central_point, *last_point, *i, *j, *projection;
  vector<Vector2D *> points;
  double avg_speed;

  WayPoints(vector<double> x_points, vector<double> y_points, vector<double> v_points) {
    n_points = x_points.size();
    double x_avg = 0, y_avg = 0;
    avg_speed = 0;
    all_waypoints_stopped = 1;
    any_waypoint_stopped = 0;
    for(int i = 0; i < n_points; i++) {
      points.push_back(new Vector2D(x_points[i], y_points[i]));
      x_avg += x_points[i];
      y_avg += y_points[i];
      avg_speed += v_points[i];
      if(abs(v_points[i]) < ALMOST_ZERO) {
        all_waypoints_stopped = 0;
        any_waypoint_stopped = 1;
      }
    }
    x_avg /= n_points;
    y_avg /= n_points;
    avg_speed /= n_points;
    central_point = new Vector2D(x_avg, y_avg);
    last_point = points[n_points - 1];
    v_points = v_points;
  }
  
  ~WayPoints() {
    delete(location);
    delete(central_point);
    delete(last_point);
  }
  
  double compute_steering_compensation() {
    double max_angle = M_PI * 0.25;
    double angle_compensation = -projection->y * 0.5; //0.25; //0.75; // * 0.5;
    if(angle_compensation > max_angle) angle_compensation = max_angle;
    if(angle_compensation < -max_angle) angle_compensation = -max_angle;
    return angle_compensation;
  }
  
  double compute_speed_compensation() {
    double max_speed = 1.5; //1;
    double offset = 0; //0.5; //-0.5; //-1;
    double speed_compensation = -(projection->x - offset) * 0.15; //0.2; //0.1;
    if(speed_compensation > max_speed) speed_compensation = max_speed;
    if(speed_compensation < -max_speed) speed_compensation = -max_speed;
    //if(speed_compensation < 0) speed_compensation *= 2;
    return speed_compensation;
  }
  
  double regulate_initial_speed(double goal_speed, double current_speed) {
    //return goal_speed;
    double diff_speed = goal_speed - current_speed;
    double max_diff = 0.75; //0.75; //0.5; //1; //2;
    if(diff_speed > max_diff) goal_speed = current_speed + max_diff;
    if(diff_speed < -max_diff) goal_speed = current_speed - max_diff;
    return goal_speed;
  }
  
  Recommendation recommended_to_stop(double current_angle, double current_speed) {
    return Recommendation {
      correct_angle(current_angle), 
      FULL_STOP
      //regulate_initial_speed(FULL_STOP, current_speed)
    };
  }
  
  Recommendation compute_recommendation(Vector2D *location, double current_angle, double current_speed, int n_spirals) {
    this->location = location;
    if(abs(avg_speed) < ALMOST_ZERO || n_spirals == 0) {
    //if(any_waypoint_stopped) {
      return recommended_to_stop(current_angle, current_speed);
    } else {
      Vector2D *direction = last_point->subtract(central_point);
      i = direction->unitary();
      j = new Vector2D(-i->y, i->x);
      Vector2D *d = location->subtract(central_point);
      projection = new Vector2D(d->dot_product(i), d->dot_product(j));
      double steering = correct_angle(direction->angle());
      double steering_compensation = correct_angle(compute_steering_compensation());
      double speed = min(avg_speed, 3);
      double speed_compensation = compute_speed_compensation();
      printf("Recommendation: steering=%f+%f, speed=%f+%f\n", steering, steering_compensation, speed, speed_compensation);
      printf("Current: steering=%f, speed=%f\n", current_angle, current_speed);
      printf("direction=(%f,%f), projection=(%f,%f)\n\n", direction->x, direction->y, projection->x, projection->y);
      if(projection->x > direction->magnitude()) 
        return recommended_to_stop(current_angle, current_speed);
      return Recommendation {
        correct_angle(steering + steering_compensation), 
        regulate_initial_speed(speed + speed_compensation, current_speed)
      };
    }
  }

};

///////////////////////////////////////////////////////////////////////////////

int main ()
{
  unsigned int seed_number = 1978;
  std::srand(seed_number);
  srand(seed_number);

  cout << "starting server" << endl;
  uWS::Hub h;
  
  double new_delta_time;
  int i = 0;
  
  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();
   
  time_t prev_timer;
  time_t timer;
  time(&prev_timer);
 
  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/
  PID pid_steer = PID();
  double max_steer = 1.2; //0.25 * M_PI; // ORIGINAL
  //pid_steer.Init(0.2, 0.05, 0.05, max_steer, -max_steer, 50);
  //pid_steer.Init(0.15, 0.005, 0.05, max_steer, -max_steer, 10); // VERY GOOD!
  //pid_steer.Init(0.25, 0.1, 0.5, max_steer, -max_steer, 10); // GREAT!!!
  //pid_steer.Init(0.25, 0.1, 0.5, max_steer, -max_steer, 10); // GREAT!!! (11 MINUTES)
  //pid_steer.Init(0.25, 0.1, 0.5, max_steer, -max_steer, 10);
  //pid_steer.Init(0.25, 0.01, 0.25, max_steer, -max_steer, 10);
  //pid_steer.Init(0.3, 0.05, 0.3, max_steer, -max_steer, 10); // good turn?
  //pid_steer.Init(0.3, 0.1, 0.2, max_steer, -max_steer, 10); // good turn!
  //pid_steer.Init(0.3, 0.1, 0.2, max_steer, -max_steer, 10);
  //pid_steer.Init(0.25, 0.01, 0.25, max_steer, -max_steer, 10); // good control
  //pid_steer.Init(0.25, 0.01, 0.25, max_steer, -max_steer, 10); // great turn and timing
  //pid_steer.Init(0.25, 0.01, 0.25, max_steer, -max_steer, 10); 
  //pid_steer.Init(0.25, 0.1, 0.4, max_steer, -max_steer, 10); // latest before future point
  //pid_steer.Init(0.2, 0.01, 0.2, max_steer, -max_steer, 10); // very good but it crashes wall
  //pid_steer.Init(0.25, 0.1, 0.5, max_steer, -max_steer, 10); // 22 minutes!!!
  //pid_steer.Init(0.25, 0.05, 0.4, max_steer, -max_steer, 10); // perfect turn
  //pid_steer.Init(0.25, 0.02, 0.4, max_steer, -max_steer, 10); // good enough for speed control and turns
  //pid_steer.Init(0.25, 0.02, 0.4, max_steer, -max_steer); // ORIGINAL
  //pid_steer.Init(0.3, 0.01, 0.4, max_steer, -max_steer);
  //pid_steer.Init(0.3, 0.01, 0.4, max_steer, -max_steer);
  //pid_steer.Init(0.3, 0.01, 0.4, max_steer, -max_steer);
  pid_steer.Init(0.3, 0.01, 0.4, max_steer, -max_steer); //25 minutes!
    
  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/
  PID pid_throttle = PID();
  double max_throttle = 1; //0.75; //1; // ORIGINAL
  double max_break = -1; //-0.25; //-0.25; //-0.15; // ORIGINAL
  //pid_throttle.Init(1, 0.05, 0.0, max_throttle, max_break, 50);
  //pid_throttle.Init(1, 0.075, 0.01, max_throttle, max_break, 50);
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10); // GREAT!!!
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10);
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10); // GREAT!!! (11 MINUTES)
  //pid_throttle.Init(0.20, 0.05, 0.1, max_throttle, max_break, 10);
  //pid_throttle.Init(0.2, 0.05, 0.2, max_throttle, max_break, 10); // good turn but too much
  //pid_throttle.Init(0.1, 0.05, 0.1, max_throttle, max_break, 10); // too low
  //pid_throttle.Init(0.2, 0.1, 0.1, max_throttle, max_break, 10); // good turn but too much
  //pid_throttle.Init(0.2, 0.05, 0.1, max_throttle, max_break, 10); // good timing
  //pid_throttle.Init(0.2, 0.05, 0.1, max_throttle, max_break, 10); // great turn and timing
  //pid_throttle.Init(0.2, 0.05, 0.1, max_throttle, max_break, 10);
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10); // 22 minutes!!!
  //pid_throttle.Init(0.25, 0.05, 0.1, max_throttle, max_break, 10); // perfect turn
  //pid_throttle.Init(0.25, 0.02, 0.1, max_throttle, max_break, 10); // good enough for speed control and turns
  //pid_throttle.Init(0.25, 0.02, 0.1, max_throttle, max_break); // ORIGINAL
  //pid_throttle.Init(0.25, 0.02, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.3, 0.02, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.4, 0.02, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.35, 0.01, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.35, 0.01, 0.2, max_throttle, max_break);
  //pid_throttle.Init(0.25, 0.01, 0.1, max_throttle, max_break);
  //pid_throttle.Init(0.25, 0.01, 0.05, max_throttle, max_break);
  pid_throttle.Init(0.35, 0.01, 0.2, max_throttle, max_break); //25 minutes!
  
  
  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);
    
        if (s != "") {
          
          auto data = json::parse(s);
          
          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");
          
          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];
          
          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;
          
          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          // Save time and compute delta time
          time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          prev_timer = timer;

          ////////////////////////////////////////
          // Steering control 
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines 
          **/
          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error
          double error_steer;
          
          
          double steer_output;
          
          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
          **/
          // modify the following line for step 3
          error_steer = 0;

          Vector2D *location = new Vector2D(x_position, y_position);
          WayPoints way_points = WayPoints(x_points, y_points, v_points);
          double current_steering = correct_angle(yaw);
          int n_spirals = spirals_x.size();
          Recommendation recommendation = way_points.compute_recommendation(location, current_steering, velocity, n_spirals);
          double desired_steering = recommendation.steering;
          double desired_speed = recommendation.speed;
          // The explanation of this calculation is in the README.md file of the github repository, section "Mathematical explanation of my vectorial fields".
          error_steer = correct_angle(desired_steering - current_steering); 
          
          /**
          * TODO (step 3): uncomment these lines 
          **/
          // Compute control to apply
          pid_steer.UpdateError(error_steer);
          //pid_steer.UpdateError(error_steer);
          steer_output = pid_steer.TotalError();

          // Save data
          file_steer.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << error_steer;
          file_steer  << " " << steer_output << endl;
          
          ////////////////////////////////////////
          // Throttle control 
          ////////////////////////////////////////

          /**
          * TODO (step 2): uncomment these lines 
          **/
          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          double error_throttle;
          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          // modify the following line for step 2
          error_throttle = 0;
          // The explanation of this calculation is in the README.md file of the github repository, section "Mathematical explanation of my vectorial fields".
          error_throttle = desired_speed - velocity;
          
          
          double throttle_output;
          double brake_output;

          /**
          * TODO (step 2): uncomment these lines 
          **/
          // Compute control to apply
          pid_throttle.UpdateError(error_throttle);
          //pid_throttle.UpdateError(error_throttle);
          double throttle = pid_throttle.TotalError();

          // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = -throttle;
          }

          // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << brake_output;
          file_throttle  << " " << throttle_output << endl;

          
          // Send control
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;
         
          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4 
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();
          
          i = i + 1;
          file_steer.close();
          file_throttle.close();
  
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); 
         
    } 

  });

    
  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
  {
      cout << "Connected!!!" << endl;
    });

  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) 
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    } 
  else 
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }
    

}
