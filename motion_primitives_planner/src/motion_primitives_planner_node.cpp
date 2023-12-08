#include "motion_primitives_planner/motion_primitives_planner_node.hpp"
#include <cmath>

bool all_collisions = false;

/* ----- Class Functions ----- */
MotionPlanner::MotionPlanner(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscriber
  subOccupancyGrid = nh.subscribe("/map/local_map/obstacle",1, &MotionPlanner::CallbackOccupancyGrid, this);
  // Publisher
  pubSelectedMotion = nh_.advertise<sensor_msgs::PointCloud2>("/points/selected_motion", 1, true);
  pubMotionPrimitives = nh_.advertise<sensor_msgs::PointCloud2>("/points/motion_primitives", 1, true);
  pubCommand = nh_.advertise<ackermann_msgs::AckermannDrive>("/car_1/command", 1, true);
  
};

MotionPlanner::~MotionPlanner() 
{    
    ROS_INFO("MotionPlanner destructor.");
}

/* ----- ROS Functions ----- */

void MotionPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
  // Subscribe to the map messages
  localMap = msg;
  // The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
  this->origin_x = msg.info.origin.position.x;
  this->origin_y = msg.info.origin.position.y;
  // Frame id of the map
  this->frame_id = msg.header.frame_id;
  // The map resolution [m/cell]
  this->mapResol = msg.info.resolution;
  // message flag
  bGetMap = true;
}

void MotionPlanner::PublishSelectedMotion(std::vector<Node> motionMinCost)
{
  // publish selected motion primitive as point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto motion : motionMinCost) {
    pcl::PointXYZI pointTmp;
    pointTmp.x = motion.x;
    pointTmp.y = motion.y;
    cloud_in_ptr->points.push_back(pointTmp);
  }

  sensor_msgs::PointCloud2 motionCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionCloudMsg);
  motionCloudMsg.header.frame_id = this->frame_id;
  motionCloudMsg.header.stamp = ros::Time::now();
  pubSelectedMotion.publish(motionCloudMsg);
}

void MotionPlanner::PublishMotionPrimitives(std::vector<std::vector<Node>> motionPrimitives)
{
  // publish motion primitives as point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto& motionPrimitive : motionPrimitives) {
    for (auto motion : motionPrimitive) {
      pcl::PointXYZI pointTmp;
      pointTmp.x = motion.x;
      pointTmp.y = motion.y;
      cloud_in_ptr->points.push_back(pointTmp);
    }
  }
  
  sensor_msgs::PointCloud2 motionPrimitivesCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionPrimitivesCloudMsg);
  motionPrimitivesCloudMsg.header.frame_id = this->frame_id;
  motionPrimitivesCloudMsg.header.stamp = ros::Time::now();
  pubMotionPrimitives.publish(motionPrimitivesCloudMsg);
}

void MotionPlanner::PublishCommand(std::vector<Node> motionMinCost)
{
  /*
    TODO: Publish control commands
    - Two components in the AckermannDrive message are used: steering_angle and speed.
    - Compute steering angle using your controller or other method.
    - Calculate proper speed command
    - Publish data  
  */

  //double steeringControl = atan2(motionMinCost.back().y - motionMinCost.front().y, motionMinCost.back().x - motionMinCost.front().x);
  //double steeringControl = 0.0;
  //for (const auto& node : motionMinCost) {
    // Extract the steering delta from the current node

  double steeringControl = motionMinCost.front().delta; //- node.yaw;
  //}
  //double steeringControl = motionMinCost.i().delta;
  //double steeringControl = atan2((motionMinCost.front().x - motionMinCost.back().x), (motionMinCost.front().y - motionMinCost.back().y));
  double speedControl = std::max(0.5 * (1 - std::abs(motionMinCost.front().delta) / this->MAX_DELTA), 0.05);//0.5;
  //double speedControl = (1.0 / (1 + motionMinCost.front().delta));

  std::cout << "Collisions: " << all_collisions << std::endl;

  if (all_collisions) {
    steeringControl = 0;
    speedControl = -0.2;

    all_collisions = false;
  }
  
  // Apply saturation limits to control signals if needed

  // Set the computed control signals to the AckermannDrive message
  ackermann_msgs::AckermannDrive command;
  command.steering_angle = steeringControl;
  command.speed = speedControl;
  pubCommand.publish(command);

  std::cout << "command steer/speed : " << command.steering_angle * 180 / M_PI << " " << command.speed << std::endl;
}


/* ----- Algorithm Functions ----- */

void MotionPlanner::Plan()
{
  // Motion generation
  std::vector<std::vector<Node>> motionPrimitives = GenerateMotionPrimitives(this->localMap);
  
  // Select motion
  std::vector<Node> motionMinCost = SelectMotion(motionPrimitives);

  // Publish data
  PublishData(motionMinCost, motionPrimitives);
}

std::vector<std::vector<Node>> MotionPlanner::GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: Generate motion primitives
    - you can change the below process if you need.
    - you can calculate cost of each motion if you need.
  */

  try {
    // initialize motion primitives
    std::vector<std::vector<Node>> motionPrimitives;

    // number of candidates
    int num_candidates = this->MAX_DELTA*2 / this->DELTA_RESOL; // *2 for considering both left/right direction

    // max progress of each motion
    double maxProgress = this->MAX_PROGRESS;
    for (int i=0; i<num_candidates+1; i++) {

      // std::cout << "i: " << i;

      // current steering delta
      double angle_delta = this->MAX_DELTA - i * this->DELTA_RESOL;

      // init start node
      Node startNode(0, 0, 0, angle_delta, 0, 0, 0, -1, false);
      
      // rollout to generate motion
      std::vector<Node> motionPrimitive = RolloutMotion(startNode, maxProgress, localMap);

      // add current motionPrimitive
      motionPrimitives.push_back(motionPrimitive);
    }

    return motionPrimitives;
    }
  catch (...) {
    std::cout << "An error occured in GenerateMotionPrimitives()\n";
  }
}

std::vector<Node> MotionPlanner::RolloutMotion(Node startNode,
                                              double maxProgress,
                                              nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: Rollout to generate a motion primitive based on the current steering angle
    - Calculate cost terms here if you need
    - Check collision / sensor range if you need
    1. Update motion node using the current steering angle delta based on the vehicle kinematics equation.
    2. Collision checking
    3. Range checking
  */

  try {
    // Initialize motionPrimitive
    std::vector<Node> motionPrimitive;
    // Init current motion node
    Node currMotionNode(startNode.x, startNode.y, 0, startNode.delta, 0, 0, 0, -1, false);
    // Start from a small progress value
    double progress = this->DIST_RESOL;

    // 1. Update motion node using the current steering angle delta based on the vehicle kinematics equation
    // - while loop until the maximum progress of a motion
    while (progress < maxProgress) {
      // - you can use params in the header file (MOTION_VEL, WHEELBASE, TIME_RESOL)
      // - steering angle value is 'startNode.delta' or 'currMotionNode.delta'
      // x_t+1   := x_t + x_dot * dt
      // y_t+1   := y_t + y_dot * dt
      // yaw_t+1 := yaw_t + yaw_dot * dt
      currMotionNode.x += cos(currMotionNode.yaw) * this->MOTION_VEL * this->TIME_RESOL;
      currMotionNode.y += sin(currMotionNode.yaw) * this->MOTION_VEL * this->TIME_RESOL;
      currMotionNode.yaw += (this->MOTION_VEL)/(this->WHEELBASE) * tan(currMotionNode.delta) * this->TIME_RESOL; //currMotionNode.delta was changed to currMotionNode.yaw
      currMotionNode.delta = startNode.delta; 

      // 2. Collision checking
      // - local to map coordinate transform
      Node collisionPointNode(currMotionNode.x, currMotionNode.y, currMotionNode.yaw, currMotionNode.delta, 0, 0, 0, -1, false);
      Node collisionPointNodeMap = LocalToMapCorrdinate(collisionPointNode);

      if (CheckCollision(collisionPointNodeMap, localMap)) {
        // - Do some process when a collision occurs.
        // - You can save collision information & calculate collision cost here.
        // - You can break and return the current motion primitive or keep generating rollout.
        // For example, you can set a collision flag and break the loop
        motionPrimitive.back().collision = true;

        // Calculate distance to the obstacle (Euclidean distance)
        double distanceToObstacle = std::sqrt(std::pow(currMotionNode.x - collisionPointNodeMap.x, 2)
                                            + std::pow(currMotionNode.y - collisionPointNodeMap.y, 2));

        // Define a maximum distance that corresponds to minimal severity
        const double MAX_COLLISION_DISTANCE = 3.0; // was 5

        // Ensure that the distance is capped at the maximum collision distance
        distanceToObstacle = std::min(distanceToObstacle, MAX_COLLISION_DISTANCE);

        // Calculate collision severity
        double collisionSeverity = 1.0 - (distanceToObstacle / MAX_COLLISION_DISTANCE);
        collisionSeverity = std::max(collisionSeverity, 0.0);  // Ensure that severity is non-negative

        // Assign collision severity as the collision cost
        currMotionNode.cost_colli = collisionSeverity;

        // Break the loop after collision
        break;
      }
      // 3. Range checking
      // - if you want to filter out motion points out of the sensor range, calculate the line-of-sight (LOS) distance & yaw angle of the node
      // - LOS distance := sqrt(x^2 + y^2)
      // - LOS yaw := atan2(y, x)
      // - if LOS distance > MAX_SENSOR_RANGE or abs(LOS_yaw) > FOV*0.5 <-- outside of the sensor range
      double LOS_DIST = sqrt(currMotionNode.x * currMotionNode.x + currMotionNode.y * currMotionNode.y);
      double LOS_YAW = atan2(currMotionNode.y, currMotionNode.x);

      if (LOS_DIST > this->MAX_SENSOR_RANGE || std::abs(LOS_YAW) > this->FOV * 0.5) {
        // - Do some process when out-of-range occurs.
        // - You can break and return the current motion primitive or keep generating rollout.
        // For example, you can set an out-of-range flag and break the loop
        currMotionNode.out_of_range = true;
        break;
      }

      // append collision-free motion in the current motionPrimitive
      motionPrimitive.push_back(currMotionNode);

      // update progress of motion
      progress += this->DIST_RESOL;
    }

    // return the current motion
    return motionPrimitive;
  }
  catch (...) {
    std::cout << "An error occured in RolloutMotion().\n";
  }
}



std::vector<Node> MotionPlanner::SelectMotion(std::vector<std::vector<Node>> motionPrimitives)
{
  // Initialization
  double minCost = 9999999;
  std::vector<Node> motionMinCost; // Initialize as odom

  int num_collisions = 0;

  try {
    // Check size of motion primitives
    if (!motionPrimitives.empty()) {
      // Iterate all motion primitive (motionPrimitive) in motionPrimitives
      for (auto& motionPrimitive : motionPrimitives) {
        // 1. Calculate cost terms
        double cost_dir_weight = 0.1; // Weight for the steering angle deviation cost
        double cost_colli_weight = 1000.0; // Weight for the collision cost

        // Example for calculating cost_dir based on the absolute deviation of the steering angle from a reference angle
        //double reference_steering_angle = 0.0; // This could be a desired or reference steering angle
        double cost_dir = fabs(motionPrimitive.back().delta) ; //- reference_steering_angle

        // Example for calculating cost_colli based on collision checking
        double cost_colli = 0;//25.0; // Initialize collision cost to zero
        for (const auto& node : motionPrimitive) {
          Node nodeMap = LocalToMapCorrdinate(node);
          if (node.collision) {
            // Increase collision cost when collision occurs
            cost_colli += 25;

            num_collisions += 1;
            // std::cout << "----------------- Num Colloisions: " << num_collisions << " --------------------\n";
          //cost_colli = 1 / (sizeof(motionPrimitive));
          }
        }

        // You may add more cost terms based on your specific requirements.

        // 2. Calculate total cost
        double cost_total = cost_dir_weight * cost_dir + cost_colli_weight * cost_colli;

        // 3. Compare & Find minimum cost & minimum cost motion
        if (cost_total < minCost) {
          motionMinCost = motionPrimitive;
          minCost = cost_total;
        }
      }

      // // 4. If all the of them are colliding - reverse out. 
      if ((minCost == 25000) && (num_collisions == 20)){
        std::cout << "****************** all collisions. *************************\n";

        // All paths invalid back up 
        // motionMinCost = -1;
        all_collisions = true;
      } 
    }

    else {
      printf("Motion Primitives are empty.\n");
    }

    // 4. Return minimum cost motion
    std::cout << "Mini Cost: " << minCost;
    return motionMinCost;
  }
  catch (...) {
    std::cout << "An error occured in selectMotion().\n";
  }
}




/* ----- Util Functions ----- */

bool MotionPlanner::CheckCollision(Node currentNodeMap, nav_msgs::OccupancyGrid localMap)
{
  // Check if the position x of the node is in the range [0, map width]
  if (currentNodeMap.x < 0 || currentNodeMap.x >= localMap.info.width) {
    return true;  // Out of map width range
  }

  // Check if the position y of the node is in the range [0, map height]
  if (currentNodeMap.y < 0 || currentNodeMap.y >= localMap.info.height) {
    return true;  // Out of map height range
  }

  // Check all map values within the inflation area of the current node
  for (int i = 0; i < this->INFLATION_SIZE; ++i) {
    for (int j = 0; j < this->INFLATION_SIZE; ++j) {
      // Calculate tmp_x and tmp_y
      double tmp_x = currentNodeMap.x + i - 0.5 * this->INFLATION_SIZE;
      double tmp_y = currentNodeMap.y + j - 0.5 * this->INFLATION_SIZE;

      // Check if tmp_x is in [0, map width]
      if (tmp_x < 0 || tmp_x >= localMap.info.width) {
        continue;  // Skip if out of map width range
      }

      // Check if tmp_y is in [0, map height]
      if (tmp_y < 0 || tmp_y >= localMap.info.height) {
        continue;  // Skip if out of map height range
      }

      // Calculate map index
      int map_index = static_cast<int>(tmp_y) * localMap.info.width + static_cast<int>(tmp_x);

      // Get map value
      int16_t map_value = static_cast<int16_t>(localMap.data[map_index]);

      // Check collision conditions
      if (map_value > this->OCCUPANCY_THRES || map_value < 0) {
        return true;  // Collision detected
      }
    }
  }

  // No collision detected within the inflation area
  return false;
}


bool MotionPlanner::CheckRunCondition()
{
  if (this->bGetMap) {
    return true;
  }
  else {
    std::cout << "Run condition is not satisfied!!!" << std::endl;
    return false;
  }
}

Node MotionPlanner::LocalToMapCorrdinate(Node nodeLocal)
{
  /*
    TODO: Transform from local to occupancy grid map coordinate
    - local coordinate ([m]): x [map min x, map max x], y [map min y, map max y]
    - map coordinate ([cell]): x [0, map width], y [0, map height]
    - convert [m] to [cell] using map resolution ([m]/[cell])
  */
  Node nodeMap;

  // Transform from local (min x, max x) [m] to map (0, map width) [grid] coordinate
  nodeMap.x = static_cast<int>((nodeLocal.x - this->mapMinX) / this->mapResol);
  
  // Transform from local (min y, max y) [m] to map (0, map height) [grid] coordinate
  nodeMap.y = static_cast<int>((nodeLocal.y - this->mapMinY) / this->mapResol);

  // Copy other data from nodeLocal to nodeMap
  nodeMap.yaw = nodeLocal.yaw;
  nodeMap.delta = nodeLocal.delta;
  nodeMap.cost_dir = nodeLocal.cost_dir;
  nodeMap.cost_colli = nodeLocal.cost_colli;
  nodeMap.cost_total = nodeLocal.cost_total;
  nodeMap.idx = nodeLocal.idx;
  nodeMap.collision = nodeLocal.collision;

  return nodeMap;
}



/* ----- Publisher ----- */

void MotionPlanner::PublishData(std::vector<Node> motionMinCost, std::vector<std::vector<Node>> motionPrimitives)
{
  // Publisher
  // - visualize selected motion primitive
  PublishSelectedMotion(motionMinCost);

  // - visualize motion primitives
  PublishMotionPrimitives(motionPrimitives);

  // - publish command
  PublishCommand(motionMinCost);
}

/* ----- Main ----- */

int main(int argc, char* argv[])
{ 
  std::cout << "start main process" << std::endl;

  ros::init(argc, argv, "motion_primitives_planner");
  // for subscribe
  ros::NodeHandle nh;
  ros::Rate rate(50.0);
  MotionPlanner MotionPlanner(nh);

  // Planning loop
  while (MotionPlanner.nh_.ok()) {
      // Spin ROS
      ros::spinOnce();
      // check run condition
      if (MotionPlanner.CheckRunCondition()) {
        // Run algorithm
        MotionPlanner.Plan();
      }
      rate.sleep();
  }

  return 0;

}
