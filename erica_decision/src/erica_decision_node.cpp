
/*
 * erica_decision_node.cpp
 *
 *  Created on: Jan 8, 2019
 *      Author: robotemperor
 */

#include <erica_decision/erica_decision_node.h>

void initialize()
{
  simulation_robot_speed = 0.0;
  robot_trj_time  = 0.0;
  detect_distance = 0.0;
  people_detection_check = false;
  people_detection_check_lidar = false;
  rotation_check = false;
  head_yaw_position = 0.0;
  lidar_detect_angle = 0.0;
  lidar_detect_distance = 0.0;
  sampling_count = 0;
  lidar_sampling_count = 0;
  simulation_check = false;
  action_movement_done_check =false;
  rotation_done_check = false;
  action_count = 1;

  fifth_trj_x = new heroehs_math::FifthOrderTrajectory();
  fifth_trj_y = new heroehs_math::FifthOrderTrajectory();

  goal_desired_vector_x = 0.0;
  goal_desired_vector_y = 0.0;
}

//callback
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if((ros::Time::now() - count).toSec() > 0.03)
  {
    if(pow(msg->axes[0],2)+pow(msg->axes[1],2) > 0.01)
    {
      goal_desired_vector_x = msg->axes[1];
      goal_desired_vector_y = msg->axes[0];
    }
    else
    {
      goal_desired_vector_x = 0;
      goal_desired_vector_y = 0;
    }
  }
}
void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   for(int angle_number = 0; angle_number < lidar_detect_angle; angle_number++)
  {
    if((double) msg->ranges[angle_number] < lidar_detect_distance) // CW check
    {
      sampling_count ++;
    }
    if((double) msg->ranges[359-angle_number] < lidar_detect_distance) // CCW check
    {
      sampling_count ++;
    }
  }
  // ROS_INFO("s_count :: %d \n", sampling_count);
  if(sampling_count > lidar_sampling_count)
  {
    sampling_count = 0;
    people_detection_check_lidar = true;
    return;
  }

  //  ROS_INFO("sampling_count :: %d \n", sampling_count);
  sampling_count = 0;
  //rotation_check = false;
  people_detection_check_lidar = false;
}
void people_position_callback(const erica_perception_msgs::PeoplePositionArray::ConstPtr& msg)
{
  double temp_distance = 0.0;

  if(msg->people_position.size() == 0)
  {
    goal_desired_vector_x = 0;
    goal_desired_vector_y = 0;
    return;
  }
  else
  {
    temp_distance = sqrt(pow(fabs(msg->people_position[0].x),2)+ pow(fabs(msg->people_position[0].y),2));

    if(std::isnan(temp_distance)) //when there is no people, the robot stops.
    {
      goal_desired_vector_x = 0;
      goal_desired_vector_y = 0;
      arrivals_action_command_msg.data = 0;
      return;
    }

    goal_desired_vector_x = (double) msg->people_position[0].x;
    goal_desired_vector_y = (double) msg->people_position[0].y;

    for(int people_num = 1; people_num < msg->people_position.size(); people_num ++) // Select closed people
    {
      if((pow((double) msg->people_position[people_num].x,2) +  pow((double) msg->people_position[people_num].y,2)) < (pow(desired_vector_msg.position.x,2)+pow(desired_vector_msg.position.y,2)))
      {
        goal_desired_vector_x = (double) msg->people_position[people_num].x;
        goal_desired_vector_y = (double) msg->people_position[people_num].y;
      }
    }
  }
  // if person is close, the robot keeps going or stops.
  if((temp_distance <= 0.7))
  {
    people_detection_check = true;
    return;
  }
  if(temp_distance <= detect_distance && temp_distance >=0.7)// detect_distance initial value 1.0
  {
    //unit vector
    goal_desired_vector_x = goal_desired_vector_x/temp_distance;
    goal_desired_vector_y = goal_desired_vector_y/temp_distance;
    //
    rotation_check = false;
    people_detection_check = false;
  }
  else
  {
    rotation_check = false;
    people_detection_check = false;
    goal_desired_vector_x = 0;
    goal_desired_vector_y = 0;
  }
}
void present_joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int joint_num=0; joint_num < msg->name.size(); joint_num ++)
  {
    if(!msg->name[joint_num].compare("head_yaw")) // if it matches head_yaw, return 0, so added !
    {
      head_yaw_position = msg->position[joint_num];
      return;
    }
  }
}

void movement_done_callback(const std_msgs::String::ConstPtr& msg)
{
  //action_movement_done_check = true;
  //action_count ++;
}
//simulation
void simulation_rviz(geometry_msgs::Pose desired_vector) // cpp 분리
{
  desired_vector_rviz_msg.header.frame_id = "map";

  desired_vector_rviz_msg.pose.orientation.z = atan2(desired_vector.position.y, desired_vector.position.x);

  rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,desired_vector_rviz_msg.pose.orientation.z);

  desired_vector_rviz_msg.pose.orientation.x = rqyToQ.x();
  desired_vector_rviz_msg.pose.orientation.y = rqyToQ.y();
  desired_vector_rviz_msg.pose.orientation.z = rqyToQ.z();
  desired_vector_rviz_msg.pose.orientation.w = rqyToQ.w();
}



int main (int argc, char **argv)
{
  ros::init(argc, argv, "erica_decision_node");
  ros::NodeHandle nh;

  initialize();

  //launch initialize
  nh.param("simulation_check",  simulation_check, false);
  nh.param("simulation_robot_speed", simulation_robot_speed, 0.2);
  nh.param("robot_trj_time", robot_trj_time, 2.0);
  nh.param("detect_distance", detect_distance, 2.0);
  nh.param("lidar_detect_angle", lidar_detect_angle, 15.0);
  nh.param("lidar_detect_distance", lidar_detect_distance, 0.3);
  nh.param("lidar_sampling_count", lidar_sampling_count, 15);

  //pub
  desired_vector_pub = nh.advertise<geometry_msgs::Pose>("/erica/desired_vector",1);
  desired_vector_rviz_pub = nh.advertise<geometry_msgs::PoseStamped>("/erica/desired_vector_rviz",1);
  arrivals_action_command_pub = nh.advertise<std_msgs::Int8>("/erica/arrivals_action_command",1);
  people_tracking_command_pub = nh.advertise<std_msgs::String>("/erica/people_tracking_command",1);

  //sub
  ros::Subscriber people_position_sub = nh.subscribe("/erica/people_position", 100, people_position_callback);
  ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scan_callback);
  ros::Subscriber joy_sub   = nh.subscribe("/joy", 1, joy_callback);
  ros::Subscriber present_joint_states_sub   = nh.subscribe("/robotis/present_joint_states", 1, present_joint_states_callback);

  //sub motion done
  ros::Subscriber movement_done_sub   = nh.subscribe("/robotis/movement_done", 1, movement_done_callback);

  people_tracking_command_msg.data = "start";

  while(ros::ok())
  {
    if(!rotation_done_check)
    {
    //people_tracking_command_msg.data = "start";
      //ROS_INFO("rotation_check :: %d\n",rotation_check);
      if(!rotation_check)
      {
        fifth_trj_x->detect_change_final_value(goal_desired_vector_x, 0, robot_trj_time);
        fifth_trj_y->detect_change_final_value(goal_desired_vector_y, 0, robot_trj_time);
        desired_vector_msg.position.x = fifth_trj_x -> fifth_order_traj_gen(0,goal_desired_vector_x,0,0,0,0,0,robot_trj_time);
        desired_vector_msg.position.y = fifth_trj_y -> fifth_order_traj_gen(0,goal_desired_vector_y,0,0,0,0,0,robot_trj_time);

        if(people_detection_check)
        {
          desired_vector_msg.position.x = 0;
          desired_vector_msg.position.y = 0;
          goal_desired_vector_x = 0;
          goal_desired_vector_y = 0;
          fifth_trj_x ->current_pose = 0;
          fifth_trj_x ->current_velocity = 0;
          fifth_trj_x ->current_acc = 0;
          fifth_trj_x ->current_time = 0;
          fifth_trj_y ->current_pose = 0;
          fifth_trj_y ->current_velocity = 0;
          fifth_trj_y ->current_acc = 0;
          fifth_trj_y ->current_time = 0;
          rotation_check = true;
        }
        if(simulation_check == true)
        {
          simulation_rviz(desired_vector_msg);
          desired_vector_rviz_pub.publish(desired_vector_rviz_msg);
        }
        arrivals_action_command_msg.data = 0;
        arrivals_action_command_pub.publish(arrivals_action_command_msg);
        desired_vector_pub.publish(desired_vector_msg);
      }
      else // rotation starts!
      {
        if(head_yaw_position > -5*DEGREE2RADIAN && head_yaw_position < 5*DEGREE2RADIAN)// if the yaw angle is closed in 0, stop
        {
          rotation_done_check = true;
          arrivals_action_command_msg.data = 0;
          arrivals_action_command_pub.publish(arrivals_action_command_msg);
          people_tracking_command_msg.data = "stop";
          //rotation stops and action starts!
        }
        else
        {
          rotation_done_check = false;
          //action_movement_done_check = false;
          if(head_yaw_position > 5*DEGREE2RADIAN)
            arrivals_action_command_msg.data = 4;
          if(head_yaw_position < -5*DEGREE2RADIAN)
            arrivals_action_command_msg.data = 5;
        }
        arrivals_action_command_pub.publish(arrivals_action_command_msg);
        arrivals_action_command_msg.data = 0; // initial!
      }
      usleep(8000);
    }
    else
    {
      if(!action_movement_done_check && action_count == 1)
      {
        arrivals_action_command_msg.data = 1;
        arrivals_action_command_pub.publish(arrivals_action_command_msg);
      }
      people_tracking_command_pub.publish(people_tracking_command_msg);
      usleep(40000000); // 60s
      people_tracking_command_msg.data = "start";
      arrivals_action_command_msg.data = 0;
      arrivals_action_command_pub.publish(arrivals_action_command_msg);
      rotation_done_check = false; // 재시작
      ROS_INFO("Restart! \n");
      //waiting
    }
    people_tracking_command_pub.publish(people_tracking_command_msg);
    ros::spinOnce();
  }

  delete fifth_trj_y;
  delete fifth_trj_x;
  return 0;
}

/*#define POPULATION_SIZE 100

// C++ program to create target string, starting from 
// random string using Genetic Algorithm 

#include <bits/stdc++.h> 
using namespace std; 

// Number of individuals in each generation 

// Valid Genes 
const string GENES = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOP"
"QRSTUVWXYZ 1234567890, .-;:_!\"#%&/()=?@${[]}"; 

// Target string to be generated 
const string TARGET = "I love GeeksforGeeks"; 


void initialize()
{

}
//callback
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
}

// Function to generate random numbers in given range  
int random_num(int start, int end) 
{ 
    int range = (end-start)+1; 
    int random_int = start+(rand()%range); 
    return random_int; 
} 

// Create random genes for mutation 
char mutated_genes() 
{ 
    int len = GENES.size(); 
    int r = random_num(0, len-1); 
    return GENES[r]; 
} 

// create chromosome or string of genes 
string create_gnome() 
{ 
    int len = TARGET.size(); 
    string gnome = ""; 
    for(int i = 0;i<len;i++) 
        gnome += mutated_genes(); 
    return gnome; 
} 

// Class representing individual in population 
class Individual 
{ 
public: 
    string chromosome; 
    int fitness; 
    Individual(string chromosome); 
    Individual mate(Individual parent2); 
    int cal_fitness(); 
}; 

Individual::Individual(string chromosome) 
{ 
    this->chromosome = chromosome; 
    fitness = cal_fitness();  
}; 

// Perform mating and produce new offspring 
Individual Individual::mate(Individual par2) 
{ 
    // chromosome for offspring 
    string child_chromosome = ""; 

    int len = chromosome.size(); 
    for(int i = 0;i<len;i++) 
    { 
        // random probability  
        float p = random_num(0, 100)/100; 

        // if prob is less than 0.45, insert gene 
        // from parent 1  
        if(p < 0.45) 
            child_chromosome += chromosome[i]; 

        // if prob is between 0.45 and 0.90, insert 
        // gene from parent 2 
        else if(p < 0.90) 
            child_chromosome += par2.chromosome[i]; 

        // otherwise insert random gene(mutate),  
        // for maintaining diversity 
        else
            child_chromosome += mutated_genes(); 
    } 

    // create new Individual(offspring) using  
    // generated chromosome for offspring 
    return Individual(child_chromosome); 
}; 


// Calculate fittness score, it is the number of 
// characters in string which differ from target 
// string. 
int Individual::cal_fitness() 
{ 
    int len = TARGET.size(); 
    int fitness = 0; 
    for(int i = 0;i<len;i++) 
    { 
        if(chromosome[i] != TARGET[i]) 
            fitness++; 
    } 
    return fitness;     
}; 

// Overloading < operator 
bool operator<(const Individual &ind1, const Individual &ind2) 
{ 
    return ind1.fitness < ind2.fitness; 
} 


//////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mobile_manager_node");


  ros::NodeHandle nh;
  initialize();

  //motor1_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_1",10);

  //joy_sub   = nh.subscribe("/joy", 1, joy_callback);

   srand((unsigned)(time(0))); 

    // current generation 
    int generation = 0; 

    vector<Individual> population; 
    bool found = false; 

    // create initial population 
    for(int i = 0;i<POPULATION_SIZE;i++) 
    { 
        string gnome = create_gnome(); 
        population.push_back(Individual(gnome)); 
    } 

  while(ros::ok())
  {

    while(! found) 
    { 
        // sort the population in increasing order of fitness score 
        sort(population.begin(), population.end()); 

        // if the individual having lowest fitness score ie.  
        // 0 then we know that we have reached to the target 
        // and break the loop 
        if(population[0].fitness <= 0) 
        { 
            found = true; 
            break; 
        } 

        // Otherwise generate new offsprings for new generation 
        vector<Individual> new_generation; 

        // Perform Elitism, that mean 10% of fittest population 
        // goes to the next generation 
        int s = (10*POPULATION_SIZE)/100; 
        for(int i = 0;i<s;i++) 
            new_generation.push_back(population[i]); 

        // From 50% of fittest population, Individuals 
        // will mate to produce offspring 
        s = (90*POPULATION_SIZE)/100; 
        for(int i = 0;i<s;i++) 
        { 
            int len = population.size(); 
            int r = random_num(0, 50); 
            Individual parent1 = population[r]; 
            r = random_num(0, 50); 
            Individual parent2 = population[r]; 
            Individual offspring = parent1.mate(parent2); 
            new_generation.push_back(offspring);  
        } 
        population = new_generation; 
        cout<< "Generation: " << generation << "\t"; 
        cout<< "String: "<< population[0].chromosome <<"\t"; 
        cout<< "Fitness: "<< population[0].fitness << "\n"; 

        generation++; 
     } 
     cout<< "Generation: " << generation << "\t"; 
    cout<< "String: "<< population[0].chromosome <<"\t"; 
    cout<< "Fitness: "<< population[0].fitness << "\n"; 


    usleep(100);

    printf("---------------------------------------\n");
    printf("DIR Motor1 :: %d \n", motor_cmd_msg_1.motor_desired_direction);

    ros::spinOnce();
  }
  return 0;
}*/
