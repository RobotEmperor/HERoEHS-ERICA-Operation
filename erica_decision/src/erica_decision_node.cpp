
/*
 * erica_decision_node.cpp
 *
 *  Created on: Jan 8, 2019
 *      Author: robotemperor
 */

#include <erica_decision/erica_decision_node.h>
#include <cmath>
void initialize()
{
  simulation_robot_speed = 0.0;
  robot_trj_time = 0.0 ;
  simulation_check = false;

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
void people_position_callback(const erica_perception_msgs::PeoplePositionArray::ConstPtr& msg)
{
  if(msg->people_position.size() == 0)
  {
    goal_desired_vector_x = 0;
    goal_desired_vector_y = 0;
    return;
  }
  else
  {
    if(std::isnan(msg->people_position[0].x) || std::isnan(msg->people_position[0].y))
      return;
    desired_vector_msg.position.x = (double) msg->people_position[0].x;
    desired_vector_msg.position.y = (double) msg->people_position[0].y;

    for(int people_num = 1; people_num < msg->people_position.size(); people_num ++)
    {
      if((pow((double) msg->people_position[people_num].x,2) +  pow((double) msg->people_position[people_num].y,2)) < (pow(desired_vector_msg.position.x,2)+pow(desired_vector_msg.position.y,2)))
      {
        desired_vector_msg.position.x = (double) msg->people_position[people_num].x;
        desired_vector_msg.position.y = (double) msg->people_position[people_num].y;
      }
    }
  }
  // if person is close, the robot stops.
  if(sqrt(pow(fabs(desired_vector_msg.position.x),2)+ pow(fabs(desired_vector_msg.position.y),2)) <= 0.7)
  {
    goal_desired_vector_x = 0;
    goal_desired_vector_y = 0;
    return;
  }
  if(sqrt(pow(fabs(desired_vector_msg.position.x),2)+pow(fabs(desired_vector_msg.position.y),2)) > 1)
  {
    goal_desired_vector_x = 0;
    goal_desired_vector_y = 0;
    return;
  }
  //unit vector
  /*  double temp_absolute_size = 0.0;
  temp_absolute_size = sqrt(pow(desired_vector_msg.position.x,2)+pow(desired_vector_msg.position.y,2));
  desired_vector_msg.position.x = desired_vector_msg.position.x/temp_absolute_size ;
  desired_vector_msg.position.y = desired_vector_msg.position.y/temp_absolute_size ;*/
  //  }

 
  goal_desired_vector_x = desired_vector_msg.position.x;
  goal_desired_vector_y = desired_vector_msg.position.y;
}
//simulation
void simulation_rviz(geometry_msgs::Pose desired_vector) // cpp 분리
{
  desired_vector_rviz_msg.header.frame_id = "map";

  desired_vector_rviz_msg.pose.orientation.z = atan2(desired_vector.position.y,desired_vector.position.x);

  rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,desired_vector_rviz_msg.pose.orientation.z);

  desired_vector_rviz_msg.pose.orientation.x = rqyToQ.x();
  desired_vector_rviz_msg.pose.orientation.y = rqyToQ.y();
  desired_vector_rviz_msg.pose.orientation.z = rqyToQ.z();
  desired_vector_rviz_msg.pose.orientation.w = rqyToQ.w();
}

void simulation_gazebo(geometry_msgs::Pose desired_vector)
{
  /*  double desired_theta_ = 0;

  if(sqrt(pow(fabs(desired_vector_msg.position.x),2)+pow(fabs(desired_vector_msg.position.y),2)) > 0.01)
  {
    if(desired_vector_msg.position.x > 0)
      cmd_vel_msg.linear.x = 0.2;
    else
      cmd_vel_msg.linear.x = -0.2;

    if(desired_vector_msg.position.x == 0)
    {
      if(desired_vector_msg.position.y < 0)
        cmd_vel_msg.linear.x = 0.2;
      else
        cmd_vel_msg.linear.x = -0.2;
    }

    desired_theta_ = atan2(desired_vector.position.y, desired_vector.position.x);

    if(desired_theta_ > M_PI/2 && desired_theta_ <= M_PI)
      desired_theta_ =  desired_theta_ - M_PI;
    if(desired_theta_ < -M_PI/2 && desired_theta_ >= -M_PI)
      desired_theta_ =  desired_theta_ + M_PI;

    left_wheel_front_steering_position_msg.data  = desired_theta_;
    right_wheel_front_steering_position_msg.data = desired_theta_;
    left_wheel_rear_steering_position_msg.data   = desired_theta_;
    right_wheel_rear_steering_position_msg.data  = desired_theta_;
  }
  else
  {
    cmd_vel_msg.linear.x = 0;
    left_wheel_front_steering_position_msg.data  = 0;
    right_wheel_front_steering_position_msg.data = 0;
    left_wheel_rear_steering_position_msg.data   = 0;
    right_wheel_rear_steering_position_msg.data  = 0;
  }

  left_wheel_front_steering_position_pub.publish(left_wheel_front_steering_position_msg);
  right_wheel_front_steering_position_pub.publish(right_wheel_front_steering_position_msg);
  left_wheel_rear_steering_position_pub.publish(left_wheel_rear_steering_position_msg);
  right_wheel_rear_steering_position_pub.publish(right_wheel_rear_steering_position_msg);

  cmd_vel_pub.publish(cmd_vel_msg);*/

  if(sqrt(pow(fabs(desired_vector.position.x),2)+pow(fabs(desired_vector.position.y),2)) > 0.01)
  {
    cmd_vel_x_msg.data = desired_vector.position.x;
    cmd_vel_y_msg.data = desired_vector.position.y;

  }
  else
  {
    cmd_vel_x_msg.data = 0;
    cmd_vel_y_msg.data = 0;
  }

  cmd_vel_x_pub.publish(cmd_vel_x_msg);
  cmd_vel_y_pub.publish(cmd_vel_y_msg);
}

//algorithm function
/*void desired_vector_trj(geometry_msgs::Pose *out_desired_vector_, double desired_value_x_ , double desired_value_y_)
{
 *out_desired_vector_= fifth_trj_x ->fifth_order_traj_gen(0,desired_value_x_,0,0,0,0,0,robot_trj_time);
}*/


int main (int argc, char **argv)
{
  ros::init(argc, argv, "erica_decision_node");
  ros::NodeHandle nh;

  initialize();

  //launch initialize
  nh.param("simulation_check",  simulation_check, false);
  nh.param("simulation_robot_speed", simulation_robot_speed, 0.2);
  nh.param("robot_trj_time", robot_trj_time, 5.0);

  //pub
  desired_vector_pub = nh.advertise<geometry_msgs::Pose>("/erica/desired_vector",1);
  desired_vector_rviz_pub = nh.advertise<geometry_msgs::PoseStamped>("/erica/desired_vector_rviz",1);

  left_wheel_front_steering_position_pub  = nh.advertise<std_msgs::Float64>("/erica_robot/left_wheel_front_steering_position/command",1);
  right_wheel_front_steering_position_pub = nh.advertise<std_msgs::Float64>("/erica_robot/right_wheel_front_steering_position/command",1);
  left_wheel_rear_steering_position_pub   = nh.advertise<std_msgs::Float64>("/erica_robot/left_wheel_rear_steering_position/command",1);
  right_wheel_rear_steering_position_pub  = nh.advertise<std_msgs::Float64>("/erica_robot/right_wheel_rear_steering_position/command",1);


  cmd_vel_x_pub = nh.advertise<std_msgs::Float64>("/erica_robot/fake_x_velocity/command",1);
  cmd_vel_y_pub = nh.advertise<std_msgs::Float64>("/erica_robot/fake_y_velocity/command",1);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);


  //sub
  ros::Subscriber people_position_sub = nh.subscribe("/erica/people_position", 100, people_position_callback);
  ros::Subscriber joy_sub   = nh.subscribe("/joy", 1, joy_callback);


  while(ros::ok())
  {
    fifth_trj_x->detect_change_final_value(goal_desired_vector_x, 0, robot_trj_time);
    fifth_trj_y->detect_change_final_value(goal_desired_vector_y, 0, robot_trj_time);
    desired_vector_msg.position.x = fifth_trj_x -> fifth_order_traj_gen(0,goal_desired_vector_x,0,0,0,0,0,robot_trj_time);
    desired_vector_msg.position.y = fifth_trj_y -> fifth_order_traj_gen(0,goal_desired_vector_y,0,0,0,0,0,robot_trj_time);

    if(simulation_check == true)
    {
      simulation_rviz(desired_vector_msg);
      simulation_gazebo(desired_vector_msg);
      desired_vector_rviz_pub.publish(desired_vector_rviz_msg);
    }

    desired_vector_pub.publish(desired_vector_msg);

    ros::spinOnce();
    usleep(8000);
  }
  delete fifth_trj_x;
  delete fifth_trj_y;
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
