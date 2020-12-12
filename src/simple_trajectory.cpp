#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64MultiArray.h>
#include <jointspace/OptStates.h>
#include "sensor_msgs/JointState.h"
#include <Eigen/Eigen>
#include <kdl/jntarray.hpp>
#include <array>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <ros/console.h>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/frames.hpp>

using namespace std;
//typedef sensor_msgs::JointState msg1;
//typedef jointspace::OptStates msg2;


#define N_JOINT 6


class Ur3Arm
{
private:
  std::string tauTopicNames[N_JOINT] = {
    "/urbot/endeffector_frc_trq_controller_1/command",
    "/urbot/endeffector_frc_trq_controller_2/command",
    "/urbot/endeffector_frc_trq_controller_3/command",
    "/urbot/endeffector_frc_trq_controller_4/command",
    "/urbot/endeffector_frc_trq_controller_5/command",
    "/urbot/endeffector_frc_trq_controller_6/command",
  };
  message_filters::Subscriber<sensor_msgs::JointState> sub1;
  message_filters::Subscriber<jointspace::OptStates> sub2;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::JointState, jointspace::OptStates> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

public:

  int ind = 0; //! always zero
  ros::NodeHandle n;
  //ros::Subscriber opt_states_sub; ros::Subscriber joint_states_sub;
  std::vector<ros::Publisher> pose_multiple_pub;
  ros::Publisher pose_pub;
  KDL::JntArray jointPosCurrent, jointVelCurrent;
  Eigen::MatrixXf q_cur, qdot_cur;
  Eigen::MatrixXf q_des, qdot_des, qddot_des;
  KDL::Tree mytree; KDL::Chain mychain;


  Ur3Arm()
  {
    jointPosCurrent.resize(N_JOINT), jointVelCurrent.resize(N_JOINT);
    q_cur.resize(N_JOINT,1); qdot_cur.resize(N_JOINT,1);
    q_des.resize(N_JOINT, 1), qdot_des.resize(N_JOINT,1), qddot_des.resize(N_JOINT,1);
    //opt_states_sub = n.subscribe("/opt_states", 1000, &Ur3Arm::optStatesCallback, this);
    //joint_states_sub = n.subscribe("/urbot/joint_states", 1000, &Ur3Arm::jointStatesCallback, this);

    // BEGIN
    float mass = 5 ; double Ixx, Iyy, Izz; double l= 0.08, r = l/2.0;

    Ixx = (1.0/12.0) * mass * ( 3*r*r + l*l);
    Iyy = Ixx;
    Izz =  (1.0/2.0) * mass * ( r*r );
    double I[6]={Ixx, Iyy, Izz, 0, 0, 0};   // Ixy = Ixz= Iyz = 0;
    double offset[6] = {0, 0, 0, 0, 0, 0} ; std::string tool_name = "new_tool";
    KDL::Vector r_cog(r, r, l/2.0); //! for a cylinder
    KDL::Joint fixed_joint = KDL::Joint(KDL::Joint::None);
    KDL::Frame tip_frame = KDL::Frame(KDL::Rotation::RPY(offset[0],offset[1],offset[2]),KDL::Vector(offset[3],offset[4],offset[5]));

    // rotational inertia in the cog
    KDL::RotationalInertia Inertia_cog = KDL::RotationalInertia(I[0], I[1], I[2], I[3], I[4], I[5]);
    KDL::RigidBodyInertia Inertia = KDL::RigidBodyInertia(mass, r_cog, Inertia_cog);
    KDL::Segment segment = KDL::Segment(tool_name, fixed_joint, tip_frame, Inertia);

    //parse kdl tree from Urdf
    if(!kdl_parser::treeFromFile("/home/mujib/test_ws/src/universal_robot/ur_description/urdf/ur5_joint_test.urdf", mytree)){
      ROS_ERROR("[ST] Failed to construct kdl tree for elfin ! ");
    }

    if (!mytree.addSegment(segment, "wrist_3_link")) {  //! adding segment to the tree
      ROS_ERROR("[ST] Could not add segment to kdl tree");
    }

    if (!mytree.getChain("base_link", tool_name, mychain)){
      ROS_ERROR("[ST] Failed to construct kdl chain for elfin ! ");
    }
    // END

    /*if(!kdl_parser::treeFromFile("/home/mujib/test_ws/src/universal_robot/ur_description/urdf/ur5_joint_test.urdf", mytree)){
      ROS_ERROR("[ST] Failed to construct kdl tree for ur5 ! ");
    }
    if (!mytree.getChain("base_link", "wrist_3_link", mychain)){ // (TODO: Add mass)
      ROS_ERROR("[ST] Failed to construct kdl chain for ur5 ! ");
    }*/

    unsigned int nj, ns; // resize variables using # of joints & segments
    nj =  mytree.getNrOfJoints(); ns = mychain.getNrOfSegments();
    if (ns == 0 || nj == 0){
      ROS_ERROR("[ST] Number of segments/joints are zero ! ");
    }
    if(jointPosCurrent.rows()!=nj || jointVelCurrent.rows()!=nj)
    {
      ROS_ERROR("[JS] ERROR in size of joint variables ! ");
    }

    sub1.subscribe(n, "/urbot/joint_states", 1000);
    sub2.subscribe(n, "/opt_states", 1000);
    sync.reset(new Sync(MySyncPolicy(1000), sub1, sub2)); //10
    sync->registerCallback(boost::bind(&Ur3Arm::callback, this, _1, _2));

    for (short int j = 0; j < N_JOINT; j ++)
    {
      pose_pub = n.advertise<std_msgs::Float64>(tauTopicNames[j], 1);
      pose_multiple_pub.push_back(pose_pub);
    }
    cout << "[ST] Constructed !" << endl;
  }

  // -> Destructor of class Ur3Arm
  ~Ur3Arm()
  {
  }

  void callback(const sensor_msgs::JointStateConstPtr &msg1, const jointspace::OptStatesConstPtr &msg2)
  {
    KDL::JntArray C(N_JOINT), gravity(N_JOINT);
    Eigen::MatrixXf qdotdot_k(N_JOINT,1), c(N_JOINT,1), g(N_JOINT,1), M_(N_JOINT, N_JOINT), tau(N_JOINT,1);
    double k_p[] = {10, 12, 10,  5,  0.01,  1}; // specify p and d gains
    double k_d[] = {15, 13, 14, 10,  8,    3};         // 15, 5, 5, 5, 5, 2
    int map_joint_states[N_JOINT]={2, 1, 0, 3, 4, 5};
    KDL::JntSpaceInertiaMatrix M(N_JOINT);
    KDL::ChainDynParam dyn_param(mychain, KDL::Vector(0, 0, -9.80665));

    for (short int i=0; i< N_JOINT; i++){
      jointPosCurrent(i) = msg1->position[map_joint_states[i]]; // joint_state
      jointVelCurrent(i) = msg1->velocity[map_joint_states[i]];
      q_cur(i,0) = msg1->position[map_joint_states[i]];
      qdot_cur(i,0) = msg1->velocity[map_joint_states[i]];

      q_des(i,0) = msg2->q.data[i];  // opt_states
      qdot_des(i,0) = msg2->qdot.data[i];
      qddot_des(i,0) = msg2->qddot.data[i];

      qdotdot_k(i,0)=k_p[i]*(q_des(i,0)-q_cur(i,0))+k_d[i]*(qdot_des(i,0)-qdot_cur(i,0));
    }

    dyn_param.JntToMass(jointPosCurrent, M);
    dyn_param.JntToGravity(jointPosCurrent, gravity);
    dyn_param.JntToCoriolis(jointPosCurrent, jointVelCurrent, C);

    for(short int i = 0; i < N_JOINT; i++){
      c(i,0) = C(i);
      g(i,0) = gravity(i);
      for(int j = 0; j < N_JOINT; j++){
        M_(i,j) = M(i,j);
      }
    }

    tau = M_ * qdotdot_k + c + g;
    cout << "tau: " << tau.transpose() << endl;
    //cout << "qddot: " << qdotdot_k.transpose() << endl;
    std::vector<std_msgs::Float64> msg_(N_JOINT);
    for (short int j = 0; j < N_JOINT; j++)
    {
      msg_[j].data = tau(j,0);
      pose_multiple_pub[j].publish(msg_[j]);
    }
  }

  /*void optStatesCallback(const jointspace::OptStates::ConstPtr& msg){

    for (short int i=0; i< N_JOINT; i++){ //perfectly sync with publisher, no data is lost till here
      q_des(i,0) = msg->q.data[i];
      qdot_des(i,0) = msg->qdot.data[i];
      qddot_des(i,0) = msg->qddot.data[i];
      std::cout << "Vel: " << q_des.transpose()    << std::endl;
      std::cout << "Acc: " << qdot_des.transpose() << std::endl;

    }
  }

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg){

    int map_joint_states[N_JOINT]={2, 1, 0, 3, 4, 5};
    for (short int i=0; i< N_JOINT; i++){
      jointPosCurrent(i) = msg->position[map_joint_states[i]];
      jointVelCurrent(i) = msg->velocity[map_joint_states[i]];
      q_cur(i,0) = msg->position[map_joint_states[i]];
      qdot_cur(i,0) = msg->velocity[map_joint_states[i]];
    }
    //cout << " [ST] Joint Pos:" << jointPosCurrent.data.transpose() << endl;
  }*/
};


int main (int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_trajectory");
  cout << "[JS] Hello World !" << endl;
  Ur3Arm arm;  ros::Rate loop_rate(50);
  /*KDL::Tree mytree; KDL::Chain mychain;
  KDL::JntSpaceInertiaMatrix M(N_JOINT);
  KDL::JntArray C(N_JOINT), gravity(N_JOINT), Acc(N_JOINT);
  Eigen::MatrixXf qdotdot_k(N_JOINT,1), c(N_JOINT,1), g(N_JOINT,1), M_(N_JOINT, N_JOINT), tau(N_JOINT,1);

  std::vector<std_msgs::Float64> msg_(N_JOINT);
  double k_p[] = {0, 5, 200, 0, 0, 0}; // specify p and d gains
  double k_d[] = {0, 0, 0, 0, 0, 0};

  //parse kdl tree from Urdf
  if(!kdl_parser::treeFromFile("/home/mujib/test_ws/src/universal_robot/ur_description/urdf/ur5_joint_test.urdf", mytree)){
    ROS_ERROR("[ST] Failed to construct kdl tree for ur5 ! ");
  }
  if (!mytree.getChain("base_link", "wrist_3_link", mychain)){
    ROS_ERROR("[ST] Failed to construct kdl chain for ur5 ! ");
  }

  unsigned int nj, ns; // resize variables using # of joints & segments
  nj =  mytree.getNrOfJoints(); ns = mychain.getNrOfSegments();
  if (ns == 0 || nj == 0){
    ROS_ERROR("[ST] Number of segments/joints are zero ! ");
  }
  if(arm.jointPosCurrent.rows()!=nj || arm.jointVelCurrent.rows()!=nj)
  {
    ROS_ERROR("[JS] ERROR in size of joint variables ! ");
  }

  KDL::ChainDynParam dyn_param(mychain, KDL::Vector(0, 0, -9.80665));*/

  while(ros::ok())
  {
    /*dyn_param.JntToMass(arm.jointPosCurrent, M);
    dyn_param.JntToGravity(arm.jointPosCurrent, gravity);
    dyn_param.JntToCoriolis(arm.jointPosCurrent, arm.jointVelCurrent, C);

    cout << "gravity: " << gravity.data.transpose() << endl;
    cout << "C: " << C.data.transpose() << endl;

    for(short int i = 0; i < N_JOINT; i++){
      c(i,0) = C(i);
      g(i,0) = gravity(i);
      for(int j = 0; j < N_JOINT; j++){
        M_(i,j) = M(i,j);
      }
    }

    for(int jnt = 0; jnt < N_JOINT; jnt++){

      qdotdot_k(jnt,0)=k_p[jnt]*(arm.q_des(jnt,0)-arm.q_cur(jnt,0))+k_d[jnt]*(arm.qdot_des(jnt,0)-arm.qdot_cur(jnt,0));
      //qdotdot_k = 10*( arm.q_des - arm.q_cur ) + 10*( arm.qdot_des - arm.qdot_cur );
    }

    if(jnt==2)
      cout << "[Simple_Trajectory] q_des: " << arm.q_des(jnt,0) << endl;
    cout << "q_des: " << arm.q_des(2,0) << endl;
    cout << "##############################"   << endl;
    cout << "q_cur: " << arm.q_cur(2,0) << endl;

    tau = M_ * qdotdot_k + c + g;
    cout << "tau: " << tau.transpose() << endl;

    for (short int j = 0; j < N_JOINT; j++)  // publish desired pose on joints in reihenfolge
    {
      msg_[j].data = tau(j,0);               // [200, 200, 104, 104, 34, 34]
      arm.pose_multiple_pub[j].publish(msg_[j]);
    }*/

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
