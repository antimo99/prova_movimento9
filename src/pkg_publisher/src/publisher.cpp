#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "pkg_publisher/quintic.hpp"
#include "std_msgs/Float64MultiArray.h"



class MyNode
{
    public: 
      std_msgs::Float64MultiArray q0,qf; 
      ros::NodeHandle n;
      ros::Publisher command_pub;
      std::vector<std::string> joint_names_ = {"panda_joint1",
                                                "panda_joint2",
                                                "panda_joint3",
                                                "panda_joint4",
                                                "panda_joint5",
                                                "panda_joint6",
                                                "panda_joint7"};
     MyNode()
     {
            command_pub = n.advertise<std_msgs::Float64MultiArray>("/joint_group_velocity_controller/command", 1000);
            q0.data.resize(joint_names_.size());
            q0.data={0,0,0,0,0,0,0};
            qf.data.resize(joint_names_.size());

            // Attendi il singolo messaggio e salvalo nella variabile globale
            auto msg=ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states",ros::Duration(1.0));
            for (size_t i = 0; i < msg->name.size(); ++i) 
            {
                ROS_INFO("Nome: %s, Posizione: %f", msg->name[i].c_str(), msg->position[i]);
            }

            q0.data=msg->position;
            qf.data=q0.data; 
            qf.data[6]+=M_PI/4;//configurazione da raggiungere
           
            using namespace std::chrono_literals;
            ros::Time t0=ros::Time::now();
            ros::Duration t(0, 0); 
            ros::Rate loop_rate(1000); //1 kHz
            double traj_duration=8.0;
            
            /*Ciclo di "generazione traiettoria"**/
            while (ros::ok() && t.toSec() <= traj_duration)
            {
                t=ros::Time::now()-t0;

                //riempio il goal
                std_msgs::Float64MultiArray goal;
                goal.data.resize(joint_names_.size());

                // riempio il vettore usando la funzione quintic 
                for (int i = 0; i <(int)joint_names_.size(); i++)
                 goal.data[i] = quintic(t.toSec(), q0.data[i], qf.data[i], traj_duration);
                
                command_pub.publish(goal);
                loop_rate.sleep();
            }
     }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "prova");
  MyNode my_node;
  ros::spin();
  return 0;
}
