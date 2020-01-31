#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <vector>
#include <thread>
#include <cmath>
#include <algorithm>

namespace gazebo 
{
    class AckermanPlugin : public ModelPlugin {
        public:
            AckermanPlugin() {}
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
                // ROS_INFO("\nThe Ackerman plugin is attached sucessfully to model[" + );
                std::cout << "\nThe Ackerman plugin is attached sucessfully to model["
                          <<_model->GetName() << "]\n";
                if ( _sdf->HasElement("L")) {
                    this->length = _sdf->Get<double>("L");
                }
                if ( _sdf->HasElement("T")) {
                    this-> thickness = _sdf->Get<double>("T");
                }
                if ( _sdf->HasElement("right_front_wheel_joint")) {
                    this->rightFrontWheelJoint = _sdf->Get<std::string>("right_front_wheel_joint"); 
                }
                if ( _sdf->HasElement("left_front_wheel_joint")) {
                    this->leftFrontWheelJoint = _sdf->Get<std::string>("left_front_wheel_joint"); 
                }
                if ( _sdf->HasElement("right_rear_wheel_joint")) {
                    this->rightRearWheelJoint = _sdf->Get<std::string>("right_rear_wheel_joint"); 
                }
                if ( _sdf->HasElement("left_rear_wheel_joint")) {
                    this->leftRearWheelJoint = _sdf->Get<std::string>("left_rear_wheel_joint"); 
                }
                std::cout << "\nRobot Tickness="
                            <<this->thickness << "\n";
                std::cout << "\nRobot Lengeth="
                            <<this->length << "\n";
                std::cout << "\nRobot lfw="
                            <<this->leftFrontWheelJoint << "\n";
                std::cout << "\nRobot frw="
                            <<this->rightFrontWheelJoint << "\n";
                std::cout << "\nRobot rrw="
                            <<this->rightRearWheelJoint << "\n";
                std::cout << "\nRobot lrw="
                            <<this->leftRearWheelJoint << "\n";
                this->model = _model;
                this->joint = _model->GetJoints()[0]; //first joint
                this->jointController = this->model->GetJointController();
                //this->model->GetJointController()->SetVelocityPID(this->joint->GetScopedName(), this->pid);
                
                // setting pid for our joint controlllers
                this->setVelPid(this->leftRearWheelJoint, 0.1, 0.0, 0.0);
                this->setVelPid(this->rightRearWheelJoint, 0.1, 0.0, 0.0);
                this->setPosPid(this->leftFrontWheelJoint, 90, 0, 1.2);
                this->setPosPid(this->rightFrontWheelJoint, 90, 0, 1.2);
                // this->setPosPid("arm1_arm2", 90, 0, 1.5);
                
                if (!ros::isInitialized()) {
                    int argc = 0;
                    char **argv = NULL;
                    ros::init(argc, argv, "gazebo_client",  ros::init_options::NoSigintHandler);
                }
                this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
                ros::SubscribeOptions turnLso = ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->model->GetName() + "/steer_left_cmd", 1,
                     boost::bind(&AckermanPlugin::onSteerLeft, this, _1),
                     ros::VoidPtr(), &this->rosQueue);
                this->turnLSub = this->rosNode->subscribe(turnLso);

                ros::SubscribeOptions turnRso = ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->model->GetName() + "/steer_right_cmd", 1,
                     boost::bind(&AckermanPlugin::onSteerRight, this, _1),
                     ros::VoidPtr(), &this->rosQueue);
                this->turnRSub = this->rosNode->subscribe(turnRso);
                this->rosQueueThread = std::thread(std::bind(&AckermanPlugin::QueueThread, this));

                ros::SubscribeOptions moveFso = ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->model->GetName() + "/move_forward_cmd", 1,
                     boost::bind(&AckermanPlugin::onMoveForward, this, _1),
                     ros::VoidPtr(), &this->rosQueue);
                this->mvFdSub = this->rosNode->subscribe(moveFso);

                ros::SubscribeOptions moveBso = ros::SubscribeOptions::create<std_msgs::Float32>(
                    "/" + this->model->GetName() + "/move_backward_cmd", 1,
                     boost::bind(&AckermanPlugin::onMoveBackWard, this, _1),
                     ros::VoidPtr(), &this->rosQueue);
                this->mvBdSub = this->rosNode->subscribe(moveBso);

                
                
                
            }
            

        private:
            double thickness;
            double length;
            std::string rightFrontWheelJoint;
            std::string leftFrontWheelJoint;
            std::string rightRearWheelJoint;
            std::string leftRearWheelJoint;
            physics::ModelPtr model;
            physics::JointPtr joint;
            physics::JointControllerPtr jointController;
            common::PID pid;
            std::unique_ptr<ros::NodeHandle> rosNode;
            ros::Subscriber rosSub;
            ros::Subscriber turnRSub;
            ros::Subscriber turnLSub;
            ros::Subscriber mvFdSub;
            ros::Subscriber mvBdSub;
            ros::CallbackQueue rosQueue;
            std::thread rosQueueThread;

            double radTodeg(double rad) {
                return (rad * 180) / M_PI;
            }
            double degTorad(double deg) {
                return (deg * M_PI) / 180;
            }
            std::vector<double> getAckermanAngles(double theta, bool thetaInRad=false) {
                // theta is in degree
                double innerAng, outerAng, radius;
                // thetas will hold innner_wheel_angle, outer_wheel_angle in rad
                std::vector<double> thetas(2);
                // radius is radius of axis of rotation
                if (thetaInRad) {
                    radius = this->length / tan(theta);
                } else {
                    radius = this->length / tan(degTorad(theta));
                }
                
                
                innerAng = atan(this->length / (radius - (this->thickness / 2.0)));
                outerAng = atan(this->length / (radius + (this->thickness / 2.0)));   
                thetas[0] = innerAng;
                thetas[1] = outerAng;

                return thetas;     
            }
            void steer(const double &_theta, bool right=false) {
                // assuming steering left is postive
                double theta;
                theta = abs(_theta);
                double inner_theta = this->getAckermanAngles(theta)[0];
                double outer_theta = this->getAckermanAngles(theta)[1];
                std::cout << "Inner angle: " << this->radTodeg(inner_theta) << "deg\n";
                std::cout << "Outer angle: " << this->radTodeg(outer_theta) << "deg\n";
                std::string rightName = this->model->GetJoint(this->rightFrontWheelJoint)->GetScopedName();
                std::string leftName = this->model->GetJoint(this->leftFrontWheelJoint)->GetScopedName();
                
                if (right) {
                    // steer right, inner wheel is right wheel
                    this->jointController->SetPositionTarget(rightName, -1 * inner_theta);
                    this->jointController->SetPositionTarget(leftName, -1 * outer_theta);
                    this->jointController->Update();     
                } else {
                    // steer left, inner wheel is left wheel
                    this->jointController->SetPositionTarget(leftName, inner_theta);
                    this->jointController->SetPositionTarget(rightName, outer_theta);
                    this->jointController->Update(); 
                }
                
            }
            void onSteerRight(const std_msgs::Float32ConstPtr &_msg) {
                this->steer(_msg->data, true);
            }
            void onSteerLeft(const std_msgs::Float32ConstPtr &_msg) {
                this->steer(_msg->data);
            }
            void move(const double &_vel, bool back=false) {
                double vel = _vel;
                vel = abs(vel);
                std::string rightName = this->model->GetJoint(this->rightRearWheelJoint)->GetScopedName();
                std::string leftName = this->model->GetJoint(this->leftRearWheelJoint)->GetScopedName();
                if (back) {
                     vel *= -1;     
                }
                this->jointController->SetVelocityTarget(rightName, vel);
                this->jointController->SetVelocityTarget(leftName, vel);
                this->jointController->Update();
                return;
            }
            void onMoveForward(const std_msgs::Float32ConstPtr &_vel) {
                this->move(_vel->data);           
            }
            void onMoveBackWard(const std_msgs::Float32ConstPtr &_vel) {
                this->move(_vel->data, true);;
            }

            void setPosPid(std::string jointName, double P, double I, double D){
                std::string name = this->model->GetJoint(jointName)->GetScopedName();
                this->jointController->SetPositionPID(name, common::PID(P, I, D));
                
                return;
            }
            void setVelPid(std::string jointName, double P, double I, double D){
                std::string name = this->model->GetJoint(jointName)->GetScopedName();
                this->jointController->SetVelocityPID(name, common::PID(P, I, D));
                return;
            }
            // void stop() {
            //     return;
            // }
            
            void QueueThread() {
                static const double timeout = 0.01;
                while(this->rosNode->ok()) {
                    this->rosQueue.callAvailable(ros::WallDuration(timeout));
                }
            }


    };
    GZ_REGISTER_MODEL_PLUGIN(AckermanPlugin);
}

