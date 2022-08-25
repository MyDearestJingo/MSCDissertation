#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math.hh>
#include <ros/console.h>
#include <ctime>
#include <cmath>
#include <cstring>
#include <algorithm>  // copy()
#include <iterator>   // istream_iterator, back_inserter
#include <sstream>    // istringstream
#include <string>
#include <vector>

using ignition::math::Vector3;
using ignition::math::Pose3;
using ignition::math::Quaternion;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::istringstream;
using std::istream_iterator;
using std::back_inserter;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      // Record the beginning time in sec
      this->begin = clock();
      this->delay = 10;
      this->lastUpd = clock();
      this->updFreq = 5;

      // this->strLinkName = "cracker_box_link";
      this->direction = Vector3<double>({0.1, 0.1, 0.1});
      this->initPose = Pose3<double>(0.6, 0, 0.02, 1, 0, 0, 0); // quat: wxyz
      this->maxDist = 0.8;
      this->vel = 0.1;

      // load params
      string elem;
      if(_sdf->HasElement("link_name")){
        elem = _sdf->GetElement("link_name")->Get<string>();
        this->strLinkName = elem;
      }
      else{
        ROS_FATAL("plugin move uniform linear missing <link_name>, cannot proceed");
        return;
      }

      if(_sdf->HasElement("direction")){
        elem = _sdf->GetElement("direction")->Get<string>();
        double dElem[3];
        istringstream ss( elem );
        copy(
          istream_iterator <float> ( ss ),
          istream_iterator <float> (),
          dElem
          );
        this->direction = (Vector3<double>(dElem[0], dElem[1], dElem[2])).Normalize();
      }
      else{
        ROS_FATAL("plugin move uniform linear missing <direction>, cannot proceed");
        return;
      }

      if(_sdf->HasElement("vel")){
        elem = _sdf->GetElement("vel")->Get<string>();
        this->vel = std::stod(elem);
      }
      else{
        ROS_FATAL("plugin move uniform linear missing <vel>, cannot proceed");
        return;
      }

      if(_sdf->HasElement("init_pose")){
        elem = _sdf->GetElement("init_pose")->Get<string>();
        double dElem[6]; // pos: xyz & ori: rpy
        istringstream ss( elem );
        copy(
          istream_iterator <double> ( ss ),
          istream_iterator <double> (),
          dElem
          );
        Vector3<double> pos(dElem[0], dElem[1], dElem[2]);
        Quaternion<double> ori(dElem[3], dElem[4], dElem[5]);
        
        this->initPose = Pose3<double>(pos, ori);
      }
      else{
        ROS_FATAL("plugin move uniform linear missing <init_pose>, cannot proceed");
        return;
      }

      if(_sdf->HasElement("max_distance")){
        elem = _sdf->GetElement("max_distance")->Get<string>();
        this->maxDist = std::stod(elem);
      }
      else{
        ROS_FATAL("plugin move uniform linear missing <max_distance>, cannot proceed");
        return;
      }
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

      // if(clock() < this->begin + this->delay*CLOCKS_PER_SEC){
      //   return;
      // }
      // auto linkPtr_ = this->model->GetLink("cracker_box_link");

      // ignition::math::Pose3d currPose_ = linkPtr_->WorldPose();
      // Vector3<double> currPos = currPose_.Pos();


      // Vector3<double> initPos(-0.5, 0.2, 0.1);
      // double radian = 0.1;
      // Vector3<double> center = initPos;
      // center[2] += radian;
      // Vector3<double> baseRadianVec = initPos - center;

      // Vector3<double> currRadianVec = currPos - center;

      // double linSpd = 0.1;

      // double theta = acos(baseRadianVec.Dot(currRadianVec)
      //   /(currRadianVec.Length()*baseRadianVec.Length()));
      // theta = currRadianVec[1] > 0 ? theta : -theta;

      // linkPtr_->SetLinearVel({
      //   0.0,
      //   cos(theta)*linSpd,
      //   sin(theta)*linSpd
      // });

      // if(clock()-this->lastUpd > CLOCKS_PER_SEC/this->updFreq){
      //   this->lastUpd = clock();
      //   cout << "------\n"
      //     << "Current Time: " << this->lastUpd << '\n'
      //     << "Current Position: " << currPos << '\n'
      //     << "Theta (rad) = " << theta << '\n'
      //     << endl;
      // }

      physics::LinkPtr pLink = this->model->GetLink(this->strLinkName);
      Vector3<double> currPos = pLink->WorldPose().Pos();

      double currDist = (currPos - this->initPose.Pos()).Length();
      if(currDist > this->maxDist){
        pLink->SetWorldPose(this->initPose);
      }
      pLink->SetLinearVel(this->direction * this->vel);
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: string strLinkName;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Timer
    private: long int begin;
    private: long int delay;
    private: long int lastUpd;
    private: long int updFreq;
    // private: int lastTimeStamp

    // Config
    private: Vector3<double> direction;
    private: Pose3<double> initPose;
    private: double vel;
    private: double maxDist;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}