#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ctime>
#include <cmath>
using ignition::math::Vector3;
using std::cout;
using std::endl;
namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
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
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

      // if(clock() < this->begin + this->delay*CLOCKS_PER_SEC){
      //   return;
      // }
      auto linkPtr_ = this->model->GetLink("tomato_soup_can_link");

      ignition::math::Pose3d currPose_ = linkPtr_->WorldPose();
      Vector3<double> currPos = currPose_.Pos();


      Vector3<double> initPos(-0.5, 0.2, 0.1);
      double radian = 0.1;
      Vector3<double> center = initPos;
      center[2] += radian;
      Vector3<double> baseRadianVec = initPos - center;

      Vector3<double> currRadianVec = currPos - center;

      double linSpd = 0.1;

      double theta = acos(baseRadianVec.Dot(currRadianVec)
        /(currRadianVec.Length()*baseRadianVec.Length()));
      theta = currRadianVec[1] > 0 ? theta : -theta;

      linkPtr_->SetLinearVel({
        0.0,
        cos(theta)*linSpd,
        sin(theta)*linSpd
      });

      if(clock()-this->lastUpd > CLOCKS_PER_SEC/this->updFreq){
        this->lastUpd = clock();
        cout << "------\n"
          << "Current Time: " << this->lastUpd << '\n'
          << "Current Position: " << currPos << '\n'
          << "Theta (rad) = " << theta << '\n'
          << endl;
      }

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Timer
    private: long int begin;
    private: long int delay;
    private: long int lastUpd;
    private: long int updFreq;
    // private: int lastTimeStamp
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}