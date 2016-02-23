#include <ros/ros.h>
#include <tf2_ros/buffer_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <giskard_msgs/WholeBodyPositionGoal.h>
#include <memory>

class GoalTransformer
{
  public:
    GoalTransformer(const ros::NodeHandle& nh, 
        const std::string& ns, const std::string& frame_id) : 
      nh_( nh ), frame_id_( frame_id ) 
    {
      buffer_ = std::make_shared<tf2_ros::BufferClient>(ns);
    }

    ~GoalTransformer() {}

    bool start(const ros::Duration& duration)
    {
      if(!buffer_->waitForServer(duration))
      {
        ROS_ERROR("Wait for TF2 BufferServer timed out. Aborting.");
        return false;
      }

      pub_ = nh_.advertise<giskard_msgs::WholeBodyPositionGoal>("out", 1);
      sub_ = nh_.subscribe("in", 1000, &GoalTransformer::callback, this);
      return true;
    }

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::BufferClient> buffer_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string frame_id_;

    void callback(const giskard_msgs::WholeBodyPositionGoal::ConstPtr& in_msg)
    {
      giskard_msgs::WholeBodyPositionGoal out_msg;

      try
      {
        buffer_->transform(in_msg->left_ee_goal, out_msg.left_ee_goal, frame_id_);
      }
      catch(tf2::TransformException& ex)
      {
        ROS_ERROR("Could not transform goal for left EE. Error: %s", ex.what());
        return;
      }

      try
      {
        buffer_->transform(in_msg->right_ee_goal, out_msg.right_ee_goal, frame_id_);
      }
      catch(tf2::TransformException& ex)
      {
        ROS_ERROR("Could not transform goal for right EE. Error: %s", ex.what());
        return;
      }

      pub_.publish(out_msg);
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_transformer");
  ros::NodeHandle nh("~");

  std::string frame_id;

  if (!nh.getParam("frame_id", frame_id))
  {
    ROS_ERROR("Parameter 'frame_id' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  GoalTransformer pst(nh, "/tf2_buffer_server", frame_id);
  if (!pst.start(ros::Duration(2.0)))
    return 0;

  ros::spin();

  return 0;
}
