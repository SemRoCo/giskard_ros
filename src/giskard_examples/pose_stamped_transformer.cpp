#include <ros/ros.h>
#include <tf2_ros/buffer_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>

class PoseStampedTransformer
{
  public:
    PoseStampedTransformer(const ros::NodeHandle& nh, 
        const std::string& ns, const std::string& frame_id) : 
      nh_( nh ), frame_id_( frame_id ) 
    {
      buffer_ = std::make_shared<tf2_ros::BufferClient>(ns);
    }

    ~PoseStampedTransformer() {}

    bool start(const ros::Duration& duration)
    {
      if(!buffer_->waitForServer(duration))
      {
        ROS_ERROR("Wait for TF2 BufferServer timed out. Aborting.");
        return false;
      }

      pub_ = nh_.advertise<geometry_msgs::PoseStamped>("out", 1);
      sub_ = nh_.subscribe("in", 1000, &PoseStampedTransformer::callback, this);
      return true;
    }

  private:
    ros::NodeHandle nh_;
    std::shared_ptr<tf2_ros::BufferClient> buffer_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string frame_id_;

    void callback(const geometry_msgs::PoseStamped::ConstPtr& in_msg)
    {
      try
      {
        geometry_msgs::PoseStamped out_msg;
        buffer_->transform(*in_msg, out_msg, frame_id_, ros::Duration(0.1));
        pub_.publish(out_msg);
      }
      catch(tf2::TransformException& ex)
      {
        ROS_ERROR("Could not transform PoseStamped. Error: %s", ex.what());
      }
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "pose_stamped_transformer");
  ros::NodeHandle nh("~");

  std::string frame_id;

  if (!nh.getParam("frame_id", frame_id))
  {
    ROS_ERROR("Parameter 'frame_id' not found in namespace '%s'.", nh.getNamespace().c_str());
    return 0;
  }

  PoseStampedTransformer pst(nh, "/tf2_buffer_server", frame_id);
  if (!pst.start(ros::Duration(2.0)))
    return 0;

  ros::spin();

  return 0;
}
