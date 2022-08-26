#include <geometry_msgs/PoseArray.h>
#include <rviz/message_filter_display.h>

#include <boost/ptr_container/ptr_vector.hpp>

namespace rviz
{
class EnumProperty;
class ColorProperty;
class FloatProperty;
class Arrow;
class Axes;

class ToolPathDisplay : public rviz::MessageFilterDisplay<geometry_msgs::PoseArray>
{
  Q_OBJECT
public:
  ToolPathDisplay();
  virtual ~ToolPathDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void processMessage(const geometry_msgs::PoseArray::ConstPtr& msg) override;

private:
  bool setTransform(std_msgs::Header const& header);
  void updateAxes();
  void updateDisplay();
  rviz::Axes* makeAxes();

  struct OgrePose
  {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };

  std::vector<OgrePose> poses_;

  rviz::EnumProperty* shape_property_;

  // Axes Display
  boost::ptr_vector<rviz::Axes> axes_;
  Ogre::SceneNode* axes_node_;
  rviz::FloatProperty* axes_length_property_;
  rviz::FloatProperty* axes_radius_property_;

private Q_SLOTS:
  /// Update the interface and visible shapes based on the selected shape type.
  void updateShapeChoice();

  /// Update the axes geometry.
  void updateAxesGeometry();
};

}  // namespace rviz
