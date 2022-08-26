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
  void updateArrows2d();
  void updateArrows3d();
  void updateAxes();
  void updateDisplay();
  rviz::Axes* makeAxes();
  rviz::Arrow* makeArrow3d();

  struct OgrePose
  {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };

  std::vector<OgrePose> poses_;
  boost::ptr_vector<rviz::Arrow> arrows3d_;
  boost::ptr_vector<rviz::Axes> axes_;

  Ogre::SceneNode* arrow_node_;
  Ogre::SceneNode* axes_node_;
  Ogre::ManualObject* manual_object_;

  rviz::EnumProperty* shape_property_;
  rviz::ColorProperty* arrow_color_property_;
  rviz::FloatProperty* arrow_alpha_property_;

  rviz::FloatProperty* arrow2d_length_property_;

  rviz::FloatProperty* arrow3d_head_radius_property_;
  rviz::FloatProperty* arrow3d_head_length_property_;
  rviz::FloatProperty* arrow3d_shaft_radius_property_;
  rviz::FloatProperty* arrow3d_shaft_length_property_;

  rviz::FloatProperty* axes_length_property_;
  rviz::FloatProperty* axes_radius_property_;

private Q_SLOTS:
  /// Update the interface and visible shapes based on the selected shape type.
  void updateShapeChoice();

  /// Update the arrow color.
  void updateArrowColor();

  /// Update the flat arrow geometry.
  void updateArrow2dGeometry();

  /// Update the 3D arrow geometry.
  void updateArrow3dGeometry();

  /// Update the axes geometry.
  void updateAxesGeometry();
};

}  // namespace rviz
