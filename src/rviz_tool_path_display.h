#include <OgreMaterialManager.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_rendering/objects/axes.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

namespace rviz_common
{

class ToolPathDisplay : public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseArray>
{
  Q_OBJECT
public:
  ToolPathDisplay();
  virtual ~ToolPathDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;
  void processMessage(const std::shared_ptr<const geometry_msgs::msg::PoseArray> msg) override;

private:
  bool setTransform(std_msgs::msg::Header const& header);
  void updateAxes();
  void updateDisplay();
  rviz_rendering::Axes* makeAxes();
  void updatePoints();
  void updateLines();
  void updateText();

  struct OgrePose
  {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };

  std::vector<OgrePose> poses_;

  // Axes Display
  boost::ptr_vector<rviz_rendering::Axes> axes_;
  Ogre::SceneNode* axes_node_;
  properties::FloatProperty* axes_length_property_;
  properties::FloatProperty* axes_radius_property_;
  BoolProperty* axes_visibility_property_;

  // Points Display
  Ogre::ManualObject* pts_object_;
  Ogre::MaterialPtr pts_material_;
  BoolProperty* pts_visibility_property_;
  properties::ColorProperty* pts_color_property_;
  properties::FloatProperty* pts_size_property_;

  // Lines Display
  Ogre::ManualObject* lines_object_;
  Ogre::MaterialPtr lines_material_;
  BoolProperty* lines_visibility_property_;
  properties::ColorProperty* lines_color_property_;

  // Text
  Ogre::SceneNode* start_text_node_;
  rviz_rendering::MovableText* start_text_;
  Ogre::SceneNode* end_text_node_;
  rviz_rendering::MovableText* end_text_;
  BoolProperty* text_visibility_property_;
  properties::FloatProperty* text_size_property_;

private Q_SLOTS:
  void updateAxesGeometry();
  void updateAxesVisibility();

  void updatePtsVisibility();
  void updatePtsColor();
  void updatePtsSize();

  void updateLinesVisibility();
  void updateLinesColor();

  void updateTextVisibility();
  void updateTextSize();
};

}  // namespace rviz_common
