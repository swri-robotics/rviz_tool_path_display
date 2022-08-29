#include <OgreMaterial.h>
#include <geometry_msgs/PoseArray.h>
#include <rviz/message_filter_display.h>

#include <boost/ptr_container/ptr_vector.hpp>

namespace rviz
{
class ColorProperty;
class FloatProperty;
class Axes;
class MovableText;
class BillboardLine;

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
  boost::ptr_vector<rviz::Axes> axes_;
  Ogre::SceneNode* axes_node_;
  rviz::FloatProperty* axes_length_property_;
  rviz::FloatProperty* axes_radius_property_;
  BoolProperty* axes_visibility_property_;

  // Points Display
  Ogre::ManualObject* pts_object_;
  Ogre::MaterialPtr pts_material_;
  BoolProperty* pts_visibility_property_;
  ColorProperty* pts_color_property_;
  FloatProperty* pts_size_property_;

  // Lines Display
  BillboardLine* lines_object_;
  //  Ogre::ManualObject* lines_object_;
  //  Ogre::MaterialPtr lines_material_;
  BoolProperty* lines_visibility_property_;
  ColorProperty* lines_color_property_;

  // Text
  Ogre::SceneNode* start_text_node_;
  MovableText* start_text_;
  Ogre::SceneNode* end_text_node_;
  MovableText* end_text_;
  BoolProperty* text_visibility_property_;
  FloatProperty* text_size_property_;

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

}  // namespace rviz
