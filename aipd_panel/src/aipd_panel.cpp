#include "aipd_panel/aipd_panel.hpp"
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(aipd_panel::aipdPanel, rviz::Panel)

namespace aipd_panel
{
    aipdPanel::aipdPanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::aipd_panel>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        // Define ROS publisher

        speed_limit_sub_ = nh_.subscribe<std_msgs::Int16>("speed_limit", 1, &aipdPanel::speed_limit_callback, this);
        detected_objects_sub_ = nh_.subscribe<aipd_msgs::DetectedObjectArray>("detected_objects", 4, &aipdPanel::detected_objects_callback, this);
        speeding_tickets_sub_ = nh_.subscribe<aipd_msgs::Ticket>("tickets", 5, &aipdPanel::speeding_tickets_callback, this);
    }


    void aipdPanel::speed_limit_callback(const std_msgs::Int16::ConstPtr& msg)
    {

    }

    void detected_objects_callback(const aipd_msgs::DetectedObjectArray::ConstPtr& msg)
    {

    }

    void speeding_tickets_callback(const aipd_msgs::Ticket::ConstPtr& msg)
    {

    }


    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void aipdPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void aipdPanel::load(const rviz::Config & config)
    {
        rviz::Panel::load(config);
    }
} // namespace aipd_panel