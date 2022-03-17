#include "aipd_panel/aipd_panel.hpp"
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(aipd_panel::aipdPanel, rviz::Panel)

namespace aipd_panel
{
    aipdPanel::aipdPanel(QWidget * parent)
    :   rviz::Panel(parent),
        ui_(std::make_shared<Ui::two_button>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        // Define ROS publisher
        button_1_pub_ = nh_.advertise<std_msgs::Bool>("button_1_topic", 1);
        button_2_pub_ = nh_.advertise<std_msgs::Bool>("button_2_topic", 1);

        // Declare ROS msg_
        msg_.data = true;

        connect(ui_->pushButton_1, SIGNAL(clicked()), this, SLOT(button_one()));
        connect(ui_->pushButton_2, SIGNAL(clicked()), this, SLOT(button_two()));
    }


    void aipdPanel::button_one()
    {
        ROS_INFO_STREAM("Button one pressed.");
        button_1_pub_.publish(msg_);
    }


    void aipdPanel::button_two()
    {
        ROS_INFO_STREAM("Button two pressed.");
        button_2_pub_.publish(msg_);
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