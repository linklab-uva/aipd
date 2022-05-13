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

        detected_objects_sub_ = nh_.subscribe<std_msgs::Int16>("num_objects", 2, &aipdPanel::num_objects_callback, this);
        speeding_tickets_sub_ = nh_.subscribe<std_msgs::String>("ticket_description", 2, &aipdPanel::ticket_description_callback, this);
        ego_velocity_sub_ = nh_.subscribe<std_msgs::Int16>("ego_velocity", 2, &aipdPanel::ego_speed_callback, this);
        speed_limit_pub_ = nh_.advertise<std_msgs::Int16>("speed_limit", 2);

        connect(this, SIGNAL(display_changed()), this, SLOT(update_display()));
        
        std::string sign_path = ros::package::getPath("aipd_panel") + "/resource/images/sign.png";
        std::string logo_path = ros::package::getPath("aipd_panel") + "/resource/images/aipd_logo.png";
        
        QPixmap sign_image(QString::fromStdString(sign_path));
        QPixmap logo_image(QString::fromStdString(logo_path));

        ui_->sign->setPixmap(sign_image);
        ui_->logo->setPixmap(logo_image);

        ui_->sign->setAlignment(Qt::AlignCenter | Qt::AlignVCenter);
        ui_->speed_limit->raise();
 
        ego_speed = 0;
        num_objects = 0;
        num_tickets = 0;
        speed_limit = 0;

        ego_speed_text_ = ui_->ego_speed->text();
        num_objects_text_ = ui_->num_detections->text();
        num_tickets_text_ = ui_->num_tickets->text();
        speed_limit_text_ = ui_->speed_limit->text();


        connect(ui_->speed_slider, SIGNAL(sliderReleased()), this, SLOT(send_speed_limit()));
    }

    void aipdPanel::num_objects_callback(const std_msgs::Int16::ConstPtr& msg)
    {
        num_objects = msg->data;
        Q_EMIT display_changed();
    }

    void aipdPanel::ticket_description_callback(const std_msgs::String::ConstPtr& msg)
    {
        ticket_queue.push_back(msg->data);
        num_tickets++;
        Q_EMIT display_changed();
    }

    void aipdPanel::ego_speed_callback(const std_msgs::Int16::ConstPtr& msg)
    {
        ego_speed = msg->data;
        Q_EMIT display_changed();
    }

    void aipdPanel::update_display(void)
    {
        ui_->num_detections->setText(num_objects_text_.arg((QString) ("Number of Detections: " + std::to_string(num_objects)).c_str()));
        ui_->ego_speed->setText(ego_speed_text_.arg((QString) ("Ego Speed: " + std::to_string(ego_speed) + " mph").c_str()));
        ui_->num_tickets->setText(num_tickets_text_.arg((QString) ("Ticket Issued: " + std::to_string(num_tickets)).c_str()));
        ui_->speed_limit->setText(speed_limit_text_.arg((QString) std::to_string(speed_limit).c_str()));
        for (std::string ticket : ticket_queue)
        {
            ui_->ticket_log->addItem((QString) ticket.c_str());
        }
        ticket_queue.clear();
    }

    QString aipdPanel::format_string(std::string text)
    {
        std::string formatted_text;
        if (isdigit(text[0])) {
            formatted_text = "<html><head/><body><p align=\"center\"><br/></p><p align=\"center\"><span style=\" font-size:26pt; font-weight:600;\">" + text + "</span></p></body></html>;";
        } else {
            formatted_text = "<html><head/><body><p><span style=\" font-size:14pt;\">"+ text + "</span></p></body></html>";
        }
        return (QString) formatted_text.c_str();
    }

    void aipdPanel::send_speed_limit()
    {
        speed_limit = ui_->speed_slider->value();
        std_msgs::Int16 speed_limit_msg;
        speed_limit_msg.data = speed_limit;
        speed_limit_pub_.publish(speed_limit_msg);
        Q_EMIT display_changed();
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