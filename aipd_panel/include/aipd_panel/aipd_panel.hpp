#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <ros/ros.h>
#include <rviz/panel.h>
#include <vector>
#include <string>
#include <std_srvs/Trigger.h>

/** 
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_aipd_panel.h>

// Other ROS dependencies
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>


namespace aipd_panel
{
    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class aipdPanel : public rviz::Panel
    {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
        Q_OBJECT

        public:
            /**
             *  QWidget subclass constructors usually take a parent widget
             *  parameter (which usually defaults to 0).  At the same time,
             *  pluginlib::ClassLoader creates instances by calling the default
             *  constructor (with no arguments). Taking the parameter and giving
             *  a default of 0 lets the default constructor work and also lets
             *  someone using the class for something else to pass in a parent
             *  widget as they normally would with Qt.
             */
            aipdPanel(QWidget * parent = 0);

            void speed_limit_callback(const std_msgs::Int16::ConstPtr& msg);

            void num_objects_callback(const std_msgs::Int16::ConstPtr& msg);

            void ticket_description_callback(const std_msgs::String::ConstPtr& msg);

            void ego_speed_callback(const std_msgs::Int16::ConstPtr& msg);

            /**
             *  Now we declare overrides of rviz::Panel functions for saving and
             *  loading data from the config file.  Here the data is the topic name.
             */
            virtual void save(rviz::Config config) const;
            virtual void load(const rviz::Config & config);


        /**
         *  Here we declare some internal slots.
         */
        private Q_SLOTS:

            void update_display(void);

        Q_SIGNALS:

            void display_changed();

        /**
         *  Finally, we close up with protected member variables
         */
        protected:
            // UI pointer
            std::shared_ptr<Ui::aipd_panel> ui_;
            // ROS declaration
            ros::NodeHandle nh_;
            ros::Subscriber speed_limit_sub_;
            ros::Subscriber detected_objects_sub_;
            ros::Subscriber speeding_tickets_sub_;
            ros::Subscriber ego_velocity_sub_;
        
        private:
            // Display variables
            int num_objects;
            int num_tickets;
            int speed_limit;
            int ego_speed;
            std::vector<std::string> ticket_queue;
            // Helper function to format Qt strings
            QString format_string(std::string text);
            
    };
} // namespace aipd_panel
#endif