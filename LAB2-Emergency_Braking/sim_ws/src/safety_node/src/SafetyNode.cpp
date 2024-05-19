/*********************************************************************************************************************/

#include <memory>
#include <math.h>
#include <algorithm>
#include <functional>
#include <limits> 
#include <bits/stdc++.h> 

/*********************************************************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

/*********************************************************************************************************************/

#define         ANGLE_MIN               -2.35           // Laser scan minimum angle
#define         ANGLE_MAX               2.35            // Laser scan maximum angle
#define         ANGLE_INCREMENT         0.004352        // Laser scan angle increment
#define         RANGES_SIZE             1080            // Size of the ranges[] array
#define         RANGE_MIN               0               // Laser scan minimum range
#define         RANGE_MAX               30              // Laser scan maximum range

// The below thresholds have been tested for a speed 0.5 only because the simulator was glitching at higher speeds
#define         ITTC_THRESHOLD_FWD      1               // iTTC braking threshold for forward driving
#define         ITTC_THRESHOLD_REV      2               // iTTC braking threshold for reverse driving (braking in reverse was very slow)


#define         QUALITY_OF_SERVICE      10              // Quality of service for ROS nodes

/*********************************************************************************************************************/

using std::placeholders::_1;
using namespace std;


class SafetyNode : public rclcpp::Node
{
    public:

        SafetyNode() : Node( "safety_node" )  
        {
            /*
             * Constructor for the class SafetyNode. Initializes ROS nodes and allocates memory for vectors
             */
            
            // Create subscriber to the LaserScan message of the /scan topic
            sub_laser_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
                            "scan", 10, std::bind( &SafetyNode::laserScanCallback, this, _1 ) );
            
            // Create subscriber to the Odometry message of the /ego_racecar/odom topic
            sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
                            "ego_racecar/odom", 10, std::bind( &SafetyNode::odometryCallback, this, _1 ) );

            // Create publisher to /drive topic 
            pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>( "drive", 10 );

            // Allocate space for all the vectors
            current_ranges.reserve( RANGES_SIZE );
            angle_cosines.reserve( RANGES_SIZE );
            range_rate.reserve( RANGES_SIZE );
            iTTC.reserve( RANGES_SIZE );

            // Initialize the vector storing the cosine of the laser beam angles
            angle_cosines = makeCosinesVector( ANGLE_MIN, ANGLE_INCREMENT, ANGLE_MAX );
        }


    private:

        // Class properties
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr sub_laser_scan;   
        rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_odometry;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;

        std::vector<float> current_ranges;
        std::vector<float> angle_cosines;
        std::vector<float> range_rate;
        std::vector<float> iTTC;
        double linear_vel_x;
        float min_iTTC ; 


        // Class Functions
        void laserScanCallback( const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg )
        {
            /*
             * Callback function for receiving LaserScan messages.
             * Processes the laser scan data, calculates iTTC for all scan angles, and brakes the car if needed.
             */

            float iTTC_threshold;
            
            // Extract the range measurements
            current_ranges = scan_msg->ranges;

            min_iTTC = std::numeric_limits<float>::max();

            // Iterate over all the range measurements
            for ( int index = 0; index < RANGES_SIZE; index++ )
            {
                if ( range_rate[ index ] <= 0 )     // moving away from object
                {
                    // This was chosen instead of replacing the range_rate by 0 to avoid the -0 that was being stored
                    iTTC[ index ] = std::numeric_limits<float>::max();
                }
                else        // moving towards object
                {
                    // Calculate iTTC
                    iTTC[ index ] = current_ranges [ index ] / range_rate [ index ];
                }

                // Update the smallest iTTC
                if ( iTTC[ index ] < min_iTTC )
                {
                    min_iTTC = iTTC[ index ];
                }
            }

            // Set the threshold for braking based on driving direction
            if ( 0 < linear_vel_x)      // Driving forward
            {
                iTTC_threshold = ITTC_THRESHOLD_FWD;
            }
            else               // Driving in reverse
            {
                iTTC_threshold = ITTC_THRESHOLD_REV;
            }

            // Braking logic
            if ( min_iTTC < iTTC_threshold )       // Collision is imminent
            {
                RCLCPP_INFO( this->get_logger(), "Min iTTC less than threshold: %f", min_iTTC );

                // Publish AckermannDrive message with speed 0 to brake
                auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

                drive_msg.header.stamp = this->now();
                drive_msg.header.frame_id = "drive_frame";
                drive_msg.drive.speed = 0;

                pub_drive->publish( drive_msg );
            }
            
        }


        void odometryCallback( const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg )
        {
            /*
             * Callback function for receiving Odometry messages. 
             * Updates the current velocity and its projection on every laser scan angle.
             */

            // Update Vx
            linear_vel_x = odometry_msg->twist.twist.linear.x;

            // Update the velocity projections Vx cos(theta)
            std::transform( angle_cosines.begin(), angle_cosines.end(), range_rate.begin(),
                    std::bind( std::multiplies<float>(), std::placeholders::_1, linear_vel_x ) );
        }


        static std::vector<float> makeCosinesVector( double start_angle, double step, double end_angle ) 
        {
            /*
             * Initializes the vector of cosines of the laser scan angles given the start angle, end angle, and angle step.
             */

            std::vector<float> vec;
            float value = start_angle;

            // Reserve memory for the vector
            vec.reserve( ( ( end_angle - start_angle ) / step ) + 1 );

            // Populate the vector
            while ( value <= end_angle ) 
            {
                vec.push_back( cos( value ) );
                value += step;
            }
            
            return vec;
        }

};

/*********************************************************************************************************************/

int main(int argc, char * argv[])
{
    /* 
     * Main function that is called when the node is run.
     */

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();

    return 0;
}

/*********************************************************************************************************************/