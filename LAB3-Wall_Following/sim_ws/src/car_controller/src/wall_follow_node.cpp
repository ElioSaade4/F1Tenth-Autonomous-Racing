/*********************************************************************************************************************/

#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

/*********************************************************************************************************************/

#define         RANGES_SIZE             ( 1080 )                            // Size of the ranges[] array
#define         ANGLE_MIN               ( -2.35 )                           // Laser scan minimum angle in radians
#define         ANGLE_MAX               ( 2.35 )                            // Laser scan maximum angle in radians
#define         ANGLE_LEFT_1            ( 1.5708 )                          // First laser scan angle in radians used to calculate distance from the left wall ( 90 degrees )
#define         ANGLE_LEFT_2            ( 0.698132 )                        // Second laser scan angle in radians used to calculate distance from the left wall ( 40 degrees )
#define         THETA                   ( ANGLE_LEFT_1 - ANGLE_LEFT_2 )     // Angle difference of the laser scans sued to calculate distance from the left wall
#define         DESIRED_DISTANCE        ( 0.8 )                             // Desired distance to the wall
#define         AC                      ( 0.25 )

#define         KP                      ( 25 )                              // PID proportional gain
#define         KI                      ( 0 )                               // PID integral gain
#define         KD                      ( 0.1 )                             // PID derivative gain

#define         STEERING_ACTUATOR_MIN   ( -25 )                             // Minimum actuator steering angle (degrees) 
#define         STEERING_ACTUATOR_MAX   ( 25 )                              // Maximum actuator steering angle (degrees)

#define         MAX_NANOSEC             ( 10000000000 )                     // Max number of nanoseconds. Used for when the timer counter overflows

#define         QOS                     ( 10 )                              // Quality of service for ROS nodes

/*********************************************************************************************************************/

using std::placeholders::_1;
using namespace std;


class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node( "wall_follow_node" )
    {
        /*
         * Constructor for the class WallFollow. Initializes ROS nodes and allocates memory for vectors
         */

        // Create subscriber to the LaserScan message of the /scan topic
        sub_laser_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
                            laserscan_topic, QOS, std::bind( &WallFollow::laserScanCallback, this, _1 ) );

        // Create publisher to /drive topic 
        pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>( drive_topic, QOS );

        // Initialize variables
        prev_error = 0;
        prev_time = ( this->now() ).nanoseconds();
        integral = 0;
        derivative = 0;

        // Allocate space for all the vectors
        ranges.reserve( RANGES_SIZE );

    }

private:
    
    double prev_error;              // Error term of PID controller in the previous time step
    double error;                   // Error term of PID controller in the current time step
    double integral;                // Intergral term of PID controller in the current time step
    double derivative;              // Derivative term of the PID controller in the current time step
    uint32_t prev_time;             // Time stamp of the previous time step
    uint32_t current_time;          // Time stamp of the current time step
    uint32_t dt;                    // Time difference between previous and current time steps
    double steering_angle;          // Value of steering angle command
    double velocity;                // Value of velocity command

    // Topics
    std::string laserscan_topic = "/scan";
    std::string drive_topic = "/drive";
    
    // ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr sub_laser_scan;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;

    // Range vector
    std::vector<float> ranges;


    double getRange( std::vector<float>* range_data, double angle )
    {
        /*
         * Helper function to return the range measurement corresponding to a given angle. Make sure you take care of NaNs and infs.
         * 
         * Args:
         *    range_data: single range array from the LiDAR
         *    angle: between angle_min and angle_max of the LiDAR
         * 
         * Returns:
         *    range: range measurement in meters at the given angle
         */
        
        int index;
        float angle_step;
        float range;
        
        angle_step = ( ANGLE_MAX - ANGLE_MIN ) / ( RANGES_SIZE - 1 );
        index = round( ( angle - ANGLE_MIN ) / angle_step );
        range = ( *range_data )[ index ];

        return range;
    }


    void getError( std::vector<float>* range_data, double desired_distance )
    {
        /*
         * Calculates the error to the wall. Follow the wall to the left (driving counter clockwise).
         *
         * Args:
         *    range_data: single range array from the LiDAR
         *    desired_distance: desired distance to the wall
         *
         * Returns:
         *    error: calculated error
         */

        double b;
        double a;
        double alpha;

        b = getRange( range_data, ANGLE_LEFT_1 );

        a = getRange( range_data, ANGLE_LEFT_2 );

        alpha = atan( ( ( a * cos( THETA ) ) - b ) / ( a * sin( THETA ) ) ); 

        error = -( desired_distance - ( ( b * cos( alpha ) ) + ( AC * sin ( alpha ) ) ) );
    }

    void pidControl()
    {
        /*
         * PID controller to publish vehicle controls based on the calculated error
         */

        // Use kp, ki & kd to implement a PID controller
        if( dt > 0 )
        {
            integral += ( error ) * dt * ( 10 ^ ( -6 ) );
            derivative =  ( error - prev_error ) / ( dt * ( 10 ^ ( -6 ) ) );

            steering_angle = ( KP * error ) + ( KI * integral ) + ( KD * derivative );

            RCLCPP_INFO( this->get_logger(), "Steering angle: %f", steering_angle );

            // Steering angle actuator limits
            if( steering_angle < STEERING_ACTUATOR_MIN )
            {
                steering_angle = STEERING_ACTUATOR_MIN;
            }
            else if( steering_angle > STEERING_ACTUATOR_MAX )
            {
                steering_angle = STEERING_ACTUATOR_MAX;
            }

            //TODO: add antiwindup for integral term
            
            // Piecewsie linear function of velocity vs. steering angle to go faster in straights and slower when turning
            if( abs( steering_angle ) < 3 )
            {
                velocity = ( - 0.667 * abs( steering_angle ) ) + 4.5;
            }
            else if( abs( steering_angle ) < 10 )
            {
                velocity = ( - 0.214 * abs( steering_angle ) ) + 3.142;
            }
            else
            {
                velocity = 1;
            }

            // Convert steering angle to radians for publishing
            steering_angle = steering_angle * 3.14159265359 / 180;

            // Fill in drive message and publish
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->now();
            drive_msg.header.frame_id = "drive_frame";
            drive_msg.drive.speed = velocity;
            drive_msg.drive.steering_angle = steering_angle;
            pub_drive->publish( drive_msg );
        }
        
    }

    void laserScanCallback( const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg ) 
    {
        /*
         * Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.
         * 
         * Args:
         *     msg: Incoming LaserScan message
         *
         * Returns:
         *     None
         */
        
        current_time = ( this->now() ).nanoseconds();
        dt = current_time - prev_time;

        if( current_time < prev_time )
        {
            dt += MAX_NANOSEC;
        }

        ranges = scan_msg->ranges;

        getError( &ranges, DESIRED_DISTANCE );

        pidControl();

        prev_time = current_time;

    }
    
};


int main( int argc, char ** argv ) 
{
    /* 
     * Main function that is called when the node is run.
     */

    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<WallFollow>() );
    rclcpp::shutdown();
    return 0;
}