/*********************************************************************************************************************/

#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

/*********************************************************************************************************************/

#define         RANGES_SIZE         ( 1080 )            // Size of the ranges[] array
#define         ANGLE_MIN           ( -2.35 )           // Laser scan minimum angle in radians
#define         ANGLE_MAX           ( 2.35 )            // Laser scan maximum angle in radians
#define         ANGLE_LEFT_1        ( 1.5708 )          // First laser scan angle in radians used to calculate distance from the left wall ( 90 degrees )
#define         ANGLE_LEFT_2        ( 0.698132 )        // Second laser scan angle in radians used to calculate distance from the left wall ( 40 degrees )
#define         DESIRED_DISTANCE    ( 0.8 )             // Desired distance to the wall
#define         AC                  ( 0.25 )

#define         KP                  ( 25 )              // PID proportional gain
#define         KI                  ( 0 )               // PID integral gain
#define         KD                  ( 0.1 )             // PID derivative gain

#define         MAX_NANOSEC         ( 10000000000 )

#define         QOS                 ( 10 )              // Quality of service for ROS nodes

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

        /*
        angles.reserve( RANGES_SIZE );

        angles = makeAnglesVector( ANGLE_MIN, ANGLE_MAX, RANGES_SIZE );

        for( int i = 0; i < RANGES_SIZE; i++ )
        {
            RCLCPP_INFO( this->get_logger(), "[ %i ]: %f", i, angles[ i ] );
        }
        */
    }

private:
    
    double prev_error;
    double error;
    double integral;
    double derivative;
    uint32_t prev_time;
    uint32_t current_time;
    uint32_t dt;
    double steering_angle;
    double velocity;

    // Topics
    std::string laserscan_topic = "/scan";
    std::string drive_topic = "/drive";
    
    // ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr sub_laser_scan;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;

    // Range vector
    std::vector<float> ranges;
    //std::vector<float> angles;


    double getRange( std::vector<float>* range_data, double angle )
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
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
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            desired_distance: desired distance to the wall

        Returns:
            error: calculated error
        */

        double b;
        double a;
        double alpha;
        double theta;

        b = getRange( range_data, ANGLE_LEFT_1 );

        a = getRange( range_data, ANGLE_LEFT_2 );

        theta = ANGLE_LEFT_1 - ANGLE_LEFT_2;

        alpha = atan( ( ( a * cos( theta ) ) - b ) / ( a * sin( theta ) ) ); 

        error = -( desired_distance - ( ( b * cos( alpha ) ) + ( AC * sin ( alpha ) ) ) );
    }

    void pidControl()
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */

        // Use kp, ki & kd to implement a PID controller
        if( dt > 0 )
        {
            integral += ( error - prev_error ) * dt * ( 10 ^ ( -6 ) );
            derivative =  ( error - prev_error ) / ( dt * ( 10 ^ ( -6 ) ) );

            steering_angle = ( KP * error ) + ( KI * integral ) + ( KD * derivative );

            RCLCPP_INFO( this->get_logger(), "Steering angle: %f", steering_angle );

            steering_angle = steering_angle * 3.14159265359 / 180;

            // TODO: add saturation for steering angle
            
            if( abs( steering_angle ) < 0.174533 )
            {
                velocity = 1.5;
            }
            else if( abs( steering_angle ) < 0.349066 )
            {
                velocity = 1;
            }
            else
            {
                velocity = 0.5;
            }

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
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
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

    /*
    static std::vector<float> makeAnglesVector( double start_angle, double end_angle, double n_elements ) 
    {

        std::vector<float> angles_vector;
        float step = ( end_angle - start_angle ) / ( n_elements - 1 );

        // Reserve memory for the vector
        angles_vector.reserve( n_elements );

        // Populate the vector
        for( int index = 0; index < n_elements; index++ )
        {
            angles_vector.push_back( start_angle + ( index * step ) );
        }
        
        return angles_vector;
    }
    */
    
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