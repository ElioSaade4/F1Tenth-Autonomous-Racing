#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

/*********************************************************************************************************************/

#define         RANGES_SIZE                 ( 1080 )                                                            // Size of the laser scan array (ranges[])
#define         RANGES_ANGLE_MIN            ( -2.35 )                                                           // Laser scan minimum angle in radians
#define         RANGES_ANGLE_MAX            ( 2.35 )                                                            // Laser scan maximum angle in radians
#define         RANGES_ANGLE_STEP           ( ( RANGES_ANGLE_MAX - RANGES_ANGLE_MIN ) / ( RANGES_SIZE - 1 ) )   // Angle difference between 2 consecutive laser scan measurements

#define         STEERING_ACTUATOR_MIN       ( -0.636332 )                                                       // Minimum actuator steering angle (radians) 
#define         STEERING_ACTUATOR_MAX       ( 0.636332 )                                                        // Maximum actuator steering angle (radians)

#define         MAX_NANOSEC                 ( 10000000000 )                                                     // Max number of nanoseconds. Used for when the timer counter overflows
#define         QOS                         ( 10 )                                                              // Quality of service for ROS nodes

#define         CAR_WIDTH                   ( 0.25 )                                                             // Car width in meters

#define         RANGE_THRESHOLD             ( 1.5 )

/*********************************************************************************************************************/

using std::placeholders::_1;
using namespace std;

class ReactiveGapFollow : public rclcpp::Node {

public:
    ReactiveGapFollow() : Node( "gap_follow_node" )
    {
        /*
         * Constructor for the class WallFollow. Initializes ROS nodes and allocates memory for vectors
         */

        // Create subscriber to the LaserScan message of the /scan topic
        sub_laser_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
                            laserscan_topic, QOS, std::bind( &ReactiveGapFollow::laserScanCallback, this, _1 ) );

        // Create publisher to /drive topic 
        pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>( drive_topic, QOS ); 

        // Allocate space for all the vectors
        ranges.reserve( RANGES_SIZE ); 

        // Set the start and end indices for a laser scan view of 180 degrees (-90 to +90) instead of 270 degrees to prevent the car
        // from turning backwards in some edge cases
        range_180_degrees[ 0 ] = 0; //ceil( ( -1.5708 - RANGES_ANGLE_MIN ) / RANGES_ANGLE_STEP );
        range_180_degrees[ 1 ] = 1079; //ceil( ( 1.5708 - RANGES_ANGLE_MIN ) / RANGES_ANGLE_STEP );             
    }

private:

    // Variables
    uint16_t range_180_degrees[ 2 ];        // vector containing the start and end indices of 180 degrees view in the ranges vector
    uint16_t range_car_bubble[ 2 ];         // vector containing the start and end indices of the car bubble around the closest object in the ranges vector 
    uint16_t range_free_space[ 2 ];         // vector containing the start and end indices of the biggest gap in the ranges vector
    float steering_angle;                   // value of car steering wheel command in radians
    float velocity;                         // value of car velocity in m/s

    // Topics
    std::string laserscan_topic = "/scan";  // name of the topic where the LaserScan message is published
    std::string drive_topic = "/drive";     // name of the topic where the AckermannDriveStamped message is published
    
    // ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr sub_laser_scan;       // subscriber to the LaserScan message
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;     // publisher to the AckermannDriveStamped message

    // Range vector
    std::vector<float> ranges;              // vector of laser scan values starting from -2.35 radians (right side of the car) to +2.35 radians (left of the car)
    
    
    void laserScanPreprocess( void )
    {   
        uint16_t index_closest;
        float distance_closest;
        float distance_between_scans;
        uint16_t n_car_width;

        // Get the index of the closest laser scan
        index_closest = 0;
        distance_closest = ranges[ index_closest ];

        for( uint16_t iterator = range_180_degrees[ 0 ]; iterator < range_180_degrees[ 1 ]; iterator++ )
        {
            if( ranges[ iterator ] < ranges[ index_closest ] )
            {
                index_closest = iterator;
            }
        }

        distance_closest = ranges[ index_closest ];

        // Approximated separation between 2 adjacent laser scans based on the distance to the closest object
        distance_between_scans = tan( RANGES_ANGLE_STEP ) * distance_closest;

        // Number of scans that cover the car width bubble
        n_car_width = ceil( CAR_WIDTH / distance_between_scans );
        
        // Indices of the bubble of the car around the point with the closest distance
        // for min index use 'floor' to have extra element in case of odd index & 'max' with to stay in the 180 degrees view bounds
        range_car_bubble[ 0 ] = std::max( ( uint16_t )( floor( index_closest - ( n_car_width / 2 ) ) ), range_180_degrees[ 0 ] );
        
        // for max index use ceil to have extra element in case of odd index & min with 1079 to stay in the 180 degrees view bounds
        range_car_bubble[ 1 ] = std::min( ( uint16_t )( ceil( index_closest + ( n_car_width / 2 ) ) ), range_180_degrees[ 1 ] );

        return;
    }

    
    void findMaxGap( void )
    {   
        uint16_t range_max_gap[ 2 ];
        uint16_t gap_temp[ 2 ];
        bool b_gap_started;
        uint16_t gap_length;

        range_max_gap[0] = 0;
        range_max_gap[1] = 0;
        gap_temp[0] = 0;
        gap_temp[1] = 0;
        gap_length = 0;
        b_gap_started = false;

        for( uint16_t i = range_180_degrees[ 0 ]; i < range_180_degrees[ 1 ]; i++ )
        {   
            if( i >= range_car_bubble[ 0 ] && i <= range_car_bubble[ 1 ] )    // point in the car bubble
            {
                if( b_gap_started )   // we just reached the car bubble after starting a range
                {
                    b_gap_started = false;

                    if( ( gap_temp[ 1 ] - gap_temp[ 0 ] + 1 ) > gap_length )    // the gap found is greater than the previous one
                    {
                        gap_length = gap_temp[ 1 ] - gap_temp[ 0 ] + 1;
                        range_max_gap[0] = gap_temp[0];
                        range_max_gap[1] = gap_temp[1];
                    }
                }
            }
            else    // point outside the car bubble
            {
                if( ranges[ i ] >= RANGE_THRESHOLD )    // point satisfies the range threshold
                {
                    if( !b_gap_started )        // found first point of a gap
                    {
                        gap_temp[ 0 ] = i;
                        b_gap_started = true;
                    }
                    else
                    {
                        gap_temp[ 1 ] = i;
                    }
                }
                else    // point does not satisfy the range threshold
                {
                    if( b_gap_started )     // the gap was already started
                    {
                        b_gap_started = false;

                        if( ( gap_temp[ 1 ] - gap_temp[ 0 ] + 1 ) > gap_length )    // the gap found is greater than the previous one
                        {
                            gap_length = gap_temp[ 1 ] - gap_temp[ 0 ] + 1;
                            range_max_gap[0] = gap_temp[0];
                            range_max_gap[1] = gap_temp[1];
                        }
                    }
                }
            }
        }

        if( b_gap_started )     // reached the end of the array with a gap started
        {
            b_gap_started = false;

            if( ( gap_temp[ 1 ] - gap_temp[ 0 ] + 1 ) > gap_length )    // the gap found is greater than the previous one
            {
                gap_length = gap_temp[ 1 ] - gap_temp[ 0 ] + 1;
                range_max_gap[0] = gap_temp[0];
                range_max_gap[1] = gap_temp[1];
            }
        }

        if( gap_length > 0 )
        {
            range_free_space[0] = range_max_gap[0];
            range_free_space[1] = range_max_gap[1];
        }
        else
        {
            RCLCPP_INFO( this->get_logger(), "findMaxGap(): No gap that satisfies the threshold was found" );
        }

        return;
    }

    
    void findBestPoint( void )
    {   
        uint16_t index_best_point;

        index_best_point = ( uint16_t )( ( range_free_space[ 1 ] + range_free_space[ 0 ] ) / 2 );
        
        steering_angle = -2.35 + ( index_best_point * RANGES_ANGLE_STEP );

        return ;
    }
    
    

    void laserScanCallback( const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg ) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        ranges = scan_msg->ranges;

        // Find closest point to LiDAR
        // Eliminate all points inside 'bubble' (set them to zero) 
        laserScanPreprocess();

        // Find max length gap 
        findMaxGap();

        // Find the best point in the gap 
        findBestPoint();

        // Actuator limits
        if( steering_angle < STEERING_ACTUATOR_MIN )
        {
            steering_angle = STEERING_ACTUATOR_MIN;
        }
        else if( steering_angle > STEERING_ACTUATOR_MAX )
        {
            steering_angle = STEERING_ACTUATOR_MAX;
        }

        // Piecewsie linear function of velocity vs. steering angle to go faster in straights and slower when turning
        /*
        if( abs( steering_angle ) < 0.0523599 )
        {
            velocity = ( - 0.667 * abs( steering_angle ) ) + 4.5;
        }
        else if( abs( steering_angle ) < 0.174533 )
        {
            velocity = ( - 0.214 * abs( steering_angle ) ) + 3.142;
        }
        else
        {
            velocity = 1;
        }*/

        velocity = 1;

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "drive_frame";
        drive_msg.drive.speed = velocity;
        drive_msg.drive.steering_angle = steering_angle;
        pub_drive->publish( drive_msg );
    }



};

int main( int argc, char ** argv ) {

    /* 
     * Main function that is called when the node is run.
     */

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveGapFollow>());
    rclcpp::shutdown();
    return 0;
}