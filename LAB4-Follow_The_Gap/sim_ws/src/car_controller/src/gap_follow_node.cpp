#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

/*********************************************************************************************************************/

#define         RANGES_SIZE                 ( 1080 )                            // Size of the ranges[] array
#define         RANGES_ANGLE_MIN            ( -2.35 )                           // Laser scan minimum angle in radians
#define         RANGES_ANGLE_MAX            ( 2.35 )                            // Laser scan maximum angle in radians
//#define         RANGES_ANGLE_STEP           ( ( RANGES_ANGLE_MAX - RANGES_ANGLE_MIN ) / ( RANGES_SIZE - 1 ) )

#define         STEERING_ACTUATOR_MIN       ( -0.436332 )                             // Minimum actuator steering angle (radians) 
#define         STEERING_ACTUATOR_MAX       ( 0.436332 )                              // Maximum actuator steering angle (radians)

#define         MAX_NANOSEC                 ( 10000000000 )                     // Max number of nanoseconds. Used for when the timer counter overflows

#define         QOS                         ( 10 )                              // Quality of service for ROS nodes

#define         CAR_WIDTH                   ( 0.5 )                             // Car width in meters

#define         RANGE_THRESHOLD             ( 2 )

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
    }

private:

    // Variables
    uint16_t range_car_bubble[ 2 ];
    uint16_t range_free_space[ 2 ];
    float steering_angle;
    float velocity = 2;

    // Topics
    std::string laserscan_topic = "/scan";
    std::string drive_topic = "/drive";
    
    // ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr sub_laser_scan;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;

    // Range vector
    std::vector<float> ranges;
    
    
    void laserScanPreprocess( void )
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        uint16_t index_closest;
        float distance_closest;
        float distance_between_scans;
        uint16_t n_car_width;

        // Get the index of the closest laser scan
        index_closest = 0;
        distance_closest = ranges[ index_closest ];

        for( uint16_t iterator = 0; iterator < RANGES_SIZE; iterator++ )
        {
            //RCLCPP_INFO( this->get_logger(), "Range[%u]: %f", iterator, ranges[ iterator ] );

            if( ranges[ iterator ] < ranges[ index_closest ] )
            {
                index_closest = iterator;
            }
        }

        //RCLCPP_INFO( this->get_logger(), "Index Closest: %u", index_closest );

        distance_closest = ranges[ index_closest ];

        //RCLCPP_INFO( this->get_logger(), "Distance Closest: %f", distance_closest );

        float angle_step = ( ( RANGES_ANGLE_MAX - RANGES_ANGLE_MIN ) / ( RANGES_SIZE - 1 ) );
        // Approximated separation between 2 adjacent laser scans based on the distance to the closest object
        distance_between_scans = tan( angle_step ) * distance_closest;

        //RCLCPP_INFO( this->get_logger(), "Distance between scans: %f", distance_between_scans );

        // Number of scans that cover the car width bubble
        n_car_width = ceil( CAR_WIDTH / distance_between_scans );

        //RCLCPP_INFO( this->get_logger(), "Car Width: %u", n_car_width );

        
        // Indices of the bubble of the car
        // for min index use floor to have extra element in case of odd index & max with  to stay in vector bounds
        range_car_bubble[ 0 ] = std::max( ( uint16_t )( floor( index_closest - ( n_car_width / 2 ) ) ), ( uint16_t )0 );
        //RCLCPP_INFO( this->get_logger(), "laserScanPreprocess(): Car min index %u", range_car_bubble[ 0 ] );

        
        // for max index use ceil to have extra element in case of odd index & min with 1079 to stay in vector bounds
        range_car_bubble[ 1 ] = std::min( ( uint16_t )( ceil( index_closest + ( n_car_width / 2 ) ) ), ( uint16_t ) ( RANGES_SIZE - 1 ) );
        //RCLCPP_INFO( this->get_logger(), "laserScanPreprocess(): Car max index %u", range_car_bubble[ 1 ] );

        /*
        // Set the values in the bubble to 0
        for( uint16_t i = min_index; i <= max_index; i++ )
        {
            ranges[ i ] = 0;
        }

        for(int i = 0; i < 1079; i++ ){
            RCLCPP_INFO( this->get_logger(), "New Range[%u]: %f", i, ranges[ i ] );
        }
        
        */
        // TODO: Limit the laser scan to 180 degrees view so that the car doesn't go backwards

        return;
    }

    
    void findMaxGap( void )
    {   
        // Return the start index & end index of the max gap in free_space_ranges

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

        for( uint16_t i = 0; i < RANGES_SIZE; i++ )
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
        
        //RCLCPP_INFO( this->get_logger(), "findMaxGap(): gap start %u", range_free_space[ 0 ] );
        //RCLCPP_INFO( this->get_logger(), "findMaxGap(): gap end %u", range_free_space[ 1 ] );

        return;
    }

    
    void findBestPoint( void )
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there

        uint16_t index_farthest;

        /*
        index_farthest = range_free_space[ 0 ];

        for( uint16_t i = range_free_space[ 0 ]; i <= range_free_space[ 1 ]; i++ )
        {
            //RCLCPP_INFO( this->get_logger(), "Range[%u]: %f", iterator, ranges[ iterator ] );

            if( ranges[ i ] > ranges[ index_farthest ] )
            {
                index_farthest = i;
            }
        }
        */

        index_farthest = ( uint16_t )( ( range_free_space[ 1 ] + range_free_space[ 0 ] ) / 2 );

        RCLCPP_INFO( this->get_logger(), "findBestPoint(): Index farthest %u", index_farthest );
        
        steering_angle = -2.35 + ( index_farthest * ( ( RANGES_ANGLE_MAX - RANGES_ANGLE_MIN ) / ( RANGES_SIZE - 1 ) ) );

        RCLCPP_INFO( this->get_logger(), "findBestPoint(): Requested steering angle %f", steering_angle );

        return ;
    }
    
    

    void laserScanCallback( const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg ) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        ranges = scan_msg->ranges;
        //RCLCPP_INFO( this->get_logger(), "Range[0]: %f", ranges[ 0 ] );

        /// TODO:
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