#include "visual_follow_motor.h"

MotorSerial::MotorSerial( ros::NodeHandle &nh, ros::NodeHandle &private_nh ):
nh_( nh ), private_nh_( private_nh ), current_angular_( 0 ), time_num_(0),
search_status_( false ), fx_( 709.1549 ), cx_( 641.4906 ), center_x_( CENTER_X ), target_update_( false )
{
    private_nh_.param<std::string>( "twist_com", serialPortNum_, "/dev/ttyUSB0" );
    private_nh_.param<int32_t>( "baudrate", serialBaudrate_, 115200 );
    private_nh_.param<std::string>( "detect_box_topic_", detect_box_topic_, "/vision_detector/box_result" );

    detect_result_sub_ = nh_.subscribe( detect_box_topic_, 1, &MotorSerial::detectResultCallback, this );
    sub_pointcloud_ = nh_.subscribe("/velodyne_points", 1, &MotorSerial::currCloudHandler, this);
    fixed_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("motor_fixed_points", 1);
    motor_angle_pub_ = nh_.advertise<std_msgs::Int32>("motor_angle", 2);
    fixed_free_trans_ = Eigen::Matrix4f::Identity();
}

MotorSerial::~MotorSerial()
{
    motorSerial_.close();
}

//Receive event function for the current frame of the point cloud
void MotorSerial::currCloudHandler( const sensor_msgs::PointCloud2ConstPtr& msg  )
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr motor_free_pointcloud( new pcl::PointCloud<pcl::PointXYZI>() );
    pcl::PointCloud<pcl::PointXYZI>::Ptr motor_fixed_pointcloud( new pcl::PointCloud<pcl::PointXYZI>() );
    pcl::fromROSMsg(*msg, *motor_free_pointcloud);
    pcl::transformPointCloud( *motor_free_pointcloud, *motor_fixed_pointcloud, fixed_free_trans_ );
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( *motor_fixed_pointcloud, output);
    output.header.stamp = msg->header.stamp;
    output.header.frame_id = "motor_fixed";
    fixed_points_pub_.publish( output );
}

void MotorSerial::detectResultCallback( const geometry_msgs::PoseStampedConstPtr& detect_result_msg )
{
    time_num_ = 0; //Counter Zero
    search_status_ = false;

    int16_t x_min = std::max(0, static_cast<int>(detect_result_msg->pose.orientation.x));
    int16_t y_min = std::max(0, static_cast<int>(detect_result_msg->pose.orientation.y));
    int16_t x_max = std::max(0, static_cast<int>(detect_result_msg->pose.orientation.z));
    int16_t y_max = std::max(0, static_cast<int>(detect_result_msg->pose.orientation.w)); 

    center_x_ = ( x_min + x_max ) / 2;
    //std::cout << center_x_ << std::endl;
    target_update_ = true;

    //angle control model
    angleControlModel();

    //Speed control mode
    // speedControlModel( pixel_bias );

    //Position Control Mode
    // positionControlModel( pixel_bias );
}

void MotorSerial::angleControlModel()
{
    if( !target_update_ )
        return;

    // std::cout << "center_x_: " << center_x_ << std::endl;
    // std::cout << "cx_: " << cx_ << std::endl;
    // std::cout << "fx_: " << fx_ << std::endl;
    double d_angle_radian = - atan( ( center_x_ - cx_ ) / fx_ );
    int32_t d_angle = d_angle_radian * 180 / 3.14;
    // std::cout << "d_angle: " << d_angle << std::endl;
    int32_t target_angular = current_angular_ + d_angle;

    int32_t input_angular;
    if( target_angular <= 0 )
        input_angular = -target_angular;
    else
        input_angular = 360 - target_angular;

    int16_t e_pixel_bias = CENTER_X - center_x_;
    //std::cout << "e_pixel_bias: " << e_pixel_bias << std::endl;
    static std::vector<uint16_t> e_vector;
    e_vector.push_back( e_pixel_bias );
    if( e_vector.size() > 50 )
    {
        e_vector.pop_back();
    }
    static int16_t pre_pixel_bias = e_pixel_bias;
    static double p_e = 1.3;//1.6;
    static double d_e = 1.6;//1.1;
    static double k_e = -0.3;//-0.3;
    static double k_factor = 0.0;
    if( e_vector.size() > 10 )
    {
        k_factor = k_e * e_vector[ e_vector.size() - 4 ];
        if( k_factor > 10 )
            k_factor = 10;
    }
    else
        k_factor = 0.0;
    uint32_t speed = abs( p_e * abs( e_pixel_bias ) + d_e * ( abs( e_pixel_bias ) - abs( pre_pixel_bias ) ) );
    //std::cout << e_pixel_bias - pre_pixel_bias << std::endl;
    //std::cout << "speed: " << speed << std::endl;
    if( speed > 800 )
        speed = 800;

    // std::cout << "input_angular:   " << input_angular <<  "   speed:   " << speed << std::endl;
    if( speed > 5 )
    {
        if( d_angle > 0 )
        {
            motorPositionControlCmd( input_angular, 0x01, speed ); //turn counterclockwise
        }
        else if( d_angle < 0 )
        {
            motorPositionControlCmd( input_angular, 0x00, speed ); //turn clockwise
        }
    }
    else
        motorSpeedControlCmd( 0 );

    pre_pixel_bias = e_pixel_bias;
    target_update_ = false;
}

void MotorSerial::speedControlModel()
{
    int16_t pixel_bias = center_x_ - CENTER_X;
    double speed_ratio = static_cast<double>( abs( pixel_bias ) ) / CENTER_X;
    int32_t speed = speed_ratio * 300;
    std::cout << "speed:" << speed << std::endl;

    if( pixel_bias < -100 )
    {
        motorSpeedControlCmd( -speed ); //turn counterclockwise
    }
    else if( pixel_bias > 100 )
    {
        motorSpeedControlCmd( speed ); //turn clockwise
    }
    else
        motorSpeedControlCmd( 0 );
}

 //Position Mode Control
void MotorSerial::positionControlModel()
{
    int16_t pixel_bias = center_x_ - CENTER_X;
    double speed_ratio = static_cast<double>( abs( pixel_bias ) ) / CENTER_X;
    uint32_t speed = speed_ratio * 400;
    std::cout << "speed:" << speed << std::endl;

    int32_t target_angular = current_angular_;

    if( pixel_bias < -100 )
    {
        target_angular += 5;
        if( target_angular > ANGULAR_THNRESHOLD )
            target_angular = ANGULAR_THNRESHOLD;
        if( target_angular < -ANGULAR_THNRESHOLD )
            target_angular = -ANGULAR_THNRESHOLD;

        if( target_angular < 0 )
            motorPositionControlCmd( -target_angular, 0x01, speed ); //turn counterclockwise
        else
            motorPositionControlCmd( 360 - target_angular, 0x01, speed ); //turn counterclockwise
    }
    else if( pixel_bias > 100 )
    {
        target_angular -= 5;
        if( target_angular > ANGULAR_THNRESHOLD )
            target_angular = ANGULAR_THNRESHOLD;
        if( target_angular < -ANGULAR_THNRESHOLD )
            target_angular = -ANGULAR_THNRESHOLD;

        if( target_angular < 0 )
            motorPositionControlCmd( -target_angular, 0x00, speed ); //turn clockwise
        else
            motorPositionControlCmd( 360 - target_angular, 0x00, speed ); //turn clockwise
    }
}

void MotorSerial::statusCheck()
{
    time_num_++;
    if( time_num_ > 300 )
        time_num_ = 300;
    
    //Cyclic scanning if the recognition frame is not received for a long time
    if( time_num_ > 50 )
    {
        if( !search_status_ )
        {
            motorPositionControlCmd( 90, 0x00, 100 ); //clockwise cruise
        }
        search_status_ = true;
        if( current_angular_ < - 87 )
        {
            motorPositionControlCmd( 270, 0x01, 100 ); //counterclockwise cruise
        }
        if( current_angular_ > 87 )
        {
            motorPositionControlCmd( 90, 0x00, 100 ); //clockwise cruise
        }
        return;
    }

    //Stops movement if it does not receive the recognition frame for a short period of time
    if( time_num_ > 20 )
    {
        motorSpeedControlCmd( 0 );
        std::cout << "Stoping motor......" << std::endl;
    }
}

//Serial port on
uint8_t MotorSerial::motorSerialStart()
{
    motorSerial_.setPort( serialPortNum_ );
    motorSerial_.setBaudrate( serialBaudrate_ );
    serial::Timeout to = serial::Timeout::simpleTimeout( 1000 );
    motorSerial_.setTimeout( to );
    try
    {
        motorSerial_.open();
    }
    catch ( serial::IOException& e )
    {
        /* code for Catch */
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if( motorSerial_.isOpen() )
        ROS_INFO_STREAM("Serial Port Initialized");
    else
        return -1;
}

void MotorSerial::getMotorRotation()
{
    std::cout << "motor rotation getting .... "<< std::endl;
    uint8_t rotation_cmd[5];
    rotation_cmd[0] = 0x3E;
    rotation_cmd[1] = 0x94;
    rotation_cmd[2] = 0x01;
    rotation_cmd[3] = 0x00;
    rotation_cmd[4] = rotation_cmd[0] + rotation_cmd[1] + rotation_cmd[2] + rotation_cmd[3];

    //Write data to serial port
    if( motorSerial_.isOpen() )
    {
        motorSerial_.write( rotation_cmd, 5 );
        usleep( 50000 );
        serialDataReading();
    }
    else
        motorSerialStart(); //Reboot the serial port
    
}

//position control
void MotorSerial::motorPositionControlCmd( uint32_t angle, uint8_t spin_direction, uint32_t speed )
{
    std::cout << "motor position control, angle value: " << (int)angle << std::endl;

    uint32_t angle_control = angle * 9 * 100;

    uint32_t max_speed = speed * 100; //Motor speed
    uint8_t motor_cmd[14];
    motor_cmd[0] = 0x3E; //header
    motor_cmd[1] = 0xA6; //command
    motor_cmd[2] = 0x01; //ID
    motor_cmd[3] = 0x08; //data length
    motor_cmd[4] = motor_cmd[0] + motor_cmd[1] + motor_cmd[2] + motor_cmd[3]; //Frame Command Checksum Byte

    motor_cmd[5] = spin_direction; //Direction of rotation byte, 0x00 clockwise, 0x01 counterclockwise
    motor_cmd[6] = *( uint8_t* )( &angle_control ); 
    motor_cmd[7] = *( ( uint8_t* )( &angle_control ) + 1 ); 
    motor_cmd[8] = *( ( uint8_t* )( &angle_control ) + 2 ); 
    motor_cmd[9] = *( uint8_t* )( &max_speed );
    motor_cmd[10] = *( ( uint8_t* )( &max_speed ) + 1 );
    motor_cmd[11] = *( ( uint8_t* )( &max_speed ) + 2 );
    motor_cmd[12] = *( ( uint8_t* )( &max_speed ) + 3 );
    motor_cmd[13] = motor_cmd[5] + motor_cmd[6] + motor_cmd[7] + motor_cmd[8]
                  + motor_cmd[9] + motor_cmd[10] + motor_cmd[11] + motor_cmd[12];

    //Write data to serial port
    if( motorSerial_.isOpen() )
    {
        motorSerial_.write( motor_cmd, 14 );
        usleep( 50000 );
        serialDataReading();
        // ROS_INFO_STREAM( "Serial Port is OK, sending motor position control command......" );
    }
    else
        motorSerialStart(); //Reboot the serial port
    
}

void MotorSerial::motorSpeedControlCmd( int32_t speed )
{
    int32_t speed_control = speed * 100;
    uint8_t speed_cmd[10];
    speed_cmd[0] = 0x3E; //header
    speed_cmd[1] = 0xA2; //command
    speed_cmd[2] = 0x01; //ID
    speed_cmd[3] = 0x04; //data length
    speed_cmd[4] = speed_cmd[0] + speed_cmd[1] + speed_cmd[2] + speed_cmd[3]; //Frame Command Checksum Byte

    speed_cmd[5] = *( uint8_t* )( &speed_control ); 
    speed_cmd[6] = *( ( uint8_t* )( &speed_control ) + 1 ); 
    speed_cmd[7] = *( ( uint8_t* )( &speed_control ) + 2 );
    speed_cmd[8] = *( ( uint8_t* )( &speed_control ) + 3 );
    speed_cmd[9] = speed_cmd[5] + speed_cmd[6] + speed_cmd[7] + speed_cmd[8];

    //Write data to serial port
    if( motorSerial_.isOpen() )
    {
        motorSerial_.write( speed_cmd, 10 );
        usleep( 50000 );
        serialDataReading();
        // ROS_INFO_STREAM( "Serial Port is OK, sending control command......" );
    }
    else
        motorSerialStart(); //Reboot the serial port
}

//Serial data reading
void MotorSerial::serialDataReading()
{
    //serialport.available()When the serial port is not cached, this function will wait until it is cached before returning the number of characters
    size_t data_size = motorSerial_.available(); 

    uint8_t buf[data_size];
    motorSerial_.read( buf, data_size );

    serialDataProcessing( buf, data_size );
}

//Serial Data Processing
inline void MotorSerial::serialDataProcessing( uint8_t *buf, size_t data_size )
{
    for( size_t i = 0; i < data_size; i++ )
    {
        // 1、Finding motor angle valid frames
        if( static_cast<uint8_t>( buf[i] ) == 0X3E && static_cast<uint8_t>( buf[i + 1] ) == 0X94 && data_size - i >= 10 )
        {
            //Sum the data frames and do the checksum.
            uint8_t sum = static_cast<uint8_t>(buf[i]) + static_cast<uint8_t>(buf[i+1]) 
                        + static_cast<uint8_t>(buf[i+2]) + static_cast<uint8_t>(buf[i+3]);
            //Data Bits and Checksums
            if( sum == static_cast<uint8_t>(buf[i+4]) )
            {
                angleCompute( buf, i );
                i += 10;
            }
        }
    }
}

//Motor angle calculation
inline void MotorSerial::angleCompute( const uint8_t *buf, int32_t i )
{
    uint8_t sum = static_cast<uint8_t>( buf[i + 5] ) + static_cast<uint8_t>( buf[i + 6] )
                + static_cast<uint8_t>( buf[i + 7] ) + static_cast<uint8_t>( buf[i + 8] );
    if( sum == static_cast<uint8_t>(buf[i+9]) )
    {
        uint32_t angle_0 = static_cast<uint32_t>( static_cast<uint8_t>( buf[i + 5] ) );
        uint32_t angle_1 = static_cast<uint32_t>( static_cast<uint8_t>( buf[i + 6] ) );
        uint32_t angle_2 = static_cast<uint32_t>( static_cast<uint8_t>( buf[i + 7] ) );
        uint32_t angle_3 = static_cast<uint32_t>( static_cast<uint8_t>( buf[i + 8] ) );

        uint32_t circle_angle = ( angle_3 << 24 ) | ( angle_2 << 16 ) | ( angle_1 << 8 ) | angle_0;
        int32_t angle_single = static_cast<int32_t>( circle_angle / 100 / 9 );
        if( angle_single >= 0 && angle_single <= 180 )
            current_angular_ = -angle_single;
        else if( angle_single > 180 && angle_single < 360 )
            current_angular_ = 360 - angle_single;
        
        std_msgs::Int32 motor_angle_value;
        motor_angle_value.data = current_angular_;
        motor_angle_pub_.publish( motor_angle_value );

        double current_angular_radian = current_angular_ * M_PI / 180.0;

        //Updating the transformation matrix
        fixed_free_trans_(0,0) = cos( current_angular_radian );
        fixed_free_trans_(0,1) = -sin( current_angular_radian );
        fixed_free_trans_(1,0) = sin( current_angular_radian );
        fixed_free_trans_(1,1) = cos( current_angular_radian );

        //Update angle values and post tf
        tf::Transform transform;
        tf::Quaternion q1;

        transform.setOrigin( tf::Vector3( 0, 0, 0 ) );
        q1.setRPY( 0, 0 ,current_angular_radian);
        transform.setRotation( q1 );
        br_.sendTransform( tf::StampedTransform( transform, ros::Time::now(), "motor_fixed", "motor_free" ) );
    }
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "visual_follow_motor" );
    ros::NodeHandle nh_, private_nh_( "~" );

    MotorSerial* motorSerial = new MotorSerial( nh_, private_nh_ );
    motorSerial->motorSerialStart();
    ros::Rate loop_rate( 50 );

    motorSerial->motorPositionControlCmd( 0, 0x00, 300 );
    loop_rate.sleep();
    motorSerial->motorPositionControlCmd( 0, 0x01, 300 );

    for( size_t i = 0; i < 10; i++ )
    {
        loop_rate.sleep();
    }

    while( ros::ok() )
    {
        motorSerial->getMotorRotation();
        motorSerial->statusCheck();
        ros::spinOnce();
        loop_rate.sleep();
    }

    motorSerial->motorPositionControlCmd( 0, 0x00, 300 );
    loop_rate.sleep();
    motorSerial->motorPositionControlCmd( 0, 0x01, 300 );

    return 0;
}
