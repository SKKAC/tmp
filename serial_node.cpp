#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <boost/ref.hpp>
#include <fstream>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "serial_manager/robot_status.h"





ros::Time last_heartbeat_reply_time;

float ang_x, ang_y, ang_z; // Global variables for angle data

// Calculate checksum
uint8_t calculateChecksum(const std::vector<uint8_t>& frame) {
    uint8_t checksum = 0;
    for(size_t i = 0; i < frame.size() - 2; ++i) { // Exclude the checksum and end byte
        checksum += frame[i];
    }
    return checksum;
}

void logFrameToFile(const std::vector<uint8_t>& frame) {
    std::ofstream outFile("/home/ujs/ser.txt", std::ios_base::app);
    if (outFile.is_open()) {
        ros::Time currentTime = ros::Time::now();
        outFile << currentTime << " ";
        for (const auto& byte : frame) {
	    outFile  << " ";
            if(byte<0x09)
            outFile <<"0x"<< std::hex << std::uppercase << static_cast<int>(byte);
	    else
	    outFile <<"0x0"<< std::hex << std::uppercase << static_cast<int>(byte);
        }
        outFile << std::endl;
        outFile.close();
    } else {
        ROS_ERROR_STREAM("Unable to open log file");
    }
}


// Function to record angle data
void recordAngleData(const std::vector<uint8_t>& frame) {
    if (frame.size() < 15) { // Ensure the frame has enough data for angles
        ROS_WARN_STREAM("Invalid angle frame size: " << frame.size());
        return;
    }

    uint8_t cmd = frame[2];
    if (cmd != 0x27) {
        ROS_WARN_STREAM("Invalid command for angle data: 0x" << std::hex << static_cast<int>(cmd));
        return;
    }

    // Function to convert 4 bytes from little-endian to float
    auto bytesToFloat = [](const uint8_t* bytes) -> float {
        uint32_t asInt = (static_cast<uint32_t>(bytes[3]) << 24) |
                         (static_cast<uint32_t>(bytes[2]) << 16) |
                         (static_cast<uint32_t>(bytes[1]) << 8)  |
                         static_cast<uint32_t>(bytes[0]);
        float asFloat;
        std::memcpy(&asFloat, &asInt, sizeof(float));
        return asFloat;
    };

    // Extract float values from the frame
    float ang_x = bytesToFloat(&frame[3]);
    float ang_y = bytesToFloat(&frame[7]);
    float ang_z = bytesToFloat(&frame[11]);

    // Print the angle data
    ROS_INFO_STREAM("Angle data received - Roll: " << ang_x << " Pitch: " << ang_y << " Yaw: " << ang_z);
}







tf2::Quaternion calculateQuaternion(const geometry_msgs::Vector3& accel, const geometry_msgs::Vector3& mag) {
    // Calculate roll and pitch angles using accelerometer data
    double roll = atan2(accel.y, accel.z);
    double pitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
    
    // Calculate yaw angle using magnetometer data (assuming no roll and pitch error)
    double yaw = atan2(-mag.y, mag.x);

    // Convert Euler angles to quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);

    return quaternion;
}






// Function to parse IMU frame and publish IMU data
void publishImuData(const std::vector<uint8_t>& imuFrame, ros::Publisher& imu_pub) {
    // Check if the frame size is correct
    if (imuFrame.size() != 23) {
        ROS_WARN_STREAM("Invalid IMU frame size: " << imuFrame.size());
        return;
    }
	
    // Parse IMU data from the frame
    int16_t gyro_x = (imuFrame[4] << 8) | imuFrame[3];
    int16_t gyro_y = (imuFrame[6] << 8) | imuFrame[5];
    int16_t gyro_z = (imuFrame[8] << 8) | imuFrame[7];
    int16_t acc_x = (imuFrame[10] << 8) | imuFrame[9];
    int16_t acc_y = (imuFrame[12] << 8) | imuFrame[11];
    int16_t acc_z = (imuFrame[14] << 8) | imuFrame[13];
    int16_t mag_x = (imuFrame[16] << 8) | imuFrame[15];
    int16_t mag_y = (imuFrame[18] << 8) | imuFrame[17];
    int16_t mag_z = (imuFrame[20] << 8) | imuFrame[19];




    // Create a sensor_msgs::Imu message
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = ros::Time::now();
    // Populate the IMU message
    // Linear acceleration
    imu_msg.linear_acceleration.x = static_cast<double>(acc_x) /32768.0f*16.0f/9.81; // Convert to m/s^2
    imu_msg.linear_acceleration.y = static_cast<double>(acc_y) /32768.0f*16.0f/9.81 ;
    imu_msg.linear_acceleration.z = static_cast<double>(acc_z) /32768.0f*16.0f/9.81 ;

    // Angular velocity
    imu_msg.angular_velocity.x = static_cast<double>(gyro_x) / 1000.0; // Convert to rad/s
    imu_msg.angular_velocity.y = static_cast<double>(gyro_y) / 1000.0;
    imu_msg.angular_velocity.z = static_cast<double>(gyro_z) / 1000.0;
    geometry_msgs::Vector3 accel;
    accel.x = acc_x;
    accel.y = acc_y;
    accel.z = acc_z;
    geometry_msgs::Vector3 mag;
    mag.x = mag_x;
    mag.y = mag_y;
    mag.z = mag_z;
    tf2::Quaternion quaternion = calculateQuaternion(accel, mag);
    geometry_msgs::Quaternion imu_quat;
    tf2::convert(quaternion, imu_quat);
    imu_msg.orientation = imu_quat;
    // Magnetic field
    //imu_msg.orientation_covariance[0] = -1; // Indicates that orientation data is not available

    // Publish the IMU message
    imu_pub.publish(imu_msg);
}


void publishStatusData(const std::vector<uint8_t>& status, ros::Publisher& pub) {
    // Check if the frame size is correct
    if (status.size() != 8) {
        ROS_WARN_STREAM("Invalid status frame size: " << status.size());
        return;
    }
	
    serial_manager::robot_status rstatus;
    rstatus.robot_status=0;
    rstatus.work_status=0;
    rstatus.error_code=0;
    rstatus.pick_one=0;
    rstatus.sum_ball=0;
    rstatus.task=0;
    rstatus.target_area=0;
    int16_t v =static_cast<uint32_t>(status[3]);
    rstatus.voltage= static_cast<double>(v);
    pub.publish(rstatus);
}



// Function to read data from serial port and return complete frame
/*std::vector<uint8_t> readSerialData(serial::Serial& ser) {
    std::vector<uint8_t> buffer;
    std::string completeFrame;
    std::vector<uint8_t> frame2;
    if (ser.available()) {
        size_t available_bytes = ser.available();
        std::vector<uint8_t> temp_buffer(available_bytes);
        ser.read(temp_buffer, available_bytes);
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());

        while (buffer.size() >= 6) { // Minimum frame length
            if (buffer[0] == 0xA5 && buffer[buffer.size() - 1] == 0xAA) {
                uint8_t length = buffer[1]; // Length includes length byte itself
                if (buffer.size() >= length + 3) { // Include length and checksum
                    std::vector<uint8_t> frame(buffer.begin(), buffer.begin() + length);
                    frame2 = frame;
                    // Calculate checksum
                    uint8_t calculated_checksum = calculateChecksum(frame);
                    uint8_t received_checksum = frame[length - 2];

                    // Parse frame
                    uint8_t cmd = frame[2];
                    std::vector<uint8_t> data(frame.begin() + 3, frame.begin() + 3 + (length - 2));
                    uint8_t end = frame[length - 1];

                    // Construct log message
                    std::stringstream ss;
                    std::stringstream datas;
                    ss << "Frame: ";
                    for (auto byte : frame) {
                        ss << "0x" << std::hex << std::uppercase << static_cast<int>(byte) << " ";
                        datas << std::hex << std::uppercase << static_cast<int>(byte);
                    }
                    ss << "| Length: " << std::dec << static_cast<int>(length);
                    ss << " | CMD: 0x" << std::hex << std::uppercase << static_cast<int>(cmd);
                    ss << " | Received Checksum: 0x" << std::hex << std::uppercase << static_cast<int>(received_checksum);
                    ss << " | Calculated Checksum: 0x" << std::hex << std::uppercase << static_cast<int>(calculated_checksum);
                    ss << " | End byte: 0x" << std::hex << std::uppercase << static_cast<int>(end);

                    // Print log message
                    if (calculated_checksum == received_checksum && end == 0xAA) {
                        // ROS_INFO_STREAM(ss.str());
                    } else {
                        ROS_WARN_STREAM(ss.str());
                    }

                    // Set completeFrame to the constructed log message
                    completeFrame = datas.str();

                    // Clear processed frame
                    buffer.erase(buffer.begin(), buffer.begin() + length + 3);
                } else {
                    // Wait for more data
                    break;
                }
            } else {
                buffer.erase(buffer.begin()); // Remove invalid bytes
            }
        }
    }

    return frame2;
}*/

// Function to read data from serial port and return complete frame
std::vector<uint8_t> readSerialData(serial::Serial& ser) {
    std::vector<uint8_t> buffer;
    std::vector<uint8_t> frame;

    if (ser.available()) {
        size_t available_bytes = ser.available();
        std::vector<uint8_t> temp_buffer(available_bytes);
        ser.read(temp_buffer, available_bytes);
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());

        while (!buffer.empty()) {
            // Find the index of frame header 0xA5
            auto header_it = std::find(buffer.begin(), buffer.end(), 0xA5);
            if (header_it == buffer.end()) {
                // No frame header found, discard the buffer
                buffer.clear();
                break;
            }

            // Check if there's enough data for a complete frame
            if (std::distance(header_it, buffer.end()) >= 2) {
                // Extract length and check if it's valid
                uint8_t length = *(header_it + 1);
                if (length >= 6 && length <= 255) {  // Minimum frame length is 6 bytes
                    // Check if we have received enough bytes for the complete frame
                    if (std::distance(header_it, buffer.end()) >= length) {
                        // Extract the complete frame
                        frame = std::vector<uint8_t>(header_it, header_it + length);
                        
                        // Verify frame integrity (start byte, end byte, checksum)
                        if (frame.front() == 0xA5 && frame.back() == 0xAA) {
                            uint8_t received_checksum = frame[length - 2];
                            uint8_t calculated_checksum = calculateChecksum(frame);
                            if (received_checksum == calculated_checksum) {
                                // Frame is valid, remove it from the buffer
                                buffer.erase(buffer.begin(), header_it + length);
                                break;
                            } else {
                                // Invalid checksum, discard the frame
                                ROS_WARN_STREAM("Invalid checksum, discarding the frame");
                                buffer.erase(buffer.begin(), header_it + 1);  // Discard up to the next potential header
                            }
                        } else {
                            // Invalid start or end byte, discard the frame
                            ROS_WARN_STREAM("Invalid start or end byte, discarding the frame");
                            buffer.erase(buffer.begin(), header_it + 1);  // Discard up to the next potential header
                        }
                    } else {
                        // Not enough bytes for a complete frame, wait for more data
                        break;
                    }
                } else {
                    // Invalid length, discard the byte with invalid length
                    ROS_WARN_STREAM("Invalid frame length, discarding the byte");
                    buffer.erase(header_it);
                }
            } else {
                // Not enough bytes for length field, wait for more data
                break;
            }
        }
    }

    return frame;
}





void sendHeartbeatReply(serial::Serial& ser) {
    std::vector<uint8_t> heartbeat_reply = {0xA5, 0x05, 0x07, 0x00, 0xAA}; // Initialize heartbeat reply frame
    // Calculate checksum
    uint8_t checksum = calculateChecksum(heartbeat_reply);
    // Set the checksum
    heartbeat_reply[3] = checksum;
    // Send the heartbeat reply frame
    ser.write(heartbeat_reply);
}

// Function to manage tasks based on received frame
void manageTask(std::vector<uint8_t> completeFrame, ros::Publisher& pub, ros::Publisher& s_pub ,serial::Serial& ser) {
    if (completeFrame.size() < 3) {
        ROS_WARN_STREAM("Incomplete frame received");
        return;
    }
    //std::vector<uint8_t> confirm = {0xA5, 0x05, 0x06, 0xB1, 0xAA};
    // Extract CMD from the third byte
    uint8_t cmd = completeFrame[2];//, checksum2 = calculateChecksum(confirm);
    // confirm[3]=checksum2;

    // Call other functions based on CMD
    switch (cmd) {
        case 0x07:
            ROS_INFO_STREAM("Confirm received!");
            break;
        case 0x06:
            last_heartbeat_reply_time = ros::Time::now();
            sendHeartbeatReply(ser);
            // ROS_INFO_STREAM("Heartbeat sent!  " << std::hex << std::uppercase << static_cast<int>(confirm[3]));
            //ser.write(confirm);
            // Handle command 0x02
            break;
        case 0x23:
            // ROS_INFO_STREAM("IMU received!");
            publishImuData(completeFrame, pub);
            break;
        case 0x27:
            // Handle angle data
            //recordAngleData(completeFrame);
            break;
	case 0x21:
	    publishStatusData(completeFrame, s_pub);
            break;
        default:
            ROS_WARN_STREAM("Unknown command: 0x" << std::hex << std::uppercase << static_cast<int>(cmd));
            break;
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, serial::Serial& ser) {
    // 计算速度指令
    int speed = static_cast<int>(50 * msg->linear.x); // 将线速度转换为0-100范围内的整数
    // int speed = static_cast<int>(0);
    // 计算方向指令
    float tmp = 180 / M_PI;
    int direction = static_cast<int>(0.8*tmp * msg->angular.z); // 将角速度转换为-180到180范围内的整数
    // int direction = static_cast<int>(0);
    // 构造指令帧
    std::vector<uint8_t> cmd_frame = {0xA5, 0x08, 0x40, static_cast<uint8_t>(speed & 0xFF), static_cast<uint8_t>((direction >> 8) & 0xFF), static_cast<uint8_t>(direction & 0xFF), 0x00, 0xAA};
    // std::vector<uint8_t> cmd_frame = {0xA5, 0x08, 0x40, 0x00, 0x00, 0x00, 0xAA};
    // 计算校验和
    uint8_t checksum = calculateChecksum(cmd_frame);

    // 将校验和添加到指令帧中
    cmd_frame.insert(cmd_frame.end() - 2, checksum);
    ROS_INFO_STREAM("control sent!  speed:" << speed << "angle:  " << direction);
    ros::Time tim = ros::Time::now();
    // 将指令帧发送到串口



    ser.write(cmd_frame);
    logFrameToFile(cmd_frame);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_reader_node");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::Publisher s_pub = nh.advertise<serial_manager::robot_status>("robot_info", 10);

    // Open serial port
    serial::Serial ser;

    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel_final", 10, boost::bind(&cmdVelCallback, _1, boost::ref(ser)));

    try {
        ser.setPort("/dev/ttyACM0"); // Modify to your serial device name
        ser.setBaudrate(115200); // Set baud rate
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    ros::Rate loop_rate(10); // Set loop rate
    std::vector<uint8_t> online = {0xA5, 0x08, 0x05, 0x01, 0x02, 0x02, 0x00, 0xAA}; // Initialize frame

    // Calculate checksum
    uint8_t checksum = calculateChecksum(online);
    online[6] = checksum;
    ser.write(online);
   // ROS_INFO_STREAM("Online message sent!" << std::hex << std::uppercase << static_cast<int>(checksum));

    

    while (ros::ok()) {
        // Call function to read serial data and get complete frame
        std::vector<uint8_t> completeFrame = readSerialData(ser);
        manageTask(completeFrame, imu_pub,s_pub, ser);
	//ser.write(online);
        // Do something with completeFrame if needed
        /*if ((ros::Time::now() - last_heartbeat_reply_time).toSec() > 0.1) { // 如果距离上次心跳回复超过200ms
            //ser.write(online);
            ROS_INFO_STREAM("Online message sent!");
           
            //sendHeartbeatReply(ser); // 重新发送心跳回复
            last_heartbeat_reply_time = ros::Time::now(); // 更新心跳回复时间戳
        }*/
        ros::spinOnce();
        loop_rate.sleep();
    }

    ser.close();
    return 0;
}

