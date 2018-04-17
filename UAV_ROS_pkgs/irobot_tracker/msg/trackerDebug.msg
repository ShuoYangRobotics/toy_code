std_msgs/Header header
int8 is_tracking
geometry_msgs/Quaternion imu_ort
geometry_msgs/Vector3 imu_linear_a
geometry_msgs/Vector3 imu_angular_v

geometry_msgs/Quaternion filter_ort
geometry_msgs/Vector3 vision_pos

geometry_msgs/Vector3 filter_pos
geometry_msgs/Vector3 filter_speed

geometry_msgs/Vector3 target_speed
geometry_msgs/Vector3 error_speed
geometry_msgs/Vector3 p_speed
geometry_msgs/Vector3 i_speed
geometry_msgs/Vector3 d_speed

geometry_msgs/Vector3 target_pos
geometry_msgs/Vector3 error_pos
geometry_msgs/Vector3 p_pos
geometry_msgs/Vector3 i_pos
geometry_msgs/Vector3 d_pos

geometry_msgs/Vector3 target_acc
geometry_msgs/Vector3 error_acc
geometry_msgs/Vector3 p_acc
geometry_msgs/Vector3 i_acc
geometry_msgs/Vector3 d_acc

geometry_msgs/Vector3 target_angle

geometry_msgs/Vector3 angle_output_b
geometry_msgs/Vector3 angle_output_t
geometry_msgs/Vector3 angle_output_c
