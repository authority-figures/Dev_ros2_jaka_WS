/**
* @last update Nov 30 2021 
* @Maintenance star@jaka
*/



#ifndef _JAKAAPI_H_
#define _JAKAAPI_H_

#include <stdio.h>
#include <string>
#include <stdint.h>
#include "jkerr.h"
#include "jktypes.h"

#if defined(_WIN32) || defined(WIN32)
/**
    * @brief Constructor for the robotic arm control class
 */
#if __cpluscplus

#ifdef DLLEXPORT_API
#undef DLLEXPORT_API
#endif // DLLEXPORT_API

#ifdef DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllexport)
#else // DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllimport)
#endif // DLLEXPORT_EXPORTS

#else // __cpluscplus

#define DLLEXPORT_API

#endif // __cpluscplus

#elif defined(__linux__)

#define DLLEXPORT_API __attribute__((visibility("default")))

#else

#define DLLEXPORT_API

#endif // defined(_WIN32) || defined(WIN32)

class DLLEXPORT_API JAKAZuRobot
{
public:
	/**
	* @brief Robotic arm control class constructor
	*/
	JAKAZuRobot();

///@name general part
///@{
	/**
    * @brief Create a control handle for the robot
    * @param ip IP address of the controller
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t login_in(const char *ip);

	/**
    * @brief Disconnect the controller
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t login_out();

	/**
    * @param handle Handle for robot control
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t power_on();

	/**
    * @brief Turn off the robot's power supply
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t power_off();

	/**
    * @brief Shut down the robot's control cabinet
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t shut_down();

	/**
    * @brief Enable the robot
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t enable_robot();

	/**
    * @brief Disable the robot
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t disable_robot();

	/**
    * @brief Get DH parameters
    * @param dh_param DH parameters
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_dh_param(DHParam* dh_param);

	/**
    * @brief Set the installation angle
    * @param angleX Rotation angle around the X-axis
    * @param angleZ Rotation angle around the Z-axis
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_installation_angle(double angleX, double angleZ);

	/**
    * @brief Get the installation angle
    * @param quat Installation angle quaternion
    * @param appang Installation angle RPY angles
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_installation_angle(Quaternion* quat, Rpy* appang);

	/**
    * @brief Get robot status data
    * @param status Robot status
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_robot_status(RobotStatus *status);
///@}
	
///@name motion part
///@{
	/**
    * @brief Manually control the robot's motion
    * @param aj_num 1-based index value, represents joint number 0-5 in joint space, and x, y, z, rx, ry, rz in Cartesian space
    * @param move_mode Robot motion mode, incremental motion or continuous motion
    * @param coord_type Robot motion coordinate system, tool coordinate system, base coordinate system (current world/user coordinate system) or joint space
    * @param vel_cmd Commanded velocity, unit degree/s for rotary axis or joint motion, mm/s for moving axis
    * @param pos_cmd Commanded position, unit degree for rotary axis or joint motion, mm for moving axis
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t jog(int aj_num, MoveMode move_mode, CoordType coord_type, double vel_cmd, double pos_cmd);

	/**
    * @brief Stop the robot's motion in manual mode
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t jog_stop(int num);

	/**
    * @brief Joint motion of the robot
    * @param joint_pos Target position for robot joint motion
    * @param move_mode Specifies the motion mode: incremental motion (relative motion) or absolute motion
    * @param is_block Set whether the interface is a blocking interface, TRUE for blocking interface, FALSE for non-blocking interface
    * @param speed Robot joint motion speed, unit: degree/s
    * @param acc Robot joint motion angular acceleration, unit: degree/s^2
    * @param tol Robot joint motion end point error, unit: mm
    * @param option_cond Optional parameters for robot joints, if not needed, the value can be left unassigned, just fill in a null pointer
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed, double acc = 90, double tol = 0, const OptionalCond *option_cond= nullptr);

	/**
	 *@brief joint move with single param
	*/
	errno_t joint_move(MoveJParam param);

	/**
	* @brief Robot end linear motion
	* @param end_pos The target position of the robot's end motion.
	* @param move_mode Specify the motion mode: incremental (relative) or absolute.
	* @param is_block Set if the interface is a blocking interface, TRUE for blocking interface FALSE for non-blocking interface.
	* @param speed Robot linear motion speed, unit: mm/s
	* @param acc Acceleration of the robot in mm/s^2.
	* @param tol The robot's endpoint error in mm.
	* @param option_cond robot joint optional parameters, if not needed, the value can not be assigned, fill in the empty pointer can be
	* @param ori_vel Attitude velocity, unit degree/s
	* @param ori_acc Attitude acceleration, unit degree/s^2.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t linear_move(const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel = 500, double tol = 0, const OptionalCond *option_cond = nullptr, double ori_vel=3.14, double ori_acc=12.56);

	/**
	 *@brief linear move with single param
	*/
	errno_t linear_move(MoveLParam param);

	/**
	* @brief The robot's end circular motion.
	* @param end_pos The target position of the robot's end motion.
	* @param mid_pos The midpoint of the robot's end motion.
	* @param move_mode Specify the motion mode: incremental (relative) or absolute.
	* @param is_block Set if the interface is a blocking interface, TRUE for blocking interface FALSE for non-blocking interface.
	* @param speed the robot arc speed, unit: degree/s
	* @param acc The acceleration of the robot's arc motion, in degrees/s^2.
	* @param tol endpoint error of the robot's circular motion, in millimeters.
	* @param option_cond Optional parameter for robot joint, if not needed, this value can not be assigned, just fill in the empty pointer.
	* @param circle_cnt Specifies the number of circles of the robot. A value of 0 is equivalent to circle_move.
	* @param circle_mode Specifies the mode of the robot's circular motion, the parameter explanation is as follows:
	- 0: Fixed to use the axis angle of rotation angle less than 180° from the start attitude to the end attitude for attitude change; (current program)
	- 1: Fixedly adopts the axis angle of the rotation angle from the start attitude to the termination attitude which is greater than 180° for attitude change;
	- 2: Selection of whether the angle is less than 180° or more than 180° is automatically chosen according to the midpoint attitude;
	- 3: The attitude pinch angle is always consistent with the arc axis. (Current whole circle motion)

	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t circular_move(const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond = nullptr, int circle_cnt = 0, int circle_mode = 0);

	/**
	 *@brief circular move with single param
	*/
	errno_t circular_move(MoveCParam param);

	/**
	* @brief Setting the robot run rate
	* @param rapid_rate is the rapidity of the program, set in the range of [0,1]
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_rapidrate(double rapid_rate);

	/**
	* @brief Get the robot runtime rate
	* @param rapid_rate current control system rate
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_rapidrate(double *rapid_rate);

	/**
	* @brief Sets the tool information for the specified number.
	* @param id tool number
	* @param tcp The tool coordinate system offset from the flange coordinate system.
	* @param name Specifies the tool alias.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_tool_data(int id, const CartesianPose *tcp, const char *name);

	/**
	* @brief Set the ID of the currently used tool
	* @param id tool coordinate system ID
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_tool_id(const int id);

	/**
	* @brief Query the ID of the currently used tool
	* @param id Tool ID query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_tool_id(int *id);

	/**
	* @brief Query information about the tools used
	* @param id Tool ID query result
	* @param tcp tool coordinate system offset relative to flange coordinate system
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_tool_data(int id, CartesianPose *tcp);

	/**
	* @brief Set the user coordinate system information for the specified number.
	* @param id user_frame user_coordinate_system_number
	* @param user_frame The user coordinate system offset value.
	* @param name user_frame user_coordinate_system_alias
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_user_frame_data(int id, const CartesianPose *user_frame, const char *name);

	/**
	* @brief Sets the user coordinate system ID currently in use.
	* @param id user coordinate system ID
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_user_frame_id(const int id);

	/**
	* @brief Query the ID of the currently used user coordinate system.
	* @param id Get result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_user_frame_id(int *id);

	/**
	* @brief Query information about the user coordinate system being used.
	* @param id User coordinate system ID query result
	* @param tcp The user coordinate system bias value.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_user_frame_data(int id, CartesianPose *tcp);

	/**
	* @brief Robot load settings
	* @param payload load center of mass, mass data
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_payload(const PayLoad *payload);

	/**
	* @brief Getting robot load data
	* @param payload load query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_payload(PayLoad *payload);

	/**
	* @brief Get the position of the end of the tool in the current setting.
	* @param tcp_position Tool end position lookup result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_tcp_position(CartesianPose *tcp_position);

	/**
	* @brief Get the current joint angle of the robot.
	* @param joint_position Joint angle query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_joint_position(JointValue *joint_position);

	/**
	 * @brief is in estop
	 */	
	errno_t is_in_estop(BOOL *estop);

	/**
	* @brief Query if the robot exceeded the limit
	* @param on_limit query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t is_on_limit(BOOL *on_limit);

	/**
	* @brief Query whether robot motion has stopped
	* @param in_pos query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t is_in_pos(BOOL *in_pos);

	/**
	* @brief Terminate current arm motion
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t motion_abort();
	
	/**
	 *@brief get motion status
	 *@param status struct of motion status
	 */
	errno_t get_motion_status(MotionStatus *status);

///@}

///@name TIO part
///@{

	/**
	* @brief set tioV3 voltage parameter
	* @param vout_enable Voltage enable, 0:off, 1:on
	* @param vout_vol Voltage size 0:24v 1:12v
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_tio_vout_param(int vout_enable, int vout_vol);

	/**
	* @brief Get tioV3 voltage parameter
	* @param vout_enable Voltage enable, 0:off, 1:on
	* @param vout_vol Voltage size 0:24v 1:12v
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_tio_vout_param(int* vout_enable, int* vout_vol);

	/**
	* @brief Add or modify a semaphore
	* @param sign_info Signal parameter
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t add_tio_rs_signal(SignInfo sign_info);

	/**
	* @brief Delete a semaphore
	* @param sig_name Signal name
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t del_tio_rs_signal(const char* sig_name);

	/**
	* @brief RS485 transmit command
	* @param chn_id Channel number
	* @param data Data field
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t send_tio_rs_command(int chn_id, uint8_t* data,int buffsize);

	/**
	* @brief Getting Signal Information
	* @param SignInfo* Array of Signal Information
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_rs485_signal_info(SignInfo* sign_info_array, int* array_len);

	/**
	* @brief Set tio mode
	* @param pin_type tio type 0 for DI Pins, 1 for DO Pins, 2 for AI Pins
	* @param pin_type tio mode DI Pins: 0:0x00 DI2 is NPN,DI1 is NPN,1:0x01 DI2 is NPN,DI1 is PNP, 2:0x10 DI2 is PNP,DI1 is NPN,3:0x11 DI2 is PNP,DI1 is PNP
							 DO Pins: Low 8-bit data high 4-bit for DO2 configuration, low 4-bit for DO1 configuration, 0x0 DO for NPN output, 0x1 DO for PNP output, 0x2 DO for push-pull output, 0xF RS485H interface
							 AI Pins: 0: analog input function enable, RS485L disable, 1: RS485L interface enable, analog input function disable
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_tio_pin_mode(int pin_type, int pin_mode);

	/**
	* @brief Get tio mode
	* @param pin_type tio type 0 for DI Pins, 1 for DO Pins, 2 for AI Pins
	* @param pin_type tio mode DI Pins: 0:0x00 DI2 is NPN,DI1 is NPN,1:0x01 DI2 is NPN,DI1 is PNP, 2:0x10 DI2 is PNP,DI1 is NPN,3:0x11 DI2 is PNP,DI1 is PNP
							 DO Pins: Low 8-bit data high 4-bit for DO2 configuration, low 4-bit for DO1 configuration, 0x0 DO for NPN output, 0x1 DO for PNP output, 0x2 DO for push-pull output, 0xF RS485H interface
							 AI Pins: 0: analog input function enable, RS485L disable, 1: RS485L interface enable, analog input function disable
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_tio_pin_mode(int pin_type, int* pin_mode);

	/**
	* @brief RS485 communication parameter configuration
	* @param ModRtuComm When the channel mode is set to Modbus RTU, you need to specify the Modbus slave node ID additionally.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_rs485_chn_comm(ModRtuComm mod_rtu_com);

	/**
	* @brief RS485 communication parameter query
	* @param ModRtuComm query chn_id as input parameter
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_rs485_chn_comm(ModRtuComm* mod_rtu_com);

	/**
	* @brief RS485 communication mode configuration
	* @param chn_id 0: RS485H, channel 1; 1: RS485L, channel 2
	* @param chn_mode 0: Modbus RTU, 1: Raw RS485, 2, torque sensor
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_rs485_chn_mode(int chn_id, int chn_mode);

	/**
	* @brief RS485 communication mode query
	* @param chn_id Input parameter 0: RS485H, channel 1; 1: RS485L, channel 2
	* @param chn_mode Output parameters 0: Modbus RTU, 1: Raw RS485, 2, torque sensor
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_rs485_chn_mode(int chn_id, int* chn_mode);

///@}

///@name traj part
///@{

	/**
	* @brief Setting the Trajectory Replication Configuration Parameters
	* @param para Track Replication Configuration Parameters
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_traj_config(const TrajTrackPara *para);

	/**
	* @brief Get track replication configuration parameters
	* @param para Track Replication Configuration Parameters
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_traj_config(TrajTrackPara *para);

	/**
	* @brief Acquisition track reproduction data control switch
	* @param mode select TRUE, start data collection, select FALSE, end data collection
	* @param filename capture data storage file name, when filename is a null pointer, the storage file named after the current date
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_traj_sample_mode(const BOOL mode, char *filename);

	/**
	* @brief Acquisition track reproduction data status query
	* @param mode for TRUE, the data is being collected, for FALSE, the data collection is over, in the data collection state is not allowed to turn on the data collection switch again.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_traj_sample_status(BOOL *sample_status);

	/**
	* @brief Query the filename of the track replication data that already exists in the controller.
	* @param filename The filename of the track replication data that already exists in the controller.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_exist_traj_file_name(MultStrStorType *filename);

	/**
	* @brief Rename the filename of the track replication data.
	* @param src original file name
	* @param dest target file name, the length of the file name can not exceed 100 characters, the file name can not be empty, the target file name does not support the Chinese language
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t rename_traj_file_name(const char *src, const char *dest);

	/**
	* @brief Deletes the track replication data file from the controller.
	* @param filename The filename of the file to be deleted, filename is the name of the data file.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t remove_traj_file(const char *filename);

	/**
	* @brief Trajectory replication data file in the controller to generate controller execution scripts
	* @param filename filename of the data file, filename is the name of the data file without a suffix.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t generate_traj_exe_file(const char *filename);

///@}

///@name servo part
///@{
	/**
	* @brief Robot SERVO MOVE mode enable
	* @param enable TRUE to enter SERVO MOVE mode, FALSE to exit the mode
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t servo_move_enable(BOOL enable);

	/**
	 *@brief is in servo mode
	 */
	errno_t is_in_servomove(BOOL *in_servo);

	/**
	* @brief Robot joint spatial position control mode
	* @param joint_pos The target position of the robot joint motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	* @param step_num times the period, servo_j movement period for step_num * 8ms, where step_num> = 1
	* @return ERR_SUCC success Other failures
	*/
	errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num = 1);

	/**
	* @brief Robot Cartesian spatial position control mode
	* @param cartesian_pose The target position of the robot's Cartesian motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	* @param step_num times the period, servo_p movement period is step_num * 8ms, where step_num>=1
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num = 1);
	
	/**
	* @brief No filter is used in SERVO mode, this command cannot be set in SERVO mode, but can be set after exiting SERVO.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t servo_move_use_none_filter();

	/**
	* @brief SERVO mode joint space first-order low-pass filter, the command in the SERVO mode can not be set, after the exit SERVO can be set
	* @param cutoffFreq First-order low-pass filter cutoff frequency.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t servo_move_use_joint_LPF(double cutoffFreq);

	/**
	* @brief SERVO mode joint space nonlinear filtering, the command in SERVO mode can not be set, after the exit SERVO can be set
	* @param max_vr The upper speed limit (absolute value) of the velocity of attitude change in Cartesian space.
	* @param max_ar Upper value (absolute value) of acceleration for the velocity of attitude change in Cartesian space °/s^2
	* @param max_jr Upper limit value (absolute value) of acceleration for the velocity of attitude change in Cartesian space °/s^3
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t servo_move_use_joint_NLF(double max_vr, double max_ar, double max_jr);

	/**
	* @brief SERVO mode Cartesian space nonlinear filtering, the instruction in SERVO mode can not be set, after the exit SERVO can be set
	* @param max_vp The upper limit of the speed of the move command in Cartesian space (absolute value). Unit: mm/s
	* @param max_ap The upper limit (absolute value) of the acceleration of the move command in Cartesian space. Unit: mm/s^2
	* @param max_jp Upper limit value (absolute value) of the acceleration of the move command plus acceleration in Cartesian space in mm/s^3
	* @param max_vr Upper limit value (absolute value) of velocity in Cartesian space for attitude change velocity in °/s
	* @param max_ar Upper limit value of acceleration (absolute value) in °/s^2 for the velocity of attitude change in Cartesian space
	* @param max_jr Upper limit value (absolute value) of acceleration for the velocity of attitude change in Cartesian space °/s^3
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t servo_move_use_carte_NLF(double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr);

	/**
	* @brief SERVO mode joint space multi-order mean value filter, this instruction can not be set in SERVO mode, after the exit SERVO can be set.
	* @param max_buf The size of the mean value filter buffer.
	* @param kp acceleration filter coefficient
	* @param kv velocity filter coefficient
	* @param ka position filter coefficient
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t servo_move_use_joint_MMF(int max_buf, double kp, double kv, double ka);

	/**
	* @brief SERVO mode speed look-ahead parameter setting
	* @param max_buf buffer size
	* @param kp Acceleration filter coefficient
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t servo_speed_foresight(int max_buf, double kp);

///@}

///@name IO part
///@{
	/**
	* @brief Sets the value of a digital output variable (DO).
	* @param type DO type
	* @param index DO index
	* @param value DO set value
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_digital_output(IOType type, int index, BOOL value);

	/**
	* @brief set the value of the analog output variable (AO) value
	* @param type AO type
	* @param index AO index
	* @param value AO set value
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_analog_output(IOType type, int index, float value);

	/**
	* @brief Query Digital Input (DI) Status
	* @param type DI type
	* @param index DI index
	* @param result DI status query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_digital_input(IOType type, int index, BOOL *result);

	/**
	* @brief Query digital output (DO) status
	* @param type DO type
	* @param index DO index
	* @param result DO status query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_digital_output(IOType type, int index, BOOL *result);

	/**
	* @brief Get the value of the analog input variable (AI)
	* @param type The type of AI
	* @param index AI index
	* @param result Specify the result of AI status query.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_analog_input(IOType type, int index, float *result);

	/**
	* @brief Get the value of analog output variable (AO)
	* @param type The type of AO
	* @param index AO index
	* @param result Specify the result of AO status query.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_analog_output(IOType type, int index, float *result);


	/**
	* @brief Sets the value of a digital output variable (DO).
	* @param type DO type
	* @param index DO index
	* @param value DO set value
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_digital_output(IOType type, int index, BOOL* value, int len);

	/**
	* @brief set the value of the analog output variable (AO) value
	* @param type AO type
	* @param index AO index
	* @param value AO set value
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_analog_output(IOType type, int index, float* value, int len);

	/**
	* @brief Query Digital Input (DI) Status
	* @param type DI type
	* @param index DI index
	* @param result DI status query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_digital_input(IOType type, int index, BOOL *result, int len);

	/**
	* @brief Query digital output (DO) status
	* @param type DO type
	* @param index DO index
	* @param result DO status query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_digital_output(IOType type, int index, BOOL *result, int len);

	/**
	* @brief Get the value of the analog input variable (AI)
	* @param type The type of AI
	* @param index AI index
	* @param result Specify the result of AI status query.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_analog_input(IOType type, int index, float *result, int len);

	/**
	* @brief Get the value of analog output variable (AO)
	* @param type The type of AO
	* @param index AO index
	* @param result Specify the result of AO status query.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_analog_output(IOType type, int index, float *result, int len);


	/**
	* @brief Query whether the extended IO module is running.
	* @param is_running Query result of the running status of the extended IO module.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t is_extio_running(BOOL *is_running);


///@}

///@name program part
///@{
	/**
	* :: @brief Run the currently loaded job program
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t program_run();

	/**
	* :: @brief Suspend the currently running operational program
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t program_pause();

	/**
	* :: @brief Continue to run currently suspended job programs
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t program_resume();

	/**
	* @brief terminates the currently executing procedure.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t program_abort();

	/**
	* @brief Load the specified job program.
	* @param file Program file path
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t program_load(const char *file);

	/**
	* @brief Get the name of the loaded job program.
	* @param file Program file path
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_loaded_program(char *file);

	/**
	* @brief Get the current line number of the robot's job program.
	* @param curr_line The current line number query result.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_current_line(int *curr_line);

	/**
	* @brief Get the status of the robot's program execution.
	* @param status The result of the job program execution status query.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_program_state(ProgramState *status);

	/**
	 *@brief get user_var list
	 */
	errno_t get_user_var(UserVariableList* vlist);

	/**
	 *@brief set user var
	 */
	errno_t set_user_var(UserVariable v);

///@}

///@name traditional drag
///@{
	/**
	* @brief Controls whether the robot enters or exits drag-and-drop mode.
	* @param enable TRUE to enter drag and drop mode, FALSE to exit drag and drop mode.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t drag_mode_enable(BOOL enable);

	/**
	* @brief Query if the robot is in drag and drop mode
	* @param in_drag query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t is_in_drag_mode(BOOL *in_drag);
///@}

///@name collision part
///@{

	/**
	* @brief Query if the robot is in collision protection mode
	* @param in_collision query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t is_in_collision(BOOL *in_collision);

	/**
	* @brief Recovery from crash-protected mode after a collision
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t collision_recover();

	/**
	* @brief Error status clearing
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t clear_error();

	/**
	* @brief Set the collision level of the robot.
	* @param level collision level, level 0-5, 0 is off collision, 1 is collision threshold 25N, 2 is collision threshold 50N, 3 is collision threshold 75N, 4 is collision threshold 100N, 5 is collision threshold 125N
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_collision_level(const int level);

	/**
	* @brief Get the collision level set by the robot.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_collision_level(int *level);


///@}

///@name math
///@{
	/**
	* @brief Calculate the inverse solution for the specified pose in the current tool, current mounting angle, and current user coordinate system settings.
	* @param ref_pos The reference joint space position for the inverse solution calculation.
	* @param cartesian_pose Cartesian space pose value.
	* @param joint_pos The result of the joint space position calculation when the calculation is successful.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);

	/**
	* @brief Calculate the position of the specified joint under the current tool, current mounting angle, and current user coordinate system settings.
	* @param joint_pos The position of the joint in joint space.
	* @param cartesian_pose Cartesian space pose calculation result.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t kine_forward(const JointValue *joint_pos, CartesianPose *cartesian_pose);

	/**
	* @brief Conversion of Euler angles to rotation matrices.
	* @param rpy The Euler angle data to be converted.
	* @param rot_matrix The converted rotation matrix.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t rpy_to_rot_matrix(const Rpy *rpy, RotMatrix *rot_matrix);

	/**
	* @brief Conversion of rotation matrix to Euler angles
	* @param rot_matrix The rotation matrix data to be converted.
	* @param rpy The result of the converted RPY Euler angles.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t rot_matrix_to_rpy(const RotMatrix *rot_matrix, Rpy *rpy);

	/**
	* @brief Quaternion to Rotation Matrix Conversion
	* @param quaternion The quaternion data to be converted.
	* @param rot_matrix The result of the converted rotation matrix.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t quaternion_to_rot_matrix(const Quaternion *quaternion, RotMatrix *rot_matrix);

	/**
	* @brief Conversion of a rotation matrix to quaternions
	* @param rot_matrix The rotation matrix to be converted.
	* @param quaternion The result of the quaternion conversion.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t rot_matrix_to_quaternion(const RotMatrix *rot_matrix, Quaternion *quaternion);


///@}

///@name SDK support
///@{
	/**
	* @brief Set how long the robot controller will terminate the current motion of the robot arm after the SDK loses connection with the robot controller due to a network exception.
	* @param millisecond time parameter, in milliseconds
	* @param mnt The type of action the robot needs to perform in case of a network exception.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_network_exception_handle(float millisecond, ProcessType mnt);

	/**
	* @brief Get SDK version number
	* @param version SDK version number
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_sdk_version(char *version);

	/**
	* @brief Get controller IP
	* @param controller_name The controller name.
	* @param ip_list Controller ip list, controller name for the specific value to return the name of the corresponding controller IP address, controller name is empty, return to the segment class of all the controller IP address
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_controller_ip(char *controller_name, char *ip_list);

	/**
	* @brief Set the path to the error code file, if you need to use the get_last_error interface you need to set the path to the error code file, if you don't use the get_last_error interface, you don't need to set the interface.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_errorcode_file_path(char *path);

	/**
	* @brief Get the last error code of the robot, the last error code will be cleared when clear_error is called.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_last_error(ErrorCode *code);

	/**
	* @brief set whether to enable debugging mode, select TRUE, start debugging mode, at this time will output debugging information in the standard output stream, select FALSE, do not output debugging information.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_debug_mode(BOOL mode);

	/**
	* @brief get SDK log path
	* @param path path of SDK log
	* @param size size of char* buffer
	* @return error code 
	*/
	static errno_t static_Get_SDK_filepath(char* path, int size);

	/**
	* @brief get SDK log path
	* @param path path of SDK log
	* @param size size of char* buffer
	* @return error code 
	*/
	errno_t get_SDK_filepath(char* path, int size);

	/**
	* @brief set SDK log path
	* @param filepath SDK log path
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_SDK_filepath(const char *filepath);

	/**
	* @brief same as set_SDK_filepath
	*/
	static errno_t static_Set_SDK_filepath(const char *filepath);



///@}

///@name torque sensor and force control
///@{

	/**
	* @brief Setting the sensor brand
	* @param sensor_brand The brand of the sensor, the available values are 1,2,3 for different brands of torque sensors.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_torsenosr_brand(int sensor_brand);

	/**
	* @brief Get the sensor brand
	* @param sensor_brand The brand of the sensor, the available values are 1,2,3 for different brands of torque sensors.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_torsenosr_brand(int *sensor_brand);

	/**
	* @brief Turn the torque sensor on or off.
	* @param sensor_mode 0 means turn off the sensor, 1 means turn on the torque sensor
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_torque_sensor_mode(int sensor_mode);

	/**
	* @brief Setting the parameters of the flex control
	* @param axis means which axis to configure, the available values are 0~5.
	* @param opt soft direction, the available values are 1 2 3 4 5 6 corresponding to fx fy fz mx my mz 0 means unchecked.
	* @param ftUser damping force, it means how much force the user can use to make the robot move in a certain direction with maximum speed.
	* @param ftConstant stands for constant force, set to 0 for manual operation.
	* @param ftNormalTrack represents constant force, set to 0 for manual operation.
	* @param ftReboundFK is the rebound force, it represents the ability of the robot to return to the initial state.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_admit_ctrl_config(int axis, int opt, double ftUser, double ftConstant, int ftNnormalTrack, double ftReboundFK);

	/**
	* @brief Start recognizing end-of-tool loads.
	* @param joint_pos End position for automatic load recognition using the torque transducer.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t start_torq_sensor_payload_identify(const JointValue *joint_pos);

	/**
	* @brief Get end-load identification status.
	* @param identify_status 0 means identification complete, 1 means not complete, 2 means identification failed
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_torq_sensor_identify_staus(int *identify_status);

	/**
	* @brief Get end-load identification result
	* @param payload End-load
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_torq_sensor_payload_identify_result(PayLoad *payload);

	/**
	* @brief Setting the sensor payload
	* @param payload End-load
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_torq_sensor_tool_payload(const PayLoad *payload);

	/**
	* @brief Get the sensor payload
	* @param payload End-load
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_torq_sensor_tool_payload(PayLoad *payload);

	/**
	* @brief Force Control Towing Enablement
	* @param enable_flag 0 disables force drag enablement, 1 enables it.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t enable_admittance_ctrl(const int enable_flag);
	errno_t enable_tool_drive(const int enable_flag);

	/**
	* @brief Set force control type and sensor initialization status.
	* @param sensor_compensation Whether to enable sensor compensation, 1 means enable is initialized, 0 means not initialized.
	* @param compliance_type 0 for not using any kind of compliance control method 1 for constant force compliance control, 2 for speed compliance control
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_compliant_type(int sensor_compensation, int compliance_type);

	/**
	* @brief Get force control type and sensor initialization status.
	* @param sensor_compensation Whether to enable sensor compensation, 1 means enable is initialized, 0 means not initialized.
	* @param compliance_type 0 for not using any kind of compliance control method 1 for constant force compliance control, 2 for speed compliance control
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_compliant_type(int *sensor_compensation, int *compliance_type);

	/**
	* @brief Acquire force control tenderness control parameter
	* @param admit_ctrl_cfg The address where the machine's force control softness control parameters are stored.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_admit_ctrl_config(RobotAdmitCtrl *admit_ctrl_cfg);

	/**
	* @brief Setting the ip address of the force control sensor
	* @param type 0 is using tcp/ip protocol, 1 is using RS485 protocol.
	* @param ip_addr is the address of the force sensor.
	* @param port is the port number of the force sensor when using tcp/ip protocol.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_torque_sensor_comm(const int type, const char *ip_addr, const int port);

	/**
	* @brief Get the ip address of the force control sensor.
	* @param type 0 is using tcp/ip protocol, 1 is using RS485 protocol.
	* @param ip_addr is the address of the force sensor.
	* @param port is the port number of the force sensor when using tcp/ip protocol.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_torque_sensor_comm(int *type, char *ip_addr, int *port);

	/**
	* @brief Set the value of the low-pass filter of the torque sensor.
	* @param torque_sensor_filter Value of the low-pass filter in Hz.
	*/
	errno_t set_torque_sensor_filter(const float torque_sensor_filter);

	/**
	* @brief Get the value of the low-pass filter of the torque sensor.
	* @param torque_sensor_filter Value of the low-pass filter in Hz.
	*/
	errno_t get_torque_sensor_filter(float *torque_sensor_filter);

	/**
	* @brief Sets the configuration of the sensor limit parameter of the force sensor.
	* @param torque_sensor_soft_limit Sensor limit parameter of the force sensor
	*/
	errno_t set_torque_sensor_soft_limit(const FTxyz torque_sensor_soft_limit);

	/**
	* @brief Get the configuration of the sensor limit parameter of the force sensor.
	* @param torque_sensor_soft_limit Sensor limit parameter of the force sensor
	*/
	errno_t get_torque_sensor_soft_limit(FTxyz *torque_sensor_soft_limit);

	/**
	* @brief shut down force control
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t disable_force_control();

	/**
	* @brief Setting the velocity soft control parameter
	* @param vel_cfg is the velocity control parameter.
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_vel_compliant_ctrl(const VelCom *vel_cfg);

	/**
	* @brief Setting the soft control moment condition
	* @param ft is the soft control moment condition
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_compliance_condition(const FTxyz *ft);



	/**
	* @brief Setting the coordinate system of the guided control motion
	* @param ftFrame 0 tool 1 world 
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t set_ft_ctrl_frame(const int ftFrame);

	/**
	* @brief Get the coordinate system of the conductor-controlled motion.
	* @param ftFrame 0 tool 1 world
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_ft_ctrl_frame(int* ftFrame);

///@}

///@name FTP part
///@{
	/**
	* @brief Establish ftp link with controller
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t init_ftp_client();

	/**
	* @brief Establish an encrypted ftp link with the controller (requires app login and controller version support)
	* @param password Robot login password
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t init_ftp_client_with_ssl(char* password);

	/**
	* @brief Disconnect ftp from controller
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t close_ftp_client();
	/**
	* @brief Download a file of the specified type and name from the controller to the local
	* @param remote The absolute path to the file name inside the controller.
	* @param local The absolute path to the file name to be downloaded locally.
	* @param opt 1 single file 2 folder
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t download_file(char* local, char* remote, int opt);

	/**
	* @brief Upload a file of a specified type and name from the controller to the local
	* @param remote Absolute path of the file name to be uploaded inside the controller.
	* @param local Absolute path to local file name.
	* @param opt 1 single file 2 folder
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t upload_file(char* local, char* remote, int opt);


	/**
	* @brief Delete a file of the specified type and name from the controller.
	* @param remote Controller internal file name
	* @param opt 1 single file 2 folder
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t del_ftp_file(char* remote, int opt);

	/**
	* @brief Renames a file of the type and name specified by the controller.
	* @param remote Original name of the controller's internal file name
	* @param des The target name to rename.
	* @param opt 1 single file 2 folder
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t rename_ftp_file(char* remote, char* des, int opt);

	/**
	* @brief Query the controller directory
	* @param remotedir Controller internal folder name
	* @param type 0 files and folders 1 files 2 folders
	* @param ret query result
	* @return ERR_SUCC on success, otherwise failure
	*/
	errno_t get_ftp_dir(const char* remotedir, int type, char* ret);
///@}

///@name subscription part
///@{
	/**
	 @brief subscribe a resource, the combination of section and key represent a path to the resource
	 		section/key:
				robot/status
				tool_frame/id
				IO/DI
				motion/status
	 @param op true: subscribe, false: unsubscribe
	 */
	errno_t subscribe_info(char* section, char* key, bool op = true); // op: true/false means subscribe/unsubscribe

	/**
	 *@brief callback regist before subscription
	 */
	void set_subscription_callback(CallBackFuncType func);
///@}

	~JAKAZuRobot();

private:
	class BIFClass;
	BIFClass *ptr;
};


#undef DLLEXPORT_API
#endif
