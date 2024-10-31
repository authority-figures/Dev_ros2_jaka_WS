/**
* @last update Nov 30 2021 
* @Maintenance star@jaka
*/
#ifndef _JHTYPES_H_
#define _JHTYPES_H_

#define TRUE 1
#define FALSE 0
#include <stdio.h>
#include <stdint.h>

typedef int BOOL;	 //SDK BOOL
typedef int JKHD;	 //robot control handler
typedef int errno_t; //SDK error code

/**
* @brief cartesian trans, unit: mm
*/
typedef struct
{
	double x;
	double y;
	double z;
} CartesianTran;

/**
* @brief cartesian ori, unit degree
*/
typedef struct
{
	double rx;
	double ry;
	double rz;
} Rpy;

/**
* @brief Quaternion
*/
typedef struct
{
	double s;
	double x;
	double y;
	double z;
} Quaternion;

/**
 *@brief cartesian pos including trans and ori
 */
typedef struct
{
	CartesianTran tran;
	Rpy rpy;
} CartesianPose;

/**
* @brief rot matrix
*/
typedef struct
{
	CartesianTran x; ///< x
	CartesianTran y; ///< y
	CartesianTran z; ///< z
} RotMatrix;

/**
* @brief program state enum
*/
typedef enum
{
	PROGRAM_IDLE,	 ///< idle
	PROGRAM_RUNNING, ///< running
	PROGRAM_PAUSED	 ///< paused
} ProgramState;

/**
* @brief coordinate type enum
*/
typedef enum
{
	COORD_BASE,
	COORD_JOINT,
	COORD_TOOL
} CoordType;

/**
* @brief jog mode enum
*/
typedef enum
{
	ABS = 0,
	INCR,
	CONTINUE
} MoveMode;


/**
* @brief payload
*/
typedef struct
{
	double mass;			///< unit ：kg
	CartesianTran centroid; ///< unit ：mm
} PayLoad;

/**
* @brief joint position, unit: degree
*/
typedef struct
{
	double jVal[6];
} JointValue;

/**
* @brief collision option
*/
typedef struct{
	int collisionMethod;
	int reactionType;
	float reboundAngle;
}CollisionOption;

/**
* @brief collision option setting type
*/
typedef enum {
	CollitionOption_ALL,
	CollitionOption_Method,
	CollitionOption_ReactionType,
	CollitionOption_ReboundAngle
}CollisionOptionSettingType;

/**
* @brief IO type enum
*/
typedef enum
{
	IO_CABINET, ///< cabinet panel IO
	IO_TOOL,	///< tool IO
	IO_EXTEND,	///< extend IO(work as modbus master)
	IO_RELAY,   ///< relay IO，only available in CAB V3 (DO)
	IO_MODBUS_SLAVE, ///< Modbus slave IO, index from 0
	IO_PROFINET_SLAVE, ///< Profinet slave IO, index from 0
	IO_EIP_SLAVE      ///< ETHRENET/IP slave IO, index from 0
} IOType;


/**
* @brief callback
* @param info remember to assign char array, no less than 1024 bytes
				the feedback info contains at least 3 data part: "section", "key", "data"
				which is packed as json. The "data" part represent the current(new) value
*/
typedef void (*CallBackFuncType)(char* info);


/**
* @brief basic robot stat
*/
typedef struct
{
	int errcode;	///< 0: normal, others: errorcode
	int powered_on;	///< 0: power off，1: power on
	int enabled;	///< 0: disabled，1: enabled
} RobotStatus;

/**
 *@brief motion status
 */
typedef struct
{
	int motion_line; 		///< the executing motion cmd id
	int motion_line_sdk; 	///< reserved
	BOOL inpos; 			///< previous motion cmd is done, should alway check queue info together
	BOOL err_add_line; 		///< previous motion cmd is dropped by controller, like target is already reached
	int queue; 				///< motion cmd number in buffer
	int active_queue; 		///< motion cmd number in blending buffer
	BOOL queue_full;		///< motion buffer is full and motion cmds reveived at this moment will be dropped
	BOOL paused;			///< motion cmd is paused and able to resume
} MotionStatus;

/**
* @brief error information
*/
typedef struct
{
	long code;		   ///< error code
	char message[120]; ///< error message
} ErrorCode;

/**
* @brief traj config
*/
typedef struct
{
	double xyz_interval; ///< catesian trans sampling accuracy
	double rpy_interval; ///< catesian ori sampling accuracy
	double vel;			 ///< 
	double acc;			 ///< 
} TrajTrackPara;

#define MaxLength  256
/**
* @brief 
*/
typedef struct
{
	int len;			 ///< length
	char name[MaxLength][MaxLength]; ///< 
} MultStrStorType;

/**
* @brief not used
*/
typedef struct
{
	int executingLineId; ///< cmd id
} OptionalCond;

/**
* @brief operations when lost communication
*/
typedef enum
{
	MOT_KEEP,  ///< no change
	MOT_PAUSE, ///< pause
	MOT_ABORT  ///< abort
} ProcessType;

/**
* @brief admittance config
*/
typedef struct
{
	int opt;			 ///< 0: disable, 1: enable
	int axis;
	double ft_user;		 ///< force to let robot move with max speed
	double ft_rebound;	 ///< rebound ability
	double ft_constant;	 ///< const force
	int ft_normal_track; ///< 0: disable，1: enable
} AdmitCtrlType;

/**
* @brief tool drive config
*/
typedef struct
{
	int opt;			///< 0: disable, 1: enable
	int axis;			///< axis index, [0,5]
	double rebound;	 	///< rebound ability
	double rigidity;	///< 
} ToolDriveConfig;

/**
* @brief used for get/set_cst_force_ctrl_config
*/
typedef struct
{
	int opt;			///< 0: disable, 1: enable
	int axis;			///< axis index, [0,5]
	double rebound;	 	///< rebound ability
	double constant;	///< const force
	double rigidity; 	///< ftDamping
} ConstForceConfig;



/**
* @brief admittance config group
*/
typedef struct
{
	AdmitCtrlType admit_ctrl[6];
} RobotAdmitCtrl;

/**
* @brief 
*/
typedef struct
{
	ToolDriveConfig config[6];
} RobotToolDriveCtrl;

/**
* @brief 
*/
typedef struct
{
	ConstForceConfig config[6];
} RobotConstForceCtrl;

/**
 @brief 
 */
typedef enum{
	FTFrame_Tool = 0,
	FTFrame_World = 1
}FTFrameType;

/**
* @brief vel control level config
* 1>rate1>rate2>rate3>rate4>0
* level 1: only able to set rate1,rate2。rate3,rate4 are 0
* level 2，only able to set rate1,rate2，rate3。rate4 is 0
* level 3，able to set rate1,rate2，rate3,rate4
*/
typedef struct
{
	int vc_level; // control level
	double rate1; //
	double rate2; //
	double rate3; //
	double rate4; //
} VelCom;

/**
* @brief 
*/
typedef struct
{
	double fx;
	double fy;
	double fz;
	double tx;
	double ty;
	double tz;
} FTxyz;

/**
* @brief torque sensor data
*/
typedef struct
{
	int status;
	int errorCode;
	FTxyz data;
} TorqSensorData;

/**
* @brief ftp file/folder info
*/
struct FtpFile
{
	const char *filename;
	FILE *stream;
};

/**
* @brief torque sensor value type
*/
typedef enum
{
	Actual,	 ///< actual feedback
	General, ///< used by controller
	Real	 ///< real feedback without gravity and bias
} TorqSensorDataType;

/**
 *  @brief DH parameters
 */
typedef struct
{
	double alpha[6];
	double a[6];
	double d[6];
	double joint_homeoff[6];
} DHParam;

/**
 *  @brief rs485 signal info
 */
typedef struct
{
	char sig_name[20];
	int chn_id;		
	int sig_type;	
	int sig_addr;	
	int value;		
	int frequency;	//no more than 10
}SignInfo;

/**
 *  @brief rs485RTU config
 */
typedef struct
{
	int chn_id;		//RS485 channel ID
	int slaveId;	//must set Modbus slave ID when modbus RTU
	int baudrate;	//4800,9600,14400,19200,38400,57600,115200,230400
	int databit;	//7，8
	int stopbit;	//1，2
	int parity;		//78: no check,  79: odds parity,  69: even parity
}ModRtuComm;

/**
 *@brief joint move param
 */
typedef struct{
	int id;				///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
	BOOL is_block;		///< block until this cmd is done
	JointValue joints;	///< targe joint value
	MoveMode mode;		///< motion mode
	double vel;			///< velocity
	double acc;			///< acceleration, set to 90 if you have no idea
	double tol;			///< tolerance, used for blending. set to 0 if you want to reach a fine point
} MoveJParam;

typedef struct{
	int id;					///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
	BOOL is_block;			///< block until this cmd is done
	CartesianPose end_pos;	///< taget position
	MoveMode move_mode;		///< motion mode
	double vel;				///< velocity
	double acc;				///< acceleration, set to 500 if you have no idea
	double tol;				///< tolerance, used for blending. set to 0 if you want to reach a fine point
	double ori_vel;			///< set to 3.14 if you have no idea
	double ori_acc;			///< set to 12.56 if you have no idea
} MoveLParam;

typedef struct{
	int id;					///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
	BOOL is_block;			///< block until this cmd is done
	CartesianPose mid_pos;	///< mid position
	CartesianPose end_pos;	///< end position
	MoveMode move_mode;		///< motion mode
	double vel;				///< velocity
	double acc;				///< acceleration, set to 500 if you have no idea
	double tol;				///< tolerance, used for blending. set to 0 if you want to reach a fine point
	
	double circle_cnt;		///< circule count
	int circle_mode;		///< clock wise or counter clock wise
} MoveCParam;

/**
 *@brief Controller User Variable Struct
 *@param id controller inner usage
 *@param value value type always double
 *@param alias variable alias which is less than 100 bytes
 */
typedef struct {
	int id;
	double value;
	char alias[100];
} UserVariable;

/**
 * @brief number of UserVariable is fixed to 100
 */
typedef struct{
	UserVariable v[100];
} UserVariableList;


#endif
