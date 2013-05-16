//Simulink definitions
#define S_FUNCTION_NAME  MAVLink_Interface
#define S_FUNCTION_LEVEL 2

// UDP
#include "PracticalSocket.h"

// POSIX threads
#include <pthread.h>

// For sleep()
#include <unistd.h>

//For Simulink structures
#include "simstruc.h"

//For MAVLink definitions
#include "mavlink/common/mavlink.h"
#include "mavlink/mavlink_types.h"
#include "mavlink/common/common.h"



//Simulink block parameter definitions
// Port
#define COM_BUAD_IDX 0
#define COM_BAUD(S) ssGetSFcnParam(S,COM_BUAD_IDX)
// Output messages
#define MSG_OUT_IDX  1
#define MSG_OUT(S) ssGetSFcnParam(S,MSG_OUT_IDX)

//Total number of parameters
#define NPARAMS   2

//Simulink housekeeping
#if !defined(MATLAB_MEX_FILE)
/*
 * This file cannot be used directly with the Real-Time Workshop. 
 */
# error This_file_can_be_used_only_during_simulation_inside_Simulink
#endif

//boolean governing thread loops
bool looping = true;
bool hasTime = false;
bool visionUpdae = false;
// UDP read thread
void * mavThreadRead(void * arg);


//Simulink helper functions
int_T mavlink_msg_decode(SimStruct* S, mavlink_message_t msg);

//Initialise sizes of Simulink structures based on parameters
static void mdlInitializeSizes(SimStruct *S) {
    /* Custom Data Types */
    // HANDLE handle = NULL;
    pthread_t * thread;
    DTypeId SS_THREAD = ssRegisterDataType(S, "SS_THREAD");
    if(SS_THREAD == INVALID_DTYPE_ID) return;
    if( !ssSetDataTypeSize(S, SS_THREAD, sizeof(thread)) ) return;
    ssSetDataTypeZero(S, SS_THREAD, &thread);
    
    int_T nInputPorts  = 0;  /* number of input ports  */
    int_T nOutputPorts = 0;  /* number of output ports */
    
    uint8_T *ptrMsg_id_in;
    uint8_T *ptrMsg_id_out;

    
    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        //mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return;
    }
#endif

//    ssSetSFcnParamTunable(S,COM_PORT_IDX,false);
    ssSetSFcnParamTunable(S,COM_BUAD_IDX,false);
    ssSetSFcnParamTunable(S,MSG_OUT_IDX,false);
    
    
    ptrMsg_id_out =  (uint8_T*) mxGetData(MSG_OUT(S)); /* pointer to MSG_OUT_IDs */
    nOutputPorts = (int_T) mxGetNumberOfElements(MSG_OUT(S)); /* set Num of output port according to the MAVlink MSG */


    /* Register the number and type of states the S-Function uses */

    ssSetNumContStates(S, 0);   /* number of continuous states           */
    ssSetNumDiscStates(S, 0);   /* number of discrete states             */

    /*
     * Configure the input ports.  
     */
    if (!ssSetNumInputPorts(S, 0)) return;    

    /*
     * Configure the output ports. 
     */
    if (!ssSetNumOutputPorts(S, nOutputPorts)) return;
	for (int ii = 0; ii < nOutputPorts; ii++)
    {
        switch ( *(ptrMsg_id_out + ii) ){
            case MAVLINK_MSG_ID_HIGHRES_IMU:{
                ssSetOutputPortWidth(S, ii, 9); /* acc(xyz) gyro(xyz) mag(xyz) */
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE: {
            	ssSetOutputPortWidth(S, ii, 6); /* r, p, y, r_speed, p_speed, y_speed*/
            	break;
            }
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
                 ssSetOutputPortWidth(S, ii, 4); /* q0 q1 q2 q3*/
                 break;
             }
            case MAVLINK_MSG_ID_DEBUG_VECT:{
            	ssSetOutputPortWidth(S, ii, 3); /*x,y,z*/
            	break;
            }
            case MAVLINK_MSG_ID_DEBUG:{
            	ssSetOutputPortWidth(S, ii, 1); /*x,y,z*/
            	break;
            }
            case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:{
            	ssSetOutputPortWidth(S, ii, 7); /*x,y,z,r,p,y,update*/
            	break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:{
            	ssSetOutputPortWidth(S, ii, 6);
            	break;
            }
            case 200:{
            	ssSetOutputPortWidth(S, ii, 1); /* timestamp */
            	hasTime = true;
            	break;
            }
            default:
                ssSetOutputPortWidth(S, ii, 1);
                break;
        }       
     } 
    
    
    
    /* setup work Vectors */
    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, 0, 1); // Dwork for the pthread
    ssSetDWorkDataType(S, 0, SS_THREAD);
    
    ssSetNumRWork(S, 0); // length of each input buffer
    
    ssSetNumPWork(S, nOutputPorts);
    
    /* setup bool work vector*/
    ssSetNumSampleTimes(S, 1);   /* number of sample times   */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);



}


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.005); // 5mS sample time
    // ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}

unsigned short echoServPort = 14550;     // TODO: make paramter
UDPSocket sock(echoServPort);
const int BUFF_SIZE = 4096;
string sourceAddress;              // Address of datagram source
unsigned short sourcePort;         // Port of datagram source
// try {
//
//	} catch (SocketException &e) {
//	      printf("Socket couldnt be created. %s", e.what()); // MORE HORRIBLE THINGS <<<---
//	}
//
#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{    
    int_T nOutputPorts = (int_T)mxGetNumberOfElements(MSG_OUT(S)); /* set Num of output port according to the MAVlink MSG */

    pthread_t * UDP_read = (pthread_t*) ssGetDWork(S, 0);

    real_T** ppPwork_data = (real_T**) ssGetPWork(S);
    /*Allocate memory for Output data*/
    for (int ii = 0; ii < nOutputPorts; ii++)
        ppPwork_data[ii] = (real_T*) calloc(ssGetOutputPortWidth(S,ii), sizeof(real_T));

    // !!! Maybe setup the UDP port and store a pointer to it in DWork ...

    looping = true;
    
//    pthread_create(UDP_read, NULL, mavThreadRead, S);
}
#endif /*  MDL_START */


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block. Generally outputs are placed in the output vector(s),
 *    ssGetOutputPortSignal.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
	// maybe put this in mdlUpdate ?
	try {

		char buffer[BUFF_SIZE];
		memset(buffer, 0, BUFF_SIZE);
		int bytesRcvd = 0, count = 0;
		while( bytesRcvd <= 0 && count < 50) // 50 tries
		{
			bytesRcvd = sock.recvFrom(buffer, BUFF_SIZE, sourceAddress, sourcePort); // Non-blocking horrible thing!
			count++;
		}
		if (bytesRcvd > 0)
		{
			  mavlink_message_t msg;
			  mavlink_status_t status;
			  for (int i = 0; i < bytesRcvd; ++i)
				  if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
					  mavlink_msg_decode(S, msg);
		}
	} catch (SocketException &e) {
	  printf("%s", e.what());
	}

    int_T nOutputPorts = (int_T)mxGetNumberOfElements(MSG_OUT(S));
    int_T nOutputWidth; 
    real_T** ppPwork_data = (real_T**) ssGetPWork(S);
    
    real_T** outputArray;
    
    outputArray = (real_T**)calloc(nOutputPorts,sizeof(real_T*));
    
    for (int ii = 0; ii < nOutputPorts; ii++)
    {
        *(outputArray+ii) = ssGetOutputPortRealSignal(S,ii);
        nOutputWidth = ssGetOutputPortWidth(S,ii);
        for (int jj = 0; jj < nOutputWidth; jj++)
        {
            *(*(outputArray+ii) + jj) = ppPwork_data[ii][jj]; /* assign output values*/
        }        
    }
    free(outputArray);
}


static void mdlTerminate(SimStruct *S)
{
    int_T nOutputPorts = (int_T)mxGetNumberOfElements(MSG_OUT(S));
    real_T** ppPwork_data = (real_T**) ssGetPWork(S);
    
//    pthread_t * thread = (pthread_t *) ssGetDWork(S, 0);
    
    
    for(int ii = 0; ii < nOutputPorts; ii++){
        if(ppPwork_data[ii]!=NULL){
            free(ppPwork_data[ii]);        
        }     
    }
    looping = false;
    sock.disconnect();
    printf("Socket has disconnected.\n");
    sleep(1);
//    pthread_join(*thread, NULL);
}



void * mavThreadRead(void * arg)
{
    SimStruct* S = (SimStruct*) arg;
    
    unsigned short echoServPort = 14550;     // TODO: make paramter

    try {
      UDPSocket sock(echoServPort);

      char buffer[BUFF_SIZE];
      string sourceAddress;              // Address of datagram source
      unsigned short sourcePort;         // Port of datagram source
      for(;;)
      {
      	memset(buffer, 0, BUFF_SIZE);
      	printf("Ready to receive!\n");
      	int bytesRcvd = sock.recvFrom(buffer, BUFF_SIZE, sourceAddress, sourcePort);
		  if (bytesRcvd > 0)
		  {

			  mavlink_message_t msg;
			  mavlink_status_t status;

			  printf("Bytes Received: %d", bytesRcvd);
			  for (int i = 0; i < bytesRcvd; ++i)
			  {
				  if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
				  {
					  //mavlink_msg_decode(S, msg);
					  printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
				  }
			  }
			  printf("\n");
		  }
		  usleep(20000);
      }
    } catch (SocketException &e) {
      printf("%s", e.what());
      mdlTerminate(S);
    }
    return 0;
}

uint32_t lastTIME = 0;
int8_T  indexVision = -1;

int_T mavlink_msg_decode(SimStruct* S, mavlink_message_t msg)
{   

    int_T nOutputPorts = (int_T)mxGetNumberOfElements(MSG_OUT(S));
    real_T** ppPwork_data = (real_T**) ssGetPWork(S);
    
    uint32_t TIMESTAMP = 0;

    uint8_T* ptrMsg_id_out = (uint8_T*) mxGetData(MSG_OUT(S));
    int8_T index;

    for (index = 0; index < nOutputPorts; index++)
    {
        if (msg.msgid == *(ptrMsg_id_out + index))
            break;
    }

    if (index == nOutputPorts)
    {
        index = -1;
        return -1;

    }
    else{
    	visionUpdae = false;
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_HIGHRES_IMU: {
                ppPwork_data[index][0] = (real_T) mavlink_msg_highres_imu_get_xacc(&msg);
                ppPwork_data[index][1] = (real_T) mavlink_msg_highres_imu_get_yacc(&msg);
                ppPwork_data[index][2] = (real_T) mavlink_msg_highres_imu_get_zacc(&msg);
                ppPwork_data[index][3] = (real_T) mavlink_msg_highres_imu_get_xgyro(&msg);
                ppPwork_data[index][4] = (real_T) mavlink_msg_highres_imu_get_ygyro(&msg);
                ppPwork_data[index][5] = (real_T) mavlink_msg_highres_imu_get_zgyro(&msg);
                ppPwork_data[index][6] = (real_T) mavlink_msg_highres_imu_get_xmag(&msg);
                ppPwork_data[index][7] = (real_T) mavlink_msg_highres_imu_get_ymag(&msg);
                ppPwork_data[index][8] = (real_T) mavlink_msg_highres_imu_get_zmag(&msg);
                TIMESTAMP = mavlink_msg_highres_imu_get_time_usec(&msg);
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
				ppPwork_data[index][0] = (real_T) mavlink_msg_attitude_quaternion_get_q1(&msg);
				ppPwork_data[index][1] = (real_T) mavlink_msg_attitude_quaternion_get_q2(&msg);
				ppPwork_data[index][2] = (real_T) mavlink_msg_attitude_quaternion_get_q3(&msg);
				ppPwork_data[index][3] = (real_T) mavlink_msg_attitude_quaternion_get_q4(&msg);
//				TIMESTAMP = mavlink_msg_attitude_quaternion_get_time_boot_ms(&msg);
				break;
            }
            case MAVLINK_MSG_ID_ATTITUDE:{
            	ppPwork_data[index][0] = (real_T) mavlink_msg_attitude_get_roll(&msg);
            	ppPwork_data[index][1] = (real_T) mavlink_msg_attitude_get_pitch(&msg);
            	ppPwork_data[index][2] = (real_T) mavlink_msg_attitude_get_yaw(&msg);
            	ppPwork_data[index][3] = (real_T) mavlink_msg_attitude_get_rollspeed(&msg);
            	ppPwork_data[index][4] = (real_T) mavlink_msg_attitude_get_pitchspeed(&msg);
            	ppPwork_data[index][5] = (real_T) mavlink_msg_attitude_get_yawspeed(&msg);
//            	TIMESTAMP = mavlink_msg_attitude_get_time_boot_ms(&msg);
            	break;
            }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:{
				ppPwork_data[index][0] = (real_T) mavlink_msg_local_position_ned_get_x(&msg);
				ppPwork_data[index][1] = (real_T) mavlink_msg_local_position_ned_get_y(&msg);
				ppPwork_data[index][2] = (real_T) mavlink_msg_local_position_ned_get_z(&msg);
				ppPwork_data[index][3] = (real_T) mavlink_msg_local_position_ned_get_vx(&msg);
				ppPwork_data[index][4] = (real_T) mavlink_msg_local_position_ned_get_vy(&msg);
				ppPwork_data[index][5] = (real_T) mavlink_msg_local_position_ned_get_vz(&msg);
//            	TIMESTAMP = mavlink_msg_attitude_get_time_boot_ms(&msg);
				break;
			}
            case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:{
            	ppPwork_data[index][0] = (real_T) mavlink_msg_vision_position_estimate_get_x(&msg);
            	ppPwork_data[index][1] = (real_T) mavlink_msg_vision_position_estimate_get_y(&msg);
            	ppPwork_data[index][2] = (real_T) mavlink_msg_vision_position_estimate_get_z(&msg);
            	ppPwork_data[index][3] = (real_T) mavlink_msg_vision_position_estimate_get_roll(&msg);
            	ppPwork_data[index][4] = (real_T) mavlink_msg_vision_position_estimate_get_pitch(&msg);
            	ppPwork_data[index][5] = (real_T) mavlink_msg_vision_position_estimate_get_yaw(&msg);
            	if (indexVision < 0)
            		indexVision = index;
            	visionUpdae = true;
            	break;
            }
            case MAVLINK_MSG_ID_DEBUG_VECT:{
            	ppPwork_data[index][0] = (real_T) mavlink_msg_debug_vect_get_x(&msg);
				ppPwork_data[index][1] = (real_T) mavlink_msg_debug_vect_get_y(&msg);
				ppPwork_data[index][2] = (real_T) mavlink_msg_debug_vect_get_z(&msg);
//				TIMESTAMP = mavlink_msg_debug_vect_get_time_usec(&msg);
            	break;
            }
            case MAVLINK_MSG_ID_DEBUG:{
            	ppPwork_data[index][0] =  mavlink_msg_debug_get_value(&msg);
            	break;
            }
            default:
                break;
        }

		if(hasTime && TIMESTAMP)
			ppPwork_data[nOutputPorts-1][0] =  TIMESTAMP;

		if(indexVision >= 0)
			ppPwork_data[indexVision][6] = (real_T) visionUpdae;
    }
    return (int_T)msg.msgid;
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
