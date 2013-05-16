#include "PracticalSocket.h"   // For UDPSocket and SocketException
#include <iostream>            // For cout and cerr
#include <cstdlib>             // For atoi()

#include "mavlink/common/mavlink.h"
#include <stdio.h>
const int BUFF_SIZE = 4096; // Longest string to receive

struct data {
	float x;
	float y;
	float z;
	bool good = false;
};

struct data mavlink_msg_decode(mavlink_message_t msg)
{
	struct data thisData;
	printf("Mssg Id: %d\n", msg.msgid);
	switch (msg.msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:{
			break;
		}
		case MAVLINK_MSG_ID_ATTITUDE: {

			break;
		}
//		case MAVLINK_MSG_ID_HIGHRES_IMU: {
//			printf("Inertial Data:\n");
//			thisData.x = (float) mavlink_msg_highres_imu_get_xacc(&msg);
//			thisData.y = (float) mavlink_msg_highres_imu_get_yacc(&msg);
//			thisData.z = (float) mavlink_msg_highres_imu_get_zacc(&msg);
//			thisData.good = true;
//			break;
//		}
		default:
			break;
	}
	return thisData;

}

int main(int argc, char *argv[]) {

  if (argc != 2) {                  // Test for correct number of parameters
    cerr << "Usage: " << argv[0] << " <Local Port>" << endl;
    exit(1);
  }

  unsigned short echoServPort = atoi(argv[1]);     // First arg:  local port

  try {
    UDPSocket sock(echoServPort);

    char buffer[BUFF_SIZE];
    string sourceAddress;              // Address of datagram source
    unsigned short sourcePort;         // Port of datagram source
    for(;;)
    {
    	memset(buffer, 0, BUFF_SIZE);
    	int bytesRcvd = sock.recvFrom(buffer, BUFF_SIZE, sourceAddress, sourcePort);
        if (bytesRcvd > 0)
        {
            // Something received - print out all bytes and parse packet
            mavlink_message_t msg;
            mavlink_status_t status;

//            printf("Bytes Received: %d", bytesRcvd);
            for (int i = 0; i < bytesRcvd; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
                {
                	 data recData = mavlink_msg_decode(msg);
                	 if (recData.good)
                		 printf("%f %f %f\n", recData.x,recData.y,recData.z);
                }
            }
        }
    }
  } catch (SocketException &e) {
    cerr << e.what() << endl;
    exit(1);
  }

  return 0;
}
