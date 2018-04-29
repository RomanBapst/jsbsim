
#include "FGPX4SimInterface.h"
#include "FGFDMExec.h"
#include "FGAtmosphere.h"
#include "FGPropagate.h"
#include "FGAuxiliary.h"

#include <cstdlib>
#include <fcntl.h>

#include "../../../Firmware/mavlink/include/mavlink/v2.0/mavlink_types.h"
#include "../../../Firmware/mavlink/include/mavlink/v2.0/common/mavlink.h"

#if defined(WIN32) && !defined(__CYGWIN__)
#  include <windows.h>
#else
#  include <netinet/in.h>       // htonl() ntohl()
#endif

using namespace std;

namespace JSBSim {

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS IMPLEMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

FGPX4SimInterface::FGPX4SimInterface(FGFDMExec* fdmex) : FGModel(fdmex)
{
  Name = "FGPX4SimInterface";
  socket_fd = -1;
  enabled = true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FGPX4SimInterface::~FGPX4SimInterface()
{
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGPX4SimInterface::InitModel(void)
{
  if (!FGModel::InitModel()) return false;

  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGPX4SimInterface::initSocket(unsigned port)
{
  socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd >= 0 ) {
      fcntl(socket_fd, F_SETFL, O_NONBLOCK);
      memset(&socket_name, 0, sizeof(struct sockaddr_in));
      socket_name.sin_family = AF_INET;
      socket_name.sin_port = htons(port);
      socket_name.sin_addr.s_addr =  htonl(INADDR_ANY);
      //memcpy(&scktName.sin_addr, host->h_addr_list[0], host->h_length);
      int len = sizeof(struct sockaddr_in);
      if (connect(socket_fd, (struct sockaddr*)&socket_name, len) == 0) {   // successful
        cout << "Connected to socket for PX4 interface" << endl;
      } else {                // unsuccessful
        cout << "Could not connect to socket for PX4 interface ..." << endl;
      }
    } else {          // unsuccessful
      cout << "Could not create socket for PX4 Interface" << endl;
    }

    return socket_fd >= 0;

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGPX4SimInterface::Run(void)
{
  if (FGModel::Run()) return true;

  if (enabled && !FDMExec->IntegrationSuspended()&& !FDMExec->Holding()) {
    RunPreFunctions();
    
    Cycle();

    RunPostFunctions();
  }
  return false;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPX4SimInterface::Cycle(void)
{
  mavlink_hil_sensor_t sensor_msg = {};

  sensor_msg.time_usec = FDMExec->GetSimTime() * 1e6;

  sensor_msg.xacc = fttom * (float)(Auxiliary->GetPilotAccel(1));    // X body accel, ft/s/s;
  sensor_msg.yacc = fttom * (float)(Auxiliary->GetPilotAccel(2));    // Y body accel, ft/s/s;
  sensor_msg.zacc = fttom * (float)(Auxiliary->GetPilotAccel(3));    // Z body accel, ft/s/s;
  sensor_msg.xgyro = (float)Propagate->GetPQR(eP);
  sensor_msg.ygyro = (float)Propagate->GetPQR(eQ);
  sensor_msg.zgyro = (float)Propagate->GetPQR(eR);

  FGMatrix33 Tl2b = Propagate->GetTl2b();

  FGColumnVector3 mag(0.21523, 0, 0.42741);

  mag = Tl2b * mag;

  sensor_msg.xmag = (float)mag(1);
  sensor_msg.ymag = (float)mag(2);
  sensor_msg.zmag = (float)mag(3);

  // convert from psf to millibar
  sensor_msg.abs_pressure = psftopa * (float)Atmosphere->GetPressure() / 100.0f;

  sensor_msg.pressure_alt = fttom * (float)Propagate->GetAltitudeASL();

  sensor_msg.temperature = RankineToCelsius(Atmosphere->GetTemperature());

  sensor_msg.fields_updated = 4095;

  mavlink_message_t msg;
  mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, &msg);

  mavlink_hil_gps_t hil_gps_msg;
  hil_gps_msg.time_usec = FDMExec->GetSimTime() * 1e6;
  hil_gps_msg.fix_type = 3;
  hil_gps_msg.lat = (int)(Propagate->GetLocation().GetLatitude() * radtodeg * 1e7);
  hil_gps_msg.lon = (int)(Propagate->GetLocation().GetLongitude() * radtodeg * 1e7);
  hil_gps_msg.alt = (int)(fttom * (float)Propagate->GetAltitudeASL() * 1000);
  hil_gps_msg.eph = 2.0f;
  hil_gps_msg.epv = 2.0f;
  hil_gps_msg.vel = (float)sqrt(Propagate->GetVel(eNorth) * Propagate->GetVel(eNorth) + Propagate->GetVel(eEast) * Propagate->GetVel(eEast)) * fttom * 100;
  hil_gps_msg.vn = (float)Propagate->GetVel(eNorth) * fttom * 100;
  hil_gps_msg.ve = (float)Propagate->GetVel(eEast) * fttom * 100;
  hil_gps_msg.vd = (float)Propagate->GetVel(eDown) * fttom * 100;
  hil_gps_msg.satellites_visible = 10;

  mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);

  packetlen +=  mavlink_msg_to_send_buffer(&buffer[packetlen], &msg);

  if (socket_fd >= 0) {
    send(socket_fd,(char *)&buffer[0],packetlen, 0);
    char buf[1024];
    int num_chars = 0;
    num_chars = recv(socket_fd, buf, sizeof(buf), 0);


    mavlink_message_t msg;
    mavlink_status_t udp_status = {};

    if (num_chars > 0) {
      for (int i = 0; i < num_chars; i++) {
          if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &udp_status)) {
            // have a message, handle it
            switch (msg.msgid) {
              case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
                mavlink_hil_actuator_controls_t controls;
                mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

                FGPropertyManager* node=0;
                node = PropertyManager->GetNode("fcs/aileron-cmd-norm");
                node->setDoubleValue(controls.controls[0]);

                 node = PropertyManager->GetNode("fcs/elevator-cmd-norm");
                node->setDoubleValue(-controls.controls[1]);

                 node = PropertyManager->GetNode("fcs/rudder-cmd-norm");
                node->setDoubleValue(-controls.controls[2]);

                 node = PropertyManager->GetNode("fcs/throttle-cmd-norm");
                node->setDoubleValue(controls.controls[3]);

                break;
            }
          }
        }
    }

  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGPX4SimInterface::Load(Element* element)
{
  string name="";
  int OutRate = 0;
  unsigned int port = 14560;

  
  document = element;

  if (!document) return false;

  name = FDMExec->GetRootDir() + document->GetAttributeValue("name");


  if (!document->GetAttributeValue("port").empty()) {
    port = atoi(document->GetAttributeValue("port").c_str());
  }

  if (!document->GetAttributeValue("rate").empty()) {
    OutRate = (int)document->GetAttributeValueAsNumber("rate");
  } else {
    OutRate = 1;
  }

  SetRate(OutRate);

  return initSocket(port);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGPX4SimInterface::SetRate(int rtHz)
{
  rtHz = rtHz>1000?1000:(rtHz<0?0:rtHz);
  if (rtHz > 0) {
    rate = (int)(0.5 + 1.0/(FDMExec->GetDeltaT()*rtHz));
    Enable();
  } else {
    rate = 1;
    Disable();
  }
}

}
