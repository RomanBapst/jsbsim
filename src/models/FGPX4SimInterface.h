#ifndef FGPX4SIMINTERFACE_H
#define FGPX4SIMINTERFACE_H

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "FGModel.h"
#include <sys/socket.h>
#include <netinet/in.h>


#include "input_output/FGXMLFileRead.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FORWARD DECLARATIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

namespace JSBSim {

class FGPX4SimInterface;

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DOCUMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DECLARATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

class FGPX4SimInterface : public FGModel, public FGXMLFileRead
{
public:
  FGPX4SimInterface(FGFDMExec*);
  ~FGPX4SimInterface();

  bool InitModel(void);
  bool Run(void);

  void Enable(void) { enabled = true; }
  void Disable(void) { enabled = false; }
  bool Toggle(void) {enabled = !enabled; return enabled;}
  bool Load(Element* el);
  void SetRate(int rt);

private:
  std::vector <FGPropertyManager*> OutputProperties;

  int socket_fd;  // socket file descriptor used for communication with PX4 simulation
  struct sockaddr_in socket_name;

  bool enabled;


  void Cycle();
  bool initSocket(unsigned port);

};
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#endif