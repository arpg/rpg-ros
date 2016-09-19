//Test executable to exercise the jaguar base object

#include <jaguar_base.h>
#include <jaguar_consts.h>

#define DEFAULT_PORT "/dev/ttyUSB0"
#define DEFAULT_BAUD 115200

using namespace std;
using namespace JaguarBase;

int main(int argc, char** argv)
{
  MotionSensorDriver* base = new MotionSensorDriver();
  struct MotionConfig config;
  config.boardType = Jaguar;
  config.commMethod = Serial;
  int rv;
  base->setMotionDriverConfig(&config);
  rv = base->openSerial(DEFAULT_PORT, DEFAULT_BAUD);
  if (rv)
    {
      printf("Error opening serial port %s\n", DEFAULT_PORT);
      return -1;
    }
  struct MotorSensorData motorSensorData;
  int j;

  base->disableMotorCmd(3);
  base->disableMotorCmd(4);
  base->enableMotorCmd(0);
  base->enableMotorCmd(1);
  base->enableMotorCmd(3);
  base->enableMotorCmd(4);

  base->sendMotorCtrlCmd(Position, 0, 100);
  base->sendMotorCtrlCmd(Position, 1, 0);
  //base->sendMotorCtrlCmd(PWM, 3, -10);
  //base->sendMotorCtrlCmd(PWM, 4, -10);
  while (true)
    {
      base->readMotorSensorData(&motorSensorData);
      
      for (uint32_t i = 0 ; i < MOTORSENSOR_NUM; ++i)
	{
	  printf("Motor %d: %d\t", i, motorSensorData.motorSensorEncoderPos[i]);
	}
      printf("\n");
      j++;
    }
  
  base->close();
  
}
