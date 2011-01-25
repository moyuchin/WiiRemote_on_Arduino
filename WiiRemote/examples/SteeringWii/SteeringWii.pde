
#include "WiiRemote.h"

#define STEERING_ANGLE_MAX    165
#define STEERING_ANGLE_MIN    15
#define STEERING_ANGLE_CENTER 90
#define STEERING_ANGLE_STEP   5

WiiRemote wiiremote;

void setup()
{
  Serial.begin(9600);
  wiiremote.init();

  /*
  unsigned char wiiremote_bdaddr[6] = {0x00, 0x1e, 0x35, 0xda, 0x48, 0xbc};
  wiiremote.setBDADDR(wiiremote_bdaddr, 6);
  wiiremote.setBDADDRAcquisitionMode(BD_ADDR_FIXED);
  */
}

void loop()
{
  wiiremote.task(&myapp);
}


void myapp(void) {
  if (wiiremote.buttonClicked(WIIREMOTE_TWO)) {
    int steering_angle = getSteeringAngle();
  }
}

int getSteeringAngle(void) {
  double rad;
  double deg;

  rad = acos((double) wiiremote.Report.Accel.Y);
  deg = rad * 180.0 / PI;

  /* clipping */
  if (deg > STEERING_ANGLE_MAX) { deg = STEERING_ANGLE_MAX; }
  if (deg < STEERING_ANGLE_MIN) { deg = STEERING_ANGLE_MIN; }

  Serial.print("\r\nSteering angle = ");
  Serial.print((int) deg);

  return (int) deg;
}


// vim: sts=2 sw=2 ts=2 et cin
