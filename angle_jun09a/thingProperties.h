#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>


const char THING_ID[] = "655ea764-13a9-435b-a574-57b75f11dfc4";

const char SSID[]     = SECRET_SSID;    // Network SSID (name)
const char PASS[]     = SECRET_PASS;    // Network password (use for WPA, or use as key for WEP)

void onLightChange();

int angle;
bool light;
bool toggle;

void initProperties(){

  ArduinoCloud.setThingId(THING_ID);
  ArduinoCloud.addProperty(angle, READ, ON_CHANGE, NULL, 5.000000);
  ArduinoCloud.addProperty(light, READWRITE, 1 * SECONDS, onLightChange);
  ArduinoCloud.addProperty(toggle, READ, ON_CHANGE, NULL);

}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);
