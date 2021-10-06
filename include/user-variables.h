// *******************************************************************************************************************************
// START userdefined data
// *******************************************************************************************************************************



// Give the sensor a plant name, change to true, upload sketch and then revert to false

String sensor_name = "Sensor_1";




// Device configuration and name setting
const String device_name = "LoRa_device0017"; // Can be changed, but not necessary, as it will give no added value.

#define uS_TO_S_FACTOR 1000000ULL //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  60                //Time ESP32 will go to sleep (in seconds)








//iotcrV2 CONEXION
const char broker[] = "3.142.89.107";
int        port     = 1883;
String dId = "2317";
String webhook_pass = "SYGkrATJ69";
String webhook_endpoint = "http://3.142.89.107:3001/api/getdevicecredentials";

//const char mqttuser[] = "user"; //add eventual mqtt username
//const char mqttpass[] = "pass"; //add eventual mqtt password






// *******************************************************************************************************************************
// END userdefined data
// *******************************************************************************************************************************
