#include "wifi.h"

// Configure the relevant parameters of the AP (hotspot) mode
const char *AP_SSID = "WLROBOT"; 
const char *AP_PSW = "12345678";    

IPAddress AP_IP(192, 168, 1, 11); 
IPAddress AP_GATEWAY(192, 168, 1, 11); 
IPAddress AP_SUBNET(255, 255, 255, 0); 


// Configure the relevant parameters of the STA mode
char *sta_ssid = "MUJITECH";
char *sta_password = "mujitech";



void WiFi_SetAP(void)
{
	WiFi.mode(WIFI_AP); 
	WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET); 
	WiFi.softAP(AP_SSID, AP_PSW); 
	// Serial.println();
	// Serial.print("AP IP address = ");
	// Serial.println(WiFi.softAPIP());
}


// ESP-01S access the WiFi network in AP mode
void set_sta_wifi()
{

  WiFi.begin(sta_ssid, sta_password);
  // Wait for connection
  while(WiFi.status()!=WL_CONNECTED)
  {
    //Serial.print(".");
    delay(500);
  }
  // Print the IP address of ESP-01S
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}
