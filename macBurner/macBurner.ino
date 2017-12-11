
//================ EEPROM Memory Map ================= 
// Address Byte       Data(8 bits)        Type                
//      0              ---- ----       MAC byte 0       
//      1              ---- ----       MAC byte 1       
//      2              ---- ----       MAC byte 2
//      3              ---- ----       MAC byte 3
//      4              ---- ----       MAC byte 4
//      5              ---- ----       MAC byte 5
//      6              ---- ----        Node ID 
//      7              ---- ----       unassigned
//      8              ---- ----       unassigned
//      9              ---- ----       unassigned
//      10             ---- ----       unassigned
//      .              ---- ----       unassigned
//      ..             ---- ----       unassigned
//      ...            ---- ----       unassigned
//      1024           ---- ----       unassigned



#include <Adafruit_SleepyDog.h>
#include <EEPROM.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#define RESET 4



IPAddress serverIp(10, 1, 1, 1); // Server ip address
EthernetUDP UdpRcv; // UDP object
EthernetUDP UdpSer; // UDP object to print serial debug info

byte mac[] = {0x00, 0x08, 0xDC, 0x00, 0x00, 0x6A}; //Assign MAC address of the Arduino here
unsigned int rcvPort = 8888; // Assign a port to talk over
unsigned int serPort = 8890;  // Assign port to print debug statements

// Initializing buffer and data variables for receiving packets from the server
int packetSize;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
String command; // String for data


unsigned int nodeID = 0;
unsigned int eeadr = 0; 
unsigned int eeNodeAdr = 6; // EEPROM addres that will store node ID number


void serialUdp(String);
void parseUdpPacket();

void setup() {

  Watchdog.enable(8000);
  Serial.begin(57600);
  Serial.println("Running macBurner sketch"); 

  // Setting pins appropriately. Very important to first deactivate the digital pins
  // because setting the pin as OUTPUT changes it's state and has caused problems with the reset pin 4 before

  // RESET pin; active LOW
  digitalWrite(RESET, HIGH);
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);


  // Burn MAC to first 6 EEPROM bytes
  for (int i = 0; i < 6; i++){
    EEPROM.write(eeadr, mac[i]);
    ++eeadr;
    }
    
  // burn node ID to the 7th EEPROM byte  
  EEPROM.write(eeNodeAdr, nodeID);

  // Zero all the other EEPROM cells - 255 is the default in each cell with an off the shelf Arduino
  for (int i = 7; i < 1024; i++){
    EEPROM.write(i,0);
  }  
 
  // Print out the contents of EEPROM
  Serial.println("EEPROM read prior to Ethernet.begin(mac)");
  for (int i = 0; i < 8; i++) {
    Serial.println("Data: ");
    Serial.println(String(EEPROM.read(i)));
  } 


  // Start Ethernet connection, automatically tries to get IP using DHCP
  if (Ethernet.begin(mac) == 0) {
    Serial.print("Failed to configure Ethernet using DHCP");
  }
 
  // Start UDP
  UdpSer.begin(serPort);
  UdpRcv.begin(rcvPort);
  delay(1500); // delay to give time for initialization

  // Print out the contents of EEPROM
  for (int i = 0; i < 8; i++) {
    serialUdp("Printing the contents of EEPROM");
    serialUdp("Address: ");
    serialUdp(String(i));
    Serial.print("Data: ");
    Serial.print(String(EEPROM.read(i)));
  } 

  serialUdp("IP address:");
  serialUdp(String(Ethernet.localIP()));

  
}




void loop() {
    serialUdp("Waiting to receive the reset command..");
   // Check if request was sent to Arduino
    packetSize = UdpRcv.parsePacket(); // Reads the packet size

    if (packetSize>0) { //if packetSize is >0, that means someone has sent a request
       parseUdpPacket();
    }

    //clear out the packetBuffer array
    memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);

    // Renew DHCP lease - times out eventually if this is removed
    Ethernet.maintain();
 

}



void serialUdp(String message){
  UdpSer.beginPacket(serverIp, serPort);
  UdpSer.print(message);
  UdpSer.endPacket();
  memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
  }



void parseUdpPacket(){
  UdpRcv.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); //Read the data request
  String command(packetBuffer); //Convert char array packetBuffer into a string called command

  if (packetSize>0) { //if packetSize is >0, that means someone has sent a request
  
      UdpRcv.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE); //Read the data request
      String command(packetBuffer); //Convert char array packetBuffer into a string called command
    
      if (command == "reset") {
        
          serialUdp("Resetting the microcontroller...");
          digitalWrite(RESET, LOW);
      }   
      
    }


}
