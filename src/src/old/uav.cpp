#include <Arduino.h> // serial, delay, setup/loop
#include <WiFi.h> // wifi radio mode control like STA or AP mode
#include <esp_now.h> // espnow api. init, send/recieve callbacks, peer management

// mac address of esp32_1 = F8:B3:B7:20:25:A8
// mac address of esp32_2 = F8:B3:B7:20:69:C0

// UAV sends telemetry to UGV and recieves commands from UGV
// UGV receives telemetry from UAV and sends commands to UAV



/////////////////////////////////config stuff
// giving the uav the mac adress of UGV as raw bytes
// so the ESP-NOW targets devices by MAC address (no Ip, no router, no wifi network)
// so we need to manually add the peer (UGV) to the ESP-NOW network
// this is done using esp_now_add_peer()
uint8_t UGV_MAC[] = {0xF8, 0xB3, 0xB7, 0x20, 0x69, 0xC0}; //mac address of UGV (reciver/commander)


//////////////////// protocol stuff
/* i had to add protocol cause , when i was sending raw bytes over usb, sometimes a byte got lost or corrupted
 a protocol in this case is like a set of grammar rules. if the message starts with a specific "start" byte and ends with a 
 "checksum" we can prove that the data is 100% correct before using it.
*/
//          protcol constants
//sof (start of frame). m using 0xAA (10101010) in binary
// to help the recieveer align itself at the start of a new message
static const uint8_t SOF = 0xAA;
static const uint8_t TYPE_TELEM = 1; // UAV to UGV (sensors/stats)
static const uint8_t TYPE_CMD = 2; // UGV to UAV (instructions)


/*
replaced these enums with protocol header . and used type_telem and type_CMD
/*
// adding the message type now
// as esp-now is a peer to peer communication protocol, both devices must add each other as a peer
// each device runs: a send loop (perodicoliclyy)
// each device runs: a recieve loop (perodicically)

//two different struct packet types: telemetry and command
// telemetry packet is sent from uav to ugv
// command packet is sent from ugv to uav

// for this i am using basic protocol concept: message type + fixed c struct packet

so i will define a 1 byte message type at the front of every packet 
like 
message_telemetry = 1
message_command = 2

that way, reciever can do
- if type == 1 then decode telemetry packet
- if type == 2 then decode command packet


// so here is the message types part
enum : uint8_t
{
  message_telemetry = 1,
  message_command = 2
};
*/

//////////////////////////// STRUCTS

// making the c struct packet definition for the telemetry data! this MUST always be same as the one i have in UGV to work!
// cause its like the struct packet blueprint is sent, binary layout must match exactly. 
// so the compiler doesnt add any padding bytes between fields for alignment purposes

/*
now every packet will start with "type" so we can tell what struct it is (uint8_t = 8 bits = 1 byte)
then the rest of the packet will be the struct packet
*/
typedef struct __attribute__((packed))
{
  uint32_t seq; //sequence number increments every packet
  uint32_t t_ms; //sender timestamp in milliseconds
  float vx; //simulated velocity x
  float vy; //simulated velocity y
  uint8_t marker; //marker detected by uav // flag
  uint8_t estop; //emergency stop flag
  //uint8_t lastCommandAck; //echo last command ID we recieved (debugging/tracing back if any issue)
/*
UGV: "UAV, this is Base. Command #5: Change altitude to 10 meters. Over."
UAV: "Base, this is UAV. Copy Command #5. I am now at 10 meters. Over." (This is the Echo). The UGV hears the Echo and thinks: "Good, they definitely heard me."
*/
} TelemetryPayload; // *the telemetryPacket in UAV.cpp is what the UAV packs (sends to UGV)

typedef struct __attribute__((packed))
{
  // message _command
  // command sequence
  // command // like command code (0 none,  1 arm,  2 disarm,  3 land , 4 emergency stop)
  // emergency stop // if trrue then UAV goes into failsafe state
  uint32_t cmdSeq; // copmmand ssequence
  uint8_t cmd; // command code (0 none,  1 arm,  2 disarm,  3 land , 4 emergency stop)
  uint8_t estop; // emergency stop flag
} CommandPayload; // *the commandPacket in UAV.cpp is what the UAV unpacks (recieves from UGV)

///// the structs are done

// adding a helper struct
// i had to add this casue , needed a way to seal them so i know they werent tampered with
// m using xor checksum . 
// if the math combines every byte during transmission, the reciever's calculation
// will fail and it will drop the packet
static uint8_t checksum_xor(uint8_t type, uint8_t len, const uint8_t* payload)
{
  uint8_t c = type ^ len; // start with type and length
  for (uint8_t i = 0; i < len; i++) c^=payload[i]; // xor the content
  return c;
}

// wrapping data  "seal the envelope" for serial transmission
// this function wraps the data in the protocol frame
//// this will look like this/
// [SOF] [TYPE] [LEN] [PAYLOAD] [CHK]
// lmao whoever is debugging my code good luck! discord me on : catari_express
static void serial_send_frame (uint8_t type, const uint8_t* payload, uint8_t len)
{
  unit8_t chk = checksum_xor (type, len, payload); // generating the signature
  Serial.write(SOF); //marker
  Serial.write(type); // what type is this?
  Serial.write(len); // how long is it?

  if (len) Serial.write(payload, len); // the actual data // payload bytes
  Serial.write(chk); // the finger print // sneding the checksum signature
}





// all the states
static uint32_t global_sequence = 0;

// land drone when global emergency stop is true
static volatile bool global_emergencyStop = false; // this will be set to true when the UGV sends an emergency stop command // updated when commands arrive

//Remembers the actual instruction (e.g., "Land")
static volatile uint8_t global_lastCommandAck = 0; // last command ID value we recieved (debugging/tracing back if any issue)

// Remembers which specific message that instruction came from.
static uint32_t global_lastCommandSequence = 0; // for printing and tracing back if any issue (veritifcaiton purpose_)

/**/
// dont need this anymore cause i was basic testin the esp-now test

//callback/interrrupt function
// this callback runs after a send attempt finishes
// it tells us whether the radio reported success or failure
// SEND CALLBACK
/*
static void checkDeliveryStatus (const uint8_t *mac, esp_now_send_status_t status)
{
  // mac = the mac address of the device we tried to send to
  // status = ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL
  /*
  Serial.printf("Sending to MAC: %02X:%02X:%02X:%02X:%02X:%02X | ", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (status == ESP_NOW_SEND_SUCCESS) 
  Serial.println("Success");
  else Serial.println("Fail");

  Serial.printf("(UAV TX) Sending to UGV -> %s\n", (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "FAIL");
}
*/

// RECIEVE CALLBACK
static void whenDataIsRecieved(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // first: basic sanity check
  if (len < 1) return; // if the radio glitches and sends a packet of size 0, we ignore it

  // the first byte is the message type
  uint8_t type = incomingData[0];

  // second: copy bytes into a struct instance (safe cause we verified length)
  if (type == message_command)
  {
    if (len != (int)sizeof(commandPacket))
    {
      Serial.printf("UAV RX Command wrong size: %d (expected %d)\n", len, (int)sizeof(commandPacket));
      return;
    }

    commandPacket newCmdPacket;
    memcpy(&newCmdPacket, incomingData, sizeof(newCmdPacket));

    // update state based on command
    global_emergencyStop = newCmdPacket.emergencyStop;
    global_lastCommandAck = newCmdPacket.command;
    global_lastCommandSequence = newCmdPacket.command_sequence;

    // printing what it got
    Serial.printf("(UAV RX) Recieved Command from UGV -> MAC: %02X:%02X:%02X:%02X:%02X:%02X | cmdSeq: %u | Cmd: %d | EmStop: %d\n",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    (unsigned long)newCmdPacket.command_sequence,
    newCmdPacket.command,
    newCmdPacket.emergencyStop ? 1 : 0);
    return;
  }
  // for now if it gets any other type, ignores it for now
  Serial.printf("UAV Recieved unknown type = %u len = %d\n", type, len);
}

void setup()
{
  Serial.begin(115200);  //start the serial monitor
  delay(500); //give time for serial to stablize

  WiFi.mode(WIFI_STA); //setting wifi in station mode //putting wifi radio in station mode which is required for espnow. we are not connecting to any router here. this enables STA interface

  // initlazing the ESPNOW on the uav esp after the wifi is set to station mode
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    while(true) delay(1000);
  }

  //now register the callback function so we can see if each send is successful or not
  esp_now_register_send_cb(checkDeliveryStatus); //onsent callback

  esp_now_register_recv_cb(whenDataIsRecieved); //onreceive callback

  // now we must add the reciever as a peer in the espnow before sendding.
  // a peer is basically a known MAC address target that ESPNow can send to. aka SSN of that target ESP32 (UGV)
  esp_now_peer_info_t peerInfo = {}; // Initialize with default values
  
  memcpy(peerInfo.peer_addr, UGV_MAC, 6); // copy the reciever UGV MAC address into the peer struct
  peerInfo.channel = 0; // setting 0 means telling it to use current wifi channel
                        // since we arenot connect to Wifi , channel 0 usually works
  peerInfo.encrypt = false; // disabling encryption for now for simplicity
  

  // now finally adding the peer into the ESP-Now peer list
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.print("UAV ready. My MAC is: ");
  Serial.println(WiFi.macAddress());

  Serial.printf("TelemetryPacket size: %d bytes\n", (int)sizeof(telemetryPacket));
  Serial.printf("CommandPacket size: %d bytes\n", (int)sizeof(commandPacket));
}

void loop()
{
  telemetryPacket newTelemetryPacket;
  
  //after creating the form named packet, we fill it with data
  newTelemetryPacket.type = message_telemetry;
  
  newTelemetryPacket.sequence = global_sequence++; // incrementing the sequence number
  newTelemetryPacket.timestamp_ms = millis(); // getting the current time in milliseconds (sender timestamp)
  
  //test velocity that changes over time
  newTelemetryPacket.velocity_x = 1.0f + 0.1f * (newTelemetryPacket.sequence % 20); // setting velocity x to 1.0 + 0.1 * (sequence % 20)
  newTelemetryPacket.velocity_y = 1.0f + 0.1f * (newTelemetryPacket.sequence % 20); // setting velocity y to 1.0 + 0.1 * (sequence % 20

  newTelemetryPacket.markerDetected = ((newTelemetryPacket.sequence /10) % 2) == 1; // setting marker detected to true or false
  
  newTelemetryPacket.emergencyStop = global_emergencyStop; // setting emergency stop to true or false
  newTelemetryPacket.lastCommandAck = global_lastCommandAck;

  // sending the telemetry packet to the UGV
  esp_err_t result = esp_now_send(UGV_MAC, (const uint8_t *)&newTelemetryPacket, sizeof(newTelemetryPacket));

  Serial.printf("UAV send telemetry result=%d seq=%lu (lastCmdSeq=%lu)\n",
    (int)result, (unsigned long)newTelemetryPacket.sequence, (unsigned long)global_lastCommandSequence);
    
    delay (5000); // sending every 1 second
}


