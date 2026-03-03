#include <Arduino.h> //basic arduino functions (serial, delay, setup/loop)
#include <WiFi.h> // controls ESP32 wifi raduo mode to STA or AP mode. espnow uses this radio
#include <esp_now.h> // espno2 api. it has init, send/recieve callbacks, peer management

// sending telemetry packets from uav to ugv
// STRUCT - DECODE
// packed cause it prevents compiler from inserting padding bytes (alignment gaps)
// this keeps the binary layout identical across devices/compilers

// mac address of UAV
uint8_t UAV_MAC[] = {0xF8, 0xB3, 0xB7, 0x20, 0x25, 0xA8};

enum : uint8_t
{
    message_telemetry = 1,
    message_command = 2
};

typedef struct __attribute__((packed))
{
 uint8_t type; // 1 = telemetry, 2 = command

 uint32_t sequence;  //sequence number increments every packet
 uint32_t timestamp_ms; // sender timestamp in milliseconds
 float velocity_x; //simulated velocity x
 float velocity_y; //simulated velocity y 
 bool markerDetected; //marker detected by uav // flag
 bool emergencyStop; //emergency stop flag
 uint8_t lastCommandAck; //command field for ugv
} telemetryPacket;// *the telemetryPacket in UAV.cpp is what the UAV packs (sends to UGV)


typedef struct __attribute__((packed))
{
  // message _command
  // command sequence
  // command // like command code (0 none,  1 arm,  2 disarm,  3 land , 4 emergency stop)
  // emergency stop // if trrue then UAV goes into failsafe state
  uint8_t type; // message_command
  uint32_t command_sequence; // copmmand ssequence
  uint8_t command; // command code (0 none,  1 arm,  2 disarm,  3 land , 4 emergency stop)
  bool emergencyStop; // emergency stop flag
} commandPacket; // *the commandPacket in UAV.cpp is what the UAV unpacks (recieves from UGV)

//state
static uint32_t global_commandSequence = 0; // for printing and tracing back if any issue (veritifcaiton purpose_)
static uint8_t global_lastUAVTelemetrySequence = 0;
static uint8_t global_lastCommandAck = 0; // echo last command ID we recieved (debugging/tracing back if any issue)

// SEND CALLBACK
static void checkDeliveryStatus (const uint8_t *mac, esp_now_send_status_t status)
{
  Serial.printf("(UGV TX) Sending to UAV -> %s\n", (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "FAIL");
}

//RECIEVE CALLBACK
//interrupt callback
// this callback runs automatically whenver an ESPNOW packet is recieved
static void whenDataIsRecieved(const uint8_t *mac, const uint8_t *incomingData, int len)
{ // this runs when the esp32 gets the data and interrrupt happens and thenduring that itnerrupt of the loop, aka loop being paused, this fnction will run
    
    // first: basic sanity check
    if (len < 1) return; // if the radio glitches and sends a packet of size 0, we ignore it

    // the first byte is the message type
    uint8_t type = incomingData[0];
    
    // 1) validating expected size
    if (type == message_telemetry)
    {
        if (len != (int)sizeof(telemetryPacket))
        {
            Serial.printf("(UGV RX) I recieved Wrong size: %d (expected %d)\n", len, (int)sizeof(telemetryPacket));
            return;
        }

    // 2) copying bytes into a struct instance (safe cause we verified length)
    telemetryPacket newTelemetryPacket;
    memcpy(&newTelemetryPacket, incomingData, sizeof(telemetryPacket));

    global_lastUAVTelemetrySequence = newTelemetryPacket.sequence;
    global_lastCommandAck = newTelemetryPacket.lastCommandAck;

    // 3) print who sent it (MAC) + decoded fields
    Serial.printf("Recieved from MAC: %02X:%02X:%02X:%02X:%02X:%02X | ", 
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    Serial.printf("Seq: %lu | TS: %lu | vx=%.2f vy=%.2f | Marker: %d | EmStop: %d | LastCmdAck: %d\n",
        (unsigned long)newTelemetryPacket.sequence, 
        (unsigned long)newTelemetryPacket.timestamp_ms,
        newTelemetryPacket.velocity_x,
        newTelemetryPacket.velocity_y,
        newTelemetryPacket.markerDetected ? 1 : 0,
        newTelemetryPacket.emergencyStop ? 1 : 0,
        newTelemetryPacket.lastCommandAck);

    return;
    }
    Serial.printf("UGV RX unknown type =%u len=%d\n", type, len);
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    WiFi.mode(WIFI_STA); //setting wifi in station mode //putting wifi radio in station mode which is required for espnow. we are not connecting to any router here. this enables STA interface

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init Failed");
        while(true) delay(1000);
    }

    esp_now_register_send_cb(checkDeliveryStatus); //onsent callback
    esp_now_register_recv_cb(whenDataIsRecieved); //onreceive callback


    // now we must add the reciever as a peer in the espnow before sendding.
    // a peer is basically a known MAC address target that ESPNow can send to. aka SSN of that target ESP32 (UGV)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, UAV_MAC, 6);
    peerInfo.channel = 0; // set to 0 to auto-use whatever channel the AP is on (or 11 if no AP)
    peerInfo.encrypt = false; // set to true if you want to use encryption (requires a password)

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        while(true) delay(1000);
    }

    Serial.println("Reciever (UGV) ready! My MAC is: ");
    Serial.println(WiFi.macAddress());

    Serial.println("UGV Setup Complete.");
    Serial.printf("Expected Telemetry struct: %d bytes\n", (int)sizeof(telemetryPacket));
    Serial.printf("Expected Command struct:   %d bytes\n", (int)sizeof(commandPacket));
    Serial.println("Waiting for data from UAV...");
}

void loop()
{
//every 2 seconds , send a command packet to UAV
//ill toggle emergencystop on/off to prove the bidrirectional control works

static uint32_t lastSendMs = 0;
uint32_t now = millis();

if (now - lastSendMs >=2000)
{
    lastSendMs = now;
    
    commandPacket newCommandPacket;
    newCommandPacket.type = message_command;

    newCommandPacket.command_sequence = global_commandSequence++;

    //demo commands : cycle cmd field 0,1,2,3,...
    newCommandPacket.command = (uint8_t)(newCommandPacket.command_sequence % 4);

    //demo estop toggles evry command
    newCommandPacket.emergencyStop = (newCommandPacket.command_sequence %2) ==1;

    esp_err_t result = esp_now_send(UAV_MAC, (const uint8_t *) &newCommandPacket, sizeof(commandPacket));

    Serial.printf("UGV send CMD result=%d cmdSeq=%lu cmd=%u estop=%d (lastTelemSeq=%lu cmdEcho=%u)\n",
    (int)result, 
    (unsigned long)newCommandPacket.command_sequence, 
    newCommandPacket.command,
    newCommandPacket.emergencyStop ? 1 : 0,
    (unsigned long)global_lastUAVTelemetrySequence,
    (unsigned long)global_lastCommandAck);
}

    delay(5000);
}

