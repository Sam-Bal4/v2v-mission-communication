#include <Arduino.h> // pulls in all the basic serial and time crap so the board actually works
#include <WiFi.h> // i had to add this just so the radio hardware even wakes up
#include <esp_now.h> // the actual api for screaming raw bytes into the air

#include "freertos/FreeRTOS.h" // the foundation for all the multitasking stuff i did
#include "freertos/task.h" // lets me hire background workers to do the chores
#include "freertos/queue.h" // mailboxes so the workers dont crash the chip when handing off data

// mac address of the ground rover so we know exactly where to scream the data
static uint8_t UGV_MAC[6] = {0xF8, 0xB3, 0xB7, 0x20, 0x69, 0xC0}; // the actual mac address i hardcoded so it doesnt get lost

// binary markers i added so we can actually find real messages in all the usb noise
static const uint8_t SOF = 0xAA; // the wait for it byte that tells us a packet is starting
static const uint8_t TYPE_TELEM = 1; // code for when the robot is sending its status
static const uint8_t TYPE_CMD   = 2; // code for when i am bullying it with instructions
static const uint8_t TYPE_MSG   = 3; // code for just sending raw text junk

// i made this match the exact memory layout of the python script
typedef struct __attribute__((packed)) { // i had to pack this so the compiler doesnt add stupid empty spaces
  uint32_t seq;    // counter to see if the radio dropped any packets
  uint32_t t_ms;   // drone time clock
  float vx;        // speed data so we know how fast the wheels are going
  float vy;        // more speed data cause why not
  uint8_t marker;  // random true or false detection bits
  uint8_t estop;   // this is where i sneak in the safety flags and abort stuff
} TelemetryPayload; // finishing up the status struct

// layout for the commands we shove down to the rover
typedef struct __attribute__((packed)) { // squishing the memory again
  uint32_t cmdSeq; // unique id so the robot knows which move is which
  uint8_t  cmd;    // the actual code like 1 to arm or 2 to disarm
  uint8_t  estop;  // the please stop before you crash button
} CommandPayload; // finishing the command struct

// mailboxes i had to set up so the tasks can pass bytes around safely
static QueueHandle_t qTelemToNow  = nullptr; // from the usb wire straight to the radio
static QueueHandle_t qMsgToNow    = nullptr; // shoving text from usb into the air
static QueueHandle_t qCmdToNow    = nullptr; // commands waiting to go to the ground
static QueueHandle_t qTelemToSerial = nullptr; // catching radio stuff and shoving it down usb
static QueueHandle_t qCmdToSerial = nullptr; // instructions caught from the air
static QueueHandle_t qMsgToSerial = nullptr; // raw text caught from the air

// lock i made to stop the workers from typing on the usb wire at the exact same time
static SemaphoreHandle_t serialMutex = nullptr; // keeping it null to start

// boring math i did to check if the data got scrambled during the radio hop
static uint8_t checksum_xor(uint8_t type, uint8_t len, const uint8_t* payload) { // passing in the data
  uint8_t c = type ^ len; // mix the header bytes together first
  for (uint8_t i = 0; i < len; i++) c ^= payload[i]; // do xor magic on every single byte of the meat
  return c; // what we get back is the digital signature
} // end of the math function

// wraps the raw data in our custom frame so the jetson brain can actually read it
static void serial_send_frame(uint8_t type, const uint8_t* payload, uint8_t len) { // taking in the raw chunk
  uint8_t chk = checksum_xor(type, len, payload); // calculate the seal so we know it is legit
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) { // wait until no one else is using the wire
    Serial.write(SOF);      // blast the start byte
    Serial.write(type);     // tell it what kind of data this is
    Serial.write(len);      // tell it how big the chunk is
    if (len) Serial.write(payload, len); // shove the actual binary meat down the wire if there is any
    Serial.write(chk);      // add the fingerprint at the very end
    xSemaphoreGive(serialMutex); // unlock so the other workers can use the wire again
  } // end of the lock zone
} // end of the sending frame function

// this triggers literally every time a packet smacks into the antenna
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) { // grabbing the mac and raw bytes
  if (len < 1) return; // ignore it if its just ghost static
  uint8_t fType = data[0]; // the very first byte is always the type label

  // echo the radio event back to the jetson so i can debug it
  char dbg[64]; // temp holding area for the string
  snprintf(dbg, 64, "ESP: Got Radio PKT type %d len %d", fType, len); // formatting the debug text
  if (qMsgToSerial) { // check if the mailbox is even there
    uint8_t dBuf[64]; // tiny data array
    memset(dBuf, 0, 64); // clean out whatever garbage was in memory
    memcpy(dBuf, dbg, strlen(dbg)); // copy the actual text over
    xQueueSend(qMsgToSerial, dBuf, 0); // drop it in the box for the usb worker
  } // end of debug push

  // sorting the radio data into the right mailbox so it doesnt get lost
  if (fType == TYPE_TELEM && len >= (int)sizeof(TelemetryPayload)) { // checking if it is a valid status packet
    TelemetryPayload t; // make a blank struct
    memcpy(&t, data + 1, sizeof(t)); // rip the content out of the raw bytes
    if (qTelemToSerial) xQueueSend(qTelemToSerial, &t, 0); // hand it off to the usb guy
  } // end of telem check
  else if (fType == TYPE_CMD && len >= (int)sizeof(CommandPayload)) { // checking if it is a command
    CommandPayload cmd; // make a blank command struct
    memcpy(&cmd, data + 1, sizeof(cmd)); // extract the juicy bits
    if (qCmdToSerial) xQueueSend(qCmdToSerial, &cmd, 0); // pass it down the line
  } // end of cmd check
  else if (fType == TYPE_MSG) { // if it is just a text message
    uint8_t msgBuf[64]; // buffer for random text
    memset(msgBuf, 0, 64); // wipe it clean first
    uint8_t msgLen = (len - 1 > 64) ? 64 : len - 1; // i had to add this to clip it if the text is way too long
    memcpy(msgBuf, data + 1, msgLen); // grab the string
    if (qMsgToSerial) xQueueSend(qMsgToSerial, msgBuf, 0); // shove it in the mail
  } // end of msg check
} // end of radio receive trigger

// the states i made for the gatekeeper to unpack the usb stream
enum ParseState { WAIT_SOF, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHK }; // all the modes we can be stuck in
typedef struct { // making a struct to hold the gatekeeper memory
  ParseState st; // what mode we are currently stuck in
  uint8_t type; // what kind of packet we think this is
  uint8_t len; // how long it should be
  uint8_t idx; // how many bytes we actually caught so far
  uint8_t buf[64]; // where we shove the bytes while waiting
} SerialParser; // naming the struct

// completely resets the parser to go back to hunting for that start byte
static void parser_init(SerialParser* p) { p->st = WAIT_SOF; } // just forcing the state back to zero

// i feed it one byte at a time and see if it passes the vibe check
static bool parser_step(SerialParser* p, uint8_t b, uint8_t* outType, uint8_t* outLen, uint8_t* outPayload) { // checking the grammar rules
  switch (p->st) { // checking what state we are stuck in
    case WAIT_SOF: if (b == SOF) p->st = WAIT_TYPE; break; // yay we finally found the start byte
    case WAIT_TYPE: p->type = b; p->st = WAIT_LEN; break; // caught the type label
    case WAIT_LEN: // time to figure out the length
      p->len = b; // saving how big it told us it was
      if (p->len > 64) { parser_init(p); break; } // if its stupidly big just throw it in the trash and restart
      p->idx = 0; // zero out the counter
      p->st = (p->len == 0) ? WAIT_CHK : WAIT_PAYLOAD; // jump to the next step
      break; // done with length
    case WAIT_PAYLOAD: // scooping up the meat
      p->buf[p->idx++] = b; // hoarding the data one byte at a time
      if (p->idx >= p->len) p->st = WAIT_CHK; // cool we got all of it
      break; // done with payload
    case WAIT_CHK: { // time for the math check
      uint8_t expected = checksum_xor(p->type, p->len, p->buf); // doing the math to check the fingerprint
      if (b == expected) { // if the math actually checks out
        *outType = p->type; *outLen = p->len; // spit out the info
        memcpy(outPayload, p->buf, p->len); // spit out the meat
        parser_init(p); return true; // reset and yell that we got a good packet
      } // end of good math
      parser_init(p); break; // the seal was bad so some static messed it up just restart
    } // end of check block
  } // end of switch
  return false; // nope still looking for more bytes
} // end of parser step

// the background worker i hired just to listen to the jetson 24 7
void serialRxTask(void* pv) { // passing the void pointer cause freertos wants it
  SerialParser parser; // creating our local gatekeeper
  parser_init(&parser); // making sure he starts fresh
  uint8_t fType, fLen, payload[64]; // temporary junk variables
  for (;;) { // loop forever cause workers dont get breaks
    while (Serial.available() > 0) { // looking if there are any bytes sitting in the usb port
      uint8_t b = (uint8_t)Serial.read(); // yank one byte out
      if (parser_step(&parser, b, &fType, &fLen, payload)) { // run it through the gatekeeper logic i wrote
        if (fType == TYPE_TELEM && fLen == sizeof(TelemetryPayload)) { // if its status data
          TelemetryPayload t; memcpy(&t, payload, sizeof(t)); // rip the status out
          if (qTelemToNow) xQueueSend(qTelemToNow, &t, 0); // shove it to the radio guy
        } // end of telem push
        else if (fType == TYPE_CMD && fLen == sizeof(CommandPayload)) { // if its a command
          CommandPayload c; memcpy(&c, payload, sizeof(c)); // rip the command out
          if (qCmdToNow) xQueueSend(qCmdToNow, &c, 0); // pass it to the radio task
        } // end of cmd push
        else if (fType == TYPE_MSG) { // if its just text
            uint8_t msgBuf[64]; memset(msgBuf, 0, 64); // blank out a buffer
            memcpy(msgBuf, payload, fLen); // copy the text
            if (qMsgToNow) xQueueSend(qMsgToNow, msgBuf, 0); // throw it in the box
        } // end of msg push
      } // end of valid packet check
    } // end of reading usb buffer
    vTaskDelay(pdMS_TO_TICKS(2)); // force it to chill for 2ms so the chip doesnt melt
  } // end of infinite loop
} // end of serial worker

// the heavy hitter worker that screams status updates into the air
void radioTxTelemTask(void* pv) { // standard task definition
  TelemetryPayload t; // make some storage
  uint8_t pkt[1 + sizeof(TelemetryPayload)]; // make the full packet buffer
  pkt[0] = TYPE_TELEM; // slap the type label on the front
  for (;;) { // loop forever obviously
    if (xQueueReceive(qTelemToNow, &t, portMAX_DELAY) == pdTRUE) { // literally just sleep until a letter shows up in the mailbox
      memcpy(pkt + 1, &t, sizeof(t)); // stuff the envelope
      esp_now_send(UGV_MAC, pkt, sizeof(pkt)); // blast it as hard as we can to the rover mac address
    } // end of mailbox check
  } // end of infinite loop
} // end of telem worker

// worker that strictly handles bullying the rover with commands
void radioTxCmdTask(void* pv) { // standard task definition
  CommandPayload c; // storage again
  uint8_t pkt[1 + sizeof(CommandPayload)]; // packet buffer again
  pkt[0] = TYPE_CMD; // slap the command label on
  for (;;) { // never ending loop
    if (xQueueReceive(qCmdToNow, &c, portMAX_DELAY) == pdTRUE) { // wait for someone to drop a command in
      memcpy(pkt + 1, &c, sizeof(c)); // pack the data in
      esp_now_send(UGV_MAC, pkt, sizeof(pkt)); // throw it over the radio
    } // end of mailbox check
  } // end of infinite loop
} // end of cmd worker

// worker for sending random debug text over the air
void radioTxMsgTask(void* pv) { // standard task definition
  uint8_t payload[64]; // holding area for raw bytes
  uint8_t pkt[1 + 64]; // max size packet
  pkt[0] = TYPE_MSG; // tag it as a message
  for (;;) { // endless loop
    if (xQueueReceive(qMsgToNow, payload, portMAX_DELAY) == pdTRUE) { // wake up when text arrives
      uint8_t len = 0; while (len < 64 && payload[len] != 0) len++; // i had to do this to manually find where the string actually ends
      if (len == 0) continue; // if its empty just ignore it cause why bother
      memcpy(pkt + 1, payload, len); // copy the actual letters
      esp_now_send(UGV_MAC, pkt, 1 + len); // scream the text to the other side
    } // end of mailbox check
  } // end of infinite loop
} // end of msg worker

// this is the guy who catches radio stuff and shoves it up the usb to the brain
void serialTxTask(void* pv) { // standard task definition
  CommandPayload c; TelemetryPayload t; uint8_t m[64]; // places to put the crap we catch
  for (;;) { // working forever
    if (xQueueReceive(qCmdToSerial, &c, 0) == pdTRUE) { // check if any commands came in from the air
      serial_send_frame(TYPE_CMD, (const uint8_t*)&c, (uint8_t)sizeof(c)); // package it and shove it up the usb
    } // end of cmd check
    if (xQueueReceive(qTelemToSerial, &t, 0) == pdTRUE) { // check if the rover sent a status update
      serial_send_frame(TYPE_TELEM, (const uint8_t*)&t, (uint8_t)sizeof(t)); // cram it into the usb port
    } // end of telem check
    if (xQueueReceive(qMsgToSerial, m, 0) == pdTRUE) { // check for random text
      uint8_t len = 0; while (len < 64 && m[len] != 0) len++; // count how long the text is
      if (len > 0) serial_send_frame(TYPE_MSG, m, len); // push the text to the computer
    } // end of msg check
    vTaskDelay(pdMS_TO_TICKS(5)); // i let this task sleep for 5ms so it doesnt hog the whole cpu
  } // end of infinite loop
} // end of serial tx worker

// the very first setup stuff i run when the power turns on
void setup() { // starting up
  Serial.begin(115200); // crank up the serial speed to talk to the jetson
  delay(500); // wait half a second cause the hardware is slow to wake up

  WiFi.mode(WIFI_STA); // force the wifi chip into station mode cause esp now needs it
  if (esp_now_init() != ESP_OK) { // kick the radio driver to wake up
    Serial.println("Error initializing ESP-NOW"); // complain if the radio is broken
    while(true) delay(1000); // just give up and freeze forever
  } // end of radio check

  // i had to pass my listener function to the radio so it knows what to do on receive
  esp_now_register_recv_cb(onDataRecv); // hooking up my listener function

  // explicitly telling the radio who it is allowed to even look at
  esp_now_peer_info_t peer = {}; // blank peer object
  memcpy(peer.peer_addr, UGV_MAC, 6); // copying the rover mac address in
  peer.channel = 0; peer.encrypt = false; // zero channel and no encryption cause encryption is too slow
  esp_now_add_peer(&peer); // officially register it

  // i made the lock here so workers dont fight over the usb
  serialMutex = xSemaphoreCreateMutex(); // creating the actual lock

  // physically spawning all the mailboxes into memory
  qTelemToNow  = xQueueCreate(10, sizeof(TelemetryPayload)); // box that holds 10 statuses
  qCmdToNow    = xQueueCreate(10, sizeof(CommandPayload)); // box that holds 10 commands
  qMsgToNow    = xQueueCreate(10, 64); // box for strings
  qTelemToSerial = xQueueCreate(10, sizeof(TelemetryPayload)); // catching status from the air
  qCmdToSerial = xQueueCreate(10, sizeof(CommandPayload)); // catching commands from the air
  qMsgToSerial = xQueueCreate(10, 64); // catching text from the air

  // finally hiring the workers and throwing them into their own background threads
  xTaskCreate(serialRxTask,      "SerRx",   4096, nullptr, 2, nullptr); // the guy watching the usb
  xTaskCreate(radioTxTelemTask,  "RadTelem",4096, nullptr, 2, nullptr); // the guy shipping status over radio
  xTaskCreate(radioTxCmdTask,    "RadCmd",  4096, nullptr, 2, nullptr); // the guy shipping commands
  xTaskCreate(radioTxMsgTask,    "RadMsg",  4096, nullptr, 2, nullptr); // the guy shipping raw text
  xTaskCreate(serialTxTask,      "SerTx",   4096, nullptr, 1, nullptr); // the guy shoving everything back to the brain

  Serial.println("===================================="); // just a pretty header i added
  Serial.print("UAV Bridge Ready! MAC: "); // yelling that we are ready to go
  Serial.println(WiFi.macAddress()); // printing my own mac address so i can debug easier
  Serial.println("===================================="); // nice little footer
} // end of setup

// main loop is literally completely empty cause the background workers handle all the chores
void loop() { // useless main loop
  vTaskDelay(portMAX_DELAY); // i just tell the main chip to sleep forever and let the workers sweat
} // end of file