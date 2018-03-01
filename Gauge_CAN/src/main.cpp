#include <Arduino.h>
//#include <Wire.h>
#include <I2C.h>    // Better Wire/I2C master library
#include <SPI.h>
#include <mcp_can.h>
MCP_CAN CAN0(17);   // Set CS to pin 17

#define LED 23      // Led arduino pin

unsigned long ledBlinkMillis = 0;   // led on memory time
const unsigned int ledTimerOn = 1;  // Led on time

unsigned long currentMillis = 0;
unsigned long loopMillisCount = 0;

unsigned int iRpmToGauge = 0;         // Value to send RPM gauge module 0-738 0-6000rpm
unsigned int cVss1 = 0;               // speed as char 0-255

const unsigned long speedAskTimer = 200;  // How often ask speed value
unsigned long lastSpeedAskTime = 0;       // Merory for last read time

const unsigned long rpmSendTimer = 100;   // How often ask speed value
unsigned long lastRpmSendkTime = 0;       // Merory for last read time

const unsigned long IoUpdateTimer = 50;   // How often set IO data
unsigned long lastIoUpdateTime = 0;       // Merory for last IO set time millis


//CAN
const int myCANid = 5; // CAN ID of this unit
const int msCANid = 0; // CAN ID of the Megasquirt (should almost always be 0)

typedef struct {
  uint32_t id;  // can identifier
  uint8_t ext;  // identifier is extended
  uint8_t len;  // length of data
  uint8_t buf[8]; // databytes
} CAN_message_t;

static CAN_message_t txmsg,rxmsg;

unsigned long canlastReceiveMillis = 0;   // CAN error timer memory
unsigned long canlastReceiveTimer = 2000; // CAN no messages error time
bool canErrorFlag = 0;                    // CAN error Flag

unsigned int rpmh = 0;        // RPM 11xx
unsigned int rpml = 0;        // RPM xx11
unsigned int RPM = 0;         // RPM 1111
const unsigned int RpmMax = 6000; // Gauge maximun rpm
const int ScaleMax = 738;         // Gauge max steps 1 degree = 3 steps
const int ScaleMin = 0;           // Gauge min steps 0 rpm
const float rpmZToSteps = 8.13;   // RPM / rpmZToSteps = Steps to gauge


void sendRpm(void)    // Send RPM value to gauge via I2C
{
  if (RPM >= RpmMax)  { RPM = RpmMax; //max value limit
  }
  else if (RPM <= 0) {  RPM = 0;      //min value limit
  }
  iRpmToGauge = RPM / rpmZToSteps;                  // Scale for gauge steps
  I2c.write(1,iRpmToGauge >> 8,iRpmToGauge & 255);  // Send steps at 2 bytes
}

void ledBlink() {       // Led on (led off by timer)
  ledBlinkMillis = currentMillis;
  digitalWrite(LED, 1);
}

// CANIN CANOUT
  bool canin_BIT[8];            // CANIN bits array
  bool canout_BIT[16];          // CANOUT bits array

  unsigned char rCanin1_8;          // Read from broadcast message
  unsigned char rCanout1_8;         // Read from broadcast message
  unsigned char rCanout9_16;        // Read from broadcast message
  unsigned char rExtCanin1_8;       // Write MS ext message
  unsigned char rExtCanout1_8;      // Read from MS ext message
  unsigned char rExtCanout9_16;     // Read from MS ext message

// pack/unpack the Megasquirt extended message format header
typedef struct msg_packed_int {
  unsigned char b0;
  unsigned char b1;
  unsigned char b2;
  unsigned char b3;
} msg_packed_int;

typedef struct msg_bit_info {
  unsigned int spare:2;
  unsigned int block_h:1;
  unsigned int block:4;
  unsigned int to_id:4;
  unsigned int from_id:4;
  unsigned int msg_type:3;
  unsigned int offset:11;
} msg_bit_info;

typedef union {
  unsigned long i;
  msg_packed_int b;
  msg_bit_info values;
} msg_packed;

msg_packed rxmsg_id,txmsg_id;

// unpack the vars from the payload of a MSG_REQ packet
typedef struct msg_req_data_packed_int {
  unsigned char b2;
  unsigned char b1;
  unsigned char b0;
} msg_req_data_packed_int;

typedef struct msq_req_data_bit_info {
  unsigned int varbyt:4;
  unsigned int spare:1;
  unsigned int varoffset:11;
  unsigned int varblk:4;  //5 !!!??
} msg_req_data_bit_info;

typedef union {
  msg_req_data_packed_int bytes;
  msg_req_data_bit_info values;
} msg_req_data_raw;

msg_req_data_raw msg_req_data;


void setup() {
    // Setup
    pinMode(LED, OUTPUT);     // Green led on board
    digitalWrite(LED, HIGH);   // turn the LED on
    //Serial.begin(115200);     // For debuggin
    CAN0.begin(CAN_500KBPS);  // init can bus : baudrate = 500k
    // *********************************************************
    // Basic Filtering on ID
    // *********************************************************
    // Bytes 1 and 2 of mask/filter apply to ID
    // So the Mask and filter below will only allow ID 0x00 through
    /*                                     ---- ID -----
    CAN0.init_Mask(0, 0, 0xFFFF); // 1111 1111 1111 1111
    CAN0.init_Filt(0, 0, 0x0001); // 0000 0000 0000 0001 - FAIL
    CAN0.init_Filt(1, 0, 0x0001); // 0000 0000 0000 0001 - FAIL
    CAN0.init_Mask(1, 0, 0xFFFF); // 1111 1111 1111 1111
    CAN0.init_Filt(2, 0, 0x0001); // 0000 0000 0000 0001 - FAIL
    CAN0.init_Filt(3, 0, 0x0001); // 0000 0000 0000 0001 - FAIL
    CAN0.init_Filt(4, 0, 0x05f0); // 0000 0000 0000 0000 - ACCEPT 1520 Megasquirt base id
    CAN0.init_Filt(5, 0, 0x0001); // 0000 0000 0000 0001 - FAIL
    */

    I2c.begin();              // start I2C bus
    I2c.timeOut(1000);        // Fault recovery timeout
    I2c.pullup(true);         // Set internal pullups
    //  I2c.setSpeed(fast);   // Set I2C speed to 400KHz
    digitalWrite(LED, LOW);    // turn the LED off
}

void loop() {
    // Main
    loopMillisCount = millis() - currentMillis; // How lon take one loop
    currentMillis = millis();                   // Update millis to variable

    // read Speed from gauge via I2C communication
    if(currentMillis - lastSpeedAskTime >= speedAskTimer) // Read command for speedo
    {
      I2c.read(2, 1);         // request 1 bytes from slave device #2 (speedo)
    }

    if(I2c.available())  {            // Is there in bytes from Speedo?
      cVss1 = I2c.receive();          // receive a one byte Speed
      lastSpeedAskTime = currentMillis;    // Reset timer
      //Serial.print("Speed=");
      //Serial.println(cVss1);
    }

    // CAN receive
    if(CAN_MSGAVAIL == CAN0.checkReceive())   // check if incoming Data on CAN
    {
      CAN0.readMsgBufID(&rxmsg.id, &rxmsg.len, rxmsg.buf);
      rxmsg.ext = CAN0.isExtendedFrame();

      canlastReceiveMillis = currentMillis;          // Reset error timer
      canErrorFlag = 0;                              // Reset error Flag
      ledBlink() ;
/*      Serial.print("ID=");
      Serial.print(rxmsg.id);
      Serial.print(" ext=");
      Serial.print(rxmsg.ext);
      Serial.print(" len=");
      Serial.print(rxmsg.len);
      Serial.print(" buf=");
      for(int i = 0; i<7; i++)    // print the data
      {
          Serial.print(rxmsg.buf[i]);
          Serial.print(".");
      }
      Serial.println("");
*/
    // CAN check what we got
    switch (rxmsg.id) {   // Broadcast messages
    case 1520:    //0     // Is CAN rxId for RPM, Copy hi and low hex to int
      rpmh = int(rxmsg.buf[6] << 8);
      rpml = int(rxmsg.buf[7]);
      RPM = rpmh + rpml;
      break;
    case 1572:    //52
      rCanin1_8 = rxmsg.buf[0];   // canin1_8;
      rCanout1_8 = rxmsg.buf[1];  // canout1_8;
      rCanout9_16 = rxmsg.buf[2]; // canout9_16;
      // There is more bytes but we dont need those (rxmsg.buf[3-7])
      break;
    default:
      if(rxmsg.ext)  {    // EXT frame messasge (MS protocol)
        rxmsg_id.i = rxmsg.id;
        if (rxmsg_id.values.to_id == myCANid) { //Is this to me?
          if(rxmsg_id.values.msg_type == 1) {   //MSG_REQ - request data
            // the data required for the MSG_RSP header is packed into the first 3 data bytes
            msg_req_data.bytes.b0 = rxmsg.buf[0];
            msg_req_data.bytes.b1 = rxmsg.buf[1];
            msg_req_data.bytes.b2 = rxmsg.buf[2];
            // Create the tx packet header
            txmsg_id.values.msg_type = 2;     // MSG_RSP
            txmsg_id.values.to_id = msCANid;  // Megasquirt CAN ID should normally be 0
            txmsg_id.values.from_id = myCANid;  // My CAN ID
            txmsg_id.values.block = msg_req_data.values.varblk;     // where we send reply (block)
            txmsg_id.values.offset = msg_req_data.values.varoffset; // where we send reply (offset)
            txmsg.len = msg_req_data.values.varbyt;   // how manyt bytes we send (This was 8! fault)
            txmsg.ext = 1;                            //we send extended frame
            txmsg.id = txmsg_id.i;

            if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 336) { // VSS1
              /* 42 0 336 2 - VSS1 Vehicle Speed 1 ms-1 1 10 0 -
                 VSS1             = scalar, U16,  336, "", 1,10 0-4095
                 VSS2             = scalar, U16,  338, "", 1,10 0-4095
                 VSS3             = scalar, U16,  340, "", 1,10 0-4095
                 VSS4             = scalar, U16,  342, "", 1,10 0-4095
              */
                txmsg.buf[0] = cVss1 / 256;
                txmsg.buf[1] = cVss1 % 256;
                txmsg.buf[2] = 0;
                txmsg.buf[3] = 0;
                txmsg.buf[4] = 0;
                txmsg.buf[5] = 0;
                txmsg.buf[6] = 0;
                txmsg.buf[7] = 0;

                CAN0.sendMsgBuf(txmsg.id, txmsg.ext, txmsg.len, txmsg.buf);
            }   // END if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 336)
            else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 77) { // CANIN
              /* CANIN1-8
                  Input bits
              */
                txmsg.buf[0] = rExtCanin1_8;
                txmsg.buf[1] = 0;
                txmsg.buf[2] = 0;
                txmsg.buf[3] = 0;
                txmsg.buf[4] = 0;
                txmsg.buf[5] = 0;
                txmsg.buf[6] = 0;
                txmsg.buf[7] = 0;

                CAN0.sendMsgBuf(txmsg.id, txmsg.ext, txmsg.len, txmsg.buf);
            }   // END else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 77) { // CANIN
          }   // END if(rxmsg_id.values.msg_type == 1) MSG_REQ - request data

          else if(rxmsg_id.values.msg_type == 0) {   //MSG_CMD - message to deposit data into memory.
            if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 75) { // CANOUT
              // CANOUT1-8
              rExtCanout1_8 = rxmsg.buf[0];
              if (rxmsg.len > 1)  {         // CANOUT9-16
                rExtCanout9_16 = rxmsg.buf[1];
              }
              else  {
                rExtCanout9_16 = 0;
              }


            } // END CANOUT
          }  // END else if(rxmsg_id.values.msg_type == 0) {   //MSG_CMD
        }   // END if (rxmsg_id.values.to_id == myCANid) {
      }   // END if(rxmsg.ext)
      break;
    }  // END switch (rxmsg.id)
  }   // END CAN

    // Send RPM to gauge, timed
  if(currentMillis - lastRpmSendkTime >= rpmSendTimer)
  {
    sendRpm();
  }

    // LED off timing
    if (currentMillis - ledBlinkMillis >= ledTimerOn && digitalRead(LED)) {
      digitalWrite(LED, 0);
      ledBlinkMillis = currentMillis;
    }


    // If no CAn communication set error flag and values to zero
    if(currentMillis - canlastReceiveMillis >= canlastReceiveTimer)
    {
      canErrorFlag = 1;
      RPM = 0;
      rExtCanout1_8 = 0;
      rExtCanout9_16 = 0;
      rCanout1_8  = 0;
      rCanout9_16 = 0;
    }

    // IO update
    if(currentMillis - lastIoUpdateTime >= lastIoUpdateTime)
    {
    // Set CANIO data
    // canin1_8
      canin_BIT[2] = 1;   //Test
      canin_BIT[4] = 1;   //Test
      bitWrite(rExtCanin1_8, 0, canin_BIT[0]);  // IN 1
      bitWrite(rExtCanin1_8, 1, canin_BIT[1]);  // IN 2
      bitWrite(rExtCanin1_8, 2, canin_BIT[2]);  // IN 3
      bitWrite(rExtCanin1_8, 3, canin_BIT[3]);  // IN 4
      bitWrite(rExtCanin1_8, 4, canin_BIT[4]);  // IN 5
      bitWrite(rExtCanin1_8, 5, canin_BIT[5]);  // IN 6
      bitWrite(rExtCanin1_8, 6, canin_BIT[6]);  // IN 7
      bitWrite(rExtCanin1_8, 7, canin_BIT[7]);  // IN 8

      // CANOUT
      canout_BIT[0] = bitRead(rExtCanout1_8, 0); // OUT 0
      canout_BIT[1] = bitRead(rExtCanout1_8, 1); // OUT 1
      canout_BIT[2] = bitRead(rExtCanout1_8, 2); // OUT 2
      canout_BIT[3] = bitRead(rExtCanout1_8, 3); // OUT 3
      canout_BIT[4] = bitRead(rExtCanout1_8, 4); // OUT 4
      canout_BIT[5] = bitRead(rExtCanout1_8, 5); // OUT 5
      canout_BIT[6] = bitRead(rExtCanout1_8, 6); // OUT 6
      canout_BIT[7] = bitRead(rExtCanout1_8, 7); // OUT 7
      //rCanout9_16 not in use
      //digitalWrite(LED, canout_BIT[0]);       // test with led

    } // END IO update



}   // End Loop
