#pragma once

//#include "motor_control.h"

// https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol
// https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/CrsfProtocol/crsf_protocol.h
#define CRSF_ADDRESS_CRSF_TRANSMITTER (0xEE)
#define CRSF_ADDRESS_RADIO_TRANSMITTER (0xEA)
#define CRSF_ADDRESS_FLIGHT_CONTROLLER (0xC8)
#define CRSF_ADDRESS_CRSF_RECEIVER (0xEC)



typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,

    //CRSF_FRAMETYPE_ELRS_STATUS = 0x2E, ELRS good/bad packet count and status flags

    CRSF_FRAMETYPE_COMMAND = 0x32,
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ  = 0x78,
    CRSF_FRAMETYPE_KISS_RESP = 0x79,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
} crsf_frame_type_e;


#define CRSF_MAX_PACKET_SIZE (64)
#define CRSF_MAX_PAYLOAD_SIZE (60)


typedef struct crsf_packet_finder_s
{
    uint8_t packet[CRSF_MAX_PACKET_SIZE];
    uint8_t currentSize;
    uint8_t needed_size;
} crsf_packet_finder_t;


typedef struct crsf_packet_s
{
    uint8_t dest;
    uint8_t len;
    uint8_t type;
    uint8_t payload[CRSF_MAX_PAYLOAD_SIZE];
    uint8_t payload_len;
    uint8_t crc;
    uint8_t packet_len;
} crsf_packet_t;


typedef struct crsf_channels_s
{
    uint16_t ch0 : 11;
    uint16_t ch1 : 11;
    uint16_t ch2 : 11;
    uint16_t ch3 : 11;
    uint16_t ch4 : 11;
    uint16_t ch5 : 11;
    uint16_t ch6 : 11;
    uint16_t ch7 : 11;
    uint16_t ch8 : 11;
    uint16_t ch9 : 11;
    uint16_t ch10 : 11;
    uint16_t ch11 : 11;
    uint16_t ch12 : 11;
    uint16_t ch13 : 11;
    uint16_t ch14 : 11;
    uint16_t ch15 : 11;
} __attribute__((packed)) crsf_channels_t;



volatile crsf_channels_t channel_commands;


void process_crsf_packet(uint8_t rx_buffer[], uint8_t len);
void process_crsf_channels_message(crsf_packet_t* p);
void process_crsf_ping_message(crsf_packet_t* p);
void process_crsf_link_statistics_message(crsf_packet_t* p);
void send_crsf_message(uint8_t dest, uint8_t type, uint8_t* payload, uint8_t payload_len /*, uint8_t* dest_buf*/);



void search_for_packet(crsf_packet_finder_t *p, uint8_t new_rx_byte) {
  // if this is possibly the first packet byte, make sure the dest is vaid.
  if(p->currentSize == 0){
    switch(new_rx_byte){
      case CRSF_ADDRESS_CRSF_TRANSMITTER:
      case CRSF_ADDRESS_RADIO_TRANSMITTER:
      case CRSF_ADDRESS_FLIGHT_CONTROLLER:
      case CRSF_ADDRESS_CRSF_RECEIVER:
        // Happy path. 
        p->packet[0] = new_rx_byte;
        p->currentSize = 1;
        return;
        break;
      default:
        // unknown dest, ignore.
        return;
    }
  }

  // if this is the second byte of the packet, make sure the size is allowed.
  if(p->currentSize == 1){
    if(new_rx_byte > CRSF_MAX_PAYLOAD_SIZE){
      // The size is too large
      p->currentSize = 0;
      return;
    }
    p->needed_size = new_rx_byte + 2;
  }

  p->packet[p->currentSize] = new_rx_byte;
  p->currentSize++;

  // now we have recieved the entire message, parse it. then reset the packet_finder struct
  if(p->currentSize >= 4 && p->currentSize == p->needed_size){
    process_crsf_packet(p->packet, p->currentSize);
    p->currentSize = 0;
  }

  // Check if there was an error and the current size has maxed out the buffer.
  if(p->currentSize >= CRSF_MAX_PACKET_SIZE){
    p->currentSize = 0;
  }
}


void print_crsf_packet(crsf_packet_t* p){
  Serial.print("dest: 0x");
  Serial.println(p->dest, 16);
  
  Serial.print("len: 0x");
  Serial.println(p->len, 16);

  Serial.print("type: 0x");
  Serial.println(p->type, 16);

  Serial.print("crc: 0x");
  Serial.println(p->crc, 16);

  Serial.print("payload len: ");
  Serial.println(p->payload_len);
  Serial.print("payload:");
  for(uint8_t i = 0; i < p->payload_len; i++){
    Serial.print(" 0x");
    Serial.print(p->payload[i], 16);
  }
  Serial.print("\n\n");
}


/*
Process the packet. Make sure the dest, size, and crc are correct.
Forward the packet to a message processor if the type is recognized. 
*/
void process_crsf_packet(uint8_t rx_buffer[], uint8_t len){
  if( len < 4) {
    //Serial.print("Message is too short\n");
    return;
  }
  crsf_packet_t p;
  p.dest = rx_buffer[0];

  switch(p.dest){
    case CRSF_ADDRESS_CRSF_TRANSMITTER:
    case CRSF_ADDRESS_RADIO_TRANSMITTER:
    case CRSF_ADDRESS_FLIGHT_CONTROLLER:
    case CRSF_ADDRESS_CRSF_RECEIVER:
      // Happy path. do nothing.
      break;
    default:
      //Serial.print("Unknown dest \n");
      return;
  }
  
  p.len = rx_buffer[1];
  if(p.len < 2) {
    //Serial.print("given len is too small. \n");
    return;
  }
  p.packet_len = p.len + 2;
  p.payload_len = p.len - 2;

  p.type = rx_buffer[2];

  if(p.packet_len > len) {
    //Serial.print("message length is wrong. \n");
    return;
  }

  p.crc = rx_buffer[1+p.len];

  uint8_t computed_crc = calc_elrs_crc8(&rx_buffer[2], len - 3);
  if(p.crc != computed_crc) {
    //Serial.print("crc doesn't match computed crc: ");
    //Serial.print(p.crc, 16);
    //Serial.print(" ");
    //Serial.println(computed_crc, 16);
    return;
  }

  if(p.payload_len) {
    memcpy(p.payload, &rx_buffer[3], p.payload_len);
  }

  //print_crsf_packet(&p);

  if(p.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED){
    process_crsf_channels_message(&p);
  }else if(p.type == CRSF_FRAMETYPE_DEVICE_PING){
    process_crsf_ping_message(&p);
  }else if(p.type == CRSF_FRAMETYPE_LINK_STATISTICS){
    process_crsf_link_statistics_message(&p);
  }else{
    //Serial.print("Packet type not expected: 0x");
    //Serial.println(p.type, 16);
    //print_crsf_packet(&p);
  }
  
}



/*
Process the channels message
*/
void process_crsf_channels_message(crsf_packet_t* p){
  if(p->type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED || p->payload_len != 22){
    return;
  }


  //crsf_channels_t ch;
  memcpy(&channel_commands, p->payload, p->payload_len);

  /*
  Serial.println(channel_commands.ch0);
  Serial.println(channel_commands.ch1);
  Serial.println(channel_commands.ch2);
  Serial.println(channel_commands.ch3);
  Serial.println(ch.ch4);
  Serial.println(ch.ch5);
  Serial.println(ch.ch6);
  Serial.println(ch.ch7);
  */
  //Serial.println(ch.ch8);
  //Serial.println(ch.ch9);
  //Serial.println(ch.ch10);
  //Serial.println(ch.ch11);
  //Serial.println(ch.ch12);
  //Serial.println(ch.ch13);
  //Serial.println(ch.ch14);
  //Serial.println(ch.ch15);
}





/*
Process the ping message
*/
void process_crsf_ping_message(crsf_packet_t* p){
  if(p->type != CRSF_FRAMETYPE_DEVICE_PING || p->payload_len != 4){
    return;
  }

  if(p->dest != CRSF_ADDRESS_FLIGHT_CONTROLLER){
    return;
  }

  uint8_t ping_requester = p->payload[1];
  uint8_t resp_buf[] = {/*extended packet dest*/ping_requester, \
  /*extended packet src*/CRSF_ADDRESS_FLIGHT_CONTROLLER, \
  /*display name*/'a', 'v', 'r', 0, \
  /*Serial number*/'E', 'L', 'R', 'S', \
  /*Hardware version*/0, 0, 0, 0, \
  /*Software version*/0, 0, 0, 0, \
  /*Number of config params*/0, \
  /*param protocol version*/0x00};
  
  send_crsf_message(ping_requester, CRSF_FRAMETYPE_DEVICE_INFO, resp_buf, sizeof(resp_buf));
}

/*
Process the ping message
*/
void process_crsf_link_statistics_message(crsf_packet_t* p){
  if(p->type != CRSF_FRAMETYPE_LINK_STATISTICS){
    return;
  }
  // Do nothing.
}




/*
Send a valid CRSF packet. 
*/
void send_crsf_message(uint8_t dest, uint8_t type, uint8_t* payload, uint8_t payload_len /*, uint8_t* dest_buf*/){
  if(payload_len > CRSF_MAX_PAYLOAD_SIZE){
    return;
  }

  uint8_t buf[CRSF_MAX_PACKET_SIZE];
  buf[0] = dest;
  buf[1] = payload_len + 2;
  buf[2] = type;
  memcpy(&buf[3], payload, payload_len);
  uint8_t crc = calc_elrs_crc8(&buf[2], payload_len + 1);
  buf[3 + payload_len] = crc;

  Serial.write(buf, payload_len + 4);
}



