#include <XInput.h>

#include "crc.h"
#include "crsf.h"

  const int JOY_MAX_VALUE = 32767;  // int16_t max
  const int TRIGGER_MAX_VALUE = 255;  // uint8_t max

void setup() {
  gen_elrs_crc8();


  Serial1.begin(115200);
  //while (!Serial1) {
  //  ;  // wait for serial port to connect. Needed for native USB port only
  //}
  pinMode(LED_BUILTIN, OUTPUT);

  channel_commands.ch0 = 992;
  channel_commands.ch1 = 992;
  channel_commands.ch2 = 992;
  channel_commands.ch3 = 992;

  XInput.setAutoSend(false);  // Wait for all controls before sending
	XInput.begin();

	XInput.setTriggerRange(0, TRIGGER_MAX_VALUE);
	XInput.setJoystickRange(-JOY_MAX_VALUE, JOY_MAX_VALUE);
}


float elrs_stick_value_to_float(int16_t in) {
  float x = in - 992;
  x = x / 992.0f;
  return x;
}

void send_xbox_controller_values(crsf_channels_t *cmds ) {
  float pitch = elrs_stick_value_to_float(cmds->ch1);
  float roll = elrs_stick_value_to_float(cmds->ch0);
  float throttle = elrs_stick_value_to_float(cmds->ch2);
  float yaw = elrs_stick_value_to_float(cmds->ch3);

  int32_t left_stick_x = int32_t(1.3*roll * JOY_MAX_VALUE);
  int32_t left_stick_y = int32_t(1.3*pitch * JOY_MAX_VALUE);
  int32_t right_stick_x = 0;
  int32_t right_stick_y = 0;
  XInput.setJoystick(JOY_LEFT, left_stick_x, left_stick_y);
  XInput.setJoystick(JOY_RIGHT, right_stick_x, right_stick_y);

  int32_t left_trigger_val = 0;
  int32_t right_trigger_val = 0;
  if(yaw >= .1){
    right_trigger_val = (int32_t)(1.3*yaw * TRIGGER_MAX_VALUE);
  }else if(yaw <= -.1){
    left_trigger_val = (int32_t)(-1.3*yaw * TRIGGER_MAX_VALUE);
  }
  XInput.setTrigger(TRIGGER_LEFT, left_trigger_val);
  XInput.setTrigger(TRIGGER_RIGHT, right_trigger_val);

  // throttle
   if(throttle > .33f){
    XInput.press(BUTTON_RB);
    XInput.release(BUTTON_LB);
  }else if(throttle < -.33f){
    XInput.release(BUTTON_RB);
    XInput.press(BUTTON_LB);
  }else{
    XInput.release(BUTTON_RB);
    XInput.release(BUTTON_LB);
  }


  float switch_a = elrs_stick_value_to_float(cmds->ch4);
  float switch_b = elrs_stick_value_to_float(cmds->ch5);
  float switch_c = elrs_stick_value_to_float(cmds->ch6);
  float switch_d = elrs_stick_value_to_float(cmds->ch7);

  // Flare
  if(switch_a > 0){
    XInput.press(BUTTON_L3);
    XInput.press(BUTTON_R3);
  }else{
    XInput.release(BUTTON_L3);
    XInput.release(BUTTON_R3);
  }

  // Fire Guns or missiles
   if(switch_b > .33f){
    XInput.press(BUTTON_A);
    XInput.release(BUTTON_B);
  }else if(switch_b < -.33f){
    XInput.release(BUTTON_A);
    XInput.press(BUTTON_B);
  }else{
    XInput.release(BUTTON_A);
    XInput.release(BUTTON_B);
  }


  // Change misiles
   if(switch_c > .5f){
    XInput.press(BUTTON_X);
  }else{
    XInput.release(BUTTON_X);
  }

    // Change target
  if(switch_d > .5f){
    XInput.press(BUTTON_Y);
  }else{
    XInput.release(BUTTON_Y);
  }

  // Send values to PC
  XInput.send();
}


void loop() {
  crsf_packet_finder_t packet_finder;
  memset(&packet_finder, 0, sizeof packet_finder);

  
  bool pressA = false;

  uint32_t last_update_time = millis();
  while(1) {
    // if we get a valid byte,
    while (Serial1.available() > 0) {
      uint8_t in = Serial1.read();
      search_for_packet(&packet_finder, in);
      digitalWrite(LED_BUILTIN, 1);
    }
    digitalWrite(LED_BUILTIN, 0);
    uint32_t new_time = millis();

    if( new_time > last_update_time + 10){
      last_update_time = new_time;

      crsf_channels_t cmds;
      memcpy(&cmds, &channel_commands, sizeof cmds);
      send_xbox_controller_values(&cmds);
    }
    
  }
}


