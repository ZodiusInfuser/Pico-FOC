#include "Commander.h"
#include <cstdlib>
#include <stdio.h>
#include <cstring>

//Commander::Commander(Stream& serial, char eol, bool echo){
//  com_port = &serial;
//  this->eol = eol;
//  this->echo = echo;
//}
Commander::Commander(char eol, bool echo){
  this->eol = eol;
  this->echo = echo;
}


void Commander::add(char id, CommandCallback onCommand, const char* label){
  call_list[call_count] = onCommand;
  call_ids[call_count] = id;
  call_label[call_count] = (char*)label;
  call_count++;
}


void Commander::run(){
  //if(!com_port) return;
  run('\n');
}

void Commander::run(char eol){
  //Stream* tmp = com_port; // save the serial instance
  char eol_tmp = this->eol;
  this->eol = eol;
  //com_port = &serial;
  int ch;
  while((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
    received_chars[rec_cnt++] = (char)ch;
    // end of user input
    if(echo)
      print((char)ch);
    if (is_sentinel(ch)) {
      // execute the user command
      run(received_chars);

      // reset the command buffer
      received_chars[0] = 0;
      rec_cnt=0;
    }
    if (rec_cnt>=MAX_COMMAND_LENGTH) { // prevent buffer overrun if message is too long
        received_chars[0] = 0;
        rec_cnt=0;
    }
  }
// process character

/*
  // a string to hold incoming data
  while (serial.available()) {
    // get the new uint8_t:
    int ch = serial.read();
    received_chars[rec_cnt++] = (char)ch;
    // end of user input
    if(echo)
      print((char)ch);
    if (is_sentinel(ch)) {
      // execute the user command
      run(received_chars);

      // reset the command buffer
      received_chars[0] = 0;
      rec_cnt=0;
    }
    if (rec_cnt>=MAX_COMMAND_LENGTH) { // prevent buffer overrun if message is too long
        received_chars[0] = 0;
        rec_cnt=0;
    }
  }

  com_port = tmp; // reset the instance to the internal value
  this->eol = eol_tmp;
*/
}

void Commander::run(char* user_input){
  // execute the user command
  char id = user_input[0];
  switch(id){
    case CMD_SCAN:
      for(int i=0; i < call_count; i++){
          print_machine_readable(CMD_SCAN);
          print(call_ids[i]);
          print(":");
          if(call_label[i]) println(call_label[i]);
          else println("");
      }
      break;
    case CMD_VERBOSE:
      if(!is_sentinel(user_input[1])) verbose = (VerboseMode)atoi(&user_input[1]);
      print_verbose("Verb:\n");
      print_machine_readable(CMD_VERBOSE);
      switch (verbose){
      case VerboseMode::nothing:
        printf("off!\n");
        break;
      case VerboseMode::on_request:
      case VerboseMode::user_friendly:
        printf("on!\n");
        break;
      case VerboseMode::machine_readable:
        println_machine_readable("machine\n");
        break;
      }
      break;
    case CMD_DECIMAL:
      if(!is_sentinel(user_input[1])) decimal_places = atoi(&user_input[1]);
      print_verbose("Decimal:\n");
      print_machine_readable(CMD_DECIMAL);
      println(decimal_places);
      break;
    default:
      for(int i=0; i < call_count; i++){
        if(id == call_ids[i]){
          print_machine_readable(user_input[0]);
          call_list[i](&user_input[1]);
          break;
        }
      }
      break;
  }
}

bool isDigit(char c) {
  return (c >= '0' && c <= '9');
}


void Commander::motor(FOCMotor* motor, char* user_command) {

  // if target setting
  if(isDigit(user_command[0]) || user_command[0] == '-' || user_command[0] == '+' || is_sentinel(user_command[0])){
    target(motor, user_command);
    return;
  }

  // parse command letter
  char cmd = user_command[0];
  char sub_cmd = user_command[1];
  // check if there is a subcommand or not
  int value_index = (sub_cmd >= 'A'  && sub_cmd <= 'Z') ||  (sub_cmd == '#') ?  2 :  1;
  // check if get command
  bool GET = is_sentinel(user_command[value_index]);
  // parse command values
  float value = atof(&user_command[value_index]);
  print_machine_readable(cmd);
  if (sub_cmd >= 'A'  && sub_cmd <= 'Z') {
    print_machine_readable(sub_cmd);
  }

  // a bit of optimisation of variable memory for Arduino UNO (atmega328)
  switch(cmd){
    case CMD_C_Q_PID:      //
      print_verbose("PID curr q| \n");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_q, &user_command[1]);
      else pid(&motor->PID_current_q,&user_command[1]);
      break;
    case CMD_C_D_PID:      //
      print_verbose("PID curr d| \n");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_current_d, &user_command[1]);
      else pid(&motor->PID_current_d, &user_command[1]);
      break;
    case CMD_V_PID:      //
      print_verbose("PID vel| \n");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_velocity, &user_command[1]);
      else pid(&motor->PID_velocity, &user_command[1]);
      break;
    case CMD_A_PID:      //
      print_verbose("PID angle| \n");
      if(sub_cmd == SCMD_LPF_TF) lpf(&motor->LPF_angle, &user_command[1]);
      else pid(&motor->P_angle, &user_command[1]);
      break;
    case CMD_LIMITS:      //
     print_verbose("Limits| \n");
      switch (sub_cmd){
        case SCMD_LIM_VOLT:      // voltage limit change
          print_verbose("volt: \n");
          if(!GET) {
            motor->voltage_limit = value;
            motor->PID_current_d.limit = value;
            motor->PID_current_q.limit = value;
            // change velocity pid limit if in voltage mode and no phase resistance set
            if( !_isset(motor->phase_resistance) && motor->torque_controller==TorqueControlType::voltage) motor->PID_velocity.limit = value;
          }
          println(motor->voltage_limit);
          break;
        case SCMD_LIM_CURR:      // current limit
          print_verbose("curr: \n");
          if(!GET){
            motor->current_limit = value;
            // if phase resistance specified or the current control is on set the current limit to the velocity PID
            if(_isset(motor->phase_resistance) || motor->torque_controller != TorqueControlType::voltage ) motor->PID_velocity.limit = value;
          }
          println(motor->current_limit);
          break;
        case SCMD_LIM_VEL:      // velocity limit
          print_verbose("vel: \n");
          if(!GET){
            motor->velocity_limit = value;
            motor->P_angle.limit = value;
          }
          println(motor->velocity_limit);
          break;
        default:
          print_error();
          break;
      }
      break;
    case CMD_MOTION_TYPE:
    case CMD_TORQUE_TYPE:
    case CMD_STATUS:
      motion(motor, &user_command[0]);
      break;
    case CMD_PWMMOD:
       // PWM modulation change
       print_verbose("PWM Mod | \n");
       switch (sub_cmd){
        case SCMD_PWMMOD_TYPE:      // zero offset
          print_verbose("type: \n");
          if(!GET) motor->foc_modulation = (FOCModulationType)value;
          switch(motor->foc_modulation){
            case FOCModulationType::SinePWM:
              printf("SinePWM\n");
              break;
            case FOCModulationType::SpaceVectorPWM:
              printf("SVPWM\n");
              break;
            case FOCModulationType::Trapezoid_120:
              printf("Trap 120\n");
              break;
            case FOCModulationType::Trapezoid_150:
              printf("Trap 150\n");
              break;
          }
          break;
        case SCMD_PWMMOD_CENTER:      // centered modulation
          print_verbose("center: \n");
          if(!GET) motor->modulation_centered = value;
          printf("%d", motor->modulation_centered);
          break;
        default:
          print_error();
          break;
       }
      break;
    case CMD_RESIST:
      print_verbose("R phase: \n");
      if(!GET){
        motor->phase_resistance = value;
        if(motor->torque_controller==TorqueControlType::voltage)
          motor->PID_velocity.limit= motor->current_limit;
      }
      if(_isset(motor->phase_resistance)) println(motor->phase_resistance);
      else printf("\n");
      break;
    case CMD_INDUCTANCE:
      print_verbose("L phase: \n");
      if(!GET){
        motor->phase_inductance = value;
      }
      if(_isset(motor->phase_inductance)) println(motor->phase_inductance);
      else println(0);
      break;
    case CMD_KV_RATING:
      print_verbose("Motor KV: \n");
      if(!GET){
        motor->KV_rating = value;
      }
      if(_isset(motor->KV_rating)) println(motor->KV_rating);
      else println(0);
      break;
    case CMD_SENSOR:
      // Sensor zero offset
       print_verbose("Sensor | \n");
       switch (sub_cmd){
        case SCMD_SENS_MECH_OFFSET:      // zero offset
          print_verbose("offset: \n");
          if(!GET) motor->sensor_offset = value;
          println(motor->sensor_offset);
          break;
        case SCMD_SENS_ELEC_OFFSET:      // electrical zero offset - not suggested to touch
          print_verbose("el. offset: \n");
          if(!GET) motor->zero_electric_angle = value;
          println(motor->zero_electric_angle);
          break;
        default:
          print_error();
          break;
       }
      break;
    case CMD_MONITOR:     // get current values of the state variables
      print_verbose("Monitor | \n");
      switch (sub_cmd){
        case SCMD_GET:      // get command
          switch((uint8_t)value){
            case 0: // get target
              print_verbose("target: \n");
              println(motor->target);
              break;
            case 1: // get voltage q
              print_verbose("Vq: \n");
              println(motor->voltage.q);
              break;
            case 2: // get voltage d
              print_verbose("Vd: \n");
              println(motor->voltage.d);
              break;
            case 3: // get current q
              print_verbose("Cq: \n");
              println(motor->current.q);
              break;
            case 4: // get current d
              print_verbose("Cd: \n");
              println(motor->current.d);
              break;
            case 5: // get velocity
              print_verbose("vel: \n");
              println(motor->_shaft_velocity);
              break;
            case 6: // get angle
              print_verbose("angle: \n");
              println(motor->_shaft_angle);
              break;
            case 7: // get all states
              print_verbose("all: \n");
              print(motor->target);
              print(";");
              print(motor->voltage.q);
              print(";");
              print(motor->voltage.d);
              print(";");
              print(motor->current.q);
              print(";");
              print(motor->current.d);
              print(";");
              print(motor->_shaft_velocity);
              print(";");
              println(motor->_shaft_angle);
              break;
            default:
              print_error();
              break;
          }
          break;
        case SCMD_DOWNSAMPLE:
          print_verbose("downsample: \n");
          if(!GET) motor->monitor_downsample = value;
          println((int)motor->monitor_downsample);
          break;
        case SCMD_CLEAR:
          motor->monitor_variables = (uint8_t) 0;
          println("clear\n");
          break;
        case CMD_DECIMAL:
          print_verbose("decimal: \n");
          motor->monitor_decimals = value;
          println((int)motor->monitor_decimals);
          break;
        case SCMD_SET:
          if(!GET){
            // set the variables
            motor->monitor_variables = (uint8_t) 0;
            for(int i = 0; i < 7; i++){
              if(is_sentinel(user_command[value_index+i])) break;
              motor->monitor_variables |=  (user_command[value_index+i] - '0') << (6-i);
            }
          }
          // print the variables
          for(int i = 0; i < 7; i++){
            print( (motor->monitor_variables & (1 << (6-i))) >> (6-i));
          }
          printf("\n");
          break;
        default:
          print_error();
          break;
       }
      break;
    default:  // unknown cmd
      print_verbose("unknown cmd \n");
      print_error();
  }
}

void Commander::motion(FOCMotor* motor, char* user_cmd, char* separator){
  char cmd = user_cmd[0];
  char sub_cmd = user_cmd[1];
  bool GET  = is_sentinel(user_cmd[1]);
  float value = atof(&user_cmd[(sub_cmd >= 'A'  && sub_cmd <= 'Z') ?  2 :  1]);

  switch(cmd){
    case CMD_MOTION_TYPE:
      print_verbose("Motion:\n");
      switch(sub_cmd){
        case SCMD_DOWNSAMPLE:
            print_verbose(" downsample: \n");
            if(!GET) motor->motion_downsample = value;
            println((int)motor->motion_downsample);
          break;
        default:
          // change control type
          if(!GET && value >= 0 && (int)value < 5) // if set command
            motor->controller = (MotionControlType)value;
          switch(motor->controller){
            case MotionControlType::torque:
              println("torque\n");
              break;
            case MotionControlType::velocity:
              println("vel\n");
              break;
            case MotionControlType::angle:
              println("angle\n");
              break;
            case MotionControlType::velocity_openloop:
              println("vel open\n");
              break;
            case MotionControlType::angle_openloop:
              println("angle open\n");
              break;
          }
            break;
        }
      break;
    case CMD_TORQUE_TYPE:
      // change control type
      print_verbose("Torque: \n");
      if(!GET && (int8_t)value >= 0 && (int8_t)value < 3)// if set command
        motor->torque_controller = (TorqueControlType)value;
      switch(motor->torque_controller){
        case TorqueControlType::voltage:
          println("volt\n");
          // change the velocity control limits if necessary
          if( !_isset(motor->phase_resistance) ) motor->PID_velocity.limit = motor->voltage_limit;
          break;
        case TorqueControlType::dc_current:
          println("dc curr\n");
          // change the velocity control limits if necessary
          motor->PID_velocity.limit = motor->current_limit;
          break;
        case TorqueControlType::foc_current:
          println("foc curr\n");
          // change the velocity control limits if necessary
          motor->PID_velocity.limit = motor->current_limit;
          break;
      }
      break;
    case CMD_STATUS:
      // enable/disable
      print_verbose("Status: \n");
      if(!GET) (bool)value ? motor->enable() : motor->disable();
       println(motor->enabled);
      break;
    default:
      target(motor,  user_cmd, separator);
      break;
  }
}

void Commander::pid(PIDController* pid, char* user_cmd){
  char cmd = user_cmd[0];
  bool GET  = is_sentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_PID_P:      // P gain change
      print_verbose("P: ");
      if(!GET) pid->P = value;
      println(pid->P);
      break;
    case SCMD_PID_I:      // I gain change
      print_verbose("I: ");
      if(!GET) pid->I = value;
      println(pid->I);
      break;
    case SCMD_PID_D:      // D gain change
      print_verbose("D: ");
      if(!GET) pid->D = value;
      println(pid->D);
      break;
    case SCMD_PID_RAMP:      //  ramp change
      print_verbose("ramp: ");
      if(!GET) pid->output_ramp = value;
      println(pid->output_ramp);
      break;
    case SCMD_PID_LIM:      //  limit change
      print_verbose("limit: ");
      if(!GET) pid->limit = value;
      println(pid->limit);
      break;
    default:
      print_error();
      break;
  }
}

void Commander::lpf(LowPassFilter* lpf, char* user_cmd){
  char cmd = user_cmd[0];
  bool GET  = is_sentinel(user_cmd[1]);
  float value = atof(&user_cmd[1]);

  switch (cmd){
    case SCMD_LPF_TF:      // Tf value change
      print_verbose("Tf: \n");
      if(!GET) lpf->tf = value;
      println(lpf->tf);
      break;
    default:
      print_error();
      break;
  }
}

void Commander::scalar(float* value,  char* user_cmd){
  bool GET  = is_sentinel(user_cmd[0]);
  if(!GET) *value = atof(user_cmd);
  println(*value);
}


void Commander::target(FOCMotor* motor,  char* user_cmd, char* separator){
  // if no values sent
  if(is_sentinel(user_cmd[0])) {
    println_machine_readable(motor->target);
    return;
  };

  float pos, vel, torque;
  char* next_value;
  switch(motor->controller){
    case MotionControlType::torque: // setting torque target
      torque = atof(strtok (user_cmd, separator));
      motor->target = torque;
      break;
    case MotionControlType::velocity: // setting velocity target + torque limit
      // set the target
      vel= atof(strtok (user_cmd, separator));
      motor->target = vel;

      // allow for setting only the target velocity without chaning the torque limit
      next_value = strtok (NULL, separator);
      if (next_value){
        torque = atof(next_value);
        motor->PID_velocity.limit = torque;
        // torque command can be voltage or current
        if(!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage) motor->voltage_limit = torque;
        else  motor->current_limit = torque;
      }
      break;
    case MotionControlType::angle: // setting angle target + torque, velocity limit
      // setting the target position
      pos= atof(strtok (user_cmd, separator));
      motor->target = pos;

      // allow for setting only the target position without chaning the velocity/torque limits
      next_value = strtok (NULL, separator);
      if( next_value ){
        vel = atof(next_value);
        motor->velocity_limit = vel;
        motor->P_angle.limit = vel;

        // allow for setting only the target position and velocity limit without the torque limit
        next_value = strtok (NULL, separator);
        if( next_value ){
          torque= atof(next_value);
          motor->PID_velocity.limit = torque;
          // torque command can be voltage or current
          if(!_isset(motor->phase_resistance) && motor->torque_controller == TorqueControlType::voltage) motor->voltage_limit = torque;
          else  motor->current_limit = torque;
        }
      }
      break;
    case MotionControlType::velocity_openloop: // setting velocity target + torque limit
      // set the target
      vel= atof(strtok (user_cmd, separator));
      motor->target = vel;
      // allow for setting only the target velocity without chaning the torque limit
      next_value = strtok (NULL, separator);
      if (next_value ){
        torque = atof(next_value);
        // torque command can be voltage or current
        if(!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
        else  motor->current_limit = torque;
      }
      break;
    case MotionControlType::angle_openloop: // setting angle target + torque, velocity limit
      // set the target
      pos= atof(strtok (user_cmd, separator));
      motor->target = pos;

      // allow for setting only the target position without chaning the velocity/torque limits
      next_value = strtok (NULL, separator);
      if( next_value ){
        vel = atof(next_value);
        motor->velocity_limit = vel;
        // allow for setting only the target velocity without chaning the torque limit
        next_value = strtok (NULL, separator);
        if (next_value ){
          torque = atof(next_value);
          // torque command can be voltage or current
          if(!_isset(motor->phase_resistance)) motor->voltage_limit = torque;
          else  motor->current_limit = torque;
        }
      }
      break;
  }
  print_verbose("Target: \n");
  println(motor->target);
}


bool Commander::is_sentinel(char ch)
{
  if(ch == eol)
    return true;
  else if (ch == '\r')
  {
      print_verbose("Warn: \\r detected! \n\n");
  }
  return false;
}

void Commander::print(const int number){
  //if( !com_port || verbose == VerboseMode::nothing ) return;
  //com_port->print(number);
}
void Commander::print(const float number){
  //if(!com_port || verbose == VerboseMode::nothing ) return;
  //com_port->print((float)number,(int)decimal_places);
}
void Commander::print(const char* message){
  //if(!com_port || verbose == VerboseMode::nothing ) return;
  //com_port->print(message);
}
void Commander::print(const char message){
  //if(!com_port || verbose == VerboseMode::nothing ) return;
  //com_port->print(message);
}

void Commander::println(const int number){
  //if(!com_port || verbose == VerboseMode::nothing ) return;
  //com_port->println(number);
}
void Commander::println(const float number){
  //if(!com_port || verbose == VerboseMode::nothing ) return;
  //com_port->println((float)number, (int)decimal_places);
}
void Commander::println(const char* message){
  //if(!com_port || verbose == VerboseMode::nothing ) return;
  //com_port->println(message);
}
//void Commander::println(const __FlashStringHelper *message){
  //if(!com_port || verbose == VerboseMode::nothing ) return;
  //com_port->println(message);
//}
void Commander::println(const char message){
  //if(!com_port || verbose == VerboseMode::nothing ) return;
  //com_port->println(message);
}


void Commander::print_verbose(const char* message){
  if(verbose == VerboseMode::user_friendly) print(message);
}

void Commander::print_machine_readable(const int number){
  if(verbose == VerboseMode::machine_readable) print(number);
}
void Commander::print_machine_readable(const float number){
  if(verbose == VerboseMode::machine_readable) print(number);
}
void Commander::print_machine_readable(const char* message){
  if(verbose == VerboseMode::machine_readable) print(message);
}
void Commander::print_machine_readable(const char message){
  if(verbose == VerboseMode::machine_readable) print(message);
}

void Commander::println_machine_readable(const int number){
  if(verbose == VerboseMode::machine_readable) println(number);
}
void Commander::println_machine_readable(const float number){
  if(verbose == VerboseMode::machine_readable) println(number);
}
void Commander::println_machine_readable(const char* message){
  if(verbose == VerboseMode::machine_readable) println(message);
}
void Commander::println_machine_readable(const char message){
  if(verbose == VerboseMode::machine_readable) println(message);
}

void Commander::print_error(){
 println("err\n");
}
