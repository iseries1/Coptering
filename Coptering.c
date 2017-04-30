/*
 * @brief IRIS copter follow copter code
 * @author Michael Burmeister
 * @date March 28, 2015
 * @version 1.0
 * 
*/

#include "simpletools.h"                      // Include simple tools
#include "mstimer.h"                          // Cog Timer Function
#include "fdserial.h"                         // Full Duplex Comm
#include "../Mavlink/common/mavlink.h"        // Mavlink decoder/encoder

typedef struct  _MavData
{
  int System;
  int Component;
  int Type;
  int Armed;
  int Custom;
  int Mode;
  int BatteryRemaining;
  int BatteryVoltage;
  int Latitude;
  int Longitude;
  int Altitude;
  int Satelites;
  int HDOP;
  int GPSFix;
  int Heading;
  int RelativeAltitude;
  int Throttle;
  int Pitch;
  int Roll;
  int Yaw;
  int WapPoint;
  int Channel9;
  int Mission;
  int Seq;
  int Time;
  int Pressure;
  int Power;
  int Cmd;
  int Cresults;
  int Txbuf;
  }  mavdata_t;

int MavDecode(mavdata_t *, mavlink_message_t *);
void Copter(void *);
void Copter2(void *);
void request_mavlink_rates(void);
void HeartBeat(void);
void SetMode(void);
void SetWP(float, float, float);
int SndData(int);


#define Rx 4
#define Tx 5
#define Baud 57600
#define Rx2 6
#define Tx2 7
#define LCNT 17
#define GCS 255

mavlink_system_t mavlink_system;
mavlink_system_t mavlink_system2;
static int packet_drops = 0;
static int parse_errors = 0;


fdserial *comm;
fdserial *comm2;

mavdata_t Md;
mavdata_t Md2;
mavlink_message_t *msg;
mavlink_message_t *msg2;
mavlink_message_t *buf;
mavlink_status_t *sts;
mavlink_status_t *sts2;
mavlink_heartbeat_t *Heart;
mavlink_sys_status_t *Status;
mavlink_gps_raw_int_t *GPS;
mavlink_vfr_hud_t *Hud;
mavlink_attitude_t *Att;
mavlink_mission_current_t *Msn;
mavlink_statustext_t *stx;
mavlink_rc_channels_t *chnl;
mavlink_mission_ack_t *msnack;
mavlink_mission_request_t *msnreq;
mavlink_command_int_t *msn;
mavlink_system_time_t *systm;
mavlink_scaled_pressure_t *syspr;
mavlink_power_status_t *syspw;
mavlink_command_ack_t *cmd;
mavlink_radio_status_t *radio;


int chan = 0;
int chan2 = 1;
unsigned char Buffer[200];
unsigned char Buffer2[200];
unsigned char StsMsg[51];
unsigned char Out[100];
unsigned char Out2[100];
unsigned char Sts[100];
unsigned char Sts2[100];
int Other = 0;
int NotHandled = -1;
float Slon, Slat;


int main()
{
  int b, i;
  int l, h;
  int ah, al;
  int Alt, PAlt;
  Slon = 0;
  Slat = 0;
  
  msg = (mavlink_message_t *)Buffer;
  sts = (mavlink_status_t *)Sts;
  buf = (mavlink_message_t *)Out;
  
  msg2 = (mavlink_message_t *)Buffer2;
  sts2 = (mavlink_status_t *)Sts2;
  
  Md.Custom = 12;
  Md.Seq = -1;
  Md.Mission = -1;
  Md.Cmd = -1;
  StsMsg[0] = 0;
  mavlink_system.sysid = 66;
  mavlink_system.compid = MAV_COMP_ID_IMU;
  
  cog_run(&Copter, 40);
  cog_run(&Copter2, 40);
  pause(1000);

  mstime_start();
  
  b = 0;
  while(1)
  {
    h = Md.BatteryVoltage / 1000;
    l = Md.BatteryVoltage - h * 1000;
    l = l / 10;
    ah = Md.Altitude / 1000;
    al = Md.Altitude - ah * 1000;
    al = al / 10;
    print("Mode: %d, Battery: %d.%d Satellites: %d Lon: %d, Lat: %d, Alt: %d.%d - ",
         Md.Custom, h, l, Md.Satelites, Md.Longitude, Md.Latitude, ah, al);

    print("Seq: %d, Mission: %d\n", Md.Seq, Md.Mission);
    
    if (NotHandled > 0)
    {
        print("Not Handled: %d\n", NotHandled);
        NotHandled = 0;
    }
    
    if (Md.Cmd > 0)
    {
        print("Cmd: %d, Results: %d\n", Md.Cmd, Md.Cresults);
        Md.Cmd = 0;
        Md.Mission = 0;
    }
    if (StsMsg[0] > 0)
    {
        print("Status: %s\n", StsMsg);
        StsMsg[0] = 0;
    }              
    pause(500);
    if (Other == 1)
      HeartBeat();

    if ((Md.Channel9 > 2000) && (Md.Custom != 4))
    {
      if (i++ > 4)
      {
        SetMode();
        PAlt = 0;
        i = 0;
      }
    }
    
    if ((Md.Channel9 < 2000) && (Md.Custom == 4))
    {
      Alt = Md2.RelativeAltitude;
      if (Alt < 2)
      {
        Alt = 5;
      }
      
      if ((abs(PAlt - Alt) > 1) && (Md.Mission >= 0))
      {
        PAlt = Alt;
        i = 0;
        Md.Mission = -1;
      }

      if (Md.Mission < 0)
      {
        if (i++ < 5)
        {
          SetWP(Md.Longitude, Md.Latitude, Alt);
          print("Setting Wp Alt: %d\n", Alt);
        }          
      }                  
    }      
  }
}

int MavDecode(mavdata_t *M, mavlink_message_t *m)
{
//  print("Message: %d, %d\n", m->msgid, m->seq);
  
  if (m->msgid == MAVLINK_MSG_ID_HEARTBEAT)
  {
    Heart = (mavlink_heartbeat_t *)m->payload64;
    if (Heart->base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY)
      M->Armed = 1;
    else
      M->Armed = 0;
    if (Heart->base_mode & MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE)
    {
      M->Custom = Heart->custom_mode;
    }
    else
      M->Custom = 0;
      
    M->System = m->sysid;
    M->Component = m->compid;
    M->Type = Heart->type;
    if (Other == 0)
    {
      request_mavlink_rates();
    }      
    return 2;
  }

  if (m->msgid == MAVLINK_MSG_ID_SYS_STATUS)
  {
    Status = (mavlink_sys_status_t *)m->payload64;
    M->BatteryRemaining = Status->battery_remaining;
    M->BatteryVoltage = Status->voltage_battery;
    return 2;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
  {
    GPS = (mavlink_gps_raw_int_t *)m->payload64;
    M->Latitude = GPS->lat;
    M->Longitude = GPS->lon;
    M->Altitude = GPS->alt;
    M->Satelites = GPS->satellites_visible;
    M->HDOP = GPS->eph;
    M->GPSFix = GPS->fix_type;
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_VFR_HUD)
  {
    Hud = (mavlink_vfr_hud_t *)m->payload64;
    M->Heading = Hud->heading;
    M->RelativeAltitude = Hud->alt;
    M->Throttle = Hud->throttle;
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_ATTITUDE)
  {
    Att = (mavlink_attitude_t *)m->payload64;
    M->Roll = Att->roll * 180/PI;
    M->Pitch = Att->pitch * 180/PI;
    M->Yaw = Att->yaw * 180/PI;
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_MISSION_CURRENT)
  {
    Msn = (mavlink_mission_current_t *)m->payload64;
    M->WapPoint = Msn->seq;
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_RC_CHANNELS)
  {
    chnl = (mavlink_rc_channels_t *)m->payload64;
    M->Channel9 = chnl->chan9_raw;
    return 1;
  }

  if (m->msgid == MAVLINK_MSG_ID_MISSION_ACK)
  {
    msnack = (mavlink_mission_ack_t *)m->payload64;
    M->Mission = msnack->type;
    return 1;
  }

  if (m->msgid == MAVLINK_MSG_ID_MISSION_REQUEST)
  {
    msnreq = (mavlink_mission_request_t *)m->payload64;
    M->Seq = msnreq->seq;
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_SYSTEM_TIME)
  {
    systm = (mavlink_system_time_t *)m->payload64;
    M->Time = systm->time_boot_ms;
    return 1;
  }

  if (m->msgid == MAVLINK_MSG_ID_SCALED_PRESSURE)
  {
    syspr = (mavlink_scaled_pressure_t *)m->payload64;
    M->Pressure = syspr->press_abs;
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_POWER_STATUS)
  {
    syspw = (mavlink_power_status_t *)m->payload64;
    M->Power = syspw->Vcc;
    return 1;
  }

  if (m->msgid == MAVLINK_MSG_ID_COMMAND_ACK)
  {
    cmd = (mavlink_command_ack_t *)m->payload64;
    M->Cmd = cmd->command;
    M->Cresults = cmd->result;
    return 1;
  }
      
  if (m->msgid == MAVLINK_MSG_ID_RC_CHANNELS_RAW)
  {
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT)
  {
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_RAW_IMU)
  {
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
  {
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_SCALED_IMU2)
  {
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW)
  {
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_TERRAIN_REPORT)
  {
    return 1;
  }
  
  if (m->msgid == MAVLINK_MSG_ID_RADIO_STATUS)
  {
    radio = (mavlink_radio_status_t *)m->payload64;
    M->Txbuf = radio->txbuf;
    return 1;
  }
                              
  if (m->msgid == MAVLINK_MSG_ID_STATUSTEXT)
  {
    stx = (mavlink_statustext_t *)m->payload64;
    memcpy(StsMsg, stx->text, 50);
    StsMsg[50] = 0;
    return 2;
  }
  NotHandled = m->msgid;
  return 0;
}

void Copter(void *par)
{
  int c;
  
  comm = fdserial_open(Rx, Tx, FDSERIAL_MODE_NONE, Baud);

  while(1)
  {
    c = fdserial_rxTime(comm, 500);
    if (c >= 0)
    {
	    if (mavlink_parse_char(chan, c, msg, sts))
      {
        if (MavDecode(&Md, msg) > 0)
            mstime_reset();
        memset(Sts, 0, sizeof(Sts));
      }
      if (mstime_get() > 3000)
      {
        Md.Custom = 17;
        mstime_reset();
      }
    }
    else
    {
      if (mstime_get() > 5000)
      {
        Md.Custom = 7;
        mstime_reset();
      }
    }
    packet_drops += sts->packet_rx_drop_count;
    parse_errors += sts->parse_error;
  }
  fdserial_close(comm);  
}

void Copter2(void *par)
{
  int c;
  
  comm2 = fdserial_open(Rx2, Tx2, FDSERIAL_MODE_NONE, Baud);

  while(1)
  {
    c = fdserial_rxTime(comm2, 500);
    if (c >= 0)
    {
	    if (mavlink_parse_char(chan2, c, msg2, sts2))
      {
        if (MavDecode(&Md2, msg2) > 0)
            mstime_reset();
        memset(Sts2, 0, sizeof(Sts2));
      }
      if (mstime_get() > 3000)
      {
        Md2.Custom = 17;
        mstime_reset();
      }
    }
    else
    {
      if (mstime_get() > 5000)
      {
        Md2.Custom = 7;
        mstime_reset();
      }
    }
    packet_drops += sts->packet_rx_drop_count;
    parse_errors += sts->parse_error;
  }
  fdserial_close(comm2);  
}

void request_mavlink_rates()
{
  int i,j;
  int t;
  
  const int  maxStreams = 8;
  const uint8_t MAVStreams[] = {
        MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_RAW_CONTROLLER,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2,
        MAV_DATA_STREAM_EXTRA3};
  const uint16_t MAVRates[] = {0x02, 0x02, 0x02, 0x05, 0x05, 0x02, 0x02, 0x02};
  for (i=0; i < maxStreams; i++)
  {
    t = mavlink_msg_request_data_stream_pack(mavlink_system.sysid, mavlink_system.compid, buf,
          Md.System, Md.Component, MAVStreams[i], MAVRates[i], 1);
    
    SndData(t);
  }
  Other = 1;
}

void HeartBeat()
{
  int t;
  
  t = mavlink_msg_heartbeat_pack(GCS, MAV_COMP_ID_MISSIONPLANNER, buf,
						       6, 0, 0, 0, 0);
  SndData(t);
}

void SetMode()
{
  int t;
  
  t = mavlink_msg_set_mode_pack(GCS, MAV_COMP_ID_MISSIONPLANNER, buf,
						       Md.System, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4);
  SndData(t);
}


void SetWP(float Lng, float Lat, float Alt)
{
  int t;
  
  if (Slon == 0)
  {
    Slon = Lng/10000000;
    Slat = Lat/10000000;
  }
  
  print("Long: %f, Lat: %f, Alt: %f\n", Lng, Lat, Alt);
  t = mavlink_msg_mission_item_pack(GCS, MAV_COMP_ID_MISSIONPLANNER, buf,
                      Md.System, Md.Component, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      MAV_CMD_NAV_WAYPOINT, 2, 1, 0, 0, 0, 0, Slat, Slon, Alt);
  SndData(t);
}


void TakeOff(float Alt)
{
  int t;
  print("Altitude: %f\n", Alt);
  t = mavlink_msg_command_long_pack(GCS, MAV_COMP_ID_MISSIONPLANNER, buf,
						       Md.System, Md.Component, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, Alt);
  SndData(t);
}
    
int SndData(int t)
{
  int j;
  
  for (j=2;j<t;j++)
    fdserial_txChar(comm, Out[j]);
  
  fdserial_txChar(comm, Out[0]);
  fdserial_txChar(comm, Out[1]);
}

    