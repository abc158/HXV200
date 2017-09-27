#include <ui-config.h>
#include "ui-commands.h"
#include "am_date_base.h"
#include "time.h"
#include  <am_config_macro.h>
#include "am_key.h"
#include "act.h"
#include "local_key_check.h"
#include "remote.h"
#include "display.h"
#include "ui-song-player.h"
#include "syscall_api.h"
#include "wireless/ARF2496K.H"
#include "sensor/sensor.h"
#include "motor/motor.h"
#include "monitor/robot_batter.h"
#include "ui-manager/exception.h"
#include "am_device.h"
#include "state_view.h"

#define STATE_VIEW 1

#define LT_VIEW 0

union{
    float f[4];
    short i16[8];
    unsigned int u32[4];
    unsigned char u8[16];
}uni;

U16 my_time=0;

void view_main_task(void)
{
#if STATE_VIEW
      static sys_state_info p,old_p;
      static UI_STATE_E s,old_s;
      int veiw_flag=0;
      U8 i=4;
      
      my_time++;      
      s = get_ui_state(); 
      sys_info_get(SYS_STATE_INFO, (long)(&p));
      
      uni.u32[0] = 0x10aa55aa;
      uni.u8[i++] = p.robot_state;
      uni.u8[i++] = p.charging_state;
      
      uni.u8[i++] = p.dock_state;
      uni.u8[i++] = p.sound_state;
      uni.u8[i++] = p.flush_state;
      uni.u8[i++] = p.switch_state;
      
      uni.u8[i++] = p.pause_id;
      uni.u8[i++] = p.bin_full;
      uni.u8[i++] = s;
      uni.u8[i++] = 0;
      
      veiw_flag=0;
      if(p.robot_state!=old_p.robot_state)          veiw_flag =1;
      if(p.charging_state!=old_p.charging_state)    veiw_flag =1;
      if(p.dock_state!=old_p.dock_state)            veiw_flag =1;
      if(p.sound_state!=old_p.sound_state)          veiw_flag =1;
      if(p.flush_state!=old_p.flush_state)          veiw_flag =1;
      if(p.switch_state!=old_p.switch_state)        veiw_flag =1;
      if(p.pause_id!=old_p.pause_id)                veiw_flag =1;
      if(p.bin_full!=old_p.bin_full)                veiw_flag =1;
      if(s!=old_s)                                  veiw_flag =1;

      if(veiw_flag)
      {
          uni.i16[7] = my_time;
          write(uart_ui,uni.u8,16);          
          old_p.robot_state = p.robot_state;
          old_p.charging_state = p.charging_state;
          old_p.dock_state = p.dock_state;
          old_p.sound_state = p.sound_state;
          old_p.flush_state = p.flush_state;
          old_p.switch_state = p.switch_state;
          old_p.pause_id = p.pause_id;
          old_p.bin_full= p.bin_full;
          old_s = s;
      }

#endif
}

void view_msg(U8 msg)
{
#if STATE_VIEW

    uni.u32[0] = 0x11aa55aa;
    uni.u8[4] = msg; 
    uni.u8[5] = 0;
    uni.i16[3] = my_time;
    write(uart_ui,uni.u8,8);

#endif
}

void view_LT_sensor(s16 p8,s16 p9,s16 p10,s16 p11,s16 p12,s16 p13)
{
#if LT_VIEW
    U8  i=4;
    uni.u32[0] = 0x12aa55aa;
    
    uni.u8[i++] =  p8>>5;
    uni.u8[i++] =  p9>>5;
    uni.u8[i++] =  p10>>5;
    uni.u8[i++] =  p11>>5;
    uni.u8[i++] =  p12>>5;
    uni.u8[i++] =  p13>>5;

    write(uart_ui,uni.u8,16);          
#endif
}

void view_CMD(U32 cmd_id,U8 cmd_action)
{
#if STATE_VIEW

    uni.u32[0] = 0x13aa55aa;
    uni.u32[1] =  cmd_id;
    uni.u8[8] =  cmd_action;
    uni.u8[9] = 0;
    uni.i16[5] = my_time;
    write(uart_ui,uni.u8,12);

#endif
}

void view_key(U32 key_val)
{
#if STATE_VIEW
    static U32 old_key=0;
    if(old_key!=key_val)
    {
        uni.u32[0] = 0x14aa55aa;
        uni.u32[1] = key_val;
        old_key = key_val;
        uni.i16[4] = my_time;
        write(uart_ui,uni.u8,10);
    }
#endif
}



void view_bump(U32 bump)
{
#if STATE_VIEW
    static U32 old_bump=0;
    if(old_bump!=bump)
    {
        uni.u32[0] = 0x15aa55aa;
        uni.u32[1] = bump;
        old_bump = bump;
        uni.i16[4] = my_time;
        write(uart_ui,uni.u8,10);
    }
#endif
}




