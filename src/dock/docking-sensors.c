//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include "docking-new.h"
#include "docking-sensors.h"
#include "ui-ir.h"
#include "sensor/sensor.h"
#include "dock-avoid.h"
#include "virtual-wall.h"

U8 dock_signals[IR_MAX_RECV];//need to get from core board

#define DOCK_SENSORS_DEBUG
#ifdef DOCK_SENSORS_DEBUG
enum {
	DEBUG_DOCK_SIGNAL = 0x1 << 0,
};

static U8 debug_mask = 0;
#define dprintf(level_mask, msg...)  if(debug_mask&level_mask)\
	printf(msg)
#else
#define dprintf(msg...) {}
#endif

BOOLEAN force_field(IR_local_Index chan)
{
	return ((dock_signals[chan]&DOCK_CLOSE_BEACON) == DOCK_CLOSE_BEACON);
}

BOOLEAN buoy_left(IR_local_Index chan)
{
	return ((dock_signals[chan]&LEFT_BEACON_BYTE) == LEFT_BEACON_BYTE);
}

BOOLEAN buoy_right(IR_local_Index chan)
{
	return ((dock_signals[chan]&RIGHT_BEACON_BYTE) == RIGHT_BEACON_BYTE);
}

BOOLEAN check_near_dock(void)
{
	return (((force_field(IR_LOCAL_MID_RIGHT)) || (force_field(IR_LOCAL_MID_LEFT)) || \
		(force_field(IR_LOCAL_LEFT)) || (force_field(IR_LOCAL_RIGHT)) || \
		(force_field(IR_LOCAL_BACK_RIGHT)) || (force_field(IR_LOCAL_BACK_LEFT))) 
		);
}

BOOLEAN check_near_dock_1(void)
{
	return (((force_field(IR_LOCAL_MID_RIGHT)) || (force_field(IR_LOCAL_MID_LEFT)) || \
		(force_field(IR_LOCAL_LEFT)) || (force_field(IR_LOCAL_RIGHT))));
}

BOOLEAN check_midleft_near_dock(void)
{
	return (force_field(IR_LOCAL_MID_LEFT));
}

BOOLEAN check_right_near_dock(void)
{
	return ((buoy_left(IR_LOCAL_BACK_RIGHT) || buoy_left(IR_LOCAL_RIGHT)) || \
		(force_field(IR_LOCAL_BACK_RIGHT) || force_field(IR_LOCAL_RIGHT)) || \
		(buoy_right(IR_LOCAL_BACK_RIGHT) || buoy_right(IR_LOCAL_RIGHT)));
}

u8 check_center_left_focus_beacon(void)
{
    if (buoy_right(IR_LOCAL_MID_LEFT) || buoy_left(IR_LOCAL_MID_LEFT))
		return  1;
	else  if (force_field(IR_LOCAL_MID_LEFT))
		return  2;
	else
		return  0;
}

u8 check_center_right_focus_beacon(void)
{
    if (buoy_right(IR_LOCAL_MID_RIGHT) || buoy_left(IR_LOCAL_MID_RIGHT))
		return  1;
	else  if (force_field(IR_LOCAL_MID_RIGHT))
		return  2;
	else
		return  0;
}

u8 check_left_beacon(void)
{
	if (buoy_right(IR_LOCAL_LEFT) || buoy_left(IR_LOCAL_LEFT))
		return  1;
	else  if (force_field(IR_LOCAL_LEFT))
		return  2;
	else
		return  0;
}

u8 check_right_beacon(void)
{
	if (buoy_right(IR_LOCAL_RIGHT) || buoy_left(IR_LOCAL_RIGHT))
		return  1;
	else  if (force_field(IR_LOCAL_RIGHT))
		return  2;
	else
		return  0;
}

u8 check_left_right_beacon(void)
{
	if (buoy_right(IR_LOCAL_RIGHT) || buoy_left(IR_LOCAL_RIGHT))
		return  1;
        else
		return  0;
}

u8 check_back_left_beacon(void)
{
	if (buoy_right(IR_LOCAL_BACK_LEFT) || buoy_left(IR_LOCAL_BACK_LEFT))
		return  1;
	else  if (force_field(IR_LOCAL_BACK_LEFT))
		return  2;
	else
		return  0;
}

u8 check_back_right_beacon(void)
{
	if (buoy_right(IR_LOCAL_BACK_RIGHT) || buoy_left(IR_LOCAL_BACK_RIGHT))
		return  1;
	else  if (force_field(IR_LOCAL_BACK_RIGHT))
		return  2;
	else
		return  0;
}

BOOLEAN check_signal(void)
{
	if ((check_back_left_beacon()) || (check_back_right_beacon()) || \
		(check_left_beacon()) || (check_right_beacon()) || \
		(check_center_left_focus_beacon()) || (check_center_right_focus_beacon()))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_recently_left_left(void)
{
	return (buoy_left(IR_LOCAL_LEFT));
}

BOOLEAN check_recently_right_right(void)
{
	return (buoy_right(IR_LOCAL_RIGHT));
}

BOOLEAN check_recently_left_right(void)
{
	return (buoy_left(IR_LOCAL_RIGHT));
}
BOOLEAN check_recently_right_left(void)
{
	return (buoy_right(IR_LOCAL_LEFT));
}
BOOLEAN check_recently_right_midright(void)
{
	return (buoy_right(IR_LOCAL_MID_RIGHT));
}
BOOLEAN check_recently_left_midleft(void)
{
	return (buoy_left(IR_LOCAL_MID_LEFT));
}

BOOLEAN check_recently_right_midleft(void)
{
	return (buoy_right(IR_LOCAL_MID_LEFT));
}
BOOLEAN check_right_midleft(void)
{
	if (buoy_right(IR_LOCAL_MID_LEFT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_left_midright(void)
{
	if (buoy_left(IR_LOCAL_MID_RIGHT))
		return TRUE;
	else
		return FALSE;
}
BOOLEAN check_recently_left_backleft(void)
{
	return (buoy_left(IR_LOCAL_BACK_LEFT));
}

BOOLEAN check_recently_right_backright(void)
{
	return (buoy_right(IR_LOCAL_BACK_RIGHT));
}

BOOLEAN check_recently_right_backleft(void)
{
	return (buoy_right(IR_LOCAL_BACK_LEFT));
}

BOOLEAN check_recently_left_backright(void)
{
	return (buoy_left(IR_LOCAL_BACK_RIGHT));
}

BOOLEAN check_recently_force_field_middle(void)
{
	return (force_field(IR_LOCAL_MID_RIGHT) || force_field(IR_LOCAL_MID_LEFT));
}

BOOLEAN check_recently_force_field(void)
{
	return recently_near_dock_1.current_state;
}

BOOLEAN check_recently_follow_right_force_field(void)
{
	return ((force_field(IR_LOCAL_RIGHT)));
}

BOOLEAN check_recently_midright_force_field(void)
{
	return ((force_field(IR_LOCAL_MID_RIGHT)));
}
BOOLEAN check_recently_midleft_force_field(void)
{
	return ((force_field(IR_LOCAL_MID_LEFT)));
}
BOOLEAN check_recently_follow_left_force_field(void)
{
	return ((force_field(IR_LOCAL_LEFT)));
}

BOOLEAN check_recently_center_left_focus(void)
{
	return (1 == check_center_left_focus_beacon());
}

BOOLEAN check_recently_center_right_focus(void)
{
	return (1 == check_center_right_focus_beacon());
}

BOOLEAN check_docking_go_forward_right(void)
{
	if (buoy_right(IR_LOCAL_MID_RIGHT) && buoy_left(IR_LOCAL_MID_RIGHT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_left(void)
{
	if (buoy_left(IR_LOCAL_MID_LEFT) && buoy_right(IR_LOCAL_MID_LEFT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_onlyright(void)
{
	if (buoy_right(IR_LOCAL_MID_RIGHT) && !buoy_left(IR_LOCAL_MID_RIGHT))
		return TRUE;
	else
		return FALSE;
}

BOOLEAN check_docking_go_forward_onlyleft(void)
{
	if (buoy_left(IR_LOCAL_MID_LEFT) && !buoy_right(IR_LOCAL_MID_LEFT))
		return TRUE;
	else
		return FALSE;
}

//BOOLEAN check_docking_go_forward(void)
//{
//	if ((recently_docking_go_forward_right.current_state && recently_docking_go_forward_left.current_state) || 
//		(recently_docking_go_forward_onlyright.current_state && recently_docking_go_forward_onlyleft.current_state))
//		return TRUE;
//	else
//		return FALSE;
//}
BOOLEAN check_docking_bump(void)
{
	if ((get_cliff_state() != 0) || (get_bump_state() != 0))
		return TRUE;
	else
		return FALSE;
}
BOOLEAN check_recently_near_dock(void)
{
    return recently_near_dock_1.current_state;
}
void set_near_dock_context(BOOLEAN value)
{
	if(value)
	{
		//set_slip_high_throd(6);
		set_stasis_high_throd();
		//turn_off_touch_bump();
		for(int i=0;i<3;i++)
                {
	          set_cliff_threshold((SENSOR_E)i, 20);
                }
		//set_lighttouch_enable(1);
	}
	else
	{
		//set_slip_normal();
		set_stasis_normal();
		//turn_on_touch_bump();
		reset_cliff_threshold();
		//set_lighttouch_enable(0);
	}
	return;
}

Debouncer_Data recently_near_dock = {
	.predicate = &check_near_dock,
	.trigger_on = 1,
	.trigger_off = 80,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = &set_near_dock_context
};

Debouncer_Data recently_near_dock_1 = {
	.predicate = &check_near_dock_1,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_near_dock_count = {
	.predicate = &check_recently_near_dock,
	.trigger_on = 11,
	.trigger_off = 1,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_center_left_focus = {
	.predicate = &check_recently_center_left_focus,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_center_right_focus = {
	.predicate = &check_recently_center_right_focus,
	.trigger_on = 1,
	.trigger_off =  50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_left = {
	.predicate = &check_recently_left_left,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_right = {
	.predicate = &check_recently_right_right,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_midright = {
	.predicate = &check_left_midright,
	.trigger_on = 1,
	.trigger_off = 60,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_midleft = {
	.predicate = &check_right_midleft,
	.trigger_on = 1,
	.trigger_off = 60,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_right_midright = {
	.predicate = &check_recently_right_midright,
	.trigger_on = 1,
	.trigger_off = 60,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_midleft = {
	.predicate = &check_recently_left_midleft,
	.trigger_on = 1,
	.trigger_off = 60,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_backleft = {
	.predicate = &check_recently_left_backleft,
	.trigger_on = 1,
	.trigger_off = 80,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_backright = {
	.predicate = &check_recently_right_backright,
	.trigger_on = 1,
	.trigger_off = 80,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_backleft = {
	.predicate = &check_recently_right_backleft,
	.trigger_on = 1,
	.trigger_off = 80,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_backright = {
	.predicate = &check_recently_left_backright,
	.trigger_on = 1,
	.trigger_off = 80,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_right = {
	.predicate = &check_recently_left_right,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_left = {
	.predicate = &check_recently_right_left,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_near_dock = {
	.predicate = &check_right_near_dock,
	.trigger_on = 1,
	.trigger_off = 100,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_midleft_near_dock = {
	.predicate = &check_midleft_near_dock,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_left_right_near_dock = {
	.predicate = &check_left_right_beacon,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_follow_right_force_field = {
	.predicate = &check_recently_follow_right_force_field,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_follow_left_force_field = {
	.predicate = &check_recently_follow_left_force_field,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_follow_midright_force_field = {
	.predicate = &check_recently_midright_force_field,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
Debouncer_Data recently_follow_midleft_force_field = {
	.predicate = &check_recently_midleft_force_field,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
#if 0
Debouncer_Data recently_signal = {
	.predicate = &check_signal,
	.trigger_on = 1,
	.trigger_off = 2000,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_near_dock = {
	.predicate = &check_near_dock,
	.trigger_on = 1,
	.trigger_off = 200,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = &set_near_dock_context
};

Debouncer_Data recently_near_dock_1 = {
	.predicate = &check_near_dock_1,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_near_dock = {
	.predicate = &check_left_near_dock,
	.trigger_on = 1,
	.trigger_off = 100,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_near_dock = {
	.predicate = &check_right_near_dock,
	.trigger_on = 1,
	.trigger_off = 100,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_center_left_focus = {
	.predicate = &check_recently_center_left_focus,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_center_right_focus = {
	.predicate = &check_recently_center_right_focus,
	.trigger_on = 1,
	.trigger_off =  50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_right = {
	.predicate = &check_docking_go_forward_right,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_left = {
	.predicate = &check_docking_go_forward_left,
	.trigger_on = 1,
	.trigger_off = 15,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_onlyright = {
	.predicate = &check_docking_go_forward_onlyright,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward_onlyleft = {
	.predicate = &check_docking_go_forward_onlyleft,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_go_forward = {
	.predicate = &check_docking_go_forward,
	.trigger_on = 1,
	.trigger_off = 15,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_left = {
	.predicate = &check_docking_left,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_docking_right = {
	.predicate = &check_docking_right,
	.trigger_on = 1,
	.trigger_off = 40,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_left = {
	.predicate = &check_recently_left_left,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_right = {
	.predicate = &check_recently_right_right,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_backleft = {
	.predicate = &check_recently_left_backleft,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_backright = {
	.predicate = &check_recently_right_backright,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_backleft = {
	.predicate = &check_recently_right_backleft,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_backright = {
	.predicate = &check_recently_left_backright,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_left_right = {
	.predicate = &check_recently_left_right,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_right_left = {
	.predicate = &check_recently_right_left,
	.trigger_on = 1,
	.trigger_off = 50,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_force_field_middle = {
	.predicate = &check_recently_force_field_middle,
	.trigger_on = 1,
	.trigger_off = 10,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_force_field = {
	.predicate = &check_recently_force_field,
	.trigger_on = 11,
	.trigger_off = 1,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_no_force_field = {
	.predicate = &check_docking_go_forward,
	.trigger_on = 1,
	.trigger_off = 200,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_follow_right_force_field = {
	.predicate = &check_recently_follow_right_force_field,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_follow_left_force_field = {
	.predicate = &check_recently_follow_left_force_field,
	.trigger_on = 1,
	.trigger_off = 20,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};

Debouncer_Data recently_bump = {
	.predicate = &check_docking_bump,
	.trigger_on = 1,
	.trigger_off = 30,
	.on_count = 0,
	.off_count = 0,
	.current_state = FALSE,
	.set_dock_context = NULL
};
#endif
U8 robot_get_dock_signals(U8 index)
{
#ifdef IR_WIRELESS
	dock_signals[index] = get_wireless_rx_code();
#else
	dock_signals[index] = remote_ir_get((IR_REMOT_POSITION_E)index);
#endif

	if (dock_signals[index] != 0)
	{
		dprintf(DEBUG_DOCK_SIGNAL, "IR%d : %x \r\n", index, dock_signals[index]);

		dock_avoid_get_signals(index, dock_signals[index]);
		virtual_wall_get_signals(index, dock_signals[index]);
	}

	return dock_signals[index];
}
