#ifndef ROBOT_SUCTION_H
#define ROBOT_SUCTION_H

extern void robot_suction_vols_set(u16 val);
extern void robot_suction_init(void);
extern void robot_suction_update(void);
extern int calc_cur_duty(int16_t voltage);

#endif