#ifndef BUS_H
#define BUS_H

typedef struct
{
    float v_max;
    float end_v;
    float accl;
    float decel;
    float w_max;
    float end_w;
    float alpha;
    float tgt_dist;
    float tgt_angle;
} t_tgt;

typedef struct
{
    float base_alpha;
    float base_time;
    float limit_time_count;
    float pow_n;
    char state;
    int counter;
} t_slalom;

typedef struct
{
    float v;
    float accl;
    float w;
    float alpha;
    float dist;
    float ang;
    t_slalom sla_param;
    char state;
    char pivot_state;
} t_ego;

typedef struct
{
    float v;
    float accl;
    float w;
    float alpha;
} t_mpc_out;

#define MPC_SIZE 5
typedef struct
{
    t_mpc_out next_state[MPC_SIZE];
} t_mpc_out_list;

typedef enum
{
    NONE_MODE = 0,
    ST_RUN = 1,
    PIVOT_TURN = 2,
    SLAROM_RUN = 3
} RUN_MODE;

#endif