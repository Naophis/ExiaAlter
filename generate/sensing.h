/*
 * sensing.h
 *
 *  Created on: 2020/12/06
 *      Author: nao12
 */

#ifndef SENSING_H_
#define SENSING_H_

#define QUEUE_SIZE 50          /* 待ち行列に入るデータの最大数 */

typedef int data_t; /* データ型 */
typedef struct {
	float front;
	float r45;
	float r90;
	float l45;
	float l90;
	float gyro;
} t_sensor_data;
t_sensor a;

typedef struct {
	float front[QUEUE_SIZE];
	float r45[QUEUE_SIZE];
	float r90[QUEUE_SIZE];
	float l45[QUEUE_SIZE];
	float l90[QUEUE_SIZE];
	float gyro[QUEUE_SIZE];
	int head;
	int tail;
	int size;
} t_sensor_data_buf;

t_sensor_data_buf data_buf;
int queue_head; /* データ先頭 */
int queue_num; /* データ個数 */

void add_sensor_data(float front, float r45, float r90, float l45, float l90) {
	int i = data_buf.head;
	data_buf.front[i] = front;
	data_buf.r45[i] = r45;
	data_buf.r90[i] = r90;
	data_buf.l45[i] = l45;
	data_buf.l90[i] = l90;
}

#endif /* SENSING_H_ */
