/*
 * WallOff.h
 *
 *  Created on: 2016/08/27
 *      Author: Naoto
 */

#ifndef WALLOFF_H_
#define WALLOFF_H_

char wallOff(char RorL, char ctrl);
char wallOff_D(char RorL, char ctrl);
volatile float wall_off_limit = 40;
volatile float wall_off_limit_d = 40;

void walloff1(char RorL) {
	if (RorL == R) {
		while (SEN_R < R_WALL_OFF && distance <= wall_off_limit
				&& SEN_FRONT < FRONT_WALL_ON) {
			if (!fail) {
				positionControlValueFlg = 0;
				break;
			}
		}
		globalState = WALL_OFF;
		while (SEN_R > R_WALL_OFF /*&& SEN_FRONT < FRONT_WALL_ON*/) {
			if (!fail) {
				positionControlValueFlg = 0;
				break;
			}
		}
	} else {
		while (SEN_L < L_WALL_OFF && distance <= wall_off_limit
				&& SEN_FRONT < FRONT_WALL_ON) {
			if (!fail) {
				positionControlValueFlg = 0;
				break;
			}
		}
		globalState = WALL_OFF;
		while (SEN_L > L_WALL_OFF /*&& SEN_FRONT < FRONT_WALL_ON*/) {
			if (!fail) {
				positionControlValueFlg = 0;
				break;
			}
		}
	}
}

float existRightWall4 = 6000;  //壁切れの予備
float existLeftWall4 = 6000;   //壁切れの予備

char wallOff(char RorL, char ctrl) {
	globalState = WALL_OFF_WAIT;
	cc = 1;

	distance = 0;
	img_distance = 0;
	Distance.error_now = 0;
	Distance.error_old = 0;
	Distance.error_delta = 0;

	ego_data_in.state = 0;
	ego_data_in.img_ang = 0;
	ego_data_in.img_dist = 0;
	mpc_tgt_calc_mode = (int32_T) ST_RUN;

	if (globalSkipFront) {
		return true;
	}

	if (dia == 1) {
		return wallOff_D(RorL, ctrl);
	}
	sensingMode = AtackStraight;
	positionControlValueFlg = ctrl;
	ang = 0;
	angle = 0;
	distance = 0;
	img_distance = 0;

	ego_data_in.state = 0;
	ego_data_in.img_ang = 0;
	ego_data_in.img_dist = 0;
	mpc_tgt_calc_mode = (int32_T) ST_RUN;

	if (RorL == R) {
		walloff1(R);
	} else {
		walloff1(L);
	}
	positionControlValueFlg = 0;
	cc = 0;
	return 1;
}

//float R_WALL_OFF_D = 270;  //壁切れ　閾値　斜め用
//float L_WALL_OFF_D = 430; //壁切れ　閾値　斜め用

void walloff3(char RorL) {
	if (RorL == R) {

		while (SEN_R < R_WALL_OFF_D && distance <= wall_off_limit_d
				&& SEN_L < DIA_N_LEFT_WALL && SEN_FRONT < DIA_FRONT_WALL_ORDER) {
			if (!fail) {
				positionControlValueFlg = 0;
				break;
			}
		}
		globalState = WALL_OFF;
		while (SEN_R > R_WALL_OFF_D /*&& SEN_L < DIA_N_LEFT_WALL
		 && SEN_FRONT < DIA_FRONT_WALL_ORDER*/) {
			if (!fail) {
				positionControlValueFlg = 0;
				break;
			}
		}
	} else {
		while (SEN_L < L_WALL_OFF_D && distance <= wall_off_limit_d
				&& SEN_R < DIA_N_RIGHT_WALL && SEN_FRONT < DIA_FRONT_WALL_ORDER) {
			if (!fail) {
				positionControlValueFlg = 0;
				break;
			}
		}
		globalState = WALL_OFF;
		while (SEN_L > L_WALL_OFF_D /*&& SEN_R < DIA_N_RIGHT_WALL
		 && SEN_FRONT < DIA_FRONT_WALL_ORDER*/) {
			if (!fail) {
				positionControlValueFlg = 0;
				break;
			}
		}
	}
}

char wallOff_D(char RorL, char ctrl) {
//	positionControlValueFlg = 1;
	ang = 0;
	angle = 0;
	distance = 0;
	img_distance = 0;
	sensingMode = AtackDia;
	if (RorL == R) {
		walloff3(R);
	} else {
		walloff3(L);
	}
	positionControlValueFlg = 0;
	return 1;
}
#endif /* WALLOFF_H_ */
