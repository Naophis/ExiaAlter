/*
 * PhysicalBasement.h
 *
 *  Created on: 2013/11/20
 *      Author: Naoto
 */

#ifndef PHYSICALBASEMENT_H_
#define PHYSICALBASEMENT_H_

char directionCheck_R = 0;
char directionCheck_L = 0;
float enc_r, enc_l;
long er, el;
float vr, vl;
float vr2, vl2;
#define logSize  200
float logR[logSize];
float logL[logSize];

void clearLogs() {
	for (char i = 0; i < logSize; i++) {
		logR[i] = 0;
		logL[i] = 0;
	}
}
float peekRight = 0;
float peekLeft = 0;
float lastPeekR = 0;
float lastPeekL = 0;
float peekRight2 = 0;
float peekLeft2 = 0;
float lastPeekR2 = 0;
float lastPeekL2 = 0;

float peekOrderR = 1800;
float peekOrderL = 1800;

void pushLatestSensor(float r, float l) {
	if (r > peekOrderR) {
		if (peekRight < r) {
			peekRight = r;
		}
	} else {
		if (lastPeekR == 0 && peekRight > peekOrderR) {
			lastPeekR = peekRight;
			lastPeekR2 = 0;
		}
		peekRight = 0;
	}
	if (l > peekOrderL) {
		if (peekLeft < l) {
			peekLeft = l;
		}
	} else {
		if (lastPeekL == 0 && peekLeft > peekOrderL) {
			lastPeekL = peekLeft;
			lastPeekL2 = 0;
		}
		peekLeft = 0;
	}
}

float peekOrderR2 = 2000;
float peekOrderL2 = 2000;
void pushLatestSensor2(float r, float l) {
	if (r > peekOrderR2) {
		if (peekRight2 < r) {
			peekRight2 = r;
		}
	} else {
		if (lastPeekR2 == 0 && peekRight2 > peekOrderR2) {
			lastPeekR2 = peekRight2;
			lastPeekR = 0;
		}
		peekRight2 = 0;
	}
	if (l > peekOrderL2) {
		if (peekLeft2 < l) {
			peekLeft2 = l;
		}
	} else {
		if (lastPeekL2 == 0 && peekLeft2 > peekOrderL2) {
			lastPeekL2 = peekLeft2;
			lastPeekL = 0;
		}
		peekLeft2 = 0;
	}
}

void pushLog(float r, float l) {
	for (char i = 0; i < logSize - 1; i++) {
		logR[i + 1] = logR[i];
		logL[i + 1] = logL[i];
	}
	logR[0] = r;
	logL[0] = l;
}

void searchPeek() {
	for (int i = 0; i < logSize; i++) {

	}
}

float getAverageR() {
	float sum = 0;
	for (char i = 0; i < logSize; i++) {
		sum += logR[i];
	}
	return sum / logSize;
}
float getAverageL() {
	float sum = 0;
	for (char i = 0; i < logSize; i++) {
		sum += logL[i];
	}
	return sum / logSize;
}

void enc_to_vel(void) {
	float tmp_tire = TIRE;

	if (fanStart == true) {
		if (fanMode != SearchRun) {
			tmp_tire = (*(float *) 1050060);
		}
	}
	vr = -(float) (PI * tmp_tire * enc_r * CMT_CYCLE / GEAR / 4096.0);
	vl = -(float) (PI * tmp_tire * enc_l * CMT_CYCLE / GEAR / 4096.0);

	V_Enc.r = vr; // vr * 0.1 + vr2 * 0.9;
	V_Enc.l = vl; // vl * 0.1 + vl2 * 0.9;

	vr2 = V_Enc.r;
	vl2 = V_Enc.l;

	rpmR_old = rpmR;
	rpmL_old = rpmL;

	rpmR = ((V_now - Tread * 500 * tgt_w_now) / (tmp_tire / 2)) * 60.0
			/ (2.0 * PI);
	rpmL = ((V_now + Tread * 500 * tgt_w_now) / (tmp_tire / 2)) * 60.0
			/ (2.0 * PI);

	Velocity.before = Velocity.error_now;
	Velocity.error_now = tgt_v_now - (V_Enc.r + V_Enc.l) / 2;
	// Velocity.error_now = V_now - (V_Enc.r + V_Enc.l) / 2;
	Velocity.error_old += Velocity.error_now;
	Velocity.error_delta = Velocity.error_now - Velocity.before;

}
#define KIREME_R 11 //11/12
#define KIREME_L 11
#define KIREME_R2 10
#define KIREME_L2 10
unsigned int R_WALL = 880; //820; //#define R_WALL 4000	//サーキット用
unsigned int L_WALL = 750; //600;	//制御壁閾値
volatile float FRONT_OUT = 1500;	//袋小路時前センサー閾値

int checkStr = 0;

char validateStraight(float dist) {
	return true;
	if (!enableSensorCtrl) {
		return true;
	}

	int c = (int) (dist / 180);
	float tmpdist = dist - c * 180;

	if (fanMode == SearchRun || !fanMode) {
		return !(60 <= tmpdist && tmpdist <= 90);
	} else {
		return true;
//		return !(60 + 90 <= tmpdist && tmpdist <= 180);
	}
}

char validateGyroReset() {
	const int enable = (int) (*(float *) 1049828);
	const float range = *(float *) 1049832;

	if (enable) {
		return (ang * 180 / PI) < range && (ang * 180 / PI) > -range;
	}

	return false;
}

float check_sen_error_front(void) {
	float error = 0;
	char check = 0;
	const float FRONT_EXIST = *(float *) 1049636;
	const float FRONT_REF = *(float *) 1049632;
	const float FRONT_DIFF = *(float *) 1049628;

	if (Front_SEN.now > FRONT_EXIST) {

		if (ABS(Front_SEN.now - FRONT_REF) > FRONT_DIFF) {
			error += Front_SEN.now - FRONT_REF;
			check++;
		}
	}

	if (check == 0) {
		V_now = 0;
		SeFrnt.error_now = 0;
		SeFrnt.error_old = 0;
		Velocity.error_now = 0;
		Velocity.error_old = 0;
		Velocity.error_delta = 0;
		C.v = 0;
	}
	return error;
}

float check_sen_error_frontSide(void) {
	float error = 0;
	char check = 0;

	const float R_EXIST = *(float *) 1049944;
	const float R_REF = *(float *) 1049948;

	const float L_EXIST = *(float *) 1049952;
	const float L_REF = *(float *) 1049956;

	if (RS_SEN45.now > R_EXIST) {
		error += RS_SEN45.now - R_REF;
		check++;
	}

	if (LS_SEN45.now > L_EXIST) {
		error -= LS_SEN45.now - L_REF;
		check++;
	}

	if (check == 0) {
		W_now = 0;
		SeFrntAngle.error_now = 0;
		SeFrntAngle.error_old = 0;
		Gy.error_now = 0;
		Gy.error_old = 0;
		Gy.error_delta = 0;
		C.w = 0;
	}
	return error;
}

float check_sen_error(void) {
	float error = 0;
	char check = 0;
	const float rightCtrlDiff = *(float *) 1049964;
	const float leftCtrlDiff = *(float *) 1049968;
	const int kireme_R = (int) (*(float *) 1049972);
	const int kireme_L = (int) (*(float *) 1049976);
	const float FRONT_OUT = *(float *) 1049772;
	const float R90_exist = *(float *) 1050020;
	const float L90_exist = *(float *) 1050024;

	if (Front_SEN.now < FRONT_OUT && validateStraight(distance)) {
		if (ABS(RS_SEN45.now - RS_SEN45.old) < kireme_R) {
			if (RS_SEN45.now > (RS_SEN45.ref - rightCtrlDiff)
					&& RS_SEN2.now > R90_exist) {
				error += RS_SEN45.now - RS_SEN45.ref;
				check++;
			}
		}
		if (ABS(LS_SEN45.now - LS_SEN45.old) < kireme_L) {
			if (LS_SEN45.now > (LS_SEN45.ref - leftCtrlDiff)
					&& LS_SEN2.now > L90_exist) {
				error -= LS_SEN45.now - LS_SEN45.ref;
				check++;
			}
		}
	}
	if (check == 0) {
		Se.error_old = Se.before = Se.error_delta = 0;
		C_old.s = 0;
	} else {
		if (!testRunMode && gyroErrResetEnable
				//  && validateGyroReset()
				&& globalState != WALL_OFF && globalState != WALL_OFF_WAIT
				&& globalState != WALL_OFF_WAIT_DIA) {
			Gy.error_old = 0;
			Angle.error_old = 0;
			// angle = ang = 0;
			C_old.g = 0;
		}
	}

	if (check != 0) {
		errorFlg = 1;
		globalError = error;
	} else {
		errorFlg = 0;
	}
	if (check == 2) {
		return error;
	}
	return 2 * error;
}
void resetAngleError(void) {
	char check = 0;
	if (ABS(RS_SEN45.now - RS_SEN45.old) < KIREME_R) {
		if (RS_SEN45.now > R_WALL) {
			if (Front_SEN.now < FRONT_OUT) {
				check++;
			}
		}
	} else {
		if (RS_SEN45.now > (R_WALL + orderup_r)) {
			check++;
		}
	}
	if (ABS(LS_SEN45.now - LS_SEN45.old) < KIREME_L) {
		if (LS_SEN45.now > L_WALL) {
			if (Front_SEN.now < FRONT_OUT) {
				check++;
			}
		}
	} else {
		if (LS_SEN45.now > (L_WALL + orderup_l)) {
			check++;
		}
	}
	if (check > 0) {
		Gy.error_old = 0;
		Angle.error_old = 0;
		angle = 0;
		ang = 0;
	}

}
int gyroReset = 0;

volatile char isControl = 0;
volatile int errorOld_dia = 0;
volatile int errorOld_dia_side = 0;

char validateImgIndex(int a) {
	return 20 <= a && a <= 210;
}

char validateImg90Index(int a) {
	return (0 <= a && a <= 20) || (60 <= a && a <= 130)
			|| (195 <= a && a <= 250);
}
float lastR45Data = 0;
float lastL45Data = 0;
float check_sen_error_dia_side_v2(void) {
	float error = 0;
	float error2 = 0;
	char check = 0;
	char check2 = 0;

	const float diff = *(float *) 1049732;
	const float diff90 = *(float *) 1049736;

	const float diffthread = *(float *) 1049740;
	const float diffthread90 = *(float *) 1049744;

	const float R90_order = *(float *) 1049500;
	const float L90_order = *(float *) 1049504;

	char flag45 = false;
	if (diaStrwallCount_r > 0) {
//		float tmpRightRef = sen_r_dia_img[(int) (img_dist_r)];
//		float tmpRightRef90 = sen_r90_dia[(int) (img_dist_r)];
		float tmpRightRef = getLerpValue(img_dist_r, sen_r_dia_img,
				sizeof(sen_r_dia_img));
		float tmpRightRef90 = getLerpValue(img_dist_r, sen_r90_dia,
				sizeof(sen_r90_dia));
		if (ABS(RS_SEN45.now - RS_SEN45.old) < diff
				&& RS_SEN45.now > (tmpRightRef - diffthread)
				&& validateImgIndex((int) img_dist_r)) {
			flag45 = true;
			error += RS_SEN45.now - tmpRightRef;
			check++;
		}
		if (ABS(RS_SEN2.now - RS_SEN2.old) < diff90
				&& RS_SEN2.now > (tmpRightRef90 - diffthread90)
				&& RS_SEN2.now > R90_order
				&& validateImg90Index((int) img_dist_r)) {
			error2 += RS_SEN2.now - tmpRightRef90;
			check2++;
		}
	}
	if (diaStrwallCount_l > 0) {
//		float tmpLeftRef = sen_l_dia_img[(int) (img_dist_l)];
//		float tmpLeftRef90 = sen_l90_dia[(int) (img_dist_l)];
		float tmpLeftRef = getLerpValue(img_dist_l, sen_l_dia_img,
				sizeof(sen_l_dia_img));
		float tmpLeftRef90 = getLerpValue(img_dist_l, sen_l90_dia,
				sizeof(sen_l90_dia));
		if (ABS(LS_SEN45.now - LS_SEN45.old) < diff
				&& LS_SEN45.now > (tmpLeftRef - diffthread)
				&& validateImgIndex((int) img_dist_l)) {
			error -= LS_SEN45.now - tmpLeftRef;
			check++;
		}
		if (ABS(LS_SEN2.now - LS_SEN2.old) < diff90
				&& LS_SEN2.now > (tmpLeftRef90 - diffthread90)
				&& LS_SEN2.now > L90_order && !flag45
				&& validateImg90Index((int) img_dist_l)) {
			error2 -= LS_SEN2.now - tmpLeftRef90;
			check2++;
		}
	}
	if (check == 0 || check2 == 0) {
		Se2.error_old = Se2.before = Se2.error_delta = 0;
	} else {
		int flag = (int) (*(float *) 1049764);
		if (!testRunMode && gyroErrResetEnable && validateGyroReset()) {
			Gy.error_old = 0;
			Angle.error_old = 0;
			// angle = ang = 0;
			C_old.g = 0;
		}
	}
	isControl = false;
	if (check != 0) {
		errorFlg = 1;
	} else {
		errorFlg = 0;
	}

	if (check > 0 && check2 > 0) {
		float tmpError, tmpError2;
		if (check == 1) {
			tmpError = 2 * error;
		}
		if (check2 == 1) {
			tmpError2 = 2 * error2;
		}
		return (tmpError + tmpError2) / 2;
	}
	if (check == 2) {
		return error;
	} else if (check == 1) {
		return 2 * error;
	}
	if (check2 == 2) {
		return error2;
	} else if (check2 == 1) {
		return 2 * error2;
	}

	return 2 * error;
//	}
}
float RF_WALL_FrontCTRL = 520; //斜め時 姿勢制御閾値(壁ナシ）
float LF_WALL_FrontCTRL = 450; //斜め時 姿勢制御閾値(壁ナシ）

float R_WALL_dia = 400;	//斜め時 横壁のアリ判定
float L_WALL_dia = 400;	//斜め時 横壁のアリ判定

float RF_WALL1 = 210; //斜め時 姿勢制御閾値(壁ナシ）
float LF_WALL1 = 220; //斜め時 姿勢制御閾値(壁ナシ）
#define KIREME_R_DIA 40
#define KIREME_L_DIA 40
float RF_WALL = 175; //斜め時 姿勢制御閾値(壁アリ）
float LF_WALL = 210;  //斜め時 姿勢制御閾値(壁アリ）

float D1 = 0.0001;
float k1 = 76.25;
float k2 = 10370;
float P = 100;
float lastC_s = 0;
float lastC_s2 = 0;

void errorVelocity(void) {
	lastC_s = C.s;
	lastC_s2 = C.s2;

	C.s = 0;
	C.s2 = 0;
	const float CsLimit = *(float *) 1049456;
	const float CsLimitDia = *(float *) 1049460;
//	const float CanlgesLimit = *(float *) 1049464;
	const float CgLimit = *(float *) 1049468;

	if (positionControlValueFlg == 1) {
		if (dia == 0) {
			Se.before = Se.error_now;
			Se.error_now = check_sen_error();

			if (Se.before != 0) {
				Se.error_delta = Se.error_now - Se.before;
			} else {
				Se.error_delta = 0;
			}

			Se.error_old += Se.error_now;
			const char enableSPID = (char) (*(float *) 1049812);
			float C_old2 = C_old.s;
			C_old.s = C.s;

			Gyro.Kp = interp_s_kp(ABS(tgt_v_now));
			Gyro.Ki = interp_s_ki(ABS(tgt_v_now));
			Gyro.Kd = interp_s_kd(ABS(tgt_v_now));

			C.s = Sen.Kp * Se.error_now + Sen.Ki * Se.error_old
					+ Sen.Kd * Se.error_delta;

			if (enableSPID) {
				C.s = (Sen.Kp * Se.error_now + Sen.Ki * Se.error_old
						+ Sen.Kd * Se.error_delta) + (C_old.s - C_old2) * dt;
			}

			if (C.s > CsLimit) {
				C.s = CsLimit;
			} else if (C.s < -CsLimit) {
				C.s = -CsLimit;
			}

		} else if (dia == 1 && Sen_Dia_Side.Kp != 0) {
			Se2.before = Se2.error_now;
			Se2.error_now = check_sen_error_dia_side_v2();

			if (Se2.before != 0) {
				Se2.error_delta = Se2.error_now - Se2.before;
			} else {
				Se2.error_delta = 0;
			}

			Se2.error_old += Se2.error_now;
			char enableSDiaPID = (char) (*(float *) 1049824);
			float C_old2 = C_old.s;
			C_old.s = C.s;
			C.s = Sen_Dia_Side.Kp * Se2.error_now
					+ Sen_Dia_Side.Ki * Se2.error_old;
			if (enableSDiaPID) {
//				C.s = (Sen_Dia_Side.Kp * Se2.error_delta / dt
//						+ Sen_Dia_Side.Ki * Se2.error_now) * dt + C_old.s;
				C.s = (Sen_Dia_Side.Kp * Se2.error_now
						+ Sen_Dia_Side.Ki * Se2.error_old
						+ Sen_Dia_Side.Kd * Se2.error_delta)
						+ (C_old.s - C_old2) * dt;
			}
			if (C.s > CsLimitDia) {
				C.s = CsLimitDia;
			} else if (C.s < -CsLimitDia) {
				C.s = -CsLimitDia;
			}
		}
	} else {
		Se.error_delta = Se.error_old = Se.before = 0;
		Se2.error_delta = Se2.error_old = Se2.before = 0;

		const char shusoku = (char) (*(float *) 1049532);
		const char shusoku_dia = (char) (*(float *) 1049536);
		const float shusoku_jerk = (*(float *) 1049540);
		if (shusoku) {
			if (ABS(lastC_s) < shusoku_jerk) {
				C.s2 = 0;
			} else {
				if (lastC_s > 0) {
					C.s = lastC_s - shusoku_jerk;
				} else {
					C.s = lastC_s + shusoku_jerk;
				}
			}
		} else {
			C.s = 0;
		}
		if (shusoku_dia) {
			if (ABS(lastC_s2) < shusoku_jerk) {
				C.s2 = 0;
			} else {
				if (lastC_s2 > 0) {
					C.s2 = lastC_s2 - shusoku_jerk;
				} else {
					C.s2 = lastC_s2 + shusoku_jerk;
				}
			}
		} else {
			C.s = 0;
		}
		C_old.s = 0;
	}
	if (!enablePWM) {
		C.s = 0;
		C_old.s = 0;
	}

	int isCtrlMode = (int) (*(float *) 1049848);

	if (isCtrlMode) {
		Angle.error_now = (angle - ang);
		if (C.angles > CgLimit || C.angles < -CgLimit) {
		} else {
			Angle.error_old += Angle.error_now;
		}
		if (Gy.before != 0) {
			Angle.error_delta = Angle.error_now - Angle.before;
		} else {
			Angle.error_delta = 0;
		}
		Angle.before = Angle.error_now;

		const char enableGPID = (char) (*(float *) 1049808);
		float C_old2 = C_old.angles;
		C_old.angles = C.angles;

		if (enableGPID) {
			C.angles = (Angles.Kp * Angle.error_now
					+ Angles.Ki * Angle.error_old
					+ Angles.Kd * Angle.error_delta)
					+ (C_old.angles - C_old2) * dt;
		} else {
			C.angles = Angles.Kp * Angle.error_now + Angles.Ki * Angle.error_old
					+ Angles.Kd * Angle.error_delta;
		}
		if (!enablePWM) {
			C.angles = 0;
			C_old.angles = 0;
		}
		if (C.angles > CgLimit) {
			C.angles = CgLimit;
		}
		if (C.angles < -CgLimit) {
			C.angles = -CgLimit;
		}
	} else {
		Gy.error_now = (tgt_w_now - settleGyro);
		// Gy.error_now = (W_now - settleGyro);
		if (C.g > CgLimit || C.g < -CgLimit) {
		} else {
			Gy.error_old += Gy.error_now;
		}
		if (Gy.before != 0) {
			Gy.error_delta = Gy.error_now - Gy.before;
		} else {
			Gy.error_delta = 0;
		}
		Gy.before = Gy.error_now;

		const char enableGPID = (char) (*(float *) 1049808);
		float C_old2 = C_old.g;
		C_old.g = C.g;

		Gyro.Kp = interp_w_kp(ABS(tgt_w_now));
		Gyro.Ki = interp_w_ki(ABS(tgt_w_now));
		Gyro.Kd = interp_w_kd(ABS(tgt_w_now));

		if (enableGPID) {
			C.g = (Gyro.Kp * Gy.error_now + Gyro.Ki * Gy.error_old
					+ Gyro.Kd * Gy.error_delta) + (C_old.g - C_old2) * dt;
		} else {
			C.g = Gyro.Kp * Gy.error_now + Gyro.Ki * Gy.error_old
					+ Gyro.Kd * Gy.error_delta;
		}
		if (!enablePWM) {
			C.g = 0;
			C_old.g = 0;
		}
		if (C.g > CgLimit) {
			C.g = CgLimit;
		}
		if (C.g < -CgLimit) {
			C.g = -CgLimit;
		}
	}

}

float checkDuty(float duty) {
	duty = ABS(duty);
	if (duty > DUTY_MAX) {
		duty = DUTY_MAX;
	}
	return duty;
}
void changeRotation(float duty, char RorL) {
	if (RorL == L) {
		if (duty > 0.0) {
			CW_R();
		} else {
			CCW_R();
		}
	} else if (RorL == R) {
		if (duty > 0.0) {
			CCW_L();
		} else {
			CW_L();
		}
	}
}
float getRpm(char RorL) {
	return RorL == R ? rpmR : rpmL;
}

float feadforward(char RorL) {
	float tmp_wheel = WHEEL;

	if (fanStart == true) {
		if (fanMode != SearchRun) {
			tmp_wheel = (*(float *) 1050060) / 2000;
		}
	}

	return Mass * tgt_accl_ff * tmp_wheel * Resist / (GEAR * Km * 1000);
}
float feadforward_para(char RorL) {
	float tmp_wheel = WHEEL;

	if (fanStart == true) {
		if (fanMode != SearchRun) {
			tmp_wheel = (*(float *) 1050060) / 2000;
		}
	}
	if (RorL == R) {
		return Lm * (-tgt_alpha_ff) * tmp_wheel * Resist / (GEAR * Km) / Tread;
	}
	return Lm * tgt_alpha_ff * tmp_wheel * Resist / (GEAR * Km) / Tread;
}

float FF_calc(char RorL, float w, float al) {
	// return 0;

	float tmp_tire = TIRE;

	if (fanStart == true) {
		if (fanMode != SearchRun) {
			tmp_tire = (*(float *) 1050060);
		}
	}

	float rpm = getRpm(RorL);
	float resist_str = 0;  //friction_str ? Resist * friction / Km : 0;
	float resist_roll = friction_roll ? (Resist * friction2 / Km) : 0;
	float whrlm = 0;

	if (RorL == R) {
		rpm = (tgt_v_now_ff / 1000 - Tread / 2 * tgt_w_now_ff) / (60 * tmp_tire / 2)
				* 1000;
	} else {
		rpm = (tgt_v_now_ff / 1000 + Tread / 2 * tgt_w_now_ff) / (60 * tmp_tire / 2)
				* 1000;
	}

	float counterV = (Ke + friction) * rpm;
	float ff = feadforward(RorL);
	float ffp = feadforward_para(RorL);

	if (V_now == 0) {
		resist_str = 0;
	}
	if (tgt_w_now == 0) {
		resist_roll = 0;
	}
	if (RorL == R) {
		if (rotate_r) {
			return ((ff + ffp) / 2 + counterV + resist_roll) + whrlm;
		} else {
			return ((ff + ffp) / 2 + counterV - resist_roll) - whrlm;
		}
	} else {
		if (rotate_l) {
			return ((ff + ffp) / 2 + counterV + resist_roll) + whrlm;
		} else {
			return ((ff + ffp) / 2 + counterV - resist_roll) - whrlm;
		}
	}
	return ((ff + ffp) / 2 + Ke * rpm + resist_str) + whrlm;
}
float FB_calc(char RorL) {
	float duty;
	if (RorL == R) {
		duty = Vel_r.Kp * (Vr.error_now)
				+ Vel_r.Ki * (Vr.error_now + Vr.error_old)
				+ Vel_r.Kd * (Vr.error_now - Vr.error_delta);
		return duty;
	}
	duty = Vel_l.Kp * (Vl.error_now) + Vel_l.Ki * (Vl.error_now + Vl.error_old)
			+ Vel_l.Kd * (Vl.error_now - Vl.error_delta);
	return duty;
}

float FB_velocity() {
	const char enableVPID = (char) (*(float *) 1049804);
	float C_old2 = C_old.v;
	C_old.v = C.v;

	Vel.Kp = interp_v_kp(V_now);
	Vel.Ki = interp_v_ki(V_now);
	Vel.Kd = interp_v_kd(V_now);
	C.v = (Vel.Kp * Velocity.error_now + Vel.Ki * Velocity.error_old
			+ Vel.Kd * Velocity.error_delta);
	if (enableVPID) {
		C.v = (Vel.Kp * Velocity.error_now + Vel.Ki * Velocity.error_old
				+ Vel.Kd * Velocity.error_delta) + (C_old.v - C_old2) * dt;
	}
	if (!enablePWM) {
		C.v = 0;
		C_old.v = 0;
	}
	return C.v;
}

float FB_distance() {
	float dist_before = (img_distance - distance);
	return 0;
}

float FB_calc_straight() {
	C.v = FB_velocity() + FB_distance();

	return C.v;
}

float FB_calc_pararell() {
	float tmpC_s = C.s;
	float tmpC_s2 = C.s2;
	float y_diff = C.y_diff;
	const float rate_limit = (*(float *) 1049512);
	const float rate_limit_dia = (*(float *) 1049516);
	const char rate_limit_override = (char) (*(float *) 1049520);

	const float rate_jerk = (*(float *) 1049524);
	const float rate_jerk_dia = (*(float *) 1049528);

	const float diff_y_gain_p = (*(float *) 1050420);

	C.y_diff = 0;
	if(globalState==SLA_TURN){
		C.y_diff = diff_y_gain_p * ego_data_out.trj_diff.y;
	}

	if (ABS(lastC_s - C.s) > rate_limit) {
		tmpC_s = 0;
		if (rate_limit_override) {
			if ((C.s - lastC_s) > rate_limit) {
				C.s = lastC_s + rate_jerk;
			} else {
				C.s = lastC_s - rate_jerk;
			}
		}
	}
	if (ABS(lastC_s2 - C.s2) > rate_limit_dia) {
		tmpC_s2 = 0;
		if (rate_limit_override) {
			if ((C.s2 - lastC_s2) > rate_limit) {
				C.s2 = lastC_s2 + rate_jerk_dia;
			} else {
				C.s2 = lastC_s2 - rate_jerk_dia;
			}
		}
	}

	if (gyroKeepZero) {
		return C.s + C.s2;
	}
	return C.angles + C.g + tmpC_s + tmpC_s2 + C.y_diff;
}

void set_predict_tgt_data() {
	for (int i = 0; i < PREDICT_SIZE; i++) {
		target_data_list[i].accl = target_data.accl;
		target_data_list[i].alpha = target_data.alpha;
		target_data_list[i].decel = target_data.decel;
		target_data_list[i].end_v = target_data.end_v;
		target_data_list[i].end_w = target_data.end_w;
		target_data_list[i].tgt_angle = target_data.tgt_angle;
		target_data_list[i].tgt_dist = target_data.tgt_dist;
		target_data_list[i].v_max = target_data.v_max;
		target_data_list[i].w_max = target_data.w_max;

		ego_data_in_list[i].accl = ego_data_in.accl;
		ego_data_in_list[i].alpha = ego_data_in.alpha;
		ego_data_in_list[i].ang = ego_data_in.ang;
		ego_data_in_list[i].dist = ego_data_in.dist;
		ego_data_in_list[i].img_ang = ego_data_in.img_ang;
		ego_data_in_list[i].img_dist = ego_data_in.img_dist;
		ego_data_in_list[i].pivot_state = ego_data_in.pivot_state;
		ego_data_in_list[i].sla_param.base_alpha =
				ego_data_in.sla_param.base_alpha;
		ego_data_in_list[i].sla_param.base_time =
				ego_data_in.sla_param.base_time;
		ego_data_in_list[i].sla_param.counter = ego_data_in.sla_param.counter;
		ego_data_in_list[i].sla_param.limit_time_count =
				ego_data_in.sla_param.limit_time_count;
		ego_data_in_list[i].sla_param.pow_n = ego_data_in.sla_param.pow_n;
		ego_data_in_list[i].sla_param.state = ego_data_in.sla_param.state;
		ego_data_in_list[i].v = ego_data_in.v;
		ego_data_in_list[i].w = ego_data_in.w;
	}
}

void dutyCalcuration2(void) {
	float dutyR = 0, dutyL = 0;
	float FB_straight_duty = FB_calc_straight();
	float FB_pararell_duty = FB_calc_pararell();
	float w, al;

	ffR = FF_calc(R, w, al);
	ffL = FF_calc(L, w, al);

	dutyR = (ffL + FB_straight_duty + FB_pararell_duty) / battery * 100;
	dutyL = (ffR + FB_straight_duty - FB_pararell_duty) / battery * 100;

	SeFrnt.error_old = 0;
	SeFrntL.error_old = 0;

	Duty_r = dutyR;
	Duty_l = dutyL;
	changeRotation(dutyR, R);	//R_CW/CCW
	changeRotation(dutyL, L);	//L_CW/CCW

	failCheck(dutyR, dutyL);
	dutyR = checkDuty(dutyR);
	dutyL = checkDuty(dutyL);

	gra_r = (long) (dutyR * M_CYCLE / 100.0);
	gra_l = (long) (dutyL * M_CYCLE / 100.0);

	if (enablePWM) {
		GPT0.GTCCRA = GPT0.GTCCRC = gra_l;
		GPT1.GTCCRA = GPT1.GTCCRC = gra_r;
	} else {
		GPT0.GTCCRA = GPT0.GTCCRC = GPT1.GTCCRA = GPT1.GTCCRC = 0;
	}
}
void mpc_tgt_calc_step(RT_MODEL_mpc_tgt_calc_T * const mpc_tgt_calc_M,
		t_tgt *mpc_tgt_calc_U_tgt, t_ego *mpc_tgt_calc_U_ego,
		int32_T mpc_tgt_calc_U_mode, t_ego *mpc_tgt_calc_Y_Out1);

//物理量ベース計算
void Physical_Basement(void) {
	enc_r = MTU1.TCNT - 30000;
	enc_l = MTU2.TCNT - 30000;
	if (os_escape) {
		MTU2.TCNT = MTU1.TCNT = 30000;
	}
	if (alphaMode == 1) {
		if (dt * sinCount / slaTerm < 2.0) {
			alpha = alphaTemp * Et2(dt * sinCount, slaTerm, etN);
		} else {
			alpha = 0;
		}
	}
	if (dia == 1 && originalDiaMode == true && checkStablly() == true) {
		if (targetVelocity > V_now) {
			if (tmpDiac < 0) {
				acc = 0;
			}
		}
	}

	char gyroMode2 = (char) (*(float *) 1049800);
	if (gyroMode2) {
		ang += settleGyro * dt;
	} else {
		ang += G.now * dt;
	}
	distance += (V_Enc.r + V_Enc.l) * dt / 2;

	ego_data_in.ang = ang;
	ego_data_in.dist = distance;
	ego_data_in.slip_point.w = settleGyro;

	target_data.accl_param.limit = *(float *) 1050412;
	target_data.accl_param.n = *(float *) 1050416;
	target_data.slip_gain = *(float *) 1050424;

	// set_predict_tgt_data();

	mpc_tgt_calc_step(&mpc_tgt_calc_error_status, &target_data, &ego_data_in,
			mpc_tgt_calc_mode, 1, &ego_data_out);


	int predict_time = (int) (*(float *) 1050428);
	if (predict_time < 1) {
		predict_time = 1;
	}
	mpc_tgt_calc_step(&mpc_tgt_calc_error_status, &target_data, &ego_data_in,
			mpc_tgt_calc_mode, (int)predict_time, &ego_data_out2);

	ego_data_in.accl = ego_data_out.accl;
	ego_data_in.alpha = ego_data_out.alpha;
//	ego_data_in.ang = ego_data_out.ang;
//	ego_data_in.dist = ego_data_out.dist;
	ego_data_in.pivot_state = ego_data_out.pivot_state;
	ego_data_in.sla_param = ego_data_out.sla_param;
	ego_data_in.state = ego_data_out.state;
	ego_data_in.v = ego_data_out.v;
	ego_data_in.w = ego_data_out.w;
	ego_data_in.sla_param.state = ego_data_out.sla_param.state;
	ego_data_in.sla_param.counter = ego_data_out.sla_param.counter;
	ego_data_in.sla_param.state = ego_data_out.sla_param.state;

	ego_data_in.img_ang = ego_data_out.img_ang;
	ego_data_in.img_dist = ego_data_out.img_dist;

	ego_data_in.slip_point.slip_angle = ego_data_out.slip_point.slip_angle;

	tgt_v_now = ego_data_out.v;
	tgt_w_now = ego_data_out.w;
	tgt_accl = ego_data_out.accl;
	tgt_alpha = ego_data_out.alpha;

	V_old = V_now;
	W_old = W_now;

	V_now = ego_data_in.v;
	W_now = ego_data_in.w;
	acc = ego_data_in.accl;
	alpha = ego_data_in.alpha;
	angle = ego_data_out.ang;
	img_distance = ego_data_in.dist;

	tgt_v_now_ff = ego_data_out2.v;
	tgt_w_now_ff = ego_data_out2.w;
	tgt_accl_ff = ego_data_out2.accl;
	tgt_alpha_ff = ego_data_out2.alpha;

	enc_to_vel();
	errorVelocity();

	if (enableSystemIdentification) {

		gra_r = (short) (ABS(*(float *) 1049472) / battery * M_CYCLE);
		gra_l = (short) (ABS(*(float *) 1049476) / battery * M_CYCLE);

		float dutyR = gra_r * 100 / M_CYCLE;
		float dutyL = gra_l * 100 / M_CYCLE;

		changeRotation(*(float *) 1049472, R);	//R_CW/CCW
		changeRotation(*(float *) 1049476, L);	//L_CW/CCW

		if (enablePWM) {
			GPT0.GTCCRA = GPT0.GTCCRC = gra_l;
			GPT1.GTCCRA = GPT1.GTCCRC = gra_r;
		} else {
			GPT0.GTCCRA = GPT0.GTCCRC = GPT1.GTCCRA = GPT1.GTCCRC = 0;
		}
	} else {
		dutyCalcuration2();
	}
}
#endif /* PHYSICALBASEMENT_H_ */
