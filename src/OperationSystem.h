/*
 * OperationSystem.h
 *
 *  Created on: 2013/11/22
 *      Author: Naoto
 */

#ifndef OPERATIONSYSTEM_H_
#define OPERATIONSYSTEM_H_

void operation();
char encorderOperation();

void sensourCheck() {
	positionControlValueFlg = 1;
	ledOn = 1;
	resetGyroParam();
	resetAllData();
//	mtu_start();
	V_now = 0;
	while (Swich == 1) {
		myprintf("%d	%d	%d	%d	%d\n", (int) LS_SEN45.now, (int) 0,
				(int) Front_SEN.now, (int) RS_SEN45.now, (int) (Se.error_now));
		cmt_wait(100);
//		SCI_SCI_printf("%d %d	%d	%d\n", MTU2.TCNT, MTU1.TCNT, (int) V_Enc.l,
//				(int) V_Enc.r);
	}
	while (Swich == 0)
		;
	cmt_wait(500);
	ledOn = 0;
}
void testRun() {

	float test_acc = *(float *) 1049268;
	float test_max_v = *(float *) 1049272;
	float test_dia = *(float *) 1049276;
	float gyroReset = *(float *) 1049508;
	float velocity = test_max_v;
	float accele = test_acc;
	float diaccele = test_acc;
	float vMax = 100;

	gyroZeroCheck(true);
	cmt_wait(500);

	float isgyroResetDisable = *(float *) 1049508;

	if (velocity > 2000 || isgyroResetDisable) {
		startVacume2(99);
		cmt_wait(200);
	}

	if (activate_TRANS_AM) {
		RS_SEN45.ref = *(float *) 1049944;
		LS_SEN45.ref = *(float *) 1049948;
		R_WALL_OFF = *(float *) 1049952;
		FRONT_WALL_ON = *(float *) 1049956;
	}

	cc = 1;
	gyroOn = 1;
	V_now = 0;

	mtu_start();
	dia = test_dia == 1;
	enableSensorCtrl = true;
	if (dia == 1) {
		if (gyroReset > 0) {
			testRunMode = true;
		}
		float targetDistDia = *(float *) 1049300;
		realRun(velocity, accele, diaccele, targetDistDia, vMax, NULL_FILTER);
		testRunMode = false;

	} else {
		float targetdist = *(float *) 1049296;
		realRun(velocity, accele, diaccele, targetdist, vMax, NULL_FILTER);
	}
	positionControlValueFlg = 1;
	gyroKeepZero = 1;
	C_old.g = C.g = C.angles = 0;
//	enablePWM = false;
	mtu_stop2();
	gyroKeepZero = 0;
	enableSensorCtrl = false;
	positionControlValueFlg = 0;
	logOutPut();
}
void testRun1(float v, float accele, float diaccele, float dist, float vM,
		char b1) {
	char check = 1;
	gyroZeroCheck(true);
	if (b1) {
		startVacume();
		cmt_wait(1000);
	}
	cc = 1;
	gyroOn = 1;
	V_now = 0;
	mtu_start();
	dia = 0;
	check = realRun(v, accele, diaccele, dist - 180, vM, NULL_FILTER);
	if (check == 1) {
		running(100, -diaccele, 180, 1);
	}
	cc = 0;
	mtu_stop();
	stopVacume();
}
void testRun2(float v, float accele, float diaccele, float dist, float vM) {
	char check = 1;
	gyroZeroCheck(false);
	cc = 1;
	gyroOn = 1;
	V_now = 0;
	mtu_start();
	dia = 0;
	check = realRun(v, accele, diaccele, dist - 90, vM, NULL_FILTER);
	if (check == 1) {
		running(100, -diaccele, 90, 1);
	}
	cc = 0;
	mtu_stop();
	stopVacume();
}

void inputNaiperTurnAll500() {
	float v = 500;
	set_param_map(Normal, v);
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, v);
	set_param_map(Dia135, v);
	set_param_map(Dia90, v);
}

void inputNaiperTurnAll1000() {
	float v = 1000;
	set_param_map(Normal, v);
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, v);
	set_param_map(Dia135, v);
	set_param_map(Dia90, v);
}

void inputNaiperTurnAll1500() {
	float v = 1500;
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, v);
	set_param_map(Dia135, v);
	set_param_map(Dia90, v);
}

void inputNaiperTurnAll1700() {
	float v = 1700;
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, v);
	set_param_map(Dia135, v);
	set_param_map(Dia90, v);
}
void inputNaiperTurnAll1800() {
	float v = 1800;
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, v);
	set_param_map(Dia135, v);
	set_param_map(Dia90, v);
}
void inputNaiperTurnAll1900() {
	float v = 1900;
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, v);
	set_param_map(Dia135, v);
	set_param_map(Dia90, v);
}
void inputNaiperTurnAll1950() {
	float v = 1950;
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, v);
	set_param_map(Dia135, v);
	set_param_map(Dia90, v);
}
void inputNaiperTurnAll2000() {
	float v = 2000;
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, v);
	set_param_map(Dia135, v);
	set_param_map(Dia90, 1900);
}
void inputNaiperTurnAll2050() {
	float v = 2050;
	set_param_map(Large, v);
	set_param_map(Orval, v);
	set_param_map(Dia45, 2000);
	set_param_map(Dia135, 2000);
	set_param_map(Dia90, 1900);
}
void inputNaiperTurnAll2100() {
	inputNaiperTurnAll2050();
	float v = 2100;
	set_param_map(Large, v);
}
/**
 * 	t_radiusData large;
 t_radiusData orval;
 t_radiusData dia45;
 t_radiusData dia135;
 t_radiusData dia90;
 t_radiusData kojima;
 */

void callParam(float vMax) {
	if (vMax == 500) {
		set_param_map(Normal, vMax);
	} else if (vMax == 1000) {
		set_param_map(Normal, vMax);
	}
	set_param_map(Large, vMax);
	set_param_map(Orval, vMax);
	set_param_map(Dia45, vMax);
	set_param_map(Dia135, vMax);
	set_param_map(Dia90, vMax);
}

void testSlalom3() {
	float vMax = *(float *) 1049264;
//	char test_dia = (char) (*(float *) 1049276);
	char turnType = (char) (*(float *) 1049280);
	char test_sla_scenario2 = (char) (*(float *) 1049284);
	float isgyroResetDisable = *(float *) 1049508;

	char test_sla_return_flg = (char) (*(float *) 1049288);
	char test_sla_return_scenario = (char) (*(float *) 1049292);

	float accele = *(float *) 1049268;
	float diaccele = *(float *) 1049268;

	char test_dia = (char) (*(float *) 1049276);
	float start_offset = *(float *) 1049496;
	if (activate_TRANS_AM) {
		RS_SEN45.ref = *(float *) 1049944;
		LS_SEN45.ref = *(float *) 1049948;
		R_WALL_OFF = *(float *) 1049952;
		FRONT_WALL_ON = *(float *) 1049956;
		start_offset = *(float *) 1049548;
	}
	callParam(vMax);

	char RorL = eigherRightLeft() == Right ? R : L;
	float tempdist1 = *(float *) 1049960;
	myprintf("tempdist1	%f	%d\r\n", tempdist1, 1049960);
	wall_off_limit = wall_off_limit_d = tempdist1;
	motionCheck();
	cmt_wait(500);
	gyroZeroCheck(false);
	resetAllData();
	mtu_start();

	readGyroParam();
	readAngleParam();
	readVelocityGain();
//	resetVelocityGain();

	if (vMax >= 1900 || isgyroResetDisable) {
		TRANSAM = true;
		fanMode = FastRun2;
		startVacume2(90);
	} else if (vMax >= 1200) {
		TRANSAM = true;
		fanMode = FastRun;
		startVacume2(90);
	}

	if (!test_dia) {
		Sen_Dia_Side.Kp = Sen_Dia_Side.Ki = Sen_Dia_Side.Kd = 0.0;
	}

	ang = 0;
	angle = 0;
	Gy.error_old = 0;
	mode_FF = 1;

//	dia = (char) test_dia == 1;

	save_high_param();

	globalSkipFront = test_sla_scenario2;

	char type = turnType;
	logs = 0;
	time = 0;
	cc = 1;
	logs = 0;

	if (globalSkipFront) {
		float tmpdist = start_offset + getFirstFront(type);
		float tmpAccel = vMax * vMax / 2 / tmpdist;
		testRunMode = true;
		if (tmpAccel > accele) {
			tmpAccel += 1000;
			realRun(vMax, tmpAccel, tmpAccel, tmpdist, vMax, NULL_FILTER);
		} else {
			realRun(vMax, accele, diaccele, tmpdist, vMax, NULL_FILTER);
		}
		testRunMode = false;
	} else {
		realRun(vMax, accele, diaccele, 180.0 + start_offset, vMax,
				NULL_FILTER);
	}
	if (!globalSkipFront) {
		if (isgyroResetDisable > 0) {
			gyroErrResetEnable = false;
			Sen.Kp = Sen.Ki = Sen.Kd = 0.0;
			Sen_Dia.Kp = Sen_Dia.Ki = Sen_Dia.Kd = 0;
		}
		wallOff(RorL, true);
	}
	testRunMode = true;
	slalom(RorL, type, vMax, vMax, 0);

	gyroErrResetEnable = true;
	// Sen.Kp = *(float *) 1049376;
	// Sen.Ki = *(float *) 1049380;
	// Sen.Kd = *(float *) 1049384;
	Sen_Dia_Side.Kp = *(float *) 1049388;
	Sen_Dia_Side.Ki = *(float *) 1049392;
	Sen_Dia_Side.Kd = *(float *) 1049396;

	if (!test_dia) {
		Sen_Dia_Side.Kp = Sen_Dia_Side.Ki = Sen_Dia_Side.Kd = 0.0;
	}
	peekRight = peekLeft = 0;

//	Sen.Kp = Sen.Ki = Sen.Kd = 0.0;

	if (test_sla_return_flg) {
		wallOff(RorL == R ? L : R, !dia);
		slalom(RorL == R ? L : R, test_sla_return_scenario, vMax, vMax, 0);
	}
	cc = 0;

	// testRunMode = true;
	if (isgyroResetDisable > 0) {
		gyroErrResetEnable = false;
		Sen.Kp = Sen.Ki = Sen.Kd = 0.0;
		Sen_Dia.Kp = Sen_Dia.Ki = Sen_Dia.Kd = 0;
	}

	if (test_dia) {
		float targetDistDia = *(float *) 1049300;
		float test_max_v = *(float *) 1049272;
		realRun(test_max_v, accele, diaccele, targetDistDia, 50, NULL_FILTER);
	} else {
		if (dia == 1) {
			realRun(vMax, accele, diaccele, 180.0 * ROOT2, 50, NULL_FILTER);
		} else {
			realRun(vMax, accele, diaccele, 180.0 * 1.0, 50, NULL_FILTER);
		}
	}
//	running(50, 0, 90, 0);
	mtu_stop2();
	testRunMode = false;
	stopVacume();
	cmt_wait(500);
	logOutPut();
}
void testNormalSlalom() {

	char testCmd = (char) (*(float *) 1049260);
	char test_front_ctrl = testCmd == 4;

	float accele = *(float *) 1049324;
	float diaccele = *(float *) 1049328;
	float vMax = *(float *) 1049320;
	float test_dia = *(float *) 1049276;

	char RorL = eigherRightLeft() == Right ? R : L;

	motionCheck();
	cmt_wait(500);
	gyroZeroCheck(false);

	if (vMax >= 1000) {
		fanMode = SearchRun;
		startVacume2(70);
		set_param_map(Normal, 1000);
	} else if (vMax >= 500) {
		set_param_map(Normal, 500);
	}

	mtu_start();
	readGyroParam();
	readVelocityGain();

	ang = angle = Gy.error_old = 0;
	mode_FF = 1;
	save_high_param();

	testRunMode = true;

	if (test_front_ctrl) {
		realRun(vMax, accele, diaccele, 180.0 * 1, vMax, NULL_FILTER);
		frontCtrl();
		realRun(vMax, accele, diaccele, 90.0 * 1, 50, NULL_FILTER);
		resetAllData();
		mtu_stop2();
	} else {
//		runForWallOff(vMax, accele, 270, 1);

		if (test_dia > 0) {
			runForWallOff(vMax, accele, 360, 1);
		} else {
			realRun(vMax, accele, diaccele, 180.0 * 1, vMax, NULL_FILTER);
		}

		logs = 0;
		time = 0;
		cc = 1;
		logs = 0;
		slalom(RorL, Normal, vMax, vMax, 0);
//		slalom(RorL, Normal, vMax, vMax, 0);

		char test_turn_times = (char) (*(float *) 1049308);
		if (test_turn_times > 0) {
			for (char i = 0; i < test_turn_times; i++) {
				slalom(RorL == R ? L : R, Normal, vMax, vMax, 0);
			}
		}
		cc = 0;
		realRun(vMax, accele, diaccele, 180.0 * 1, 50, NULL_FILTER);
	}
	C_old.g = 0;
	C_old.s = 0;
	mtu_stop2();
	stopVacume();
	cmt_wait(500);
	logOutPut();
}

void testNormalSlalom2() {

	char testCmd = (char) (*(float *) 1049260);
	char test_front_ctrl = testCmd == 4;

	float accele = *(float *) 1049324;
	float diaccele = *(float *) 1049328;
	float vMax = *(float *) 1049320;
	float test_dia = *(float *) 1049276;

	char RorL = eigherRightLeft() == Right ? R : L;

	motionCheck();
	cmt_wait(500);
	gyroZeroCheck(false);

	if (vMax >= 1000) {
		fanMode = SearchRun;
		startVacume2(70);
		set_param_map(Normal, 1000);
	} else {
		set_param_map(Normal, 500);
	}

	mtu_start();
	readGyroParam();
	readVelocityGain();

	ang = angle = Gy.error_old = 0;
	mode_FF = 1;
	save_high_param();

	testRunMode = true;

	if (test_front_ctrl) {
		realRun(vMax, accele, diaccele, 180.0 * 1, vMax, NULL_FILTER);
		frontCtrl();
		realRun(vMax, accele, diaccele, 90.0 * 1, 50, NULL_FILTER);
		resetAllData();
		mtu_stop2();
	} else {

		if (test_dia > 0) {
			runForWallOff(vMax, accele, 360, 1);
		} else {
			realRun(vMax, accele, diaccele, 180.0 * 1, vMax, NULL_FILTER);
		}

		logs = 0;
		time = 0;
		cc = 1;
		logs = 0;
		slalom(RorL, Normal, vMax, vMax, 0);
//		slalom(RorL, Normal, vMax, vMax, 0);

		char test_turn_times = (char) (*(float *) 1049308);
		if (test_turn_times > 0) {
			for (char i = 0; i < test_turn_times; i++) {
				slalom(RorL == R ? R : L, Normal, vMax, vMax, 0);
			}
		}
		cc = 0;
		realRun(vMax, accele, diaccele, 180.0 * 1, 50, NULL_FILTER);
	}
	mtu_stop2();
	stopVacume();
	cmt_wait(500);
	logOutPut();
}
void testWallOff() {
	float test_acc = *(float *) 1049268;
	float test_max_v = *(float *) 1049264;
	float test_dia = *(float *) 1049276;

	float maxVelocity = test_max_v;
	float vMax = maxVelocity;

	float accele = test_acc;
	float diaccele = test_acc;

	wall_off_limit = wall_off_limit_d = 400;
	inputNaiperTurnAll1800();
	char RorL = eigherRightLeft() == Right ? R : L;

	motionCheck();
	cmt_wait(500);
	gyroZeroCheck(false);
	mtu_start();
	readGyroParam();
	readVelocityGain();

	if (vMax > 1200) {
		startVacume2(70);
	}

	if (activate_TRANS_AM) {
		RS_SEN45.ref = *(float *) 1049944;
		LS_SEN45.ref = *(float *) 1049948;
		R_WALL_OFF = *(float *) 1049952;
		FRONT_WALL_ON = *(float *) 1049956;
		// start_offset = *(float *) 1049548;
	}
	Sen_Dia_Side.Kp = Sen_Dia_Side.Ki = Sen_Dia_Side.Kd = 0.0;
	ang = angle = Gy.error_old = 0;
	mode_FF = 1;
	dia = test_dia == 1;

	logs = time = 0;
	cc = 1;
	realRun(maxVelocity, accele, diaccele, 200.0, vMax, NULL_FILTER);
//	runForWallOff(maxVelocity, accele, 180, 1);

	wallOff(RorL, !dia);

	if (dia) {
		realRun(vMax, accele, diaccele, 90.0 * ROOT2, 50, NULL_FILTER);
	} else {
		realRun(vMax, accele, diaccele, 90.0 * 1, 50, NULL_FILTER);
	}
	mtu_stop2();
	stopVacume();
	cmt_wait(500);
	logOutPut();
}

void testCalibrate() {

	float test_acc = *(float *) 1049268;
	float test_max_v = *(float *) 1049272;
	float test_dia = *(float *) 1049276;

	float velocity = test_max_v;
	float accele = test_acc;
	float diaccele = test_acc;
	float vMax = 100;

	char RorL = eigherRightLeft() == Right ? L : R;

	gyroZeroCheck(true);
	cmt_wait(500);
	fanMode = SearchRun;
	startVacume2(70);
	cmt_wait(200);
	cc = 1;
	gyroOn = 1;
	V_now = 0;

	mtu_start();
	dia = test_dia == 1;
	enableSensorCtrl = true;
	float targetdist = *(float *) 1049296;
	realRun(velocity, accele, diaccele, targetdist, vMax, NULL_FILTER);

	mtu_stop();

	enableSensorCtrl = false;

	calibrateRun(RorL);

	logOutPut();
}

void testWallOffSeach() {

	float vMax = *(float *) 1049320;
	float accele = *(float *) 1049324;
	float diaccele = *(float *) 1049324;

	motionCheck();
	cmt_wait(500);
	gyroZeroCheck(false);
	mtu_start();
	readGyroParam();
	readVelocityGain();
	globalState = STRAIGHT;
	if (vMax >= 1000) {
		fanMode = SearchRun;
		startVacume2(70);
	}

	ang = 0;
	angle = 0;
	Gy.error_old = 0;
	mode_FF = 1;
	cc = 1;
//	realRun(vMax, accele, diaccele, 90.0 * 1, 50, NULL_FILTER);
	runForWallOff(vMax, accele, 360, 1);
	realRun(vMax, accele, diaccele, 90.0 * 1, 50, NULL_FILTER);
	mtu_stop2();
	stopVacume();
	cmt_wait(500);
	logOutPut();
}

void testRoll(char r) {

	float test_turn_angle = *(float *) 1049304;
	char test_turn_times = (char) (*(float *) 1049308);
	float test_turn_acc = *(float *) 1049312;
	float test_turn_w = *(float *) 1049316;

	if (test_turn_times == 0) {
		test_turn_times = 1;
	}

	r = eigherRightLeft() == Right ? R : L;
	gyroZeroCheck(true);

	gyroZeroCheck(false);
	float test_dia = *(float *) 1049276;
	if (test_dia >= 1) {
		fanMode = CtrlFan2;
		startVacume2(70);
	}

	for (char a = 0; a < test_turn_times; a++) {
		cmt_wait(150);
		cc = 1;
		gyroRollTest(r == R ? L : R, test_turn_angle, test_turn_w,
				test_turn_acc);
//		gyroZeroCheck(false);
		cc = 0;
	}
	mtu_stop2();
	logOutPut();
}

void searchMode(char X, char Y) {
	char x1 = 0, y1 = 0;
	char check = 0;
	x = 0;
	y = 0;
	next_dir = North;
	gyroZeroCheck(true);
	cmt_wait(500);
	now_dir = North;
//	check = Adachi2(X, Y, 1);
	if (check == 0) {
		return;
	}
	cmt_wait(500);
	roll(L, 180, 5, 30);
	x1 = x;
	y1 = y;
	now_dir = direction(now_dir, Back);
	next_dir = now_dir;
	x = x1;
	y = y1;
	cmt_wait(500);
	gyroZeroCheck(false);
//	Adachi(0, 0);
}

char isFullMaze(char X, char Y) {
	return ((X == 7 && Y == 7) || (X == 8 && Y == 7) || (X == 7 && Y == 8)
			|| (X == 8 && Y == 8));
}

void flushcheckQ() {
	for (int i = 0; i < checkQlength; i++) {
		checkQ[i] = 0;
	}
}

void printRealPath();
char makePath3(char goalX, char goalY, char isFull) {
	Dia2 = Dia - minus + 2;
	Dia3 = Dia - minus + 1;
	St2 = St1 - minus - 1;
	St3 = St1 - minus - 2;
	x = 0;
	y = 0;
	now_dir = North;
	vetorPathCreate(goalX, goalY, isFull);
	drawChangePathRoot(goalX, goalY, isFull);
	largePath(true);
	diagonalPath(isFull, true);
	return countPathLength2();
}
char makePath4() {
	Dia2 = Dia - minus + 2;
	Dia3 = Dia - minus + 1;
	St2 = St1 - minus;
	St3 = St1 - minus - 1;
	vetorPathCreate(0, 0, false);
	largePath(true);
	diagonalPath(false, true);
	return true;
}
void makePath(char goalX, char goalY, char isFull, float vmax, float acc,
		float diac) {
	flushcheckQ();
	printMap();
	printMap2();

	pathVmax = vmax;
	pathAcc = acc;
	pathDiac = diac;

	if (cirquitMode == false) {
		minus = 4;
		int minus3 = makePath3(goalX, goalY, isFull);
	}
	printRealPath();
	double time = getGoalTime(true);
	myprintf("time = %f\r\n", time);

}
void makePath2() {
	flushcheckQ();
	if (cirquitMode == false) {
		minus = 4;
		makePath4();
	}
	printRealPath();
}
void testVacume() {
	gyroZeroCheck(true);
	cmt_wait(500);
	mtu_start();
	readVelocityGain();
	resetGyroParam();
	startVacume();
	cmt_wait(50000);
	stopVacume();
	mtu_stop();
}
void testVacume2(int duty) {
	gyroZeroCheck(true);
	cmt_wait(500);
//	mtu_start();
//	readVelocityGain();
//	resetGyroParam();
	startVacume2(duty);
	fanMode = TestRun;
//	cmt_wait(15000);
	int mode = 0;
	while (PushCenter) {

		if (!PushTop) {
			mode++;
		}
		if (!PushBottom) {
			mode--;
		}

		if (mode > 6) {
			mode = 0;
		}

		if (mode < 0) {
			mode = 0;
		}
		ledHex(mode);
		if (mode == 0) {
			myVacumeDuty = 4;
		} else if (mode == 1) {
			myVacumeDuty = 5;
		} else if (mode == 2) {
			myVacumeDuty = 6;
		} else if (mode == 3) {
			myVacumeDuty = 7;
		} else if (mode == 4) {
			myVacumeDuty = 8;
		} else if (mode == 5) {
			myVacumeDuty = 9;
		} else if (mode == 6) {
			myVacumeDuty = 9;
		}

		while (!PushTop || !PushBottom)
			;
	}

	for (int i = 0; i < 200; i++) {
		myprintf("%d %f \r\n", (int) (FAN_AMP / battery * FAN_CYCLE), battery);
		cmt_wait(100);

	}
	stopVacume();
	mtu_stop();
}
char switchOn() {
	return !Swich;
}

char modeCheck() {
	char swTop = !PushTop;
	char swBottom = !PushBottom;
	char swRight = !PushRight;
	char swLeft = !PushLeft;
	char swCenter = !PushCenter;
	return swTop + swBottom * 2 + swLeft * 4 + swRight * 8 + swCenter * 16;
}
char encorderOperation() {
	volatile signed char mode = SEARCH;
	globalState = MODE_SELECT;
	os_escape = 0;
	MTU1.TCNT = 4500;
	MTU2.TCNT = 30000;
	MTU.TSTRA.BIT.CST4 = 0;
	while (1) {
		char swTop = !PushTop;
		char swBottom = !PushBottom;
		char swRight = !PushRight;
		char swLeft = !PushLeft;
		char swCenter = !PushCenter;

		if (mode == SEARCH) {
			led(1, 0, 0, 0);
		} else if (mode == SEARCH2) {
			led(0, 1, 0, 0);
		} else if (mode == RUN) {
			led(1, 1, 0, 0);
		} else if (mode == RUN2) {
			led(0, 0, 1, 0);
		} else if (mode == RUN3) {
			led(1, 0, 1, 0);
		} else if (mode == RUN4) {
			led(0, 1, 1, 0);
		} else if (mode == RUN5) {
			led(1, 1, 1, 0);
		} else if (mode == CONFIG) {
			led(0, 0, 0, 1);
		} else if (mode == CONFIG2) {
			led(1, 0, 0, 1);
		} else if (mode == CONFIG3) {
			led(0, 1, 0, 1);
		} else if (mode == CONFIG4) {
			led(1, 1, 0, 1);
		} else if (mode == CONFIG5) {
			led(0, 0, 1, 1);
		} else if (mode == CONFIG6) {
			led(1, 0, 1, 1);
		} else if (mode == CONFIG7) {
			led(0, 1, 1, 1);
		} else if (mode == CONFIG8) {
			led(1, 1, 1, 1);
		} else if (mode == CONFIG9) {
			led(1, 0, 0, 0);
			cmt_wait(50);
			led(0, 0, 0, 0);
			cmt_wait(50);
			led(1, 0, 0, 0);
			cmt_wait(50);
		} else if (mode == KeepZeroPoint) {
			led(0, 1, 0, 0);
			cmt_wait(50);
			led(0, 0, 0, 0);
			cmt_wait(50);
			led(0, 1, 0, 0);
			cmt_wait(50);
		} else if (mode == MapMemory) {
			led(0, 0, 1, 0);
			cmt_wait(50);
			led(0, 0, 0, 0);
			cmt_wait(50);
			led(0, 0, 1, 0);
			cmt_wait(50);
		} else if (mode == VacumeTest) {
			led(0, 0, 0, 1);
			cmt_wait(50);
			led(0, 0, 0, 0);
			cmt_wait(50);
			led(0, 0, 0, 1);
			cmt_wait(50);
		} else if (mode == Mentenance) {
			led(0, 0, 0, 0);
		}

		if (swTop || swBottom) {
			if (swTop) {
				mode++;
				if (mode > Mentenance) {
					mode = SEARCH;
				}
			}
			if (swBottom) {
				mode--;
				if (mode < 0) {
					mode = Mentenance;
				}
			}
			for (int i = 0; i < 60000; i++)
				;
		}

		while (swTop || swBottom) {
			swTop = !PushTop;
			swBottom = !PushBottom;
			swRight = !PushRight;
			swLeft = !PushLeft;
			swCenter = !PushCenter;
		};

		if (swCenter) {
			os_escape = 1;
			decide(100);
			led(1, 1, 1, 1);
			cmt_wait(500);
			led(0, 0, 0, 0);
			cmt_wait(500);
			break;
		}
		cmt_wait(10);
	}
	os_escape = true;
	MTU.TSTRA.BIT.CST4 = 1;
	return mode;
}

void cirquitRun() {
	cirquit();
}
void FF_print() {
	mtu_start();
	while (1) {
		cmt_wait(50);
		myprintf("%d %d \n", (int) (FF_calc(L, 0, 0) * 1000),
				(int) (FF_calc(R, 0, 0) * 1000));
	}
	mtu_stop();
}

#define goal7 10000
#define goal8 25000

char selectGoal() {
	char goal = 0;
	os_escape = 0;
	MTU1.TCNT = 4500;
	MTU2.TCNT = 30000;

	MTU.TSTRA.BIT.CST4 = 0;
	while (1) {
		if (MTU1.TCNT < goal7) {
			LED1 = 1;
//			LED6 = 0;
			goal = 7;
		} else if (MTU1.TCNT < goal8) {
			LED1 = 0;
//			LED6 = 1;
			goal = 8;
		}
		if (MTU1.TCNT > OVERFLOW) {
			MTU1.TCNT = 5000;
		}
		if (MTU1.TCNT < UNDERFLOW) {
			MTU1.TCNT = 15000;
		}
		if (!Swich) {
			os_escape = 1;
			LED1 = 1;
			LED2 = 1;
			LED3 = 1;
			cmt_wait(500);
			LED1 = 0;
			LED2 = 0;
			LED3 = 0;
			cmt_wait(500);
			break;
		}
	}
	led(0, 0, 0, 0);
	if (goal == 7) {
		coin(250);
		cmt_wait(100);
	}
	if (goal == 8) {
		coin(250);
		cmt_wait(100);
		coin(250);
		cmt_wait(100);
	}

	MTU.TSTRA.BIT.CST4 = 1;
	return goal;
}

void printCheckMapping() {
	myprintf("\r\n");
	for (int i = 0; i < 255; i++) {
		int x = (checkQ[i] & 0xf0) >> 4;
		int y = checkQ[i] & 0x0f;
		myprintf("(%2x,%2x)\r\n", x, y);
		if (x == 0 && y == 0) {
			break;
		}
	}
	myprintf("\r\n");
	for (int j = 15; j >= 0; j--) {
		for (int i = 0; i < 16; i++) {
			myprintf("%2x, ", checkMap[i][j]);
		}
		myprintf("\r\n");
	}
	myprintf("\r\n");
}

void easyMakePath(char isFull) {
	flushcheckQ();
	fanMode = SearchRun;
	fanMode = FastRun;
	Dia2 = Dia - minus + 2;
	Dia3 = Dia - minus + 1;
	St2 = St1 - minus - 1;
	St3 = St1 - minus - 2;
	vetorPathCreate(0, 0, isFull);
	largePath(true);
	diagonalPath(isFull, true);
	printRealPath();
}

void exportMap() {
	myprintf("{");
	for (int i = 0; i < 16; i++) {
		myprintf("{");
		for (int j = 0; j < 16; j++) {
			myprintf("%d", (map[i][j] & 0x0f));
			if (j != 15) {
				myprintf(",");
			}
		}
		myprintf("}");
		if (i != 15) {
			myprintf(",");
		}
		myprintf("\r\n");
	}
	myprintf("}\r\n");
}
void exportMap2() {
	myprintf("[");
	for (int i = 0; i < 16; i++) {
		myprintf("[");
		for (int j = 0; j < 16; j++) {
			myprintf("%d", (map[i][j] & 0x0f));
			if (j != 15) {
				myprintf(",");
			}
		}
		myprintf("]");
		if (i != 15) {
			myprintf(",");
		}
		myprintf("");
	}
	myprintf("]\r\n");
}
void checkMemory(int goalX, int goalY) {
	int normal_music = (int) (*(float *) 1050064);
	int transam_music = (int) (*(float *) 1050072);

	float normal_music_time = (int) (*(float *) 1050068);
	float transam_music_time = (int) (*(float *) 1050076);

	float accele = (int) (*(float *) 1050064);

	if (writeMap2(FLASH_DF_BLOCK_0)) {
		if (activate_TRANS_AM) {
			if (transam_music == ASH_LIKE_SNOW) {
				ashLikeSnow(transam_music_time);
			} else if (transam_music == QUARIA) {
				quoria(transam_music_time);
			} else if (transam_music == Ahead_Tears) {
				aheadTears(transam_music_time);
			} else if (transam_music == HAKANAKU) {
				startTransam(transam_music_time);
			}
		} else {
			if (normal_music == ASH_LIKE_SNOW) {
				ashLikeSnow(normal_music_time);
			} else if (normal_music == QUARIA) {
				quoria(normal_music_time);
			} else if (normal_music == Ahead_Tears) {
				aheadTears(normal_music_time);
			} else if (normal_music == HAKANAKU) {
				startTransam(normal_music_time);
			}
		}
//		quoria(400);
	} else {
//		coin(125);
	}
	readMaze();
	updateDist(goalX, goalY, 0, isFullMaze(goalX, goalY));
	for (int j = 0; j < 15; j++) {
		for (int i = 0; i < 15; i++) {
			if ((existWall(i, j, North) != existWall(i, j + 1, South))
					|| (existWall(i, j, East) != existWall(i + 1, j, West))) {
				resetFull();
				return;
			}
		}
	}
	exportMap();
	cirquitMode = false;
	myprintf("not cirquit \r\n");
//	resetMap();
	printMap();

	exportMap2();
}

void operation() {
	volatile char goalX = (char) (*(float *) 1049336);
	volatile char goalY = (char) (*(float *) 1049340); //selectGoal();
	char testCmd = (char) (*(float *) 1049260);
	myprintf("Goal=(%d, %d)\r\n", goalX, goalY);
	fanMode = FastRun;
	if (!PushBottom) {
		goalX = 7;
		goalY = 7;
		testCmd = 0;
		coin(100);
		coin(100);
		coin(100);
	}

	if (goalChangeFlg == 1) {
		goalX = 7;
		goalY = 7;
	}

	if (!PushTop) {
		printSensor();
	}
	if (!PushRight) {
		activate_TRANS_AM = true;
	}

	enableSciUpdate = true;
	globalState = IMPORT_PARM;

//	float values[4] = { 1, 3, 5, 8 };
//	float tmpv = 1.5;
	while (PushBottom) {
//		float v = getLerpValue(tmpv, values, sizeof(values));
//		cmt_wait(100);
	};
	if (!PushTop) {
		enableSciUpdate = true;
		while (PushBottom)
			;
	}

	resetAllData();

	if (testCmd > 0) {
		switch (testCmd) {
		case 1:
			cmtMusic(D2_, 100);
			testSlalom3();
			break;
		case 2:
			cmtMusic(E2_, 100);
			testRun();
			break;
		case 3:
			cmtMusic(F2_, 100);
			testRoll(R);
			break;
		case 4:
			cmtMusic(G2_, 100);
			testNormalSlalom();
			break;
		case 5:
			cmtMusic(A2_, 100);
			testWallOff();
			break;
		case 6:
			cmtMusic(B2_, 100);
			testWallOffSeach();
			break;
		case 7:
			cmtMusic(C3_, 100);
			testNormalSlalom();
			break;
		case 8:
			cmtMusic(D3_, 100);
			checkIsoukeisu();
			break;
		case 9:
			cmtMusic(E3_, 100);
			cirquit();
			break;
		case 10:
			cmtMusic(F3_, 100);
			cirquitFull();
			break;
		case 11:
			cmtMusic(G3_, 100);
			keepZeroPoint2();
			break;
		case 12: //System Identification 並進
			cmtMusic(A3_, 100);
			// detectSysId();
			break;
		case 13: //System Identification 回転
			cmtMusic(B3_, 100);
			break;
		case 14:
			cmtMusic(C3_, 100);
			cirquitDiaPractice();
			break;
		case 15:
			cmtMusic(C3_, 100);
			testNormalSlalom2();
			break;
		case 16:
			cmtMusic(C3_, 100);
			testCalibrate();
			break;
		case 17:
			cmtMusic(C3_, 100);
			gyroZeroCheck(true);
			frontctrl_pos();
			break;
		}
	} else {
		cmtMusic(C2_, 100);
	}

	if (testCmd != 9 && testCmd != 10 && testCmd != 14) {
		checkMemory(goalX, goalY);
	}

	wall_off_limit = 25;
	wall_off_limit_d = 25;

	volatile char autostart = (char) (*(float *) 1049408);

	while (1) {
		TRANSAM = false;
		int res = action(encorderOperation(), goalX, goalY, false);
		mtu_stop2();
		if (autostart && res == 2 && testCmd == 0) {
			action(RUN, goalX, goalY, true);
		}
		mtu_stop2();
	}
}

#endif /* OPERATIONSYSTEM_H_ */
