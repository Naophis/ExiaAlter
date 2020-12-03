#include <iodefine.h>
#include "stdio.h"
#include <stdint.h>
#include <string.h>
#include "machine.h"
#include <mathf.h>
#include "gen_code/mpc_tgt_calc.h"
#include "config/defines.h"
#include "config/Table.h"

#include "config/Structor.h"

#include "config/init.h"
#include "include/serial.h"
#include "spi.h"
#include "Music.h"

#include "config/ParamDef.h"
#include "sensor.h"

#include "config/ParamDef.h"
#include "config/ParamsController.h"
#include "config/ParamUartImporter.h"
#include "Util.h"
#include "calc_interp.h"
#include "PhysicalBasement.h"

#include "motion/Run.h"
#include "motion/WallOff.h"
#include "search/VectorSearch.h"
#include "search/VectorPath.h"
#include "Path.h"
#include "motion/RealRun.h"

#include "flash/r_flash_rx_config.h"
#include "flash/r_flash_rx_if.h"
#include "flash/flash.h"

#include "search/Adachi.h"
#include "search/cirkit.h"

#include "config/Setting.h"
#include "Action.h"
#include "OperationSystem.h"
#include "config/SerialMapper.h"


volatile void mtu6A() {
	PORTD.PODR.BIT.B7 = 1;
}
volatile void mtu6B() {
	PORTD.PODR.BIT.B7 = 0;
}
volatile char rightTrend = 0;
volatile char leftTrend = 0;
volatile signed short gyros[4];

volatile void cmt() {
	timer++;
	time++;
	if (singing) {
		buzzerTimer++;
		if (m_time > 0 && buzzerTimer >= (CMT_CYCLE / 1000) * m_time) {
			stopCmt1();
		}
	}
	sinCount++;
	swTop = !PushTop;
	swBottom = !PushBottom;
	swRight = !PushRight;
	swLeft = !PushLeft;
	swCenter = !PushCenter;
	if (!singing) {
		if (!PushTop) {
			cmtMusic(C2_, 100);
		}
		if (!PushRight) {
			cmtMusic(D2_, 100);
		}
		if (!PushLeft) {
			cmtMusic(E2_, 100);
		}
		if (!PushBottom) {
			cmtMusic(F2_, 100);
		}
	}
	Physical_Basement();
	if ((logs < (L_Length - 1)) && (cc == 1) && (time >= 0)) {
		if ((time % (char) (logterm)) == 0) {
			log1[logs] = (V_now);
			log3[logs] = (V_Enc.r + V_Enc.l) / 2;
			log4[logs] = (ang * 180 / PI); //ジャイロ
			log5[logs] = Duty_l * 100;
			log6[logs] = Duty_r * 100;
			log7[logs] = (battery);
			log8[logs] = (LS_SEN45.now);
			log9[logs] = (RS_SEN45.now);
			logs10[logs] = Se.error_now;
			log11[logs] = (Front_SEN.now);
			log12[logs] = (settleGyro);
			log13[logs] = (W_now);
			log14[logs] = (angle * 180 / PI); //理論値
			log15[logs] = V_Enc.r;
			log16[logs] = V_Enc.l;
			log17[logs] = (float) (alpha);
			log18[logs] = px2; //lastPeekR;
			log19[logs] = (feadforward_para(L));
			log20[logs] = (feadforward_para(R));
			log21[logs] = (LS_SEN2.now);
			log22[logs] = (RS_SEN2.now);
			float tmpRightRef = sen_r_dia_img[(int) img_dist_r];
			float tmpRightRef90 = sen_r90_dia[(int) img_dist_r];
			float tmpLeftRef = sen_l_dia_img[(int) img_dist_l];
			float tmpLeftRef90 = sen_l90_dia[(int) img_dist_l];

			log23[logs] = sen_l2[0];
			log24[logs] = sen_r2[0];

			log25[logs] = C.g;
			log26[logs] = C.s;
			log27[logs] = C.v;
			log28[logs] = globalState;
			log29[logs] = C.y_diff ;

			log30[logs] = tmpLeftRef;
			log31[logs] = tmpLeftRef90;

			log32[logs] = distance;
			log33[logs] = img_distance;
			log34[logs] = C.angles;
			log35[logs] = ego_data_in.img_ang;
			log36[logs] = ego_data_in.img_dist;

			logs++;
		}
	}

	if (enableSystemIdentification && timer < DETECT_SYS_ID_Length) {
		vrlist[timer] = V_Enc.r;
		vllist[timer] = V_Enc.l;
		gyrolist[timer] = settleGyro;
		dutylist[timer] = (*(float *) 1049472) / battery * M_CYCLE;
	}

}

volatile int timer2 = 0;

void mtu4_A() {
	tpu_count++;
	switch (tpu_count) {
	case 1:
		timer2++;
		if (mpu) {
			callMpu(0x47);
		}
		PORTD.PODR.BIT.B6 = 1; //Front
		break;
	case 2:
		if (mpu) {
			callMpu(0x47);
		}
		PORTE.PODR.BIT.B0 = 1; //Right45
		PORT2.PODR.BIT.B5 = 1; //Left45
		break;
	case 3:
		if (mpu) {
			callMpu(0x47);
		}
		PORTE.PODR.BIT.B1 = 1; //Right90
		PORT2.PODR.BIT.B4 = 1; //Left90
		break;
	case 4:
		if (fanStart) {
			const float minVoltage = *(float *)1049852;
			float tmpBattery = max(battery, minVoltage);

			if (fanMode == TestRun) {
				GPT2.GTCCRA = (short) (myVacumeDuty / tmpBattery * FAN_CYCLE);
				GPT2.GTCCRC = (short) (myVacumeDuty / tmpBattery * FAN_CYCLE);
			} else if (fanMode == FastRun) {
				GPT2.GTCCRA = (short) (FAN_AMP / tmpBattery * FAN_CYCLE);
				GPT2.GTCCRC = (short) (FAN_AMP / tmpBattery * FAN_CYCLE);
			} else if (fanMode == FastRun2) {
				GPT2.GTCCRA = (short) (fastRunFanV2 / tmpBattery * FAN_CYCLE);
				GPT2.GTCCRC = (short) (fastRunFanV2 / tmpBattery * FAN_CYCLE);
			} else if (fanMode == SearchRun) {
				GPT2.GTCCRA = (short) (FAN_AMP2 / tmpBattery * FAN_CYCLE);
				GPT2.GTCCRC = (short) (FAN_AMP2 / tmpBattery * FAN_CYCLE);
			} else if (fanMode == CtrlFan) {
				GPT2.GTCCRA = (short) (FAN_AMP3 / tmpBattery * FAN_CYCLE);
				GPT2.GTCCRC = (short) (FAN_AMP3 / tmpBattery * FAN_CYCLE);
			} else if (fanMode == CtrlFan2) {
				GPT2.GTCCRA = (short) (FAN_AMP4 / tmpBattery * FAN_CYCLE);
				GPT2.GTCCRC = (short) (FAN_AMP4 / tmpBattery * FAN_CYCLE);
			} else if (fanMode == CtrlFan3) {
				GPT2.GTCCRA = (short) (FAN_AMP5 / tmpBattery * FAN_CYCLE);
				GPT2.GTCCRC = (short) (FAN_AMP5 / tmpBattery * FAN_CYCLE);
			}else if (fanMode == CtrlFan4) {
				GPT2.GTCCRA = (short) (FAN_AMP6 / tmpBattery * FAN_CYCLE);
				GPT2.GTCCRC = (short) (FAN_AMP6 / tmpBattery * FAN_CYCLE);
			}
		} else {
			GPT2.GTCCRA = GPT2.GTCCRC = 0;
		}
		if (mpu) {
			callMpu(0x47);
		}
		break;
	}
}

void getBattery() {
	float tmp = 3.34;
	if (BATTERY != 0) {
		battery = 0.1 * tmp * BATTERY / 4096 * 4 + 0.9 * batteryOld;
	} else {
		battery = tmp * BATTERY / 4096 * 4;
	}
	batteryOld = battery;
}
void mtu4_B() {
	switch (tpu_count) {
	case 1:
		sensing_front();
		gyros[0] = (signed short) getMpuData();
		break;
	case 2:
		sensing_side();
		gyros[1] = (signed short) getMpuData();
		break;
	case 3:
		sensing_side2();
		gyros[2] = (signed short) getMpuData();
		break;
	case 4:
		sensing_in_off();
		gyros[3] = (signed short) getMpuData();

		RS_SEN45.old = RS_SEN45.now;
		LS_SEN45.old = LS_SEN45.now;
		Front_SEN.old = Front_SEN.now;
		RS_SEN2.old = RS_SEN2.now;
		LS_SEN2.old = LS_SEN2.now;

		getSensorData();
		float tmpGyros = 0.250 * (gyros[0] + gyros[1] + gyros[2] + gyros[3]);

		settleGyro2 = (tmpGyros - G.ref) * G.th;

		float diffOder = *(float *) 1049748;
		char gyroMode = (char) (*(float *) 1049752);
		float gyrooffset = *(float *) 1049756;

		if ((W_now - settleGyro2) > diffOder
				|| (settleGyro2 - W_now) > diffOder) {
			settleGyro = W_now + gyrooffset;
			settleGyroOld = settleGyro;
			G.now = settleGyro;
			G.old = settleGyro;
		} else {
			G.now = settleGyro2;
			settleGyroOld = settleGyro;
			if (gyroMode == true) {
				settleGyro = 0.1 * G.now + 0.9 * G.old;
			} else {
				settleGyro = G.now;
			}
			char gyroConfig2 = (char) (*(float *) 1049820);
			if (gyroConfig2) {
				G.old = settleGyro2;
			} else {
				G.old = settleGyro;
			}
		}

		tpu_count = 0;

		for (char i = 4; i > 0; i--) {
			sen_r2[i] = sen_r2[i - 1];
			sen_l2[i] = sen_l2[i - 1];
		}
		sen_r2[0] = RS_SEN2.now > search_wall_off_r;
		sen_l2[0] = LS_SEN2.now > search_wall_off_l;
		break;
	}
	sensor_led_off();
}

void initExiaAlter() {
	initClock();
	initLED();
	initSensorLED();
	initMotorDriverRotate();
	initS12AD();
	initSci1();
	initCmt0();
	initIsouKeisu();
	initGPT01();
//	initGPT2();
	Init_SPI();
	init_Mtu4();
	init_Mtu6();
	myprintf("Flash Open %d\r\n", R_FLASH_Open());
	importParam();

}

void main(void) {
	initExiaAlter();
	myprintf("hello world\r\n");
	batteryCheck();
	setupCmt = enableMPU = os_escape = true;

	while (1) {
		float result = getZeroPoint();
		if (ABS(result) < 0.01) {
			decide2(125);
			setupMpu6500();
		} else {
			break;
		}
	}
	coin(100);
	ledOn = 1;
	os_escape = 1;
	// myprintf("%c[33m", ESC); /* 文字を黄色に */
	operation();
	os_escape = 0;
}
