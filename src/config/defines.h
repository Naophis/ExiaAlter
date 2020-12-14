/*
 * defines.h
 *
 *  Created on: 2017/07/30
 *      Author: Naoto
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define STR(var) #var   //引数にした変数を変数名を示す文字列リテラルとして返すマクロ関数

//#define true 1
//#define false 0
volatile char setupCmt = false;
volatile int tpu_count = 0;
#define M_CYCLE 450
#define FAN_CYCLE 450

#define FRONT_AD S12AD.ADDR3 //front P43
#define RS2 S12AD.ADDR5	 //P45
#define RS45 S12AD.ADDR4 //P44
#define LS45 S12AD.ADDR1 //P41
#define LS2 S12AD.ADDR2	//P42

#define LF S12AD.ADDR6	//P46
#define BATTERY S12AD.ADDR7	//P47

#define LED1 PORTA.PODR.BIT.B7
#define LED2 PORTB.PODR.BIT.B5
#define LED3 PORTB.PODR.BIT.B6
#define LED4 PORTB.PODR.BIT.B7
#define LED_RIGHT PORTE.PODR.BIT.B2
#define LED_LEFT PORT2.PODR.BIT.B3

volatile double settleGyro2, settleGyroOld, settleGyro;

volatile char adEnable = 1;

volatile char enableMPU = false;
float battery, batteryOld;
char fanStart = false;
char fanStart2 = false;
//#define FAN_AMP2 10.50
//#define FAN_AMP 10.50

float FAN_AMP = 9.0f;	//11.35
float FAN_AMP2 = 6.5f;	//5.0
float FAN_AMP3 = 0.0f;	//5.0
float FAN_AMP4 = 4.0f;	//5.0
float FAN_AMP5 = 3.0f;	//5.0
float FAN_AMP6 = 3.0f;	//5.0
float fastRunFanV2 = 3.0f;	//5.0

volatile float myVacumeDuty = 6.5;
//#define FAN_AMP 11.0	//11.35
const float PI = 3.141592653589793;

char TRANSAM = false;
double dt = 0.001 / 4;
#define ABS(IN) ((IN) < 0 ? - (IN) : (IN))
#define Vo 0.0f				//初速度
#define Wo 0.0f			//初期角速度
#define W_max 60.0f
//#define W_max 5.5f
volatile double W_now = 0;
volatile double alpha = 0.0;			//	[rad/s^2]
volatile double ang = 0.0;				//角度 rad
volatile double distance = 0.0;		//距離 mm
volatile double img_distance = 0.0;		//距離 mm
volatile double img_dist_r = 0.0;		//距離 mm
volatile double img_dist_l = 0.0;		//距離 mm
volatile char diaStrwallCount_r = 0;
volatile char diaStrwallCount_l = 0;
#define North 1
#define East 2
#define West 4
#define South 8
#define NorthEast 3
#define SouthEast 5
#define NorthWest 6
#define SouthWest 7
//#define true 1
//#define false 0

#define R 1					//探索時ターン　右フラグ
#define L 2					//探索時ターン　左フラグ
#define ROOT2 1.41421356f
#define pathLength 512

volatile float V_r = 0;
volatile float V_l = 0;
volatile float V_max = 600.0;	//最高速度
volatile float acc = 4000;		//加速度
volatile float V_now = Vo;		//現在速度
volatile float V_old = 0;
volatile float W_old = 0;
volatile unsigned short x = 0, y = 0;		//座標保持変数
volatile unsigned char path_s[pathLength];	//直進パス
volatile unsigned char path_t[pathLength];	//ターンパス

volatile float tgt_v_now_ff = 0;
volatile float tgt_w_now_ff = 0;
volatile float tgt_accl_ff = 0;
volatile float tgt_alpha_ff = 0;

volatile unsigned int Value = 255;	//最小の評価値
volatile unsigned char next_move = North;
volatile unsigned char next_dir = North;
volatile unsigned char now_dir = North;

volatile char stop = 0;
volatile float v_sla[8];

volatile char os_escape;
volatile char fail = 0;
volatile unsigned int sinCount = 0;
volatile unsigned char sw;
volatile unsigned int time = 0;
volatile short gyro_temp[5];
volatile float gyro = 0;
volatile float tempGyro = 0;
volatile int temp_max = 0, temp_min = 5000;
volatile float oldGyroData = 0;
volatile float settlegyroData = 0;
volatile int tempGyro2 = 0;
volatile float settleGyro3 = 0;
volatile float controlSensour = 0;
volatile float controlGyro = 0;
volatile float w_r, w_l;
volatile float w_now;
volatile char positionControlValueFlg;
volatile char gyroKeepZero;
volatile char dia = 0;
volatile char frontwall_ctrl=0;
volatile float Duty_r, Duty_l;
volatile unsigned long gra_r, gra_l;

volatile unsigned int meloTimer = 0;
volatile int DutyR, DutyL;
volatile float rpmR = 0, rpmL = 0;
volatile float rpmR_old = 0, rpmL_old = 0;

volatile float logterm = 5;
volatile float isouratio = 0;
volatile float isouzure = 0;
volatile float searchrange = 0;
volatile float angle = 0;
volatile char gyroErrResetEnable = false;
volatile unsigned int cc = 0;
volatile unsigned int logs = 0;
volatile float ffR, ffL;
volatile float angle_enc;
volatile char slaFLG = 0;
volatile char gyroOn = 0;
volatile float W_now2 = 0;
volatile int EtCheck = 0;
volatile int ledOn = 0;
volatile int logData = 0;
volatile char cirquitMode = 0;
#define NAPIER 0.7632146182405746f
#define NAPIER2 0.6034501613134686f
volatile char goalChangeFlg = 0;
volatile char modeFlg = 0;
volatile char allow = 1;
volatile char weightMode = 1;
volatile char wallCut_R = 0;
volatile char wallCut_L = 0;
volatile char mode_FF = 0;
volatile char fastMode = 0;
volatile float slalomBackDistance = 0;
volatile char pathMode = 1;
volatile float tempVmax = 0;
volatile char alphaMode = 0;
volatile float alphaTemp = 0;
volatile float omegaTemp = 0;
volatile float slaTerm = 0;
volatile char runFlg = 0;
volatile char errorFlg = 0;
volatile char globalError = 0;

volatile char frontWallCtrl = 0;

volatile char isKnown = 0;

volatile float tmpData = 0;
#define MAZE_SIZE  16
volatile char dist[MAZE_SIZE][MAZE_SIZE];

volatile unsigned int c = 0;
volatile unsigned int timer = 0;
#define Swich	PORTC.PIDR.BIT.B4
#define CW_R();  PORTB.PODR.BIT.B1 = 1; PORTB.PODR.BIT.B2 = 0;
#define CCW_R(); PORTB.PODR.BIT.B1 = 0; PORTB.PODR.BIT.B2 = 1;
#define CCW_L();  PORTB.PODR.BIT.B3 = 1; PORTB.PODR.BIT.B4 = 0;
#define CW_L(); PORTB.PODR.BIT.B3 = 0; PORTB.PODR.BIT.B4 = 1;

#define PushTop PORTC.PIDR.BIT.B5
#define PushCenter PORTC.PIDR.BIT.B4
#define PushLeft PORTC.PIDR.BIT.B3
#define PushRight PORTC.PIDR.BIT.B0
#define PushBottom PORTC.PIDR.BIT.B2

#define ALL 0
#define SearchMode 1
#define AtackStraight 2
#define AtackDia 3
#define AtackDia2 4
#define CtrlMode 5
volatile char sensingMode = AtackStraight;
volatile char enablePWM = false;
#define  ESC    0x1B

#define checkQlength 256
int checkQDuality[100];
int checkQ[checkQlength];
unsigned char checkMap[16][16];
unsigned char checkTurningPoint = false;
char checkPoint = false;
char activate_TRANS_AM = false;
#define MAX 1023

char globalSkipFront = false;

void ledHex(int v) {
	char a = (v & 0x01);
	char b = (v & 0x02);
	char c = (v & 0x04);
	char d = (v & 0x08);
	LED1 = a > 0;
	LED2 = b > 0;
	LED3 = c > 0;
	LED4 = d > 0;
	LED_RIGHT = false;
	LED_LEFT = false;
}
void led(char a, char b, char c, char d) {
	LED1 = a > 0;
	LED2 = b > 0;
	LED3 = c > 0;
	LED4 = d > 0;
	LED_RIGHT = false;
	LED_LEFT = false;
}
void LED(char i) {
	LED1 = i & 0x01;
	LED2 = i & 0x02;
	LED3 = i & 0x04;
	LED4 = i & 0x08;
	LED_RIGHT = false;
	LED_LEFT = false;
}
void F_SEN_LED() {
	PORTC.PODR.BIT.B3 = 1;
}
void S1_SEN_LED() {
	PORTC.PODR.BIT.B2 = 1;
}
void S2_SEN_LED() {
//	return;
	PORT1.PODR.BIT.B5 = 1;
}
void SEN_LEDOFF() {
	PORTC.PODR.BIT.B2 = 0;
	PORTC.PODR.BIT.B3 = 0;
	PORT1.PODR.BIT.B5 = 0;
}
int gyroData = 0;
#define GYRO_Z 0x47
volatile int buzzerTimer = 0;

#define BUFSIZE		100
volatile float etN = 4; //	ネイピアべき乗数
// ｸﾞﾛｰﾊﾞﾙ変数

// SCI ﾊﾞｯﾌｧ
unsigned char g_uchBuf1[BUFSIZE];
volatile char originalDiaMode = false;
volatile char mpu = false;
// SCI ﾘﾝｸﾞﾊﾞｯﾌｧ ｲﾝﾃﾞｯｸｽ
int g_nIdx1Read;
int g_nIdx1Write;

volatile float vs[10][12];
volatile float vs2[10][12];
volatile float vs3[10][12];

volatile float tmpDiac;
volatile float targetVelocity;
#define Kata 0
#define Zentansaku 1
#define Oufuku 2
unsigned char recieved_data;

#define SCI_RX_BUF_SHIFT   4
#define SCI_RX_BUF_SIZE   (1<<SCI_RX_BUF_SHIFT)
#define SCI_RX_BUF_MASK   (SCI_RX_BUF_SIZE-1)

#define motionLogLength 2048

int nextDirection = 0;
volatile unsigned char map[MAZE_SIZE][MAZE_SIZE];

typedef struct {
	unsigned int n :10;
	unsigned int e :10;
	unsigned int w :10;
	unsigned int s :10;
	unsigned int v :8;
	unsigned int N1 :4;
	unsigned int NE :4;
	unsigned int E1 :4;
	unsigned int SE :4;
	unsigned int S1 :4;
	unsigned int SW :4;
	unsigned int W1 :4;
	unsigned int NW :4;
	unsigned int step :4;

} vector_map;
volatile vector_map m[MAZE_SIZE][MAZE_SIZE];

#define Q_LENGTH 2048
int que[Q_LENGTH][3];
#define L_Length 2900
volatile float log1[L_Length];
volatile float logs2[L_Length];
volatile float log3[L_Length];
volatile float log4[L_Length];
volatile float log5[L_Length];
volatile float log6[L_Length];
volatile float log7[L_Length];
volatile float log8[L_Length];
volatile float log9[L_Length];
volatile float logs10[L_Length];
volatile float log11[L_Length];
volatile float log12[L_Length];
volatile float log13[L_Length];
volatile float log14[L_Length];
volatile float log15[L_Length];
volatile float log16[L_Length];
volatile float log17[L_Length];
volatile float log18[L_Length];
volatile float log19[L_Length];
volatile float log20[L_Length];
volatile float log21[L_Length];
volatile float log22[L_Length];
volatile float log23[L_Length];
volatile float log24[L_Length];
volatile float log25[L_Length];
volatile float log26[L_Length];
volatile float log27[L_Length];
volatile float log28[L_Length];
volatile float log29[L_Length];
volatile float log30[L_Length];
volatile float log31[L_Length];
volatile float log32[L_Length];
volatile float log33[L_Length];
volatile float log34[L_Length];
volatile float log35[L_Length];
volatile float log36[L_Length];

volatile char enableSystemIdentification = false;
volatile float sysIdDutyR = 0;
volatile float sysIdDutyL = 0;

#define DETECT_SYS_ID_Length 20
volatile float vrlist[DETECT_SYS_ID_Length];
volatile float vllist[DETECT_SYS_ID_Length];
volatile float gyrolist[DETECT_SYS_ID_Length];
volatile float dutylist[DETECT_SYS_ID_Length];

#define Cycle 24000000.0f
#define MTU_CYCLE 16000.0f	//4khz
#define _ICLK 240000000 //240MHz
#define _PCLKA 60000000.0f //60MHz
#define _PCLKB 60000000.0f //60MHz
#define CMT_CYCLE 4000 // 4kHz
#define CMT_CYCLE2 2000 // 2kHz
#define M_CYCLE2 500

#define FastRun 0
#define SearchRun 1
#define TestRun 2
#define CtrlFan 3
#define CtrlFan2 4
#define CtrlFan3 5
#define CtrlFan4 6
#define FastRun2 7

volatile char fanMode = SearchRun;
volatile char tmpfanMode = SearchRun;

volatile char rotate_r = true;
volatile char rotate_l = true;

volatile char friction_str = true;
volatile char friction_roll = false;

volatile char RecvDat, ComFlag;
volatile char enableSciUpdate = false;

volatile char enableSensorCtrl = false;

volatile char swTop, swBottom, swLeft, swRight, swCenter;

volatile unsigned int globalState = 0;
#define NONE 0
#define STRAIGHT 1
#define PIVOT 2
//#define SLA_TURN 2
#define SLA_TURN 3
#define SLA_BEFORE 4
#define SLA_AFTER 5
#define WALL_OFF 6
#define FRONT_ctrl 7
#define PARAREL 8
//#define DIA_STRAIGHT 2
#define DIA_STRAIGHT 9
#define WALL_OFF_WAIT 10
#define WALL_OFF_WAIT_DIA 14
#define MODE_SELECT 11
#define START_WAIT 12
#define IMPORT_PARM 13
#define BACK_STRAIGHT 15
#define PIVOT_END 16

char testRunMode = false;

#define SEN_R RS_SEN45.now
#define SEN_L LS_SEN45.now
#define SEN_R2 RS_SEN2.now
#define SEN_L2 LS_SEN2.now
#define SEN_FRONT Front_SEN.now

char testMode = true;

float R_WALL_EXIST = 1900;
float L_WALL_EXIST = 1500;

float R_WALL_EXIST2 = 1600;  //探索時壁判定
float L_WALL_EXIST2 = 1500;  //探索時壁判定
float FRONT_WALL_EXIST2 = 740; //探索時壁判定

float R_WALL_EXIST3 = 420;  //探索時壁判定
float L_WALL_EXIST3 = 300;  //探索時壁判定
float FRONT_WALL_EXIST3 = 300; //探索時壁判定

float px = 0, py = 0;
float px2 = 0, py2 = 0;

float FRONT_WALL_EXIST = 800; //探索時壁判定
float R_WALL_EXIST4 = 220;  //探索時壁判定
float L_WALL_EXIST4 = 220;  //探索時壁判定

float pathVmax = 0;
float pathAcc = 0;
float pathDiac = 0;

volatile char motionDir = R;

#define STOP_VELOCITY 2.0f

volatile RT_MODEL_mpc_tgt_calc_T mpc_tgt_calc_error_status;
volatile t_tgt target_data;
volatile t_ego ego_data_in;
volatile int32_T mpc_tgt_calc_mode;
volatile t_ego ego_data_out;
volatile t_ego ego_data_out2;

#define PREDICT_SIZE 1
volatile t_tgt target_data_list[PREDICT_SIZE];
volatile t_ego ego_data_in_list[PREDICT_SIZE];
volatile t_ego ego_data_out_list[PREDICT_SIZE];

volatile float tgt_v_now = 0;
volatile float tgt_w_now = 0;
volatile float tgt_accl = 0;
volatile float tgt_alpha = 0;

volatile float accl_delay_max_cnt = 30;
volatile float decel_delay_max_cnt = 50;

float sen_l_dia_img[252] = {1168.3132,1111.1736,996.6117,883.5892,786.79645,704.73905,639.9055,588.88835,549.19795,517.1289,493.96715,476.0818,464.28115,454.3743,447.10935,439.69265,436.3953,429.947,427.4448,423.25285,419.85295,413.170075,411.208675,408.785375,408.46355,410.199625,410.781825,408.529,407.213175,407.60245,409.87725,411.84185,411.7888,411.602425,412.994775,413.987525,416.01485,418.656125,419.1417,418.51,417.0391,417.8899,419.096525,421.3874,419.9143,421.83405,420.8998,427.7468,425.85285,423.9202,426.626,426.9459,432.42945,434.53145,434.15715,435.58575,437.08745,435.9967,439.453,441.93335,444.5568,444.9137,446.0753,448.2758,449.4695,448.5288,452.99955,455.14985,457.0009,460.93325,460.4788,463.1739,465.04695,465.45545,466.41875,468.88515,473.1259,477.2445,478.68165,483.7457,487.33175,491.006,492.13615,495.852,498.87915,502.278,506.3239,509.79575,510.36195,513.348,515.0152,517.50055,519.85685,522.9608,525.42155,531.7569,537.9052,543.1276,545.1687,548.8807,553.26495,553.88385,558.3587,563.3452,563.981,567.74125,573.2542,576.4265,580.96065,584.4315,589.04335,595.7187,599.26575,604.49805,609.25085,611.6476,617.31575,619.94465,625.6914,632.09985,636.9724,643.89435,648.38855,654.71615,658.2508,662.62575,666.68835,671.75725,676.75465,683.0125,690.44735,695.1093,698.6404,703.75605,710.985,718.70705,723.4677,732.2108,732.6732,736.63725,739.9356,744.1273,751.1034,757.4477,762.62835,765.8032,770.4071,776.9244,782.78515,788.50625,795.52625,803.50775,810.49375,818.6729,826.1814,834.08985,841.2942,847.9879,852.5313,859.4958,868.5852,876.8559,882.9609,890.3027,896.2858,905.9525,915.2982,925.8532,935.3813,944.6277,950.402,959.527,968.4183,978.3167,984.5472,991.3718,995.2456,1002.2142,1007.4986,1015.6221,1018.1409,1021.9174,1027.2637,1036.0308,1043.692,1053.0524,1061.0145,1065.9445,1075.9302,1085.6154,1095.6045,1106.8234,1118.2543,1127.5417,1143.1151,1153.3534,1170.299,1181.2284,1194.9468,1208.7039,1224.1943,1235.1133,1247.167,1258.6482,1267.6575,1274.9554,1280.77,1291.2604,1297.5566,1305.2753,1310.7028,1317.6068,1323.129,1330.8762,1343.6891,1352.1254,1365.3839,1378.0952,1390.0767,1404.9089,1421.5735,1435.8208,1449.5656,1459.9286,1475.7836,1486.3538,1495.4269,1506.7544,1515.1445,1523.0464,1529.9657,1531.4398,1533.4211,1529.765,1521.3577,1513.1154,1506.42,1506.1465,1515.7095,1530.634,1544.9092,1556.1884,1559.6837,1557.0104,1545.2981,1527.5188,1505.7478,1483.1477,1456.3937,1417.6177,1358.204,1283.1167};
float sen_r_dia_img[256]  = {1476.0024,1404.8821,1257.1056,1033.2114,896.5972,832.95375,674.787,634.05745,566.4374,494.75865,462.49025,437.5775,419.66535,405.88155,394.226875,385.125925,379.3979,376.254875,370.766625,365.844575,359.924175,357.185525,354.954175,353.4331,350.514825,347.669275,345.4938,344.9665,343.2943,343.46805,341.470175,341.455775,338.8707,337.48,338.063,339.139375,341.62385,341.97115,343.09725,346.424075,346.677,346.950775,352.1167,356.119075,354.8628,354.464625,355.375825,357.88215,361.38705,361.946325,362.75495,364.938475,368.128975,373.2884,376.3823,378.1921,381.354925,384.297425,387.499825,389.413075,391.8042,392.91865,394.494075,399.1585,403.529675,404.979,408.6318,409.94175,413.301975,419.92705,421.2109,426.5442,429.63375,430.48735,432.76475,435.2123,439.838,441.7918,445.81645,446.55855,448.71775,452.80665,456.2515,458.12065,458.82485,463.49055,466.6894,469.622,472.26295,477.91635,481.0678,485.60795,489.4818,491.9936,498.0714,500.62675,503.49595,507.05625,512.0125,515.74145,518.78255,523.68825,525.73195,528.4229,532.53685,537.3883,542.5396,548.52675,551.3881,554.1264,561.8604,565.3773,569.6023,573.7342,576.91585,583.8711,585.1139,591.69965,595.60425,602.7343,608.17645,610.90415,618.23755,623.43445,626.03785,632.154,639.2747,642.92155,647.0164,651.4718,654.35215,662.57885,669.5498,675.6109,681.1764,686.8524,693.3404,700.69085,707.4287,714.5343,719.84125,726.2314,733.74505,744.1023,750.8321,760.17855,766.6392,775.2931,783.6368,791.7913,797.6092,807.30475,812.97785,821.30585,827.88695,834.2411,844.9054,852.5393,862.3392,870.0632,880.1181,890.8036,898.3327,905.5651,911.6998,919.1309,928.4302,934.7492,945.0452,954.3208,965.3854,973.1741,984.8026,994.2339,1005.7415,1012.0707,1022.0394,1030.9827,1042.6362,1054.7371,1070.3427,1085.1202,1100.6875,1113.0701,1126.3181,1138.9175,1150.463,1166.411,1173.7164,1181.8616,1187.5278,1199.6191,1209.3838,1218.9186,1231.7456,1246.984,1263.2495,1277.4733,1301.4341,1309.5914,1335.5963,1343.3829,1360.7419,1379.4684,1395.9315,1411.2969,1427.0741,1443.1948,1457.8423,1472.8124,1486.1631,1497.2693,1507.7671,1521.4756,1530.8622,1540.6127,1555.1505,1570.329,1584.7313,1601.4445,1620.3086,1641.5681,1664.6543,1685.9284,1707.9512,1730.7808,1753.0156,1771.9926,1791.9746,1809.4006,1823.533,1834.7446,1845.2914,1849.9542,1852.1734,1848.9034,1843.6268,1835.0106,1820.7556,1800.0584,1783.978,1770.3324,1765.3654,1764.2006,1774.8798,1792.0454,1809.86,1831.4104,1854.6732,1886.7994,1909.8304,1922.775,1908.0178,1866.8346,1792.6656,1680.0322};
float sen_r90_dia[256]  = {635.6486,641.6453,648.88105,662.5387,675.0945,682.00645,696.43835,700.775,711.84965,728.37435,739.99605,750.56805,758.69295,770.60665,782.2354,793.934,803.0614,810.972,821.9925,830.35595,838.35575,848.4996,858.6367,866.4791,875.7693,887.4989,896.0318,902.6822,909.6079,919.8032,925.0519,931.1295,938.3586,940.3869,944.2026,948.5751,950.3937,957.7994,968.8218,987.314,1008.3146,1020.7455,1017.6691,1002.6711,983.0795,966.8882,961.3303,967.7684,982.5429,995.9637,999.593,990.7986,974.3307,956.2054,942.0531,932.2678,924.0345,917.7786,915.9286,915.2205,910.0387,902.8167,899.9415,890.6965,886.8041,883.0913,875.1682,870.6962,867.4832,856.9991,853.2792,842.9324,834.2357,824.64735,818.62425,808.0552,800.10455,791.0133,780.18815,769.6367,759.13465,750.1944,740.3524,728.1545,719.502,709.3872,702.7024,691.15215,680.7,672.41155,661.00055,649.1173,633.2278,623.31405,613.94845,603.35435,596.80055,588.71395,578.35405,571.1155,564.53695,554.98635,547.0026,538.87915,529.519,525.2328,517.5099,510.1324,503.65615,494.1589,488.7293,479.93325,474.60105,467.55135,461.9022,455.0736,448.7396,442.83685,435.3752,430.20235,425.05505,419.4726,412.61715,406.928275,401.100525,394.156425,391.145375,390.113525,385.442775,380.485325,377.3331,367.8755,362.114575,356.97065,354.08545,348.21395,346.583275,343.563475,338.429375,335.8895,331.7288,325.70165,322.613,319.966675,315.839575,311.6556,308.067725,307.571875,301.65865,299.023675,295.165,297.31315,295.006225,292.366575,290.9567,287.438625,290.071525,289.374,290.962525,290.675075,291.509,293.740425,297.13935,299.9497,303.0964,308.704825,315.62295,321.8547,327.741175,331.42795,333.70885,337.3208,342.2687,341.2558,341.1415,340.0093,334.468375,333.157525,327.8053,322.410375,318.480225,313.9304,309.358025,304.679075,299.7328,299.188975,295.625825,294.986075,291.16385,293.111,293.7399,294.78725,295.951075,296.493475,296.58115,296.52795,298.464575,302.6784,305.7033,308.42965,314.64395,314.421575,318.210975,319.676025,322.748375,325.444025,328.8357,332.822425,337.403075,343.07145,348.97535,353.5207,357.16045,362.39275,366.37555,372.83465,378.4235,382.778025,385.324625,389.561425,396.044025,401.115775,407.259925,412.20435,418.698025,424.5301,429.6715,436.84885,442.59405,451.9897,458.4543,463.2208,469.66375,477.66615,485.8012,494.5294,503.15205,510.5476,519.0735,524.711,531.34135,538.7409,546.9091,555.1101,564.85565,572.5608,582.2337,589.5057,595.47395,605.32155,614.786,625.777,634.2156,643.1247,653.2619,663.56835};
float sen_l90_dia[252] = {470.92955,471.8329,480.22215,484.3072,489.8292,494.28205,501.8291,507.49855,512.67625,520.17375,520.94615,528.41175,532.383,538.4757,543.4104,550.32135,556.3783,560.98065,566.8899,574.4738,583.0458,588.12245,592.7643,598.2033,604.29565,608.2904,608.4881,616.1296,621.21845,624.6946,631.04055,631.7334,630.8581,632.12925,628.3491,629.8772,636.83125,647.71285,664.9623,680.7846,685.8971,684.2174,677.4392,668.5556,666.8034,663.87205,672.8232,682.62685,687.341,685.5955,672.80385,660.1469,651.74855,639.92485,630.17405,621.26385,619.05925,617.68865,611.53705,603.2548,600.58705,597.39675,592.89805,585.79615,582.46595,575.81725,572.44665,569.38215,565.2314,559.08705,557.89495,553.2511,546.59935,540.13825,533.6874,527.4916,519.1031,514.4177,510.37865,504.65535,497.74535,490.9047,482.2154,476.24935,475.16965,471.0075,464.02735,457.149,448.15855,442.6985,437.7646,430.9424,424.8621,418.894775,414.8384,412.3026,409.587675,403.362825,399.987,395.884125,393.10105,387.27105,382.1903,378.04465,372.1262,367.742975,363.77035,359.042725,354.686825,349.51435,344.92475,341.7322,335.664975,332.1367,328.99885,322.46,321.764475,316.568975,316.5071,310.579475,304.49835,302.78055,298.237375,294.182925,291.9367,291.66935,286.197325,281.76185,277.2759,275.1026,273.734675,270.8599,268.86485,266.395975,266.247475,263.17075,259.507725,259.275,255.6991,254.41335,249.6441,250.018475,248.4271,239.704,241.151275,237.37965,238.38285,238.7994,238.52865,236.746525,236.404875,235.438725,236.094325,232.517475,237.4703,239.213325,235.23875,236.620125,240.53185,241.585825,245.84085,250.58385,253.65295,253.674875,257.207375,264.139475,264.026175,266.56475,270.036125,271.8433,275.33315,278.72965,276.7653,277.8469,274.77415,271.4187,267.133575,260.0196,257.587225,255.68365,252.4802,254.30715,248.4207,246.2445,244.3541,241.8163,242.15635,241.732475,244.25715,239.458125,241.89115,244.331375,244.583,246.2665,245.378525,245.293325,247.798525,248.6899,250.704625,247.107875,248.53125,250.860925,251.60955,254.8638,256.623225,256.956075,260.683075,262.630025,264.25165,265.0344,269.528525,271.11835,275.539925,278.569925,281.4942,284.583525,285.858825,290.17185,295.37655,295.219025,300.24115,303.562725,312.18225,311.430225,314.865775,320.6068,325.5955,325.922375,330.1802,334.225025,338.8296,343.17965,347.84365,352.8396,359.04865,360.71755,366.77435,368.874825,373.00185,378.88335,379.70025,383.836975,387.551975,395.393,397.1417,403.527325,408.806125,414.807075,419.3798,426.12425,429.4522,435.41445};

#endif /* DEFINES_H_ */
