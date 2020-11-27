#ifndef CONFIG_PARAMUARTIMPORTER_H_
#define CONFIG_PARAMUARTIMPORTER_H_

void hoge1(int tgt_type, int id) {
	float p[12];
	for (char i = 0; i < 12; i++) {
		p[i] = *(float *) (id + 4 * i);
	}
	setPrms(tgt_type, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]);
	setPrms3(tgt_type, p[9], p[10], p[11]);
}

void set_param_from_normal(int tgt_type, int id) {
	if (tgt_type == Normal) {
		id += 64 * 0;
	} else if (tgt_type == Large) {
		id += 64 * 1;
	} else if (tgt_type == Orval) {
		id += 64 * 2;
	} else if (tgt_type == Dia45) {
		id += 64 * 3;
	} else if (tgt_type == Dia135) {
		id += 64 * 4;
	} else if (tgt_type == Dia90) {
		id += 64 * 5;
	}
	hoge1(tgt_type, id);
}
void set_param_from_large(int tgt_type, int id) {
	if (tgt_type == Large) {
		id += 64 * 0;
	} else if (tgt_type == Orval) {
		id += 64 * 1;
	} else if (tgt_type == Dia45) {
		id += 64 * 2;
	} else if (tgt_type == Dia135) {
		id += 64 * 3;
	} else if (tgt_type == Dia90) {
		id += 64 * 4;
	}
	hoge1(tgt_type, id);
}

void set_param_map(int tgt_type, float v) {
	int id = 0;
	if (v == 500) {
		id = 1050432;
		set_param_from_normal(tgt_type, id);
		return;
	} else if (v == 1000) {
		id = 1050816;
		set_param_from_normal(tgt_type, id);
		return;
	} else if (v == 1500) {
		id = 1051200;
	} else if (v == 1700) {
		id = 1051520;
	} else if (v == 1800) {
		id = 1051840;
	} else if (v == 1900) {
		id = 1052160;
	} else if (v == 1950) {
		id = 1052480;
	} else if (v == 2000) {
		id = 1052800;
	} else if (v == 2050) {
		id = 1053120;
	} else if (v == 2100) {
		id = 1053440;
	} else if (v == 2200) {
		id = 1053760;
	} else if (v == 2300) {
		id = 1054080;
	} else if (v == 2400) {
		id = 1054400;
	} else if (v == 2500) {
		id = 1054720;
	} else if (v == 2600) {
		id = 1055040;
	} else if (v == 2700) {
		id = 1055360;
	} else if (v == 2800) {
		id = 1055680;
	}
	set_param_from_large(tgt_type, id);
}
#endif
