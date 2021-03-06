/*
 * calc_interp.h
 *
 *  Created on: 2020/11/16
 *      Author: nao12
 */

#ifndef CALC_INTERP_H_
#define CALC_INTERP_H_

#define linergain 5

#define INTERP(xi,xi1,yi,yi1,x) (yi + ((( yi1 - yi ) * ( x - xi )) / ( xi1 - xi )))
float interp_v_kp (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050152);
	gain_v[1] = (*(float *)1050156);
	gain_v[2] = (*(float *)1050160);
	gain_v[3] = (*(float *)1050164);
	gain_v[4] = (*(float *)1050168);

	gain_val[0] = (*(float *)1050092);
	gain_val[1] = (*(float *)1050104);
	gain_val[2] = (*(float *)1050116);
	gain_val[3] = (*(float *)1050128);
	gain_val[4] = (*(float *)1050140);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;	
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}
float interp_v_ki (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050152);
	gain_v[1] = (*(float *)1050156);
	gain_v[2] = (*(float *)1050160);
	gain_v[3] = (*(float *)1050164);
	gain_v[4] = (*(float *)1050168);

	gain_val[0] = (*(float *)1050096);
	gain_val[1] = (*(float *)1050108);
	gain_val[2] = (*(float *)1050120);
	gain_val[3] = (*(float *)1050132);
	gain_val[4] = (*(float *)1050144);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;	
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}

float interp_v_kd (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050152);
	gain_v[1] = (*(float *)1050156);
	gain_v[2] = (*(float *)1050160);
	gain_v[3] = (*(float *)1050164);
	gain_v[4] = (*(float *)1050168);

	gain_val[0] = (*(float *)1050100);
	gain_val[1] = (*(float *)1050112);
	gain_val[2] = (*(float *)1050124);
	gain_val[3] = (*(float *)1050136);
	gain_val[4] = (*(float *)1050148);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;	
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}

float interp_w_kp (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050232);
	gain_v[1] = (*(float *)1050236);
	gain_v[2] = (*(float *)1050240);
	gain_v[3] = (*(float *)1050244);
	gain_v[4] = (*(float *)1050248);

	gain_val[0] = (*(float *)1050172);
	gain_val[1] = (*(float *)1050184);
	gain_val[2] = (*(float *)1050196);
	gain_val[3] = (*(float *)1050208);
	gain_val[4] = (*(float *)1050220);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;	
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}

float interp_w_ki (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050232);
	gain_v[1] = (*(float *)1050236);
	gain_v[2] = (*(float *)1050240);
	gain_v[3] = (*(float *)1050244);
	gain_v[4] = (*(float *)1050248);

	gain_val[0] = (*(float *)1050176);
	gain_val[1] = (*(float *)1050188);
	gain_val[2] = (*(float *)1050200);
	gain_val[3] = (*(float *)1050212);
	gain_val[4] = (*(float *)1050224);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;	
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}

float interp_w_kd (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050232);
	gain_v[1] = (*(float *)1050236);
	gain_v[2] = (*(float *)1050240);
	gain_v[3] = (*(float *)1050244);
	gain_v[4] = (*(float *)1050248);

	gain_val[0] = (*(float *)1050180);
	gain_val[1] = (*(float *)1050192);
	gain_val[2] = (*(float *)1050204);
	gain_val[3] = (*(float *)1050216);
	gain_val[4] = (*(float *)1050228);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;	
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}

float interp_s_kp (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050312);
	gain_v[1] = (*(float *)1050316);
	gain_v[2] = (*(float *)1050320);
	gain_v[3] = (*(float *)1050324);
	gain_v[4] = (*(float *)1050328);

	gain_val[0] = (*(float *)1050252);
	gain_val[1] = (*(float *)1050264);
	gain_val[2] = (*(float *)1050276);
	gain_val[3] = (*(float *)1050288);
	gain_val[4] = (*(float *)1050300);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}
float interp_s_ki (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050312);
	gain_v[1] = (*(float *)1050316);
	gain_v[2] = (*(float *)1050320);
	gain_v[3] = (*(float *)1050324);
	gain_v[4] = (*(float *)1050328);

	gain_val[0] = (*(float *)1050256);
	gain_val[1] = (*(float *)1050268);
	gain_val[2] = (*(float *)1050280);
	gain_val[3] = (*(float *)1050292);
	gain_val[4] = (*(float *)1050304);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}

float interp_s_kd (float v){
    float gain_v[linergain];
    float gain_val[linergain];
	gain_v[0] = (*(float *)1050312);
	gain_v[1] = (*(float *)1050316);
	gain_v[2] = (*(float *)1050320);
	gain_v[3] = (*(float *)1050324);
	gain_v[4] = (*(float *)1050328);

	gain_val[0] = (*(float *)1050260);
	gain_val[1] = (*(float *)1050272);
	gain_val[2] = (*(float *)1050284);
	gain_val[3] = (*(float *)1050296);
	gain_val[4] = (*(float *)1050308);
	if(v<=gain_v[0]){
		return gain_val[0];
	}
	if(v>=gain_v[4]){
		return gain_val[4];
	}
	int i=0;
	for(i=1;i<linergain;i++){
			if(gain_v[i]>=v)
				break;
	}
	return INTERP(gain_v[i - 1], gain_v[i], gain_val[i - 1], gain_val[i], v);
}



#endif /* CALC_INTERP_H_ */
