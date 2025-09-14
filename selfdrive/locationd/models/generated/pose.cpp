#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8209044570432856386) {
   out_8209044570432856386[0] = delta_x[0] + nom_x[0];
   out_8209044570432856386[1] = delta_x[1] + nom_x[1];
   out_8209044570432856386[2] = delta_x[2] + nom_x[2];
   out_8209044570432856386[3] = delta_x[3] + nom_x[3];
   out_8209044570432856386[4] = delta_x[4] + nom_x[4];
   out_8209044570432856386[5] = delta_x[5] + nom_x[5];
   out_8209044570432856386[6] = delta_x[6] + nom_x[6];
   out_8209044570432856386[7] = delta_x[7] + nom_x[7];
   out_8209044570432856386[8] = delta_x[8] + nom_x[8];
   out_8209044570432856386[9] = delta_x[9] + nom_x[9];
   out_8209044570432856386[10] = delta_x[10] + nom_x[10];
   out_8209044570432856386[11] = delta_x[11] + nom_x[11];
   out_8209044570432856386[12] = delta_x[12] + nom_x[12];
   out_8209044570432856386[13] = delta_x[13] + nom_x[13];
   out_8209044570432856386[14] = delta_x[14] + nom_x[14];
   out_8209044570432856386[15] = delta_x[15] + nom_x[15];
   out_8209044570432856386[16] = delta_x[16] + nom_x[16];
   out_8209044570432856386[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4042472597088507909) {
   out_4042472597088507909[0] = -nom_x[0] + true_x[0];
   out_4042472597088507909[1] = -nom_x[1] + true_x[1];
   out_4042472597088507909[2] = -nom_x[2] + true_x[2];
   out_4042472597088507909[3] = -nom_x[3] + true_x[3];
   out_4042472597088507909[4] = -nom_x[4] + true_x[4];
   out_4042472597088507909[5] = -nom_x[5] + true_x[5];
   out_4042472597088507909[6] = -nom_x[6] + true_x[6];
   out_4042472597088507909[7] = -nom_x[7] + true_x[7];
   out_4042472597088507909[8] = -nom_x[8] + true_x[8];
   out_4042472597088507909[9] = -nom_x[9] + true_x[9];
   out_4042472597088507909[10] = -nom_x[10] + true_x[10];
   out_4042472597088507909[11] = -nom_x[11] + true_x[11];
   out_4042472597088507909[12] = -nom_x[12] + true_x[12];
   out_4042472597088507909[13] = -nom_x[13] + true_x[13];
   out_4042472597088507909[14] = -nom_x[14] + true_x[14];
   out_4042472597088507909[15] = -nom_x[15] + true_x[15];
   out_4042472597088507909[16] = -nom_x[16] + true_x[16];
   out_4042472597088507909[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3109392813306738693) {
   out_3109392813306738693[0] = 1.0;
   out_3109392813306738693[1] = 0.0;
   out_3109392813306738693[2] = 0.0;
   out_3109392813306738693[3] = 0.0;
   out_3109392813306738693[4] = 0.0;
   out_3109392813306738693[5] = 0.0;
   out_3109392813306738693[6] = 0.0;
   out_3109392813306738693[7] = 0.0;
   out_3109392813306738693[8] = 0.0;
   out_3109392813306738693[9] = 0.0;
   out_3109392813306738693[10] = 0.0;
   out_3109392813306738693[11] = 0.0;
   out_3109392813306738693[12] = 0.0;
   out_3109392813306738693[13] = 0.0;
   out_3109392813306738693[14] = 0.0;
   out_3109392813306738693[15] = 0.0;
   out_3109392813306738693[16] = 0.0;
   out_3109392813306738693[17] = 0.0;
   out_3109392813306738693[18] = 0.0;
   out_3109392813306738693[19] = 1.0;
   out_3109392813306738693[20] = 0.0;
   out_3109392813306738693[21] = 0.0;
   out_3109392813306738693[22] = 0.0;
   out_3109392813306738693[23] = 0.0;
   out_3109392813306738693[24] = 0.0;
   out_3109392813306738693[25] = 0.0;
   out_3109392813306738693[26] = 0.0;
   out_3109392813306738693[27] = 0.0;
   out_3109392813306738693[28] = 0.0;
   out_3109392813306738693[29] = 0.0;
   out_3109392813306738693[30] = 0.0;
   out_3109392813306738693[31] = 0.0;
   out_3109392813306738693[32] = 0.0;
   out_3109392813306738693[33] = 0.0;
   out_3109392813306738693[34] = 0.0;
   out_3109392813306738693[35] = 0.0;
   out_3109392813306738693[36] = 0.0;
   out_3109392813306738693[37] = 0.0;
   out_3109392813306738693[38] = 1.0;
   out_3109392813306738693[39] = 0.0;
   out_3109392813306738693[40] = 0.0;
   out_3109392813306738693[41] = 0.0;
   out_3109392813306738693[42] = 0.0;
   out_3109392813306738693[43] = 0.0;
   out_3109392813306738693[44] = 0.0;
   out_3109392813306738693[45] = 0.0;
   out_3109392813306738693[46] = 0.0;
   out_3109392813306738693[47] = 0.0;
   out_3109392813306738693[48] = 0.0;
   out_3109392813306738693[49] = 0.0;
   out_3109392813306738693[50] = 0.0;
   out_3109392813306738693[51] = 0.0;
   out_3109392813306738693[52] = 0.0;
   out_3109392813306738693[53] = 0.0;
   out_3109392813306738693[54] = 0.0;
   out_3109392813306738693[55] = 0.0;
   out_3109392813306738693[56] = 0.0;
   out_3109392813306738693[57] = 1.0;
   out_3109392813306738693[58] = 0.0;
   out_3109392813306738693[59] = 0.0;
   out_3109392813306738693[60] = 0.0;
   out_3109392813306738693[61] = 0.0;
   out_3109392813306738693[62] = 0.0;
   out_3109392813306738693[63] = 0.0;
   out_3109392813306738693[64] = 0.0;
   out_3109392813306738693[65] = 0.0;
   out_3109392813306738693[66] = 0.0;
   out_3109392813306738693[67] = 0.0;
   out_3109392813306738693[68] = 0.0;
   out_3109392813306738693[69] = 0.0;
   out_3109392813306738693[70] = 0.0;
   out_3109392813306738693[71] = 0.0;
   out_3109392813306738693[72] = 0.0;
   out_3109392813306738693[73] = 0.0;
   out_3109392813306738693[74] = 0.0;
   out_3109392813306738693[75] = 0.0;
   out_3109392813306738693[76] = 1.0;
   out_3109392813306738693[77] = 0.0;
   out_3109392813306738693[78] = 0.0;
   out_3109392813306738693[79] = 0.0;
   out_3109392813306738693[80] = 0.0;
   out_3109392813306738693[81] = 0.0;
   out_3109392813306738693[82] = 0.0;
   out_3109392813306738693[83] = 0.0;
   out_3109392813306738693[84] = 0.0;
   out_3109392813306738693[85] = 0.0;
   out_3109392813306738693[86] = 0.0;
   out_3109392813306738693[87] = 0.0;
   out_3109392813306738693[88] = 0.0;
   out_3109392813306738693[89] = 0.0;
   out_3109392813306738693[90] = 0.0;
   out_3109392813306738693[91] = 0.0;
   out_3109392813306738693[92] = 0.0;
   out_3109392813306738693[93] = 0.0;
   out_3109392813306738693[94] = 0.0;
   out_3109392813306738693[95] = 1.0;
   out_3109392813306738693[96] = 0.0;
   out_3109392813306738693[97] = 0.0;
   out_3109392813306738693[98] = 0.0;
   out_3109392813306738693[99] = 0.0;
   out_3109392813306738693[100] = 0.0;
   out_3109392813306738693[101] = 0.0;
   out_3109392813306738693[102] = 0.0;
   out_3109392813306738693[103] = 0.0;
   out_3109392813306738693[104] = 0.0;
   out_3109392813306738693[105] = 0.0;
   out_3109392813306738693[106] = 0.0;
   out_3109392813306738693[107] = 0.0;
   out_3109392813306738693[108] = 0.0;
   out_3109392813306738693[109] = 0.0;
   out_3109392813306738693[110] = 0.0;
   out_3109392813306738693[111] = 0.0;
   out_3109392813306738693[112] = 0.0;
   out_3109392813306738693[113] = 0.0;
   out_3109392813306738693[114] = 1.0;
   out_3109392813306738693[115] = 0.0;
   out_3109392813306738693[116] = 0.0;
   out_3109392813306738693[117] = 0.0;
   out_3109392813306738693[118] = 0.0;
   out_3109392813306738693[119] = 0.0;
   out_3109392813306738693[120] = 0.0;
   out_3109392813306738693[121] = 0.0;
   out_3109392813306738693[122] = 0.0;
   out_3109392813306738693[123] = 0.0;
   out_3109392813306738693[124] = 0.0;
   out_3109392813306738693[125] = 0.0;
   out_3109392813306738693[126] = 0.0;
   out_3109392813306738693[127] = 0.0;
   out_3109392813306738693[128] = 0.0;
   out_3109392813306738693[129] = 0.0;
   out_3109392813306738693[130] = 0.0;
   out_3109392813306738693[131] = 0.0;
   out_3109392813306738693[132] = 0.0;
   out_3109392813306738693[133] = 1.0;
   out_3109392813306738693[134] = 0.0;
   out_3109392813306738693[135] = 0.0;
   out_3109392813306738693[136] = 0.0;
   out_3109392813306738693[137] = 0.0;
   out_3109392813306738693[138] = 0.0;
   out_3109392813306738693[139] = 0.0;
   out_3109392813306738693[140] = 0.0;
   out_3109392813306738693[141] = 0.0;
   out_3109392813306738693[142] = 0.0;
   out_3109392813306738693[143] = 0.0;
   out_3109392813306738693[144] = 0.0;
   out_3109392813306738693[145] = 0.0;
   out_3109392813306738693[146] = 0.0;
   out_3109392813306738693[147] = 0.0;
   out_3109392813306738693[148] = 0.0;
   out_3109392813306738693[149] = 0.0;
   out_3109392813306738693[150] = 0.0;
   out_3109392813306738693[151] = 0.0;
   out_3109392813306738693[152] = 1.0;
   out_3109392813306738693[153] = 0.0;
   out_3109392813306738693[154] = 0.0;
   out_3109392813306738693[155] = 0.0;
   out_3109392813306738693[156] = 0.0;
   out_3109392813306738693[157] = 0.0;
   out_3109392813306738693[158] = 0.0;
   out_3109392813306738693[159] = 0.0;
   out_3109392813306738693[160] = 0.0;
   out_3109392813306738693[161] = 0.0;
   out_3109392813306738693[162] = 0.0;
   out_3109392813306738693[163] = 0.0;
   out_3109392813306738693[164] = 0.0;
   out_3109392813306738693[165] = 0.0;
   out_3109392813306738693[166] = 0.0;
   out_3109392813306738693[167] = 0.0;
   out_3109392813306738693[168] = 0.0;
   out_3109392813306738693[169] = 0.0;
   out_3109392813306738693[170] = 0.0;
   out_3109392813306738693[171] = 1.0;
   out_3109392813306738693[172] = 0.0;
   out_3109392813306738693[173] = 0.0;
   out_3109392813306738693[174] = 0.0;
   out_3109392813306738693[175] = 0.0;
   out_3109392813306738693[176] = 0.0;
   out_3109392813306738693[177] = 0.0;
   out_3109392813306738693[178] = 0.0;
   out_3109392813306738693[179] = 0.0;
   out_3109392813306738693[180] = 0.0;
   out_3109392813306738693[181] = 0.0;
   out_3109392813306738693[182] = 0.0;
   out_3109392813306738693[183] = 0.0;
   out_3109392813306738693[184] = 0.0;
   out_3109392813306738693[185] = 0.0;
   out_3109392813306738693[186] = 0.0;
   out_3109392813306738693[187] = 0.0;
   out_3109392813306738693[188] = 0.0;
   out_3109392813306738693[189] = 0.0;
   out_3109392813306738693[190] = 1.0;
   out_3109392813306738693[191] = 0.0;
   out_3109392813306738693[192] = 0.0;
   out_3109392813306738693[193] = 0.0;
   out_3109392813306738693[194] = 0.0;
   out_3109392813306738693[195] = 0.0;
   out_3109392813306738693[196] = 0.0;
   out_3109392813306738693[197] = 0.0;
   out_3109392813306738693[198] = 0.0;
   out_3109392813306738693[199] = 0.0;
   out_3109392813306738693[200] = 0.0;
   out_3109392813306738693[201] = 0.0;
   out_3109392813306738693[202] = 0.0;
   out_3109392813306738693[203] = 0.0;
   out_3109392813306738693[204] = 0.0;
   out_3109392813306738693[205] = 0.0;
   out_3109392813306738693[206] = 0.0;
   out_3109392813306738693[207] = 0.0;
   out_3109392813306738693[208] = 0.0;
   out_3109392813306738693[209] = 1.0;
   out_3109392813306738693[210] = 0.0;
   out_3109392813306738693[211] = 0.0;
   out_3109392813306738693[212] = 0.0;
   out_3109392813306738693[213] = 0.0;
   out_3109392813306738693[214] = 0.0;
   out_3109392813306738693[215] = 0.0;
   out_3109392813306738693[216] = 0.0;
   out_3109392813306738693[217] = 0.0;
   out_3109392813306738693[218] = 0.0;
   out_3109392813306738693[219] = 0.0;
   out_3109392813306738693[220] = 0.0;
   out_3109392813306738693[221] = 0.0;
   out_3109392813306738693[222] = 0.0;
   out_3109392813306738693[223] = 0.0;
   out_3109392813306738693[224] = 0.0;
   out_3109392813306738693[225] = 0.0;
   out_3109392813306738693[226] = 0.0;
   out_3109392813306738693[227] = 0.0;
   out_3109392813306738693[228] = 1.0;
   out_3109392813306738693[229] = 0.0;
   out_3109392813306738693[230] = 0.0;
   out_3109392813306738693[231] = 0.0;
   out_3109392813306738693[232] = 0.0;
   out_3109392813306738693[233] = 0.0;
   out_3109392813306738693[234] = 0.0;
   out_3109392813306738693[235] = 0.0;
   out_3109392813306738693[236] = 0.0;
   out_3109392813306738693[237] = 0.0;
   out_3109392813306738693[238] = 0.0;
   out_3109392813306738693[239] = 0.0;
   out_3109392813306738693[240] = 0.0;
   out_3109392813306738693[241] = 0.0;
   out_3109392813306738693[242] = 0.0;
   out_3109392813306738693[243] = 0.0;
   out_3109392813306738693[244] = 0.0;
   out_3109392813306738693[245] = 0.0;
   out_3109392813306738693[246] = 0.0;
   out_3109392813306738693[247] = 1.0;
   out_3109392813306738693[248] = 0.0;
   out_3109392813306738693[249] = 0.0;
   out_3109392813306738693[250] = 0.0;
   out_3109392813306738693[251] = 0.0;
   out_3109392813306738693[252] = 0.0;
   out_3109392813306738693[253] = 0.0;
   out_3109392813306738693[254] = 0.0;
   out_3109392813306738693[255] = 0.0;
   out_3109392813306738693[256] = 0.0;
   out_3109392813306738693[257] = 0.0;
   out_3109392813306738693[258] = 0.0;
   out_3109392813306738693[259] = 0.0;
   out_3109392813306738693[260] = 0.0;
   out_3109392813306738693[261] = 0.0;
   out_3109392813306738693[262] = 0.0;
   out_3109392813306738693[263] = 0.0;
   out_3109392813306738693[264] = 0.0;
   out_3109392813306738693[265] = 0.0;
   out_3109392813306738693[266] = 1.0;
   out_3109392813306738693[267] = 0.0;
   out_3109392813306738693[268] = 0.0;
   out_3109392813306738693[269] = 0.0;
   out_3109392813306738693[270] = 0.0;
   out_3109392813306738693[271] = 0.0;
   out_3109392813306738693[272] = 0.0;
   out_3109392813306738693[273] = 0.0;
   out_3109392813306738693[274] = 0.0;
   out_3109392813306738693[275] = 0.0;
   out_3109392813306738693[276] = 0.0;
   out_3109392813306738693[277] = 0.0;
   out_3109392813306738693[278] = 0.0;
   out_3109392813306738693[279] = 0.0;
   out_3109392813306738693[280] = 0.0;
   out_3109392813306738693[281] = 0.0;
   out_3109392813306738693[282] = 0.0;
   out_3109392813306738693[283] = 0.0;
   out_3109392813306738693[284] = 0.0;
   out_3109392813306738693[285] = 1.0;
   out_3109392813306738693[286] = 0.0;
   out_3109392813306738693[287] = 0.0;
   out_3109392813306738693[288] = 0.0;
   out_3109392813306738693[289] = 0.0;
   out_3109392813306738693[290] = 0.0;
   out_3109392813306738693[291] = 0.0;
   out_3109392813306738693[292] = 0.0;
   out_3109392813306738693[293] = 0.0;
   out_3109392813306738693[294] = 0.0;
   out_3109392813306738693[295] = 0.0;
   out_3109392813306738693[296] = 0.0;
   out_3109392813306738693[297] = 0.0;
   out_3109392813306738693[298] = 0.0;
   out_3109392813306738693[299] = 0.0;
   out_3109392813306738693[300] = 0.0;
   out_3109392813306738693[301] = 0.0;
   out_3109392813306738693[302] = 0.0;
   out_3109392813306738693[303] = 0.0;
   out_3109392813306738693[304] = 1.0;
   out_3109392813306738693[305] = 0.0;
   out_3109392813306738693[306] = 0.0;
   out_3109392813306738693[307] = 0.0;
   out_3109392813306738693[308] = 0.0;
   out_3109392813306738693[309] = 0.0;
   out_3109392813306738693[310] = 0.0;
   out_3109392813306738693[311] = 0.0;
   out_3109392813306738693[312] = 0.0;
   out_3109392813306738693[313] = 0.0;
   out_3109392813306738693[314] = 0.0;
   out_3109392813306738693[315] = 0.0;
   out_3109392813306738693[316] = 0.0;
   out_3109392813306738693[317] = 0.0;
   out_3109392813306738693[318] = 0.0;
   out_3109392813306738693[319] = 0.0;
   out_3109392813306738693[320] = 0.0;
   out_3109392813306738693[321] = 0.0;
   out_3109392813306738693[322] = 0.0;
   out_3109392813306738693[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2650242605500977184) {
   out_2650242605500977184[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2650242605500977184[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2650242605500977184[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2650242605500977184[3] = dt*state[12] + state[3];
   out_2650242605500977184[4] = dt*state[13] + state[4];
   out_2650242605500977184[5] = dt*state[14] + state[5];
   out_2650242605500977184[6] = state[6];
   out_2650242605500977184[7] = state[7];
   out_2650242605500977184[8] = state[8];
   out_2650242605500977184[9] = state[9];
   out_2650242605500977184[10] = state[10];
   out_2650242605500977184[11] = state[11];
   out_2650242605500977184[12] = state[12];
   out_2650242605500977184[13] = state[13];
   out_2650242605500977184[14] = state[14];
   out_2650242605500977184[15] = state[15];
   out_2650242605500977184[16] = state[16];
   out_2650242605500977184[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4407203996993581678) {
   out_4407203996993581678[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4407203996993581678[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4407203996993581678[2] = 0;
   out_4407203996993581678[3] = 0;
   out_4407203996993581678[4] = 0;
   out_4407203996993581678[5] = 0;
   out_4407203996993581678[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4407203996993581678[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4407203996993581678[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4407203996993581678[9] = 0;
   out_4407203996993581678[10] = 0;
   out_4407203996993581678[11] = 0;
   out_4407203996993581678[12] = 0;
   out_4407203996993581678[13] = 0;
   out_4407203996993581678[14] = 0;
   out_4407203996993581678[15] = 0;
   out_4407203996993581678[16] = 0;
   out_4407203996993581678[17] = 0;
   out_4407203996993581678[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4407203996993581678[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4407203996993581678[20] = 0;
   out_4407203996993581678[21] = 0;
   out_4407203996993581678[22] = 0;
   out_4407203996993581678[23] = 0;
   out_4407203996993581678[24] = 0;
   out_4407203996993581678[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4407203996993581678[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4407203996993581678[27] = 0;
   out_4407203996993581678[28] = 0;
   out_4407203996993581678[29] = 0;
   out_4407203996993581678[30] = 0;
   out_4407203996993581678[31] = 0;
   out_4407203996993581678[32] = 0;
   out_4407203996993581678[33] = 0;
   out_4407203996993581678[34] = 0;
   out_4407203996993581678[35] = 0;
   out_4407203996993581678[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4407203996993581678[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4407203996993581678[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4407203996993581678[39] = 0;
   out_4407203996993581678[40] = 0;
   out_4407203996993581678[41] = 0;
   out_4407203996993581678[42] = 0;
   out_4407203996993581678[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4407203996993581678[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4407203996993581678[45] = 0;
   out_4407203996993581678[46] = 0;
   out_4407203996993581678[47] = 0;
   out_4407203996993581678[48] = 0;
   out_4407203996993581678[49] = 0;
   out_4407203996993581678[50] = 0;
   out_4407203996993581678[51] = 0;
   out_4407203996993581678[52] = 0;
   out_4407203996993581678[53] = 0;
   out_4407203996993581678[54] = 0;
   out_4407203996993581678[55] = 0;
   out_4407203996993581678[56] = 0;
   out_4407203996993581678[57] = 1;
   out_4407203996993581678[58] = 0;
   out_4407203996993581678[59] = 0;
   out_4407203996993581678[60] = 0;
   out_4407203996993581678[61] = 0;
   out_4407203996993581678[62] = 0;
   out_4407203996993581678[63] = 0;
   out_4407203996993581678[64] = 0;
   out_4407203996993581678[65] = 0;
   out_4407203996993581678[66] = dt;
   out_4407203996993581678[67] = 0;
   out_4407203996993581678[68] = 0;
   out_4407203996993581678[69] = 0;
   out_4407203996993581678[70] = 0;
   out_4407203996993581678[71] = 0;
   out_4407203996993581678[72] = 0;
   out_4407203996993581678[73] = 0;
   out_4407203996993581678[74] = 0;
   out_4407203996993581678[75] = 0;
   out_4407203996993581678[76] = 1;
   out_4407203996993581678[77] = 0;
   out_4407203996993581678[78] = 0;
   out_4407203996993581678[79] = 0;
   out_4407203996993581678[80] = 0;
   out_4407203996993581678[81] = 0;
   out_4407203996993581678[82] = 0;
   out_4407203996993581678[83] = 0;
   out_4407203996993581678[84] = 0;
   out_4407203996993581678[85] = dt;
   out_4407203996993581678[86] = 0;
   out_4407203996993581678[87] = 0;
   out_4407203996993581678[88] = 0;
   out_4407203996993581678[89] = 0;
   out_4407203996993581678[90] = 0;
   out_4407203996993581678[91] = 0;
   out_4407203996993581678[92] = 0;
   out_4407203996993581678[93] = 0;
   out_4407203996993581678[94] = 0;
   out_4407203996993581678[95] = 1;
   out_4407203996993581678[96] = 0;
   out_4407203996993581678[97] = 0;
   out_4407203996993581678[98] = 0;
   out_4407203996993581678[99] = 0;
   out_4407203996993581678[100] = 0;
   out_4407203996993581678[101] = 0;
   out_4407203996993581678[102] = 0;
   out_4407203996993581678[103] = 0;
   out_4407203996993581678[104] = dt;
   out_4407203996993581678[105] = 0;
   out_4407203996993581678[106] = 0;
   out_4407203996993581678[107] = 0;
   out_4407203996993581678[108] = 0;
   out_4407203996993581678[109] = 0;
   out_4407203996993581678[110] = 0;
   out_4407203996993581678[111] = 0;
   out_4407203996993581678[112] = 0;
   out_4407203996993581678[113] = 0;
   out_4407203996993581678[114] = 1;
   out_4407203996993581678[115] = 0;
   out_4407203996993581678[116] = 0;
   out_4407203996993581678[117] = 0;
   out_4407203996993581678[118] = 0;
   out_4407203996993581678[119] = 0;
   out_4407203996993581678[120] = 0;
   out_4407203996993581678[121] = 0;
   out_4407203996993581678[122] = 0;
   out_4407203996993581678[123] = 0;
   out_4407203996993581678[124] = 0;
   out_4407203996993581678[125] = 0;
   out_4407203996993581678[126] = 0;
   out_4407203996993581678[127] = 0;
   out_4407203996993581678[128] = 0;
   out_4407203996993581678[129] = 0;
   out_4407203996993581678[130] = 0;
   out_4407203996993581678[131] = 0;
   out_4407203996993581678[132] = 0;
   out_4407203996993581678[133] = 1;
   out_4407203996993581678[134] = 0;
   out_4407203996993581678[135] = 0;
   out_4407203996993581678[136] = 0;
   out_4407203996993581678[137] = 0;
   out_4407203996993581678[138] = 0;
   out_4407203996993581678[139] = 0;
   out_4407203996993581678[140] = 0;
   out_4407203996993581678[141] = 0;
   out_4407203996993581678[142] = 0;
   out_4407203996993581678[143] = 0;
   out_4407203996993581678[144] = 0;
   out_4407203996993581678[145] = 0;
   out_4407203996993581678[146] = 0;
   out_4407203996993581678[147] = 0;
   out_4407203996993581678[148] = 0;
   out_4407203996993581678[149] = 0;
   out_4407203996993581678[150] = 0;
   out_4407203996993581678[151] = 0;
   out_4407203996993581678[152] = 1;
   out_4407203996993581678[153] = 0;
   out_4407203996993581678[154] = 0;
   out_4407203996993581678[155] = 0;
   out_4407203996993581678[156] = 0;
   out_4407203996993581678[157] = 0;
   out_4407203996993581678[158] = 0;
   out_4407203996993581678[159] = 0;
   out_4407203996993581678[160] = 0;
   out_4407203996993581678[161] = 0;
   out_4407203996993581678[162] = 0;
   out_4407203996993581678[163] = 0;
   out_4407203996993581678[164] = 0;
   out_4407203996993581678[165] = 0;
   out_4407203996993581678[166] = 0;
   out_4407203996993581678[167] = 0;
   out_4407203996993581678[168] = 0;
   out_4407203996993581678[169] = 0;
   out_4407203996993581678[170] = 0;
   out_4407203996993581678[171] = 1;
   out_4407203996993581678[172] = 0;
   out_4407203996993581678[173] = 0;
   out_4407203996993581678[174] = 0;
   out_4407203996993581678[175] = 0;
   out_4407203996993581678[176] = 0;
   out_4407203996993581678[177] = 0;
   out_4407203996993581678[178] = 0;
   out_4407203996993581678[179] = 0;
   out_4407203996993581678[180] = 0;
   out_4407203996993581678[181] = 0;
   out_4407203996993581678[182] = 0;
   out_4407203996993581678[183] = 0;
   out_4407203996993581678[184] = 0;
   out_4407203996993581678[185] = 0;
   out_4407203996993581678[186] = 0;
   out_4407203996993581678[187] = 0;
   out_4407203996993581678[188] = 0;
   out_4407203996993581678[189] = 0;
   out_4407203996993581678[190] = 1;
   out_4407203996993581678[191] = 0;
   out_4407203996993581678[192] = 0;
   out_4407203996993581678[193] = 0;
   out_4407203996993581678[194] = 0;
   out_4407203996993581678[195] = 0;
   out_4407203996993581678[196] = 0;
   out_4407203996993581678[197] = 0;
   out_4407203996993581678[198] = 0;
   out_4407203996993581678[199] = 0;
   out_4407203996993581678[200] = 0;
   out_4407203996993581678[201] = 0;
   out_4407203996993581678[202] = 0;
   out_4407203996993581678[203] = 0;
   out_4407203996993581678[204] = 0;
   out_4407203996993581678[205] = 0;
   out_4407203996993581678[206] = 0;
   out_4407203996993581678[207] = 0;
   out_4407203996993581678[208] = 0;
   out_4407203996993581678[209] = 1;
   out_4407203996993581678[210] = 0;
   out_4407203996993581678[211] = 0;
   out_4407203996993581678[212] = 0;
   out_4407203996993581678[213] = 0;
   out_4407203996993581678[214] = 0;
   out_4407203996993581678[215] = 0;
   out_4407203996993581678[216] = 0;
   out_4407203996993581678[217] = 0;
   out_4407203996993581678[218] = 0;
   out_4407203996993581678[219] = 0;
   out_4407203996993581678[220] = 0;
   out_4407203996993581678[221] = 0;
   out_4407203996993581678[222] = 0;
   out_4407203996993581678[223] = 0;
   out_4407203996993581678[224] = 0;
   out_4407203996993581678[225] = 0;
   out_4407203996993581678[226] = 0;
   out_4407203996993581678[227] = 0;
   out_4407203996993581678[228] = 1;
   out_4407203996993581678[229] = 0;
   out_4407203996993581678[230] = 0;
   out_4407203996993581678[231] = 0;
   out_4407203996993581678[232] = 0;
   out_4407203996993581678[233] = 0;
   out_4407203996993581678[234] = 0;
   out_4407203996993581678[235] = 0;
   out_4407203996993581678[236] = 0;
   out_4407203996993581678[237] = 0;
   out_4407203996993581678[238] = 0;
   out_4407203996993581678[239] = 0;
   out_4407203996993581678[240] = 0;
   out_4407203996993581678[241] = 0;
   out_4407203996993581678[242] = 0;
   out_4407203996993581678[243] = 0;
   out_4407203996993581678[244] = 0;
   out_4407203996993581678[245] = 0;
   out_4407203996993581678[246] = 0;
   out_4407203996993581678[247] = 1;
   out_4407203996993581678[248] = 0;
   out_4407203996993581678[249] = 0;
   out_4407203996993581678[250] = 0;
   out_4407203996993581678[251] = 0;
   out_4407203996993581678[252] = 0;
   out_4407203996993581678[253] = 0;
   out_4407203996993581678[254] = 0;
   out_4407203996993581678[255] = 0;
   out_4407203996993581678[256] = 0;
   out_4407203996993581678[257] = 0;
   out_4407203996993581678[258] = 0;
   out_4407203996993581678[259] = 0;
   out_4407203996993581678[260] = 0;
   out_4407203996993581678[261] = 0;
   out_4407203996993581678[262] = 0;
   out_4407203996993581678[263] = 0;
   out_4407203996993581678[264] = 0;
   out_4407203996993581678[265] = 0;
   out_4407203996993581678[266] = 1;
   out_4407203996993581678[267] = 0;
   out_4407203996993581678[268] = 0;
   out_4407203996993581678[269] = 0;
   out_4407203996993581678[270] = 0;
   out_4407203996993581678[271] = 0;
   out_4407203996993581678[272] = 0;
   out_4407203996993581678[273] = 0;
   out_4407203996993581678[274] = 0;
   out_4407203996993581678[275] = 0;
   out_4407203996993581678[276] = 0;
   out_4407203996993581678[277] = 0;
   out_4407203996993581678[278] = 0;
   out_4407203996993581678[279] = 0;
   out_4407203996993581678[280] = 0;
   out_4407203996993581678[281] = 0;
   out_4407203996993581678[282] = 0;
   out_4407203996993581678[283] = 0;
   out_4407203996993581678[284] = 0;
   out_4407203996993581678[285] = 1;
   out_4407203996993581678[286] = 0;
   out_4407203996993581678[287] = 0;
   out_4407203996993581678[288] = 0;
   out_4407203996993581678[289] = 0;
   out_4407203996993581678[290] = 0;
   out_4407203996993581678[291] = 0;
   out_4407203996993581678[292] = 0;
   out_4407203996993581678[293] = 0;
   out_4407203996993581678[294] = 0;
   out_4407203996993581678[295] = 0;
   out_4407203996993581678[296] = 0;
   out_4407203996993581678[297] = 0;
   out_4407203996993581678[298] = 0;
   out_4407203996993581678[299] = 0;
   out_4407203996993581678[300] = 0;
   out_4407203996993581678[301] = 0;
   out_4407203996993581678[302] = 0;
   out_4407203996993581678[303] = 0;
   out_4407203996993581678[304] = 1;
   out_4407203996993581678[305] = 0;
   out_4407203996993581678[306] = 0;
   out_4407203996993581678[307] = 0;
   out_4407203996993581678[308] = 0;
   out_4407203996993581678[309] = 0;
   out_4407203996993581678[310] = 0;
   out_4407203996993581678[311] = 0;
   out_4407203996993581678[312] = 0;
   out_4407203996993581678[313] = 0;
   out_4407203996993581678[314] = 0;
   out_4407203996993581678[315] = 0;
   out_4407203996993581678[316] = 0;
   out_4407203996993581678[317] = 0;
   out_4407203996993581678[318] = 0;
   out_4407203996993581678[319] = 0;
   out_4407203996993581678[320] = 0;
   out_4407203996993581678[321] = 0;
   out_4407203996993581678[322] = 0;
   out_4407203996993581678[323] = 1;
}
void h_4(double *state, double *unused, double *out_992674251783283689) {
   out_992674251783283689[0] = state[6] + state[9];
   out_992674251783283689[1] = state[7] + state[10];
   out_992674251783283689[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2043125571732342042) {
   out_2043125571732342042[0] = 0;
   out_2043125571732342042[1] = 0;
   out_2043125571732342042[2] = 0;
   out_2043125571732342042[3] = 0;
   out_2043125571732342042[4] = 0;
   out_2043125571732342042[5] = 0;
   out_2043125571732342042[6] = 1;
   out_2043125571732342042[7] = 0;
   out_2043125571732342042[8] = 0;
   out_2043125571732342042[9] = 1;
   out_2043125571732342042[10] = 0;
   out_2043125571732342042[11] = 0;
   out_2043125571732342042[12] = 0;
   out_2043125571732342042[13] = 0;
   out_2043125571732342042[14] = 0;
   out_2043125571732342042[15] = 0;
   out_2043125571732342042[16] = 0;
   out_2043125571732342042[17] = 0;
   out_2043125571732342042[18] = 0;
   out_2043125571732342042[19] = 0;
   out_2043125571732342042[20] = 0;
   out_2043125571732342042[21] = 0;
   out_2043125571732342042[22] = 0;
   out_2043125571732342042[23] = 0;
   out_2043125571732342042[24] = 0;
   out_2043125571732342042[25] = 1;
   out_2043125571732342042[26] = 0;
   out_2043125571732342042[27] = 0;
   out_2043125571732342042[28] = 1;
   out_2043125571732342042[29] = 0;
   out_2043125571732342042[30] = 0;
   out_2043125571732342042[31] = 0;
   out_2043125571732342042[32] = 0;
   out_2043125571732342042[33] = 0;
   out_2043125571732342042[34] = 0;
   out_2043125571732342042[35] = 0;
   out_2043125571732342042[36] = 0;
   out_2043125571732342042[37] = 0;
   out_2043125571732342042[38] = 0;
   out_2043125571732342042[39] = 0;
   out_2043125571732342042[40] = 0;
   out_2043125571732342042[41] = 0;
   out_2043125571732342042[42] = 0;
   out_2043125571732342042[43] = 0;
   out_2043125571732342042[44] = 1;
   out_2043125571732342042[45] = 0;
   out_2043125571732342042[46] = 0;
   out_2043125571732342042[47] = 1;
   out_2043125571732342042[48] = 0;
   out_2043125571732342042[49] = 0;
   out_2043125571732342042[50] = 0;
   out_2043125571732342042[51] = 0;
   out_2043125571732342042[52] = 0;
   out_2043125571732342042[53] = 0;
}
void h_10(double *state, double *unused, double *out_7169769923752438266) {
   out_7169769923752438266[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7169769923752438266[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7169769923752438266[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3475252269168967167) {
   out_3475252269168967167[0] = 0;
   out_3475252269168967167[1] = 9.8100000000000005*cos(state[1]);
   out_3475252269168967167[2] = 0;
   out_3475252269168967167[3] = 0;
   out_3475252269168967167[4] = -state[8];
   out_3475252269168967167[5] = state[7];
   out_3475252269168967167[6] = 0;
   out_3475252269168967167[7] = state[5];
   out_3475252269168967167[8] = -state[4];
   out_3475252269168967167[9] = 0;
   out_3475252269168967167[10] = 0;
   out_3475252269168967167[11] = 0;
   out_3475252269168967167[12] = 1;
   out_3475252269168967167[13] = 0;
   out_3475252269168967167[14] = 0;
   out_3475252269168967167[15] = 1;
   out_3475252269168967167[16] = 0;
   out_3475252269168967167[17] = 0;
   out_3475252269168967167[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3475252269168967167[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3475252269168967167[20] = 0;
   out_3475252269168967167[21] = state[8];
   out_3475252269168967167[22] = 0;
   out_3475252269168967167[23] = -state[6];
   out_3475252269168967167[24] = -state[5];
   out_3475252269168967167[25] = 0;
   out_3475252269168967167[26] = state[3];
   out_3475252269168967167[27] = 0;
   out_3475252269168967167[28] = 0;
   out_3475252269168967167[29] = 0;
   out_3475252269168967167[30] = 0;
   out_3475252269168967167[31] = 1;
   out_3475252269168967167[32] = 0;
   out_3475252269168967167[33] = 0;
   out_3475252269168967167[34] = 1;
   out_3475252269168967167[35] = 0;
   out_3475252269168967167[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3475252269168967167[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3475252269168967167[38] = 0;
   out_3475252269168967167[39] = -state[7];
   out_3475252269168967167[40] = state[6];
   out_3475252269168967167[41] = 0;
   out_3475252269168967167[42] = state[4];
   out_3475252269168967167[43] = -state[3];
   out_3475252269168967167[44] = 0;
   out_3475252269168967167[45] = 0;
   out_3475252269168967167[46] = 0;
   out_3475252269168967167[47] = 0;
   out_3475252269168967167[48] = 0;
   out_3475252269168967167[49] = 0;
   out_3475252269168967167[50] = 1;
   out_3475252269168967167[51] = 0;
   out_3475252269168967167[52] = 0;
   out_3475252269168967167[53] = 1;
}
void h_13(double *state, double *unused, double *out_7070279399632189089) {
   out_7070279399632189089[0] = state[3];
   out_7070279399632189089[1] = state[4];
   out_7070279399632189089[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5255399397064674843) {
   out_5255399397064674843[0] = 0;
   out_5255399397064674843[1] = 0;
   out_5255399397064674843[2] = 0;
   out_5255399397064674843[3] = 1;
   out_5255399397064674843[4] = 0;
   out_5255399397064674843[5] = 0;
   out_5255399397064674843[6] = 0;
   out_5255399397064674843[7] = 0;
   out_5255399397064674843[8] = 0;
   out_5255399397064674843[9] = 0;
   out_5255399397064674843[10] = 0;
   out_5255399397064674843[11] = 0;
   out_5255399397064674843[12] = 0;
   out_5255399397064674843[13] = 0;
   out_5255399397064674843[14] = 0;
   out_5255399397064674843[15] = 0;
   out_5255399397064674843[16] = 0;
   out_5255399397064674843[17] = 0;
   out_5255399397064674843[18] = 0;
   out_5255399397064674843[19] = 0;
   out_5255399397064674843[20] = 0;
   out_5255399397064674843[21] = 0;
   out_5255399397064674843[22] = 1;
   out_5255399397064674843[23] = 0;
   out_5255399397064674843[24] = 0;
   out_5255399397064674843[25] = 0;
   out_5255399397064674843[26] = 0;
   out_5255399397064674843[27] = 0;
   out_5255399397064674843[28] = 0;
   out_5255399397064674843[29] = 0;
   out_5255399397064674843[30] = 0;
   out_5255399397064674843[31] = 0;
   out_5255399397064674843[32] = 0;
   out_5255399397064674843[33] = 0;
   out_5255399397064674843[34] = 0;
   out_5255399397064674843[35] = 0;
   out_5255399397064674843[36] = 0;
   out_5255399397064674843[37] = 0;
   out_5255399397064674843[38] = 0;
   out_5255399397064674843[39] = 0;
   out_5255399397064674843[40] = 0;
   out_5255399397064674843[41] = 1;
   out_5255399397064674843[42] = 0;
   out_5255399397064674843[43] = 0;
   out_5255399397064674843[44] = 0;
   out_5255399397064674843[45] = 0;
   out_5255399397064674843[46] = 0;
   out_5255399397064674843[47] = 0;
   out_5255399397064674843[48] = 0;
   out_5255399397064674843[49] = 0;
   out_5255399397064674843[50] = 0;
   out_5255399397064674843[51] = 0;
   out_5255399397064674843[52] = 0;
   out_5255399397064674843[53] = 0;
}
void h_14(double *state, double *unused, double *out_1916798049807563076) {
   out_1916798049807563076[0] = state[6];
   out_1916798049807563076[1] = state[7];
   out_1916798049807563076[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6006366428071826571) {
   out_6006366428071826571[0] = 0;
   out_6006366428071826571[1] = 0;
   out_6006366428071826571[2] = 0;
   out_6006366428071826571[3] = 0;
   out_6006366428071826571[4] = 0;
   out_6006366428071826571[5] = 0;
   out_6006366428071826571[6] = 1;
   out_6006366428071826571[7] = 0;
   out_6006366428071826571[8] = 0;
   out_6006366428071826571[9] = 0;
   out_6006366428071826571[10] = 0;
   out_6006366428071826571[11] = 0;
   out_6006366428071826571[12] = 0;
   out_6006366428071826571[13] = 0;
   out_6006366428071826571[14] = 0;
   out_6006366428071826571[15] = 0;
   out_6006366428071826571[16] = 0;
   out_6006366428071826571[17] = 0;
   out_6006366428071826571[18] = 0;
   out_6006366428071826571[19] = 0;
   out_6006366428071826571[20] = 0;
   out_6006366428071826571[21] = 0;
   out_6006366428071826571[22] = 0;
   out_6006366428071826571[23] = 0;
   out_6006366428071826571[24] = 0;
   out_6006366428071826571[25] = 1;
   out_6006366428071826571[26] = 0;
   out_6006366428071826571[27] = 0;
   out_6006366428071826571[28] = 0;
   out_6006366428071826571[29] = 0;
   out_6006366428071826571[30] = 0;
   out_6006366428071826571[31] = 0;
   out_6006366428071826571[32] = 0;
   out_6006366428071826571[33] = 0;
   out_6006366428071826571[34] = 0;
   out_6006366428071826571[35] = 0;
   out_6006366428071826571[36] = 0;
   out_6006366428071826571[37] = 0;
   out_6006366428071826571[38] = 0;
   out_6006366428071826571[39] = 0;
   out_6006366428071826571[40] = 0;
   out_6006366428071826571[41] = 0;
   out_6006366428071826571[42] = 0;
   out_6006366428071826571[43] = 0;
   out_6006366428071826571[44] = 1;
   out_6006366428071826571[45] = 0;
   out_6006366428071826571[46] = 0;
   out_6006366428071826571[47] = 0;
   out_6006366428071826571[48] = 0;
   out_6006366428071826571[49] = 0;
   out_6006366428071826571[50] = 0;
   out_6006366428071826571[51] = 0;
   out_6006366428071826571[52] = 0;
   out_6006366428071826571[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_8209044570432856386) {
  err_fun(nom_x, delta_x, out_8209044570432856386);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4042472597088507909) {
  inv_err_fun(nom_x, true_x, out_4042472597088507909);
}
void pose_H_mod_fun(double *state, double *out_3109392813306738693) {
  H_mod_fun(state, out_3109392813306738693);
}
void pose_f_fun(double *state, double dt, double *out_2650242605500977184) {
  f_fun(state,  dt, out_2650242605500977184);
}
void pose_F_fun(double *state, double dt, double *out_4407203996993581678) {
  F_fun(state,  dt, out_4407203996993581678);
}
void pose_h_4(double *state, double *unused, double *out_992674251783283689) {
  h_4(state, unused, out_992674251783283689);
}
void pose_H_4(double *state, double *unused, double *out_2043125571732342042) {
  H_4(state, unused, out_2043125571732342042);
}
void pose_h_10(double *state, double *unused, double *out_7169769923752438266) {
  h_10(state, unused, out_7169769923752438266);
}
void pose_H_10(double *state, double *unused, double *out_3475252269168967167) {
  H_10(state, unused, out_3475252269168967167);
}
void pose_h_13(double *state, double *unused, double *out_7070279399632189089) {
  h_13(state, unused, out_7070279399632189089);
}
void pose_H_13(double *state, double *unused, double *out_5255399397064674843) {
  H_13(state, unused, out_5255399397064674843);
}
void pose_h_14(double *state, double *unused, double *out_1916798049807563076) {
  h_14(state, unused, out_1916798049807563076);
}
void pose_H_14(double *state, double *unused, double *out_6006366428071826571) {
  H_14(state, unused, out_6006366428071826571);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
