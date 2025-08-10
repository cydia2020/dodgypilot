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
void err_fun(double *nom_x, double *delta_x, double *out_5046380216439218526) {
   out_5046380216439218526[0] = delta_x[0] + nom_x[0];
   out_5046380216439218526[1] = delta_x[1] + nom_x[1];
   out_5046380216439218526[2] = delta_x[2] + nom_x[2];
   out_5046380216439218526[3] = delta_x[3] + nom_x[3];
   out_5046380216439218526[4] = delta_x[4] + nom_x[4];
   out_5046380216439218526[5] = delta_x[5] + nom_x[5];
   out_5046380216439218526[6] = delta_x[6] + nom_x[6];
   out_5046380216439218526[7] = delta_x[7] + nom_x[7];
   out_5046380216439218526[8] = delta_x[8] + nom_x[8];
   out_5046380216439218526[9] = delta_x[9] + nom_x[9];
   out_5046380216439218526[10] = delta_x[10] + nom_x[10];
   out_5046380216439218526[11] = delta_x[11] + nom_x[11];
   out_5046380216439218526[12] = delta_x[12] + nom_x[12];
   out_5046380216439218526[13] = delta_x[13] + nom_x[13];
   out_5046380216439218526[14] = delta_x[14] + nom_x[14];
   out_5046380216439218526[15] = delta_x[15] + nom_x[15];
   out_5046380216439218526[16] = delta_x[16] + nom_x[16];
   out_5046380216439218526[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2577348512298300834) {
   out_2577348512298300834[0] = -nom_x[0] + true_x[0];
   out_2577348512298300834[1] = -nom_x[1] + true_x[1];
   out_2577348512298300834[2] = -nom_x[2] + true_x[2];
   out_2577348512298300834[3] = -nom_x[3] + true_x[3];
   out_2577348512298300834[4] = -nom_x[4] + true_x[4];
   out_2577348512298300834[5] = -nom_x[5] + true_x[5];
   out_2577348512298300834[6] = -nom_x[6] + true_x[6];
   out_2577348512298300834[7] = -nom_x[7] + true_x[7];
   out_2577348512298300834[8] = -nom_x[8] + true_x[8];
   out_2577348512298300834[9] = -nom_x[9] + true_x[9];
   out_2577348512298300834[10] = -nom_x[10] + true_x[10];
   out_2577348512298300834[11] = -nom_x[11] + true_x[11];
   out_2577348512298300834[12] = -nom_x[12] + true_x[12];
   out_2577348512298300834[13] = -nom_x[13] + true_x[13];
   out_2577348512298300834[14] = -nom_x[14] + true_x[14];
   out_2577348512298300834[15] = -nom_x[15] + true_x[15];
   out_2577348512298300834[16] = -nom_x[16] + true_x[16];
   out_2577348512298300834[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_490162499866022473) {
   out_490162499866022473[0] = 1.0;
   out_490162499866022473[1] = 0.0;
   out_490162499866022473[2] = 0.0;
   out_490162499866022473[3] = 0.0;
   out_490162499866022473[4] = 0.0;
   out_490162499866022473[5] = 0.0;
   out_490162499866022473[6] = 0.0;
   out_490162499866022473[7] = 0.0;
   out_490162499866022473[8] = 0.0;
   out_490162499866022473[9] = 0.0;
   out_490162499866022473[10] = 0.0;
   out_490162499866022473[11] = 0.0;
   out_490162499866022473[12] = 0.0;
   out_490162499866022473[13] = 0.0;
   out_490162499866022473[14] = 0.0;
   out_490162499866022473[15] = 0.0;
   out_490162499866022473[16] = 0.0;
   out_490162499866022473[17] = 0.0;
   out_490162499866022473[18] = 0.0;
   out_490162499866022473[19] = 1.0;
   out_490162499866022473[20] = 0.0;
   out_490162499866022473[21] = 0.0;
   out_490162499866022473[22] = 0.0;
   out_490162499866022473[23] = 0.0;
   out_490162499866022473[24] = 0.0;
   out_490162499866022473[25] = 0.0;
   out_490162499866022473[26] = 0.0;
   out_490162499866022473[27] = 0.0;
   out_490162499866022473[28] = 0.0;
   out_490162499866022473[29] = 0.0;
   out_490162499866022473[30] = 0.0;
   out_490162499866022473[31] = 0.0;
   out_490162499866022473[32] = 0.0;
   out_490162499866022473[33] = 0.0;
   out_490162499866022473[34] = 0.0;
   out_490162499866022473[35] = 0.0;
   out_490162499866022473[36] = 0.0;
   out_490162499866022473[37] = 0.0;
   out_490162499866022473[38] = 1.0;
   out_490162499866022473[39] = 0.0;
   out_490162499866022473[40] = 0.0;
   out_490162499866022473[41] = 0.0;
   out_490162499866022473[42] = 0.0;
   out_490162499866022473[43] = 0.0;
   out_490162499866022473[44] = 0.0;
   out_490162499866022473[45] = 0.0;
   out_490162499866022473[46] = 0.0;
   out_490162499866022473[47] = 0.0;
   out_490162499866022473[48] = 0.0;
   out_490162499866022473[49] = 0.0;
   out_490162499866022473[50] = 0.0;
   out_490162499866022473[51] = 0.0;
   out_490162499866022473[52] = 0.0;
   out_490162499866022473[53] = 0.0;
   out_490162499866022473[54] = 0.0;
   out_490162499866022473[55] = 0.0;
   out_490162499866022473[56] = 0.0;
   out_490162499866022473[57] = 1.0;
   out_490162499866022473[58] = 0.0;
   out_490162499866022473[59] = 0.0;
   out_490162499866022473[60] = 0.0;
   out_490162499866022473[61] = 0.0;
   out_490162499866022473[62] = 0.0;
   out_490162499866022473[63] = 0.0;
   out_490162499866022473[64] = 0.0;
   out_490162499866022473[65] = 0.0;
   out_490162499866022473[66] = 0.0;
   out_490162499866022473[67] = 0.0;
   out_490162499866022473[68] = 0.0;
   out_490162499866022473[69] = 0.0;
   out_490162499866022473[70] = 0.0;
   out_490162499866022473[71] = 0.0;
   out_490162499866022473[72] = 0.0;
   out_490162499866022473[73] = 0.0;
   out_490162499866022473[74] = 0.0;
   out_490162499866022473[75] = 0.0;
   out_490162499866022473[76] = 1.0;
   out_490162499866022473[77] = 0.0;
   out_490162499866022473[78] = 0.0;
   out_490162499866022473[79] = 0.0;
   out_490162499866022473[80] = 0.0;
   out_490162499866022473[81] = 0.0;
   out_490162499866022473[82] = 0.0;
   out_490162499866022473[83] = 0.0;
   out_490162499866022473[84] = 0.0;
   out_490162499866022473[85] = 0.0;
   out_490162499866022473[86] = 0.0;
   out_490162499866022473[87] = 0.0;
   out_490162499866022473[88] = 0.0;
   out_490162499866022473[89] = 0.0;
   out_490162499866022473[90] = 0.0;
   out_490162499866022473[91] = 0.0;
   out_490162499866022473[92] = 0.0;
   out_490162499866022473[93] = 0.0;
   out_490162499866022473[94] = 0.0;
   out_490162499866022473[95] = 1.0;
   out_490162499866022473[96] = 0.0;
   out_490162499866022473[97] = 0.0;
   out_490162499866022473[98] = 0.0;
   out_490162499866022473[99] = 0.0;
   out_490162499866022473[100] = 0.0;
   out_490162499866022473[101] = 0.0;
   out_490162499866022473[102] = 0.0;
   out_490162499866022473[103] = 0.0;
   out_490162499866022473[104] = 0.0;
   out_490162499866022473[105] = 0.0;
   out_490162499866022473[106] = 0.0;
   out_490162499866022473[107] = 0.0;
   out_490162499866022473[108] = 0.0;
   out_490162499866022473[109] = 0.0;
   out_490162499866022473[110] = 0.0;
   out_490162499866022473[111] = 0.0;
   out_490162499866022473[112] = 0.0;
   out_490162499866022473[113] = 0.0;
   out_490162499866022473[114] = 1.0;
   out_490162499866022473[115] = 0.0;
   out_490162499866022473[116] = 0.0;
   out_490162499866022473[117] = 0.0;
   out_490162499866022473[118] = 0.0;
   out_490162499866022473[119] = 0.0;
   out_490162499866022473[120] = 0.0;
   out_490162499866022473[121] = 0.0;
   out_490162499866022473[122] = 0.0;
   out_490162499866022473[123] = 0.0;
   out_490162499866022473[124] = 0.0;
   out_490162499866022473[125] = 0.0;
   out_490162499866022473[126] = 0.0;
   out_490162499866022473[127] = 0.0;
   out_490162499866022473[128] = 0.0;
   out_490162499866022473[129] = 0.0;
   out_490162499866022473[130] = 0.0;
   out_490162499866022473[131] = 0.0;
   out_490162499866022473[132] = 0.0;
   out_490162499866022473[133] = 1.0;
   out_490162499866022473[134] = 0.0;
   out_490162499866022473[135] = 0.0;
   out_490162499866022473[136] = 0.0;
   out_490162499866022473[137] = 0.0;
   out_490162499866022473[138] = 0.0;
   out_490162499866022473[139] = 0.0;
   out_490162499866022473[140] = 0.0;
   out_490162499866022473[141] = 0.0;
   out_490162499866022473[142] = 0.0;
   out_490162499866022473[143] = 0.0;
   out_490162499866022473[144] = 0.0;
   out_490162499866022473[145] = 0.0;
   out_490162499866022473[146] = 0.0;
   out_490162499866022473[147] = 0.0;
   out_490162499866022473[148] = 0.0;
   out_490162499866022473[149] = 0.0;
   out_490162499866022473[150] = 0.0;
   out_490162499866022473[151] = 0.0;
   out_490162499866022473[152] = 1.0;
   out_490162499866022473[153] = 0.0;
   out_490162499866022473[154] = 0.0;
   out_490162499866022473[155] = 0.0;
   out_490162499866022473[156] = 0.0;
   out_490162499866022473[157] = 0.0;
   out_490162499866022473[158] = 0.0;
   out_490162499866022473[159] = 0.0;
   out_490162499866022473[160] = 0.0;
   out_490162499866022473[161] = 0.0;
   out_490162499866022473[162] = 0.0;
   out_490162499866022473[163] = 0.0;
   out_490162499866022473[164] = 0.0;
   out_490162499866022473[165] = 0.0;
   out_490162499866022473[166] = 0.0;
   out_490162499866022473[167] = 0.0;
   out_490162499866022473[168] = 0.0;
   out_490162499866022473[169] = 0.0;
   out_490162499866022473[170] = 0.0;
   out_490162499866022473[171] = 1.0;
   out_490162499866022473[172] = 0.0;
   out_490162499866022473[173] = 0.0;
   out_490162499866022473[174] = 0.0;
   out_490162499866022473[175] = 0.0;
   out_490162499866022473[176] = 0.0;
   out_490162499866022473[177] = 0.0;
   out_490162499866022473[178] = 0.0;
   out_490162499866022473[179] = 0.0;
   out_490162499866022473[180] = 0.0;
   out_490162499866022473[181] = 0.0;
   out_490162499866022473[182] = 0.0;
   out_490162499866022473[183] = 0.0;
   out_490162499866022473[184] = 0.0;
   out_490162499866022473[185] = 0.0;
   out_490162499866022473[186] = 0.0;
   out_490162499866022473[187] = 0.0;
   out_490162499866022473[188] = 0.0;
   out_490162499866022473[189] = 0.0;
   out_490162499866022473[190] = 1.0;
   out_490162499866022473[191] = 0.0;
   out_490162499866022473[192] = 0.0;
   out_490162499866022473[193] = 0.0;
   out_490162499866022473[194] = 0.0;
   out_490162499866022473[195] = 0.0;
   out_490162499866022473[196] = 0.0;
   out_490162499866022473[197] = 0.0;
   out_490162499866022473[198] = 0.0;
   out_490162499866022473[199] = 0.0;
   out_490162499866022473[200] = 0.0;
   out_490162499866022473[201] = 0.0;
   out_490162499866022473[202] = 0.0;
   out_490162499866022473[203] = 0.0;
   out_490162499866022473[204] = 0.0;
   out_490162499866022473[205] = 0.0;
   out_490162499866022473[206] = 0.0;
   out_490162499866022473[207] = 0.0;
   out_490162499866022473[208] = 0.0;
   out_490162499866022473[209] = 1.0;
   out_490162499866022473[210] = 0.0;
   out_490162499866022473[211] = 0.0;
   out_490162499866022473[212] = 0.0;
   out_490162499866022473[213] = 0.0;
   out_490162499866022473[214] = 0.0;
   out_490162499866022473[215] = 0.0;
   out_490162499866022473[216] = 0.0;
   out_490162499866022473[217] = 0.0;
   out_490162499866022473[218] = 0.0;
   out_490162499866022473[219] = 0.0;
   out_490162499866022473[220] = 0.0;
   out_490162499866022473[221] = 0.0;
   out_490162499866022473[222] = 0.0;
   out_490162499866022473[223] = 0.0;
   out_490162499866022473[224] = 0.0;
   out_490162499866022473[225] = 0.0;
   out_490162499866022473[226] = 0.0;
   out_490162499866022473[227] = 0.0;
   out_490162499866022473[228] = 1.0;
   out_490162499866022473[229] = 0.0;
   out_490162499866022473[230] = 0.0;
   out_490162499866022473[231] = 0.0;
   out_490162499866022473[232] = 0.0;
   out_490162499866022473[233] = 0.0;
   out_490162499866022473[234] = 0.0;
   out_490162499866022473[235] = 0.0;
   out_490162499866022473[236] = 0.0;
   out_490162499866022473[237] = 0.0;
   out_490162499866022473[238] = 0.0;
   out_490162499866022473[239] = 0.0;
   out_490162499866022473[240] = 0.0;
   out_490162499866022473[241] = 0.0;
   out_490162499866022473[242] = 0.0;
   out_490162499866022473[243] = 0.0;
   out_490162499866022473[244] = 0.0;
   out_490162499866022473[245] = 0.0;
   out_490162499866022473[246] = 0.0;
   out_490162499866022473[247] = 1.0;
   out_490162499866022473[248] = 0.0;
   out_490162499866022473[249] = 0.0;
   out_490162499866022473[250] = 0.0;
   out_490162499866022473[251] = 0.0;
   out_490162499866022473[252] = 0.0;
   out_490162499866022473[253] = 0.0;
   out_490162499866022473[254] = 0.0;
   out_490162499866022473[255] = 0.0;
   out_490162499866022473[256] = 0.0;
   out_490162499866022473[257] = 0.0;
   out_490162499866022473[258] = 0.0;
   out_490162499866022473[259] = 0.0;
   out_490162499866022473[260] = 0.0;
   out_490162499866022473[261] = 0.0;
   out_490162499866022473[262] = 0.0;
   out_490162499866022473[263] = 0.0;
   out_490162499866022473[264] = 0.0;
   out_490162499866022473[265] = 0.0;
   out_490162499866022473[266] = 1.0;
   out_490162499866022473[267] = 0.0;
   out_490162499866022473[268] = 0.0;
   out_490162499866022473[269] = 0.0;
   out_490162499866022473[270] = 0.0;
   out_490162499866022473[271] = 0.0;
   out_490162499866022473[272] = 0.0;
   out_490162499866022473[273] = 0.0;
   out_490162499866022473[274] = 0.0;
   out_490162499866022473[275] = 0.0;
   out_490162499866022473[276] = 0.0;
   out_490162499866022473[277] = 0.0;
   out_490162499866022473[278] = 0.0;
   out_490162499866022473[279] = 0.0;
   out_490162499866022473[280] = 0.0;
   out_490162499866022473[281] = 0.0;
   out_490162499866022473[282] = 0.0;
   out_490162499866022473[283] = 0.0;
   out_490162499866022473[284] = 0.0;
   out_490162499866022473[285] = 1.0;
   out_490162499866022473[286] = 0.0;
   out_490162499866022473[287] = 0.0;
   out_490162499866022473[288] = 0.0;
   out_490162499866022473[289] = 0.0;
   out_490162499866022473[290] = 0.0;
   out_490162499866022473[291] = 0.0;
   out_490162499866022473[292] = 0.0;
   out_490162499866022473[293] = 0.0;
   out_490162499866022473[294] = 0.0;
   out_490162499866022473[295] = 0.0;
   out_490162499866022473[296] = 0.0;
   out_490162499866022473[297] = 0.0;
   out_490162499866022473[298] = 0.0;
   out_490162499866022473[299] = 0.0;
   out_490162499866022473[300] = 0.0;
   out_490162499866022473[301] = 0.0;
   out_490162499866022473[302] = 0.0;
   out_490162499866022473[303] = 0.0;
   out_490162499866022473[304] = 1.0;
   out_490162499866022473[305] = 0.0;
   out_490162499866022473[306] = 0.0;
   out_490162499866022473[307] = 0.0;
   out_490162499866022473[308] = 0.0;
   out_490162499866022473[309] = 0.0;
   out_490162499866022473[310] = 0.0;
   out_490162499866022473[311] = 0.0;
   out_490162499866022473[312] = 0.0;
   out_490162499866022473[313] = 0.0;
   out_490162499866022473[314] = 0.0;
   out_490162499866022473[315] = 0.0;
   out_490162499866022473[316] = 0.0;
   out_490162499866022473[317] = 0.0;
   out_490162499866022473[318] = 0.0;
   out_490162499866022473[319] = 0.0;
   out_490162499866022473[320] = 0.0;
   out_490162499866022473[321] = 0.0;
   out_490162499866022473[322] = 0.0;
   out_490162499866022473[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1869185704993173059) {
   out_1869185704993173059[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1869185704993173059[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1869185704993173059[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1869185704993173059[3] = dt*state[12] + state[3];
   out_1869185704993173059[4] = dt*state[13] + state[4];
   out_1869185704993173059[5] = dt*state[14] + state[5];
   out_1869185704993173059[6] = state[6];
   out_1869185704993173059[7] = state[7];
   out_1869185704993173059[8] = state[8];
   out_1869185704993173059[9] = state[9];
   out_1869185704993173059[10] = state[10];
   out_1869185704993173059[11] = state[11];
   out_1869185704993173059[12] = state[12];
   out_1869185704993173059[13] = state[13];
   out_1869185704993173059[14] = state[14];
   out_1869185704993173059[15] = state[15];
   out_1869185704993173059[16] = state[16];
   out_1869185704993173059[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1284215992985263392) {
   out_1284215992985263392[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1284215992985263392[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1284215992985263392[2] = 0;
   out_1284215992985263392[3] = 0;
   out_1284215992985263392[4] = 0;
   out_1284215992985263392[5] = 0;
   out_1284215992985263392[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1284215992985263392[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1284215992985263392[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1284215992985263392[9] = 0;
   out_1284215992985263392[10] = 0;
   out_1284215992985263392[11] = 0;
   out_1284215992985263392[12] = 0;
   out_1284215992985263392[13] = 0;
   out_1284215992985263392[14] = 0;
   out_1284215992985263392[15] = 0;
   out_1284215992985263392[16] = 0;
   out_1284215992985263392[17] = 0;
   out_1284215992985263392[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1284215992985263392[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1284215992985263392[20] = 0;
   out_1284215992985263392[21] = 0;
   out_1284215992985263392[22] = 0;
   out_1284215992985263392[23] = 0;
   out_1284215992985263392[24] = 0;
   out_1284215992985263392[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1284215992985263392[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1284215992985263392[27] = 0;
   out_1284215992985263392[28] = 0;
   out_1284215992985263392[29] = 0;
   out_1284215992985263392[30] = 0;
   out_1284215992985263392[31] = 0;
   out_1284215992985263392[32] = 0;
   out_1284215992985263392[33] = 0;
   out_1284215992985263392[34] = 0;
   out_1284215992985263392[35] = 0;
   out_1284215992985263392[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1284215992985263392[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1284215992985263392[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1284215992985263392[39] = 0;
   out_1284215992985263392[40] = 0;
   out_1284215992985263392[41] = 0;
   out_1284215992985263392[42] = 0;
   out_1284215992985263392[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1284215992985263392[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1284215992985263392[45] = 0;
   out_1284215992985263392[46] = 0;
   out_1284215992985263392[47] = 0;
   out_1284215992985263392[48] = 0;
   out_1284215992985263392[49] = 0;
   out_1284215992985263392[50] = 0;
   out_1284215992985263392[51] = 0;
   out_1284215992985263392[52] = 0;
   out_1284215992985263392[53] = 0;
   out_1284215992985263392[54] = 0;
   out_1284215992985263392[55] = 0;
   out_1284215992985263392[56] = 0;
   out_1284215992985263392[57] = 1;
   out_1284215992985263392[58] = 0;
   out_1284215992985263392[59] = 0;
   out_1284215992985263392[60] = 0;
   out_1284215992985263392[61] = 0;
   out_1284215992985263392[62] = 0;
   out_1284215992985263392[63] = 0;
   out_1284215992985263392[64] = 0;
   out_1284215992985263392[65] = 0;
   out_1284215992985263392[66] = dt;
   out_1284215992985263392[67] = 0;
   out_1284215992985263392[68] = 0;
   out_1284215992985263392[69] = 0;
   out_1284215992985263392[70] = 0;
   out_1284215992985263392[71] = 0;
   out_1284215992985263392[72] = 0;
   out_1284215992985263392[73] = 0;
   out_1284215992985263392[74] = 0;
   out_1284215992985263392[75] = 0;
   out_1284215992985263392[76] = 1;
   out_1284215992985263392[77] = 0;
   out_1284215992985263392[78] = 0;
   out_1284215992985263392[79] = 0;
   out_1284215992985263392[80] = 0;
   out_1284215992985263392[81] = 0;
   out_1284215992985263392[82] = 0;
   out_1284215992985263392[83] = 0;
   out_1284215992985263392[84] = 0;
   out_1284215992985263392[85] = dt;
   out_1284215992985263392[86] = 0;
   out_1284215992985263392[87] = 0;
   out_1284215992985263392[88] = 0;
   out_1284215992985263392[89] = 0;
   out_1284215992985263392[90] = 0;
   out_1284215992985263392[91] = 0;
   out_1284215992985263392[92] = 0;
   out_1284215992985263392[93] = 0;
   out_1284215992985263392[94] = 0;
   out_1284215992985263392[95] = 1;
   out_1284215992985263392[96] = 0;
   out_1284215992985263392[97] = 0;
   out_1284215992985263392[98] = 0;
   out_1284215992985263392[99] = 0;
   out_1284215992985263392[100] = 0;
   out_1284215992985263392[101] = 0;
   out_1284215992985263392[102] = 0;
   out_1284215992985263392[103] = 0;
   out_1284215992985263392[104] = dt;
   out_1284215992985263392[105] = 0;
   out_1284215992985263392[106] = 0;
   out_1284215992985263392[107] = 0;
   out_1284215992985263392[108] = 0;
   out_1284215992985263392[109] = 0;
   out_1284215992985263392[110] = 0;
   out_1284215992985263392[111] = 0;
   out_1284215992985263392[112] = 0;
   out_1284215992985263392[113] = 0;
   out_1284215992985263392[114] = 1;
   out_1284215992985263392[115] = 0;
   out_1284215992985263392[116] = 0;
   out_1284215992985263392[117] = 0;
   out_1284215992985263392[118] = 0;
   out_1284215992985263392[119] = 0;
   out_1284215992985263392[120] = 0;
   out_1284215992985263392[121] = 0;
   out_1284215992985263392[122] = 0;
   out_1284215992985263392[123] = 0;
   out_1284215992985263392[124] = 0;
   out_1284215992985263392[125] = 0;
   out_1284215992985263392[126] = 0;
   out_1284215992985263392[127] = 0;
   out_1284215992985263392[128] = 0;
   out_1284215992985263392[129] = 0;
   out_1284215992985263392[130] = 0;
   out_1284215992985263392[131] = 0;
   out_1284215992985263392[132] = 0;
   out_1284215992985263392[133] = 1;
   out_1284215992985263392[134] = 0;
   out_1284215992985263392[135] = 0;
   out_1284215992985263392[136] = 0;
   out_1284215992985263392[137] = 0;
   out_1284215992985263392[138] = 0;
   out_1284215992985263392[139] = 0;
   out_1284215992985263392[140] = 0;
   out_1284215992985263392[141] = 0;
   out_1284215992985263392[142] = 0;
   out_1284215992985263392[143] = 0;
   out_1284215992985263392[144] = 0;
   out_1284215992985263392[145] = 0;
   out_1284215992985263392[146] = 0;
   out_1284215992985263392[147] = 0;
   out_1284215992985263392[148] = 0;
   out_1284215992985263392[149] = 0;
   out_1284215992985263392[150] = 0;
   out_1284215992985263392[151] = 0;
   out_1284215992985263392[152] = 1;
   out_1284215992985263392[153] = 0;
   out_1284215992985263392[154] = 0;
   out_1284215992985263392[155] = 0;
   out_1284215992985263392[156] = 0;
   out_1284215992985263392[157] = 0;
   out_1284215992985263392[158] = 0;
   out_1284215992985263392[159] = 0;
   out_1284215992985263392[160] = 0;
   out_1284215992985263392[161] = 0;
   out_1284215992985263392[162] = 0;
   out_1284215992985263392[163] = 0;
   out_1284215992985263392[164] = 0;
   out_1284215992985263392[165] = 0;
   out_1284215992985263392[166] = 0;
   out_1284215992985263392[167] = 0;
   out_1284215992985263392[168] = 0;
   out_1284215992985263392[169] = 0;
   out_1284215992985263392[170] = 0;
   out_1284215992985263392[171] = 1;
   out_1284215992985263392[172] = 0;
   out_1284215992985263392[173] = 0;
   out_1284215992985263392[174] = 0;
   out_1284215992985263392[175] = 0;
   out_1284215992985263392[176] = 0;
   out_1284215992985263392[177] = 0;
   out_1284215992985263392[178] = 0;
   out_1284215992985263392[179] = 0;
   out_1284215992985263392[180] = 0;
   out_1284215992985263392[181] = 0;
   out_1284215992985263392[182] = 0;
   out_1284215992985263392[183] = 0;
   out_1284215992985263392[184] = 0;
   out_1284215992985263392[185] = 0;
   out_1284215992985263392[186] = 0;
   out_1284215992985263392[187] = 0;
   out_1284215992985263392[188] = 0;
   out_1284215992985263392[189] = 0;
   out_1284215992985263392[190] = 1;
   out_1284215992985263392[191] = 0;
   out_1284215992985263392[192] = 0;
   out_1284215992985263392[193] = 0;
   out_1284215992985263392[194] = 0;
   out_1284215992985263392[195] = 0;
   out_1284215992985263392[196] = 0;
   out_1284215992985263392[197] = 0;
   out_1284215992985263392[198] = 0;
   out_1284215992985263392[199] = 0;
   out_1284215992985263392[200] = 0;
   out_1284215992985263392[201] = 0;
   out_1284215992985263392[202] = 0;
   out_1284215992985263392[203] = 0;
   out_1284215992985263392[204] = 0;
   out_1284215992985263392[205] = 0;
   out_1284215992985263392[206] = 0;
   out_1284215992985263392[207] = 0;
   out_1284215992985263392[208] = 0;
   out_1284215992985263392[209] = 1;
   out_1284215992985263392[210] = 0;
   out_1284215992985263392[211] = 0;
   out_1284215992985263392[212] = 0;
   out_1284215992985263392[213] = 0;
   out_1284215992985263392[214] = 0;
   out_1284215992985263392[215] = 0;
   out_1284215992985263392[216] = 0;
   out_1284215992985263392[217] = 0;
   out_1284215992985263392[218] = 0;
   out_1284215992985263392[219] = 0;
   out_1284215992985263392[220] = 0;
   out_1284215992985263392[221] = 0;
   out_1284215992985263392[222] = 0;
   out_1284215992985263392[223] = 0;
   out_1284215992985263392[224] = 0;
   out_1284215992985263392[225] = 0;
   out_1284215992985263392[226] = 0;
   out_1284215992985263392[227] = 0;
   out_1284215992985263392[228] = 1;
   out_1284215992985263392[229] = 0;
   out_1284215992985263392[230] = 0;
   out_1284215992985263392[231] = 0;
   out_1284215992985263392[232] = 0;
   out_1284215992985263392[233] = 0;
   out_1284215992985263392[234] = 0;
   out_1284215992985263392[235] = 0;
   out_1284215992985263392[236] = 0;
   out_1284215992985263392[237] = 0;
   out_1284215992985263392[238] = 0;
   out_1284215992985263392[239] = 0;
   out_1284215992985263392[240] = 0;
   out_1284215992985263392[241] = 0;
   out_1284215992985263392[242] = 0;
   out_1284215992985263392[243] = 0;
   out_1284215992985263392[244] = 0;
   out_1284215992985263392[245] = 0;
   out_1284215992985263392[246] = 0;
   out_1284215992985263392[247] = 1;
   out_1284215992985263392[248] = 0;
   out_1284215992985263392[249] = 0;
   out_1284215992985263392[250] = 0;
   out_1284215992985263392[251] = 0;
   out_1284215992985263392[252] = 0;
   out_1284215992985263392[253] = 0;
   out_1284215992985263392[254] = 0;
   out_1284215992985263392[255] = 0;
   out_1284215992985263392[256] = 0;
   out_1284215992985263392[257] = 0;
   out_1284215992985263392[258] = 0;
   out_1284215992985263392[259] = 0;
   out_1284215992985263392[260] = 0;
   out_1284215992985263392[261] = 0;
   out_1284215992985263392[262] = 0;
   out_1284215992985263392[263] = 0;
   out_1284215992985263392[264] = 0;
   out_1284215992985263392[265] = 0;
   out_1284215992985263392[266] = 1;
   out_1284215992985263392[267] = 0;
   out_1284215992985263392[268] = 0;
   out_1284215992985263392[269] = 0;
   out_1284215992985263392[270] = 0;
   out_1284215992985263392[271] = 0;
   out_1284215992985263392[272] = 0;
   out_1284215992985263392[273] = 0;
   out_1284215992985263392[274] = 0;
   out_1284215992985263392[275] = 0;
   out_1284215992985263392[276] = 0;
   out_1284215992985263392[277] = 0;
   out_1284215992985263392[278] = 0;
   out_1284215992985263392[279] = 0;
   out_1284215992985263392[280] = 0;
   out_1284215992985263392[281] = 0;
   out_1284215992985263392[282] = 0;
   out_1284215992985263392[283] = 0;
   out_1284215992985263392[284] = 0;
   out_1284215992985263392[285] = 1;
   out_1284215992985263392[286] = 0;
   out_1284215992985263392[287] = 0;
   out_1284215992985263392[288] = 0;
   out_1284215992985263392[289] = 0;
   out_1284215992985263392[290] = 0;
   out_1284215992985263392[291] = 0;
   out_1284215992985263392[292] = 0;
   out_1284215992985263392[293] = 0;
   out_1284215992985263392[294] = 0;
   out_1284215992985263392[295] = 0;
   out_1284215992985263392[296] = 0;
   out_1284215992985263392[297] = 0;
   out_1284215992985263392[298] = 0;
   out_1284215992985263392[299] = 0;
   out_1284215992985263392[300] = 0;
   out_1284215992985263392[301] = 0;
   out_1284215992985263392[302] = 0;
   out_1284215992985263392[303] = 0;
   out_1284215992985263392[304] = 1;
   out_1284215992985263392[305] = 0;
   out_1284215992985263392[306] = 0;
   out_1284215992985263392[307] = 0;
   out_1284215992985263392[308] = 0;
   out_1284215992985263392[309] = 0;
   out_1284215992985263392[310] = 0;
   out_1284215992985263392[311] = 0;
   out_1284215992985263392[312] = 0;
   out_1284215992985263392[313] = 0;
   out_1284215992985263392[314] = 0;
   out_1284215992985263392[315] = 0;
   out_1284215992985263392[316] = 0;
   out_1284215992985263392[317] = 0;
   out_1284215992985263392[318] = 0;
   out_1284215992985263392[319] = 0;
   out_1284215992985263392[320] = 0;
   out_1284215992985263392[321] = 0;
   out_1284215992985263392[322] = 0;
   out_1284215992985263392[323] = 1;
}
void h_4(double *state, double *unused, double *out_3952000395503486674) {
   out_3952000395503486674[0] = state[6] + state[9];
   out_3952000395503486674[1] = state[7] + state[10];
   out_3952000395503486674[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_9012219907785707801) {
   out_9012219907785707801[0] = 0;
   out_9012219907785707801[1] = 0;
   out_9012219907785707801[2] = 0;
   out_9012219907785707801[3] = 0;
   out_9012219907785707801[4] = 0;
   out_9012219907785707801[5] = 0;
   out_9012219907785707801[6] = 1;
   out_9012219907785707801[7] = 0;
   out_9012219907785707801[8] = 0;
   out_9012219907785707801[9] = 1;
   out_9012219907785707801[10] = 0;
   out_9012219907785707801[11] = 0;
   out_9012219907785707801[12] = 0;
   out_9012219907785707801[13] = 0;
   out_9012219907785707801[14] = 0;
   out_9012219907785707801[15] = 0;
   out_9012219907785707801[16] = 0;
   out_9012219907785707801[17] = 0;
   out_9012219907785707801[18] = 0;
   out_9012219907785707801[19] = 0;
   out_9012219907785707801[20] = 0;
   out_9012219907785707801[21] = 0;
   out_9012219907785707801[22] = 0;
   out_9012219907785707801[23] = 0;
   out_9012219907785707801[24] = 0;
   out_9012219907785707801[25] = 1;
   out_9012219907785707801[26] = 0;
   out_9012219907785707801[27] = 0;
   out_9012219907785707801[28] = 1;
   out_9012219907785707801[29] = 0;
   out_9012219907785707801[30] = 0;
   out_9012219907785707801[31] = 0;
   out_9012219907785707801[32] = 0;
   out_9012219907785707801[33] = 0;
   out_9012219907785707801[34] = 0;
   out_9012219907785707801[35] = 0;
   out_9012219907785707801[36] = 0;
   out_9012219907785707801[37] = 0;
   out_9012219907785707801[38] = 0;
   out_9012219907785707801[39] = 0;
   out_9012219907785707801[40] = 0;
   out_9012219907785707801[41] = 0;
   out_9012219907785707801[42] = 0;
   out_9012219907785707801[43] = 0;
   out_9012219907785707801[44] = 1;
   out_9012219907785707801[45] = 0;
   out_9012219907785707801[46] = 0;
   out_9012219907785707801[47] = 1;
   out_9012219907785707801[48] = 0;
   out_9012219907785707801[49] = 0;
   out_9012219907785707801[50] = 0;
   out_9012219907785707801[51] = 0;
   out_9012219907785707801[52] = 0;
   out_9012219907785707801[53] = 0;
}
void h_10(double *state, double *unused, double *out_4901115569812966272) {
   out_4901115569812966272[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4901115569812966272[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4901115569812966272[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5949843926999388058) {
   out_5949843926999388058[0] = 0;
   out_5949843926999388058[1] = 9.8100000000000005*cos(state[1]);
   out_5949843926999388058[2] = 0;
   out_5949843926999388058[3] = 0;
   out_5949843926999388058[4] = -state[8];
   out_5949843926999388058[5] = state[7];
   out_5949843926999388058[6] = 0;
   out_5949843926999388058[7] = state[5];
   out_5949843926999388058[8] = -state[4];
   out_5949843926999388058[9] = 0;
   out_5949843926999388058[10] = 0;
   out_5949843926999388058[11] = 0;
   out_5949843926999388058[12] = 1;
   out_5949843926999388058[13] = 0;
   out_5949843926999388058[14] = 0;
   out_5949843926999388058[15] = 1;
   out_5949843926999388058[16] = 0;
   out_5949843926999388058[17] = 0;
   out_5949843926999388058[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5949843926999388058[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5949843926999388058[20] = 0;
   out_5949843926999388058[21] = state[8];
   out_5949843926999388058[22] = 0;
   out_5949843926999388058[23] = -state[6];
   out_5949843926999388058[24] = -state[5];
   out_5949843926999388058[25] = 0;
   out_5949843926999388058[26] = state[3];
   out_5949843926999388058[27] = 0;
   out_5949843926999388058[28] = 0;
   out_5949843926999388058[29] = 0;
   out_5949843926999388058[30] = 0;
   out_5949843926999388058[31] = 1;
   out_5949843926999388058[32] = 0;
   out_5949843926999388058[33] = 0;
   out_5949843926999388058[34] = 1;
   out_5949843926999388058[35] = 0;
   out_5949843926999388058[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5949843926999388058[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5949843926999388058[38] = 0;
   out_5949843926999388058[39] = -state[7];
   out_5949843926999388058[40] = state[6];
   out_5949843926999388058[41] = 0;
   out_5949843926999388058[42] = state[4];
   out_5949843926999388058[43] = -state[3];
   out_5949843926999388058[44] = 0;
   out_5949843926999388058[45] = 0;
   out_5949843926999388058[46] = 0;
   out_5949843926999388058[47] = 0;
   out_5949843926999388058[48] = 0;
   out_5949843926999388058[49] = 0;
   out_5949843926999388058[50] = 1;
   out_5949843926999388058[51] = 0;
   out_5949843926999388058[52] = 0;
   out_5949843926999388058[53] = 1;
}
void h_13(double *state, double *unused, double *out_4932748021563902320) {
   out_4932748021563902320[0] = state[3];
   out_4932748021563902320[1] = state[4];
   out_4932748021563902320[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6222250340591511014) {
   out_6222250340591511014[0] = 0;
   out_6222250340591511014[1] = 0;
   out_6222250340591511014[2] = 0;
   out_6222250340591511014[3] = 1;
   out_6222250340591511014[4] = 0;
   out_6222250340591511014[5] = 0;
   out_6222250340591511014[6] = 0;
   out_6222250340591511014[7] = 0;
   out_6222250340591511014[8] = 0;
   out_6222250340591511014[9] = 0;
   out_6222250340591511014[10] = 0;
   out_6222250340591511014[11] = 0;
   out_6222250340591511014[12] = 0;
   out_6222250340591511014[13] = 0;
   out_6222250340591511014[14] = 0;
   out_6222250340591511014[15] = 0;
   out_6222250340591511014[16] = 0;
   out_6222250340591511014[17] = 0;
   out_6222250340591511014[18] = 0;
   out_6222250340591511014[19] = 0;
   out_6222250340591511014[20] = 0;
   out_6222250340591511014[21] = 0;
   out_6222250340591511014[22] = 1;
   out_6222250340591511014[23] = 0;
   out_6222250340591511014[24] = 0;
   out_6222250340591511014[25] = 0;
   out_6222250340591511014[26] = 0;
   out_6222250340591511014[27] = 0;
   out_6222250340591511014[28] = 0;
   out_6222250340591511014[29] = 0;
   out_6222250340591511014[30] = 0;
   out_6222250340591511014[31] = 0;
   out_6222250340591511014[32] = 0;
   out_6222250340591511014[33] = 0;
   out_6222250340591511014[34] = 0;
   out_6222250340591511014[35] = 0;
   out_6222250340591511014[36] = 0;
   out_6222250340591511014[37] = 0;
   out_6222250340591511014[38] = 0;
   out_6222250340591511014[39] = 0;
   out_6222250340591511014[40] = 0;
   out_6222250340591511014[41] = 1;
   out_6222250340591511014[42] = 0;
   out_6222250340591511014[43] = 0;
   out_6222250340591511014[44] = 0;
   out_6222250340591511014[45] = 0;
   out_6222250340591511014[46] = 0;
   out_6222250340591511014[47] = 0;
   out_6222250340591511014[48] = 0;
   out_6222250340591511014[49] = 0;
   out_6222250340591511014[50] = 0;
   out_6222250340591511014[51] = 0;
   out_6222250340591511014[52] = 0;
   out_6222250340591511014[53] = 0;
}
void h_14(double *state, double *unused, double *out_2761179743815230965) {
   out_2761179743815230965[0] = state[6];
   out_2761179743815230965[1] = state[7];
   out_2761179743815230965[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5471283309584359286) {
   out_5471283309584359286[0] = 0;
   out_5471283309584359286[1] = 0;
   out_5471283309584359286[2] = 0;
   out_5471283309584359286[3] = 0;
   out_5471283309584359286[4] = 0;
   out_5471283309584359286[5] = 0;
   out_5471283309584359286[6] = 1;
   out_5471283309584359286[7] = 0;
   out_5471283309584359286[8] = 0;
   out_5471283309584359286[9] = 0;
   out_5471283309584359286[10] = 0;
   out_5471283309584359286[11] = 0;
   out_5471283309584359286[12] = 0;
   out_5471283309584359286[13] = 0;
   out_5471283309584359286[14] = 0;
   out_5471283309584359286[15] = 0;
   out_5471283309584359286[16] = 0;
   out_5471283309584359286[17] = 0;
   out_5471283309584359286[18] = 0;
   out_5471283309584359286[19] = 0;
   out_5471283309584359286[20] = 0;
   out_5471283309584359286[21] = 0;
   out_5471283309584359286[22] = 0;
   out_5471283309584359286[23] = 0;
   out_5471283309584359286[24] = 0;
   out_5471283309584359286[25] = 1;
   out_5471283309584359286[26] = 0;
   out_5471283309584359286[27] = 0;
   out_5471283309584359286[28] = 0;
   out_5471283309584359286[29] = 0;
   out_5471283309584359286[30] = 0;
   out_5471283309584359286[31] = 0;
   out_5471283309584359286[32] = 0;
   out_5471283309584359286[33] = 0;
   out_5471283309584359286[34] = 0;
   out_5471283309584359286[35] = 0;
   out_5471283309584359286[36] = 0;
   out_5471283309584359286[37] = 0;
   out_5471283309584359286[38] = 0;
   out_5471283309584359286[39] = 0;
   out_5471283309584359286[40] = 0;
   out_5471283309584359286[41] = 0;
   out_5471283309584359286[42] = 0;
   out_5471283309584359286[43] = 0;
   out_5471283309584359286[44] = 1;
   out_5471283309584359286[45] = 0;
   out_5471283309584359286[46] = 0;
   out_5471283309584359286[47] = 0;
   out_5471283309584359286[48] = 0;
   out_5471283309584359286[49] = 0;
   out_5471283309584359286[50] = 0;
   out_5471283309584359286[51] = 0;
   out_5471283309584359286[52] = 0;
   out_5471283309584359286[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_5046380216439218526) {
  err_fun(nom_x, delta_x, out_5046380216439218526);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2577348512298300834) {
  inv_err_fun(nom_x, true_x, out_2577348512298300834);
}
void pose_H_mod_fun(double *state, double *out_490162499866022473) {
  H_mod_fun(state, out_490162499866022473);
}
void pose_f_fun(double *state, double dt, double *out_1869185704993173059) {
  f_fun(state,  dt, out_1869185704993173059);
}
void pose_F_fun(double *state, double dt, double *out_1284215992985263392) {
  F_fun(state,  dt, out_1284215992985263392);
}
void pose_h_4(double *state, double *unused, double *out_3952000395503486674) {
  h_4(state, unused, out_3952000395503486674);
}
void pose_H_4(double *state, double *unused, double *out_9012219907785707801) {
  H_4(state, unused, out_9012219907785707801);
}
void pose_h_10(double *state, double *unused, double *out_4901115569812966272) {
  h_10(state, unused, out_4901115569812966272);
}
void pose_H_10(double *state, double *unused, double *out_5949843926999388058) {
  H_10(state, unused, out_5949843926999388058);
}
void pose_h_13(double *state, double *unused, double *out_4932748021563902320) {
  h_13(state, unused, out_4932748021563902320);
}
void pose_H_13(double *state, double *unused, double *out_6222250340591511014) {
  H_13(state, unused, out_6222250340591511014);
}
void pose_h_14(double *state, double *unused, double *out_2761179743815230965) {
  h_14(state, unused, out_2761179743815230965);
}
void pose_H_14(double *state, double *unused, double *out_5471283309584359286) {
  H_14(state, unused, out_5471283309584359286);
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
