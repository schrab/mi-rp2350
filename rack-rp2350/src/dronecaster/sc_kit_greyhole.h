// sc_kit_greyhole.h — GreyholeRaw (sc3-plugins @ 25eacaf7, GPL2+).
// The faust-generated mydsp class below is copied VERBATIM from
// sc3p_full/source/DEINDUGens/GreyholeRaw.cpp (lines 219-1052), only renamed
// mydsp -> GreyholeRawDsp plus a SetParams helper. Source header:
//   "version: 1.0 / author: Julian Parker, bug fixes by Till Bovermann /
//    license: GPL2+ / name: GreyholeRaw / (c) Julian Parker 2013"
// Faust architecture wrapper: Copyright (C) 2005-2012 Stefan Kersten, GPL2+.
// Slider mapping taken from buildUserInterface (GreyholeRaw.cpp:523-531).
#ifndef RACK_SRC_DRONECASTER_SC_KIT_GREYHOLE_H_
#define RACK_SRC_DRONECASTER_SC_KIT_GREYHOLE_H_
#include <cmath>
#include <cstddef>
namespace sc {
using FAUSTFLOAT = float;
class dsp { public: virtual ~dsp() = default; };
class UI {
 public:
  virtual ~UI() = default;
  virtual void openVerticalBox(const char*) {}
  virtual void closeBox() {}
  virtual void addHorizontalSlider(const char*, float*, float, float, float,
                                   float) {}
  virtual void addVerticalSlider(const char*, float*, float, float, float,
                                 float) {}
  virtual void addNumEntry(const char*, float*, float, float, float, float) {}
  virtual void addButton(const char*, float*) {}
  virtual void addCheckButton(const char*, float*) {}
  virtual void addHorizontalBargraph(const char*, float*, float, float) {}
  virtual void addVerticalBargraph(const char*, float*, float, float) {}
};
class Meta { public: virtual ~Meta() = default;
  virtual void declare(const char*, const char*) {} };
inline float sc_clipf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
template <typename A, typename B> inline A min(A a, B b) { return a < b ? a : b; }
template <typename A, typename B> inline A max(A a, B b) { return a > b ? a : b; }
inline int greyhole_primes(int i) {
  static const int prime_nums[] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263, 269, 271, 277, 281, 283, 293, 307, 311, 313, 317, 331, 337, 347, 349, 353, 359, 367, 373, 379, 383, 389, 397, 401, 409, 419, 421, 431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491, 499, 503, 509, 521, 523, 541, 547, 557, 563, 569, 571, 577, 587, 593, 599, 601, 607, 613, 617, 619, 631, 641, 643, 647, 653, 659, 661, 673, 677, 683, 691, 701, 709, 719, 727, 733, 739, 743, 751, 757, 761, 769, 773, 787, 797, 809, 811, 821, 823, 827, 829, 839, 853, 857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929, 937, 941, 947, 953, 967, 971, 977, 983, 991, 997, 1009, 1013, 1019, 1021, 1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069, 1087, 1091, 1093, 1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153, 1163, 1171, 1181, 1187, 1193, 1201, 1213, 1217, 1223, 1229, 1231, 1237, 1249, 1259, 1277, 1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319, 1321, 1327, 1361, 1367, 1373, 1381, 1399, 1409, 1423, 1427, 1429, 1433, 1439, 1447, 1451, 1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493, 1499, 1511, 1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579, 1583, 1597, 1601, 1607, 1609, 1613, 1619, 1621, 1627, 1637, 1657, 1663, 1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733, 1741, 1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811, 1823, 1831, 1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889, 1901, 1907, 1913, 1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993, 1997, 1999, 2003, 2011, 2017, 2027, 2029, 2039, 2053, 2063, 2069, 2081, 2083, 2087, 2089, 2099, 2111, 2113, 2129, 2131, 2137, 2141, 2143, 2153, 2161, 2179, 2203, 2207, 2213, 2221, 2237, 2239, 2243, 2251, 2267, 2269, 2273, 2281, 2287, 2293, 2297, 2309, 2311, 2333, 2339, 2341, 2347, 2351, 2357, 2371, 2377, 2381, 2383, 2389, 2393, 2399, 2411, 2417, 2423, 2437, 2441, 2447, 2459, 2467, 2473, 2477, 2503, 2521, 2531, 2539, 2543, 2549, 2551, 2557, 2579, 2591, 2593, 2609, 2617, 2621, 2633, 2647, 2657, 2659, 2663, 2671, 2677, 2683, 2687, 2689, 2693, 2699, 2707, 2711, 2713, 2719, 2729, 2731, 2741, 2749, 2753, 2767, 2777, 2789, 2791, 2797, 2801, 2803, 2819, 2833, 2837, 2843, 2851, 2857, 2861, 2879, 2887, 2897, 2903, 2909, 2917, 2927, 2939, 2953, 2957, 2963, 2969, 2971, 2999, 3001, 3011, 3019, 3023, 3037, 3041, 3049, 3061, 3067, 3079, 3083, 3089, 3109, 3119, 3121, 3137, 3163, 3167, 3169, 3181, 3187, 3191, 3203, 3209, 3217, 3221, 3229, 3251, 3253, 3257, 3259, 3271, 3299, 3301, 3307, 3313, 3319, 3323, 3329, 3331, 3343, 3347, 3359, 3361, 3371, 3373, 3389, 3391, 3407, 3413, 3433, 3449, 3457, 3461, 3463, 3467, 3469, 3491, 3499, 3511, 3517, 3527, 3529, 3533, 3539, 3541, 3547, 3557, 3559, 3571, 3581, 3583, 3593, 3607, 3613, 3617, 3623, 3631, 3637, 3643, 3659, 3671, 3673, 3677, 3691, 3697, 3701, 3709, 3719, 3727, 3733, 3739, 3761, 3767, 3769, 3779, 3793, 3797, 3803, 3821, 3823, 3833, 3847, 3851, 3853, 3863, 3877, 3881, 3889, 3907, 3911, 3917, 3919, 3923, 3929, 3931, 3943, 3947, 3967, 3989, 4001, 4003, 4007, 4013, 4019, 4021, 4027, 4049, 4051, 4057, 4073, 4079, 4091, 4093, 4099, 4111, 4127, 4129, 4133, 4139, 4153, 4157, 4159, 4177, 4201, 4211, 4217, 4219, 4229, 4231, 4241, 4243, 4253, 4259, 4261, 4271, 4273, 4283, 4289, 4297, 4327, 4337, 4339, 4349, 4357, 4363, 4373, 4391, 4397, 4409, 4421, 4423, 4441, 4447, 4451, 4457, 4463, 4481, 4483, 4493, 4507, 4513, 4517, 4519, 4523, 4547, 4549, 4561, 4567, 4583, 4591, 4597, 4603, 4621, 4637, 4639, 4643, 4649, 4651, 4657, 4663, 4673, 4679, 4691, 4703, 4721, 4723, 4729, 4733, 4751, 4759, 4783, 4787, 4789, 4793, 4799, 4801, 4813, 4817, 4831, 4861, 4871, 4877, 4889, 4903, 4909, 4919, 4931, 4933, 4937, 4943, 4951, 4957, 4967, 4969, 4973, 4987, 4993, 4999, 5003, 5009, 5011, 5021, 5023, 5039, 5051, 5059, 5077, 5081, 5087, 5099, 5101, 5107, 5113, 5119, 5147, 5153, 5167, 5171, 5179, 5189, 5197, 5209, 5227, 5231, 5233, 5237, 5261, 5273, 5279, 5281, 5297, 5303, 5309, 5323, 5333, 5347, 5351, 5381, 5387, 5393, 5399, 5407, 5413, 5417, 5419, 5431, 5437, 5441, 5443, 5449, 5471, 5477, 5479, 5483, 5501, 5503, 5507, 5519, 5521, 5527, 5531, 5557, 5563, 5569, 5573, 5581, 5591, 5623, 5639, 5641, 5647, 5651, 5653, 5657, 5659, 5669, 5683, 5689, 5693, 5701, 5711, 5717, 5737, 5741, 5743, 5749, 5779, 5783, 5791, 5801, 5807, 5813, 5821, 5827, 5839, 5843, 5849, 5851, 5857, 5861, 5867, 5869, 5879, 5881, 5897, 5903, 5923, 5927, 5939, 5953, 5981, 5987, 6007, 6011, 6029, 6037, 6043, 6047, 6053, 6067, 6073, 6079, 6089, 6091, 6101, 6113, 6121, 6131, 6133, 6143, 6151, 6163, 6173, 6197, 6199, 6203, 6211, 6217, 6221, 6229, 6247, 6257, 6263, 6269, 6271, 6277, 6287, 6299, 6301, 6311, 6317, 6323, 6329, 6337, 6343, 6353, 6359, 6361, 6367, 6373, 6379, 6389, 6397, 6421, 6427, 6449, 6451, 6469, 6473, 6481, 6491, 6521, 6529, 6547, 6551, 6553, 6563, 6569, 6571, 6577, 6581, 6599, 6607, 6619, 6637, 6653, 6659, 6661, 6673, 6679, 6689, 6691, 6701, 6703, 6709, 6719, 6733, 6737, 6761, 6763, 6779, 6781, 6791, 6793, 6803, 6823, 6827, 6829, 6833, 6841, 6857, 6863, 6869, 6871, 6883, 6899, 6907, 6911, 6917, 6947, 6949, 6959, 6961, 6967, 6971, 6977, 6983, 6991, 6997, 7001, 7013, 7019, 7027, 7039, 7043, 7057, 7069, 7079, 7103, 7109, 7121, 7127, 7129, 7151, 7159, 7177, 7187, 7193, 7207, 7211, 7213, 7219, 7229, 7237, 7243, 7247, 7253, 7283, 7297, 7307, 7309, 7321, 7331, 7333, 7349, 7351, 7369, 7393, 7411, 7417, 7433, 7451, 7457, 7459, 7477, 7481, 7487, 7489, 7499, 7507, 7517, 7523, 7529, 7537, 7541, 7547, 7549, 7559, 7561, 7573, 7577, 7583, 7589, 7591, 7603, 7607, 7621, 7639, 7643, 7649, 7669, 7673, 7681, 7687, 7691, 7699, 7703, 7717, 7723, 7727, 7741, 7753, 7757, 7759, 7789, 7793, 7817, 7823, 7829, 7841, 7853, 7867, 7873, 7877, 7879, 7883, 7901, 7907, 7919, 7927, 7933, 7937, 7949, 7951, 7963, 7993, 8009, 8011, 8017, 8039, 8053, 8059, 8069, 8081, 8087, 8089, 8093, 8101, 8111, 8117, 8123, 8147, 8161, 8167, 8171, 8179, 8191, 8209, 8219, 8221, 8231, 8233, 8237, 8243, 8263, 8269, 8273, 8287, 8291, 8293, 8297, 8311, 8317, 8329, 8353, 8363, 8369, 8377, 8387, 8389, 8419, 8423, 8429, 8431, 8443, 8447, 8461, 8467, 8501, 8513, 8521, 8527, 8537, 8539, 8543, 8563, 8573, 8581, 8597, 8599, 8609, 8623, 8627, 8629, 8641, 8647, 8663, 8669, 8677, 8681, 8689, 8693, 8699, 8707, 8713, 8719, 8731, 8737, 8741, 8747, 8753, 8761, 8779, 8783, 8803, 8807, 8819, 8821, 8831, 8837, 8839, 8849, 8861, 8863, 8867, 8887, 8893, 8923, 8929, 8933, 8941, 8951, 8963, 8969, 8971, 8999, 9001, 9007, 9011, 9013, 9029, 9041, 9043, 9049, 9059, 9067, 9091, 9103, 9109, 9127, 9133, 9137, 9151, 9157, 9161, 9173, 9181, 9187, 9199, 9203, 9209, 9221, 9227, 9239, 9241, 9257, 9277, 9281, 9283, 9293, 9311, 9319, 9323, 9337, 9341, 9343, 9349, 9371, 9377, 9391, 9397, 9403, 9413, 9419, 9421, 9431, 9433, 9437, 9439, 9461, 9463, 9467, 9473, 9479, 9491, 9497, 9511, 9521, 9533, 9539, 9547, 9551, 9587, 9601, 9613, 9619, 9623, 9629, 9631, 9643, 9649, 9661, 9677, 9679, 9689, 9697, 9719, 9721, 9733, 9739, 9743, 9749, 9767, 9769, 9781, 9787, 9791, 9803, 9811, 9817, 9829, 9833, 9839, 9851, 9857, 9859, 9871, 9883, 9887, 9901, 9907, 9923, 9929, 9931, 9941, 9949, 9967, 9973, 10007, 10009, 10037, 10039, 10061, 10067, 10069, 10079, 10091, 10093, 10099, 10103, 10111, 10133, 10139, 10141, 10151, 10159, 10163, 10169, 10177, 10181, 10193, 10211, 10223, 10243, 10247, 10253, 10259, 10267, 10271, 10273, 10289, 10301, 10303, 10313, 10321, 10331, 10333, 10337, 10343, 10357, 10369, 10391, 10399, 10427, 10429, 10433, 10453, 10457, 10459, 10463, 10477, 10487, 10499, 10501, 10513, 10529, 10531, 10559, 10567, 10589, 10597, 10601, 10607, 10613, 10627, 10631, 10639, 10651, 10657, 10663, 10667};
  return prime_nums[i];
}
#define primes greyhole_primes

class GreyholeRawDsp : public dsp {
  private:
	FAUSTFLOAT 	fslider0;
	float 	fVec0[2];
	int 	iVec1[2];
	float 	fConst0;
	float 	fConst1;
	FAUSTFLOAT 	fslider1;
	float 	fVec2[2];
	float 	fConst2;
	FAUSTFLOAT 	fslider2;
	float 	fVec3[2];
	float 	fRec39[2];
	float 	fRec40[2];
	int 	IOTA;
	float 	fVec4[131072];
	FAUSTFLOAT 	fslider3;
	float 	fRec41[2];
	float 	fRec42[2];
	float 	fRec43[2];
	float 	fRec44[2];
	FAUSTFLOAT 	fslider4;
	float 	fVec5[2];
	float 	fVec6[131072];
	float 	fVec7[16384];
	FAUSTFLOAT 	fslider5;
	float 	fRec45[2];
	float 	fRec38[2];
	float 	fRec36[2];
	float 	fVec8[16384];
	float 	fRec47[2];
	float 	fRec46[2];
	float 	fRec37[2];
	float 	fVec9[16384];
	float 	fRec48[2];
	float 	fRec35[2];
	float 	fRec33[2];
	float 	fVec10[16384];
	float 	fRec50[2];
	float 	fRec49[2];
	float 	fRec34[2];
	float 	fVec11[16384];
	float 	fRec51[2];
	float 	fRec32[2];
	float 	fRec30[2];
	float 	fVec12[16384];
	float 	fRec53[2];
	float 	fRec52[2];
	float 	fRec31[2];
	float 	fVec13[16384];
	float 	fRec54[2];
	float 	fRec29[2];
	float 	fRec27[2];
	float 	fVec14[16384];
	float 	fRec56[2];
	float 	fRec55[2];
	float 	fRec28[2];
	float 	fVec15[16384];
	float 	fRec57[2];
	float 	fRec26[2];
	float 	fRec24[2];
	float 	fVec16[16384];
	float 	fRec59[2];
	float 	fRec58[2];
	float 	fRec25[2];
	float 	fVec17[16384];
	float 	fRec60[2];
	float 	fRec23[2];
	float 	fRec21[2];
	float 	fVec18[16384];
	float 	fRec62[2];
	float 	fRec61[2];
	float 	fRec22[2];
	float 	fVec19[16384];
	float 	fRec63[2];
	float 	fRec20[2];
	float 	fRec18[2];
	float 	fVec20[16384];
	float 	fRec65[2];
	float 	fRec64[2];
	float 	fRec19[2];
	float 	fVec21[16384];
	float 	fRec66[2];
	float 	fRec17[2];
	float 	fRec15[2];
	float 	fVec22[16384];
	float 	fRec68[2];
	float 	fRec67[2];
	float 	fRec16[2];
	float 	fVec23[16384];
	float 	fRec69[2];
	float 	fRec14[2];
	float 	fRec12[2];
	float 	fVec24[16384];
	float 	fRec71[2];
	float 	fRec70[2];
	float 	fRec13[2];
	float 	fVec25[16384];
	float 	fRec72[2];
	float 	fRec11[2];
	float 	fRec9[2];
	float 	fVec26[16384];
	float 	fRec74[2];
	float 	fRec73[2];
	float 	fRec10[2];
	float 	fVec27[16384];
	float 	fRec75[2];
	float 	fRec8[2];
	float 	fRec6[2];
	float 	fVec28[16384];
	float 	fRec77[2];
	float 	fRec76[2];
	float 	fRec7[2];
	float 	fVec29[16384];
	float 	fRec78[2];
	float 	fRec5[2];
	float 	fRec3[2];
	float 	fVec30[16384];
	float 	fRec80[2];
	float 	fRec79[2];
	float 	fRec4[2];
	FAUSTFLOAT 	fslider6;
	float 	fVec31[2];
	float 	fRec2[2];
	float 	fRec0[1024];
	float 	fRec81[2];
	float 	fRec1[1024];
	int fSamplingFreq;

  public:
	void SetParams(float damping, float delayTime, float diffusion,
	               float feedback, float modDepth, float modFreq, float size) {
		fslider6 = sc_clipf(damping, 0.f, 0.99f);
		fslider3 = sc_clipf(delayTime, 0.001f, 1.45f);
		fslider0 = sc_clipf(diffusion, 0.f, 0.99f);
		fslider4 = sc_clipf(feedback, 0.f, 1.f);
		fslider1 = sc_clipf(modDepth, 0.f, 1.f);
		fslider2 = sc_clipf(modFreq, 0.f, 10.f);
		fslider5 = sc_clipf(size, 0.5f, 3.f);
	}

	virtual void metadata(Meta* m) { 
		m->declare("version", "1.0");
		m->declare("author", "Julian Parker, bug fixes by Till Bovermann");
		m->declare("license", "GPL2+");
		m->declare("name", "GreyholeRaw");
		m->declare("maths.lib/name", "Faust Math Library");
		m->declare("maths.lib/version", "2.0");
		m->declare("maths.lib/author", "GRAME");
		m->declare("maths.lib/copyright", "GRAME");
		m->declare("maths.lib/license", "LGPL with exception");
		m->declare("delays.lib/name", "Faust Delay Library");
		m->declare("delays.lib/version", "0.0");
		m->declare("filters.lib/name", "Faust Filters Library");
		m->declare("filters.lib/version", "0.0");
		m->declare("copyright", "(c) Julian Parker 2013");
		m->declare("signals.lib/name", "Faust Signal Routing Library");
		m->declare("signals.lib/version", "0.0");
		m->declare("oscillators.lib/name", "Faust Oscillator Library");
		m->declare("oscillators.lib/version", "0.0");
		m->declare("basics.lib/name", "Faust Basic Element Library");
		m->declare("basics.lib/version", "0.0");
	}

	virtual int getNumInputs() { return 2; }
	virtual int getNumOutputs() { return 2; }
	static void classInit(int samplingFreq) {
	}
	virtual void instanceConstants(int samplingFreq) {
		fSamplingFreq = samplingFreq;
		fConst0 = min(1.92e+05f, max(1e+03f, (float)fSamplingFreq));
		fConst1 = (0.00056689343f * fConst0);
		fConst2 = (3.1415927f / fConst0);
	}
	virtual void instanceResetUserInterface() {
		fslider0 = 0.5f;
		fslider1 = 0.1f;
		fslider2 = 2.0f;
		fslider3 = 0.2f;
		fslider4 = 0.9f;
		fslider5 = 1.0f;
		fslider6 = 0.0f;
	}
	virtual void instanceClear() {
		for (int i=0; i<2; i++) fVec0[i] = 0;
		for (int i=0; i<2; i++) iVec1[i] = 0;
		for (int i=0; i<2; i++) fVec2[i] = 0;
		for (int i=0; i<2; i++) fVec3[i] = 0;
		for (int i=0; i<2; i++) fRec39[i] = 0;
		for (int i=0; i<2; i++) fRec40[i] = 0;
		IOTA = 0;
		for (int i=0; i<131072; i++) fVec4[i] = 0;
		for (int i=0; i<2; i++) fRec41[i] = 0;
		for (int i=0; i<2; i++) fRec42[i] = 0;
		for (int i=0; i<2; i++) fRec43[i] = 0;
		for (int i=0; i<2; i++) fRec44[i] = 0;
		for (int i=0; i<2; i++) fVec5[i] = 0;
		for (int i=0; i<131072; i++) fVec6[i] = 0;
		for (int i=0; i<16384; i++) fVec7[i] = 0;
		for (int i=0; i<2; i++) fRec45[i] = 0;
		for (int i=0; i<2; i++) fRec38[i] = 0;
		for (int i=0; i<2; i++) fRec36[i] = 0;
		for (int i=0; i<16384; i++) fVec8[i] = 0;
		for (int i=0; i<2; i++) fRec47[i] = 0;
		for (int i=0; i<2; i++) fRec46[i] = 0;
		for (int i=0; i<2; i++) fRec37[i] = 0;
		for (int i=0; i<16384; i++) fVec9[i] = 0;
		for (int i=0; i<2; i++) fRec48[i] = 0;
		for (int i=0; i<2; i++) fRec35[i] = 0;
		for (int i=0; i<2; i++) fRec33[i] = 0;
		for (int i=0; i<16384; i++) fVec10[i] = 0;
		for (int i=0; i<2; i++) fRec50[i] = 0;
		for (int i=0; i<2; i++) fRec49[i] = 0;
		for (int i=0; i<2; i++) fRec34[i] = 0;
		for (int i=0; i<16384; i++) fVec11[i] = 0;
		for (int i=0; i<2; i++) fRec51[i] = 0;
		for (int i=0; i<2; i++) fRec32[i] = 0;
		for (int i=0; i<2; i++) fRec30[i] = 0;
		for (int i=0; i<16384; i++) fVec12[i] = 0;
		for (int i=0; i<2; i++) fRec53[i] = 0;
		for (int i=0; i<2; i++) fRec52[i] = 0;
		for (int i=0; i<2; i++) fRec31[i] = 0;
		for (int i=0; i<16384; i++) fVec13[i] = 0;
		for (int i=0; i<2; i++) fRec54[i] = 0;
		for (int i=0; i<2; i++) fRec29[i] = 0;
		for (int i=0; i<2; i++) fRec27[i] = 0;
		for (int i=0; i<16384; i++) fVec14[i] = 0;
		for (int i=0; i<2; i++) fRec56[i] = 0;
		for (int i=0; i<2; i++) fRec55[i] = 0;
		for (int i=0; i<2; i++) fRec28[i] = 0;
		for (int i=0; i<16384; i++) fVec15[i] = 0;
		for (int i=0; i<2; i++) fRec57[i] = 0;
		for (int i=0; i<2; i++) fRec26[i] = 0;
		for (int i=0; i<2; i++) fRec24[i] = 0;
		for (int i=0; i<16384; i++) fVec16[i] = 0;
		for (int i=0; i<2; i++) fRec59[i] = 0;
		for (int i=0; i<2; i++) fRec58[i] = 0;
		for (int i=0; i<2; i++) fRec25[i] = 0;
		for (int i=0; i<16384; i++) fVec17[i] = 0;
		for (int i=0; i<2; i++) fRec60[i] = 0;
		for (int i=0; i<2; i++) fRec23[i] = 0;
		for (int i=0; i<2; i++) fRec21[i] = 0;
		for (int i=0; i<16384; i++) fVec18[i] = 0;
		for (int i=0; i<2; i++) fRec62[i] = 0;
		for (int i=0; i<2; i++) fRec61[i] = 0;
		for (int i=0; i<2; i++) fRec22[i] = 0;
		for (int i=0; i<16384; i++) fVec19[i] = 0;
		for (int i=0; i<2; i++) fRec63[i] = 0;
		for (int i=0; i<2; i++) fRec20[i] = 0;
		for (int i=0; i<2; i++) fRec18[i] = 0;
		for (int i=0; i<16384; i++) fVec20[i] = 0;
		for (int i=0; i<2; i++) fRec65[i] = 0;
		for (int i=0; i<2; i++) fRec64[i] = 0;
		for (int i=0; i<2; i++) fRec19[i] = 0;
		for (int i=0; i<16384; i++) fVec21[i] = 0;
		for (int i=0; i<2; i++) fRec66[i] = 0;
		for (int i=0; i<2; i++) fRec17[i] = 0;
		for (int i=0; i<2; i++) fRec15[i] = 0;
		for (int i=0; i<16384; i++) fVec22[i] = 0;
		for (int i=0; i<2; i++) fRec68[i] = 0;
		for (int i=0; i<2; i++) fRec67[i] = 0;
		for (int i=0; i<2; i++) fRec16[i] = 0;
		for (int i=0; i<16384; i++) fVec23[i] = 0;
		for (int i=0; i<2; i++) fRec69[i] = 0;
		for (int i=0; i<2; i++) fRec14[i] = 0;
		for (int i=0; i<2; i++) fRec12[i] = 0;
		for (int i=0; i<16384; i++) fVec24[i] = 0;
		for (int i=0; i<2; i++) fRec71[i] = 0;
		for (int i=0; i<2; i++) fRec70[i] = 0;
		for (int i=0; i<2; i++) fRec13[i] = 0;
		for (int i=0; i<16384; i++) fVec25[i] = 0;
		for (int i=0; i<2; i++) fRec72[i] = 0;
		for (int i=0; i<2; i++) fRec11[i] = 0;
		for (int i=0; i<2; i++) fRec9[i] = 0;
		for (int i=0; i<16384; i++) fVec26[i] = 0;
		for (int i=0; i<2; i++) fRec74[i] = 0;
		for (int i=0; i<2; i++) fRec73[i] = 0;
		for (int i=0; i<2; i++) fRec10[i] = 0;
		for (int i=0; i<16384; i++) fVec27[i] = 0;
		for (int i=0; i<2; i++) fRec75[i] = 0;
		for (int i=0; i<2; i++) fRec8[i] = 0;
		for (int i=0; i<2; i++) fRec6[i] = 0;
		for (int i=0; i<16384; i++) fVec28[i] = 0;
		for (int i=0; i<2; i++) fRec77[i] = 0;
		for (int i=0; i<2; i++) fRec76[i] = 0;
		for (int i=0; i<2; i++) fRec7[i] = 0;
		for (int i=0; i<16384; i++) fVec29[i] = 0;
		for (int i=0; i<2; i++) fRec78[i] = 0;
		for (int i=0; i<2; i++) fRec5[i] = 0;
		for (int i=0; i<2; i++) fRec3[i] = 0;
		for (int i=0; i<16384; i++) fVec30[i] = 0;
		for (int i=0; i<2; i++) fRec80[i] = 0;
		for (int i=0; i<2; i++) fRec79[i] = 0;
		for (int i=0; i<2; i++) fRec4[i] = 0;
		for (int i=0; i<2; i++) fVec31[i] = 0;
		for (int i=0; i<2; i++) fRec2[i] = 0;
		for (int i=0; i<1024; i++) fRec0[i] = 0;
		for (int i=0; i<2; i++) fRec81[i] = 0;
		for (int i=0; i<1024; i++) fRec1[i] = 0;
	}
	virtual void init(int samplingFreq) {
		classInit(samplingFreq);
		instanceInit(samplingFreq);
	}
	virtual void instanceInit(int samplingFreq) {
		instanceConstants(samplingFreq);
		instanceResetUserInterface();
		instanceClear();
	}
	virtual GreyholeRawDsp* clone() {
		return new GreyholeRawDsp();
	}
	virtual int getSampleRate() {
		return fSamplingFreq;
	}
	virtual void buildUserInterface(UI* ui_interface) {
		ui_interface->openVerticalBox("0x00");
		ui_interface->addHorizontalSlider("damping", &fslider6, 0.0f, 0.0f, 0.99f, 0.001f);
		ui_interface->addHorizontalSlider("delayTime", &fslider3, 0.2f, 0.001f, 1.45f, 0.0001f);
		ui_interface->addHorizontalSlider("diffusion", &fslider0, 0.5f, 0.0f, 0.99f, 0.0001f);
		ui_interface->addHorizontalSlider("feedback", &fslider4, 0.9f, 0.0f, 1.0f, 0.01f);
		ui_interface->addHorizontalSlider("modDepth", &fslider1, 0.1f, 0.0f, 1.0f, 0.001f);
		ui_interface->addHorizontalSlider("modFreq", &fslider2, 2.0f, 0.0f, 1e+01f, 0.01f);
		ui_interface->addHorizontalSlider("size", &fslider5, 1.0f, 0.5f, 3.0f, 0.0001f);
		ui_interface->closeBox();
	}
	virtual void compute (int count, FAUSTFLOAT** input, FAUSTFLOAT** output) {
		float 	fSlow0 = float(fslider0);
		float 	fSlow1 = float(fslider1);
		float 	fSlow2 = float(fslider2);
		float 	fSlow3 = floorf(min((float)65533, (fConst0 * float(fslider3))));
		float 	fSlow4 = float(fslider4);
		float 	fSlow5 = float(fslider5);
		float 	fSlow6 = (49 * fSlow5);
		int 	iSlow7 = primes(fSlow6);
		float 	fSlow8 = (0.0001f * iSlow7);
		int 	iSlow9 = primes((fSlow6 + 10));
		float 	fSlow10 = (0.0001f * iSlow9);
		float 	fSlow11 = (36 * fSlow5);
		int 	iSlow12 = primes(fSlow11);
		float 	fSlow13 = (0.001f * iSlow12);
		int 	iSlow14 = primes((fSlow11 + 10));
		float 	fSlow15 = (0.001f * iSlow14);
		float 	fSlow16 = (23 * fSlow5);
		int 	iSlow17 = primes(fSlow16);
		float 	fSlow18 = (0.001f * iSlow17);
		int 	iSlow19 = primes((fSlow16 + 10));
		float 	fSlow20 = (0.001f * iSlow19);
		int 	iSlow21 = primes((10 * fSlow5));
		float 	fSlow22 = (0.001f * iSlow21);
		int 	iSlow23 = primes((10 * (fSlow5 + 1)));
		float 	fSlow24 = (0.001f * iSlow23);
		float 	fSlow25 = (68 * fSlow5);
		int 	iSlow26 = primes(fSlow25);
		float 	fSlow27 = (0.0001f * iSlow26);
		int 	iSlow28 = primes((fSlow25 + 10));
		float 	fSlow29 = (0.0001f * iSlow28);
		float 	fSlow30 = (55 * fSlow5);
		int 	iSlow31 = primes(fSlow30);
		float 	fSlow32 = (0.001f * iSlow31);
		int 	iSlow33 = primes((fSlow30 + 10));
		float 	fSlow34 = (0.001f * iSlow33);
		float 	fSlow35 = (42 * fSlow5);
		int 	iSlow36 = primes(fSlow35);
		float 	fSlow37 = (0.001f * iSlow36);
		int 	iSlow38 = primes((fSlow35 + 10));
		float 	fSlow39 = (0.001f * iSlow38);
		float 	fSlow40 = (29 * fSlow5);
		int 	iSlow41 = primes(fSlow40);
		float 	fSlow42 = (0.001f * iSlow41);
		int 	iSlow43 = primes((fSlow40 + 10));
		float 	fSlow44 = (0.001f * iSlow43);
		float 	fSlow45 = (87 * fSlow5);
		int 	iSlow46 = primes(fSlow45);
		float 	fSlow47 = (0.0001f * iSlow46);
		int 	iSlow48 = primes((fSlow45 + 10));
		float 	fSlow49 = (0.0001f * iSlow48);
		float 	fSlow50 = (74 * fSlow5);
		int 	iSlow51 = primes(fSlow50);
		float 	fSlow52 = (0.001f * iSlow51);
		int 	iSlow53 = primes((fSlow50 + 10));
		float 	fSlow54 = (0.001f * iSlow53);
		float 	fSlow55 = (61 * fSlow5);
		int 	iSlow56 = primes(fSlow55);
		float 	fSlow57 = (0.001f * iSlow56);
		int 	iSlow58 = primes((fSlow55 + 10));
		float 	fSlow59 = (0.001f * iSlow58);
		float 	fSlow60 = (48 * fSlow5);
		int 	iSlow61 = primes(fSlow60);
		float 	fSlow62 = (0.001f * iSlow61);
		int 	iSlow63 = primes((fSlow60 + 10));
		float 	fSlow64 = (0.001f * iSlow63);
		float 	fSlow65 = float(fslider6);
		FAUSTFLOAT* input0 = input[0];
		FAUSTFLOAT* input1 = input[1];
		FAUSTFLOAT* output0 = output[0];
		FAUSTFLOAT* output1 = output[1];
		for (int i=0; i<count; i++) {
			fVec0[0] = fSlow0;
			iVec1[0] = 1;
			float fTemp0 = (0.5f * (fSlow0 + fVec0[1]));
			float fTemp1 = cosf(fTemp0);
			float fTemp2 = (0 - fTemp0);
			float fTemp3 = sinf(fTemp2);
			float fTemp4 = (0 - fTemp3);
			float fTemp5 = cosf(fTemp2);
			float fTemp6 = sinf(fTemp0);
			float fTemp7 = (0 - fTemp6);
			fVec2[0] = fSlow1;
			float fTemp8 = (fSlow1 + fVec2[1]);
			fVec3[0] = fSlow2;
			float fTemp9 = (fConst2 * (fSlow2 + fVec3[1]));
			float fTemp10 = sinf(fTemp9);
			float fTemp11 = cosf(fTemp9);
			fRec39[0] = ((fTemp10 * fRec40[1]) + (fTemp11 * fRec39[1]));
			int iTemp12 = (1 - iVec1[1]);
			fRec40[0] = (((fTemp11 * fRec40[1]) + ((0 - fTemp10) * fRec39[1])) + iTemp12);
			float fTemp13 = (fConst1 * (fTemp8 * (fRec40[0] + 1.0f)));
			float fTemp14 = (fTemp13 + 8.500005f);
			int iTemp15 = int(fTemp14);
			float fTemp16 = floorf(fTemp14);
			float fTemp17 = (fTemp13 + (9.0f - fTemp16));
			float fTemp18 = (fTemp13 + (8.0f - fTemp16));
			float fTemp19 = (fTemp13 + (7.0f - fTemp16));
			float fTemp20 = (fTemp13 + (6.0f - fTemp16));
			float fTemp21 = (fTemp17 * fTemp18);
			float fTemp22 = (((((fRec0[(IOTA-int((int(min(512, max(0, iTemp15))) + 1)))&1023] * (0 - fTemp17)) * (0 - (0.5f * fTemp18))) * (0 - (0.33333334f * fTemp19))) * (0 - (0.25f * fTemp20))) + ((fTemp13 + (1e+01f - fTemp16)) * ((((((fRec0[(IOTA-int((int(min(512, max(0, (iTemp15 + 1)))) + 1)))&1023] * (0 - fTemp18)) * (0 - (0.5f * fTemp19))) * (0 - (0.33333334f * fTemp20))) + (0.5f * (((fTemp17 * fRec0[(IOTA-int((int(min(512, max(0, (iTemp15 + 2)))) + 1)))&1023]) * (0 - fTemp19)) * (0 - (0.5f * fTemp20))))) + (0.16666667f * ((fTemp21 * fRec0[(IOTA-int((int(min(512, max(0, (iTemp15 + 3)))) + 1)))&1023]) * (0 - fTemp20)))) + (0.041666668f * ((fTemp21 * fTemp19) * fRec0[(IOTA-int((int(min(512, max(0, (iTemp15 + 4)))) + 1)))&1023])))));
			fVec4[IOTA&131071] = fTemp22;
			float fTemp23 = ((int((fRec41[1] != 0.0f)))?((int(((fRec42[1] > 0.0f) & (fRec42[1] < 1.0f))))?fRec41[1]:0):((int(((fRec42[1] == 0.0f) & (fSlow3 != fRec43[1]))))?4.5351473e-05f:((int(((fRec42[1] == 1.0f) & (fSlow3 != fRec44[1]))))?-4.5351473e-05f:0)));
			fRec41[0] = fTemp23;
			fRec42[0] = max(0.0f, min(1.0f, (fRec42[1] + fTemp23)));
			fRec43[0] = ((int(((fRec42[1] >= 1.0f) & (fRec44[1] != fSlow3))))?fSlow3:fRec43[1]);
			fRec44[0] = ((int(((fRec42[1] <= 0.0f) & (fRec43[1] != fSlow3))))?fSlow3:fRec44[1]);
			int iTemp24 = int(min((float)65536, max((float)0, fRec43[0])));
			float fTemp25 = (1.0f - fRec42[0]);
			int iTemp26 = int(min((float)65536, max((float)0, fRec44[0])));
			fVec5[0] = fSlow4;
			float fTemp27 = (fSlow4 + fVec5[1]);
			float fTemp28 = ((float)input0[i] + (0.5f * (((fVec4[(IOTA-iTemp24)&131071] * fTemp25) + (fVec4[(IOTA-iTemp26)&131071] * fRec42[0])) * fTemp27)));
			float fTemp29 = ((fRec27[1] * fTemp7) + (fTemp1 * fTemp28));
			float fTemp30 = ((fRec30[1] * fTemp7) + (fTemp1 * fTemp29));
			float fTemp31 = ((fRec33[1] * fTemp7) + (fTemp1 * fTemp30));
			float fTemp32 = ((fRec36[1] * fTemp7) + (fTemp1 * fTemp31));
			float fTemp33 = (fConst1 * (fTemp8 * (fRec39[0] + 1.0f)));
			float fTemp34 = (fTemp33 + 8.500005f);
			int iTemp35 = int(fTemp34);
			float fTemp36 = floorf(fTemp34);
			float fTemp37 = (fTemp33 + (9.0f - fTemp36));
			float fTemp38 = (fTemp33 + (8.0f - fTemp36));
			float fTemp39 = (fTemp33 + (7.0f - fTemp36));
			float fTemp40 = (fTemp33 + (6.0f - fTemp36));
			float fTemp41 = (fTemp37 * fTemp38);
			float fTemp42 = (((((fRec1[(IOTA-int((int(min(512, max(0, iTemp35))) + 1)))&1023] * (0 - fTemp37)) * (0 - (0.5f * fTemp38))) * (0 - (0.33333334f * fTemp39))) * (0 - (0.25f * fTemp40))) + ((fTemp33 + (1e+01f - fTemp36)) * ((((((fRec1[(IOTA-int((int(min(512, max(0, (iTemp35 + 1)))) + 1)))&1023] * (0 - fTemp38)) * (0 - (0.5f * fTemp39))) * (0 - (0.33333334f * fTemp40))) + (0.5f * (((fTemp37 * fRec1[(IOTA-int((int(min(512, max(0, (iTemp35 + 2)))) + 1)))&1023]) * (0 - fTemp39)) * (0 - (0.5f * fTemp40))))) + (0.16666667f * ((fTemp41 * fRec1[(IOTA-int((int(min(512, max(0, (iTemp35 + 3)))) + 1)))&1023]) * (0 - fTemp40)))) + (0.041666668f * ((fTemp41 * fTemp39) * fRec1[(IOTA-int((int(min(512, max(0, (iTemp35 + 4)))) + 1)))&1023])))));
			fVec6[IOTA&131071] = fTemp42;
			float fTemp43 = ((float)input1[i] + (0.5f * (fTemp27 * ((fVec6[(IOTA-iTemp26)&131071] * fRec42[0]) + (fVec6[(IOTA-iTemp24)&131071] * fTemp25)))));
			float fTemp44 = ((fRec28[1] * fTemp7) + (fTemp1 * fTemp43));
			float fTemp45 = ((fRec31[1] * fTemp7) + (fTemp1 * fTemp44));
			float fTemp46 = ((fRec34[1] * fTemp7) + (fTemp1 * fTemp45));
			float fTemp47 = ((fRec37[1] * fTemp7) + (fTemp1 * fTemp46));
			float fTemp48 = ((6.123234e-17f * fTemp32) - fTemp47);
			fVec7[IOTA&16383] = fTemp48;
			fRec45[0] = (fSlow8 + (0.9999f * ((iSlow7 * iTemp12) + fRec45[1])));
			float fTemp49 = (fRec45[0] + -1.49999f);
			int iTemp50 = int(min(8192, max(0, int(fTemp49))));
			float fTemp51 = floorf(fTemp49);
			float fTemp52 = (fTemp51 + (2.0f - fRec45[0]));
			float fTemp53 = (fRec45[0] - fTemp51);
			fRec38[0] = (fVec7[(IOTA-int((iTemp50 + 1)))&16383] + ((fRec38[1] * (0 - (fTemp52 / fTemp53))) + ((fTemp52 * fVec7[(IOTA-iTemp50)&16383]) / fTemp53)));
			fRec36[0] = fRec38[0];
			float fTemp54 = (fTemp32 + (6.123234e-17f * fTemp47));
			fVec8[IOTA&16383] = fTemp54;
			fRec47[0] = (fSlow10 + (0.9999f * ((iSlow9 * iTemp12) + fRec47[1])));
			float fTemp55 = (fRec47[0] + -1.49999f);
			int iTemp56 = int(min(8192, max(0, int(fTemp55))));
			float fTemp57 = floorf(fTemp55);
			float fTemp58 = (fTemp57 + (2.0f - fRec47[0]));
			float fTemp59 = (fRec47[0] - fTemp57);
			fRec46[0] = (fVec8[(IOTA-int((iTemp56 + 1)))&16383] + ((fRec46[1] * (0 - (fTemp58 / fTemp59))) + ((fTemp58 * fVec8[(IOTA-iTemp56)&16383]) / fTemp59)));
			fRec37[0] = fRec46[0];
			float fTemp60 = ((fRec36[1] * fTemp1) + (fTemp31 * fTemp6));
			float fTemp61 = ((fTemp1 * fRec37[1]) + (fTemp46 * fTemp6));
			float fTemp62 = ((6.123234e-17f * fTemp60) - fTemp61);
			fVec9[IOTA&16383] = fTemp62;
			fRec48[0] = (fSlow13 + (0.999f * ((iSlow12 * iTemp12) + fRec48[1])));
			float fTemp63 = (fRec48[0] + -1.49999f);
			int iTemp64 = int(min(8192, max(0, int(fTemp63))));
			float fTemp65 = floorf(fTemp63);
			float fTemp66 = (fTemp65 + (2.0f - fRec48[0]));
			float fTemp67 = (fRec48[0] - fTemp65);
			fRec35[0] = (fVec9[(IOTA-int((iTemp64 + 1)))&16383] + ((fRec35[1] * (0 - (fTemp66 / fTemp67))) + ((fTemp66 * fVec9[(IOTA-iTemp64)&16383]) / fTemp67)));
			fRec33[0] = fRec35[0];
			float fTemp68 = ((6.123234e-17f * fTemp61) + fTemp60);
			fVec10[IOTA&16383] = fTemp68;
			fRec50[0] = (fSlow15 + (0.999f * ((iSlow14 * iTemp12) + fRec50[1])));
			float fTemp69 = (fRec50[0] + -1.49999f);
			int iTemp70 = int(min(8192, max(0, int(fTemp69))));
			float fTemp71 = floorf(fTemp69);
			float fTemp72 = (fTemp71 + (2.0f - fRec50[0]));
			float fTemp73 = (fRec50[0] - fTemp71);
			fRec49[0] = ((((fVec10[(IOTA-iTemp70)&16383] * fTemp72) / fTemp73) + fVec10[(IOTA-int((iTemp70 + 1)))&16383]) + (fRec49[1] * (0 - (fTemp72 / fTemp73))));
			fRec34[0] = fRec49[0];
			float fTemp74 = ((fRec33[1] * fTemp1) + (fTemp30 * fTemp6));
			float fTemp75 = ((fTemp1 * fRec34[1]) + (fTemp45 * fTemp6));
			float fTemp76 = ((6.123234e-17f * fTemp74) - fTemp75);
			fVec11[IOTA&16383] = fTemp76;
			fRec51[0] = (fSlow18 + (0.999f * ((iSlow17 * iTemp12) + fRec51[1])));
			float fTemp77 = (fRec51[0] + -1.49999f);
			int iTemp78 = int(min(8192, max(0, int(fTemp77))));
			float fTemp79 = floorf(fTemp77);
			float fTemp80 = (fTemp79 + (2.0f - fRec51[0]));
			float fTemp81 = (fRec51[0] - fTemp79);
			fRec32[0] = (fVec11[(IOTA-int((iTemp78 + 1)))&16383] + ((fRec32[1] * (0 - (fTemp80 / fTemp81))) + ((fTemp80 * fVec11[(IOTA-iTemp78)&16383]) / fTemp81)));
			fRec30[0] = fRec32[0];
			float fTemp82 = (fTemp74 + (6.123234e-17f * fTemp75));
			fVec12[IOTA&16383] = fTemp82;
			fRec53[0] = (fSlow20 + (0.999f * ((iSlow19 * iTemp12) + fRec53[1])));
			float fTemp83 = (fRec53[0] + -1.49999f);
			int iTemp84 = int(min(8192, max(0, int(fTemp83))));
			float fTemp85 = floorf(fTemp83);
			float fTemp86 = (fTemp85 + (2.0f - fRec53[0]));
			float fTemp87 = (fRec53[0] - fTemp85);
			fRec52[0] = (fVec12[(IOTA-int((iTemp84 + 1)))&16383] + ((fRec52[1] * (0 - (fTemp86 / fTemp87))) + ((fTemp86 * fVec12[(IOTA-iTemp84)&16383]) / fTemp87)));
			fRec31[0] = fRec52[0];
			float fTemp88 = ((fRec30[1] * fTemp1) + (fTemp29 * fTemp6));
			float fTemp89 = ((fTemp1 * fRec31[1]) + (fTemp44 * fTemp6));
			float fTemp90 = ((6.123234e-17f * fTemp88) - fTemp89);
			fVec13[IOTA&16383] = fTemp90;
			fRec54[0] = (fSlow22 + (0.999f * ((iSlow21 * iTemp12) + fRec54[1])));
			float fTemp91 = (fRec54[0] + -1.49999f);
			int iTemp92 = int(min(8192, max(0, int(fTemp91))));
			float fTemp93 = floorf(fTemp91);
			float fTemp94 = (fTemp93 + (2.0f - fRec54[0]));
			float fTemp95 = (fRec54[0] - fTemp93);
			fRec29[0] = (fVec13[(IOTA-int((iTemp92 + 1)))&16383] + ((fRec29[1] * (0 - (fTemp94 / fTemp95))) + ((fTemp94 * fVec13[(IOTA-iTemp92)&16383]) / fTemp95)));
			fRec27[0] = fRec29[0];
			float fTemp96 = (fTemp88 + (6.123234e-17f * fTemp89));
			fVec14[IOTA&16383] = fTemp96;
			fRec56[0] = (fSlow24 + (0.999f * ((iSlow23 * iTemp12) + fRec56[1])));
			float fTemp97 = (fRec56[0] + -1.49999f);
			int iTemp98 = int(min(8192, max(0, int(fTemp97))));
			float fTemp99 = floorf(fTemp97);
			float fTemp100 = (fTemp99 + (2.0f - fRec56[0]));
			float fTemp101 = (fRec56[0] - fTemp99);
			fRec55[0] = (fVec14[(IOTA-int((iTemp98 + 1)))&16383] + ((fRec55[1] * (0 - (fTemp100 / fTemp101))) + ((fTemp100 * fVec14[(IOTA-iTemp98)&16383]) / fTemp101)));
			fRec28[0] = fRec55[0];
			float fTemp102 = ((fRec27[1] * fTemp1) + (fTemp28 * fTemp6));
			float fTemp103 = ((fTemp4 * fRec15[1]) + (fTemp102 * fTemp5));
			float fTemp104 = ((fTemp4 * fRec18[1]) + (fTemp5 * fTemp103));
			float fTemp105 = ((fTemp4 * fRec21[1]) + (fTemp5 * fTemp104));
			float fTemp106 = ((fRec24[1] * fTemp4) + (fTemp5 * fTemp105));
			float fTemp107 = ((fTemp1 * fRec28[1]) + (fTemp43 * fTemp6));
			float fTemp108 = ((fTemp4 * fRec16[1]) + (fTemp5 * fTemp107));
			float fTemp109 = ((fTemp4 * fRec19[1]) + (fTemp5 * fTemp108));
			float fTemp110 = ((fTemp4 * fRec22[1]) + (fTemp5 * fTemp109));
			float fTemp111 = ((fTemp4 * fRec25[1]) + (fTemp5 * fTemp110));
			float fTemp112 = ((6.123234e-17f * fTemp106) - fTemp111);
			fVec15[IOTA&16383] = fTemp112;
			fRec57[0] = (fSlow27 + (0.9999f * ((iSlow26 * iTemp12) + fRec57[1])));
			float fTemp113 = (fRec57[0] + -1.49999f);
			int iTemp114 = int(min(8192, max(0, int(fTemp113))));
			float fTemp115 = floorf(fTemp113);
			float fTemp116 = (fTemp115 + (2.0f - fRec57[0]));
			float fTemp117 = (fRec57[0] - fTemp115);
			fRec26[0] = (fVec15[(IOTA-int((iTemp114 + 1)))&16383] + ((fRec26[1] * (0 - (fTemp116 / fTemp117))) + ((fTemp116 * fVec15[(IOTA-iTemp114)&16383]) / fTemp117)));
			fRec24[0] = fRec26[0];
			float fTemp118 = (fTemp106 + (6.123234e-17f * fTemp111));
			fVec16[IOTA&16383] = fTemp118;
			fRec59[0] = (fSlow29 + (0.9999f * ((iSlow28 * iTemp12) + fRec59[1])));
			float fTemp119 = (fRec59[0] + -1.49999f);
			int iTemp120 = int(min(8192, max(0, int(fTemp119))));
			float fTemp121 = floorf(fTemp119);
			float fTemp122 = (fTemp121 + (2.0f - fRec59[0]));
			float fTemp123 = (fRec59[0] - fTemp121);
			fRec58[0] = (fVec16[(IOTA-int((iTemp120 + 1)))&16383] + ((fRec58[1] * (0 - (fTemp122 / fTemp123))) + ((fTemp122 * fVec16[(IOTA-iTemp120)&16383]) / fTemp123)));
			fRec25[0] = fRec58[0];
			float fTemp124 = ((fRec24[1] * fTemp5) + (fTemp3 * fTemp105));
			float fTemp125 = ((fTemp5 * fRec25[1]) + (fTemp3 * fTemp110));
			float fTemp126 = ((6.123234e-17f * fTemp124) - fTemp125);
			fVec17[IOTA&16383] = fTemp126;
			fRec60[0] = (fSlow32 + (0.999f * ((iSlow31 * iTemp12) + fRec60[1])));
			float fTemp127 = (fRec60[0] + -1.49999f);
			int iTemp128 = int(min(8192, max(0, int(fTemp127))));
			float fTemp129 = floorf(fTemp127);
			float fTemp130 = (fTemp129 + (2.0f - fRec60[0]));
			float fTemp131 = (fRec60[0] - fTemp129);
			fRec23[0] = (fVec17[(IOTA-int((iTemp128 + 1)))&16383] + (((0 - (fTemp130 / fTemp131)) * fRec23[1]) + ((fVec17[(IOTA-iTemp128)&16383] * fTemp130) / fTemp131)));
			fRec21[0] = fRec23[0];
			float fTemp132 = (fTemp124 + (6.123234e-17f * fTemp125));
			fVec18[IOTA&16383] = fTemp132;
			fRec62[0] = (fSlow34 + (0.999f * ((iSlow33 * iTemp12) + fRec62[1])));
			float fTemp133 = (fRec62[0] + -1.49999f);
			int iTemp134 = int(min(8192, max(0, int(fTemp133))));
			float fTemp135 = floorf(fTemp133);
			float fTemp136 = (fTemp135 + (2.0f - fRec62[0]));
			float fTemp137 = (fRec62[0] - fTemp135);
			fRec61[0] = (fVec18[(IOTA-int((iTemp134 + 1)))&16383] + ((fRec61[1] * (0 - (fTemp136 / fTemp137))) + ((fTemp136 * fVec18[(IOTA-iTemp134)&16383]) / fTemp137)));
			fRec22[0] = fRec61[0];
			float fTemp138 = ((fRec21[1] * fTemp5) + (fTemp3 * fTemp104));
			float fTemp139 = ((fTemp5 * fRec22[1]) + (fTemp3 * fTemp109));
			float fTemp140 = ((6.123234e-17f * fTemp138) - fTemp139);
			fVec19[IOTA&16383] = fTemp140;
			fRec63[0] = (fSlow37 + (0.999f * ((iSlow36 * iTemp12) + fRec63[1])));
			float fTemp141 = (fRec63[0] + -1.49999f);
			int iTemp142 = int(min(8192, max(0, int(fTemp141))));
			float fTemp143 = floorf(fTemp141);
			float fTemp144 = (fTemp143 + (2.0f - fRec63[0]));
			float fTemp145 = (fRec63[0] - fTemp143);
			fRec20[0] = ((((fVec19[(IOTA-iTemp142)&16383] * fTemp144) / fTemp145) + fVec19[(IOTA-int((iTemp142 + 1)))&16383]) + (fRec20[1] * (0 - (fTemp144 / fTemp145))));
			fRec18[0] = fRec20[0];
			float fTemp146 = (fTemp138 + (6.123234e-17f * fTemp139));
			fVec20[IOTA&16383] = fTemp146;
			fRec65[0] = (fSlow39 + (0.999f * ((iSlow38 * iTemp12) + fRec65[1])));
			float fTemp147 = (fRec65[0] + -1.49999f);
			int iTemp148 = int(min(8192, max(0, int(fTemp147))));
			float fTemp149 = floorf(fTemp147);
			float fTemp150 = (fTemp149 + (2.0f - fRec65[0]));
			float fTemp151 = (fRec65[0] - fTemp149);
			fRec64[0] = (fVec20[(IOTA-int((iTemp148 + 1)))&16383] + ((fRec64[1] * (0 - (fTemp150 / fTemp151))) + ((fTemp150 * fVec20[(IOTA-iTemp148)&16383]) / fTemp151)));
			fRec19[0] = fRec64[0];
			float fTemp152 = ((fRec18[1] * fTemp5) + (fTemp3 * fTemp103));
			float fTemp153 = ((fTemp5 * fRec19[1]) + (fTemp3 * fTemp108));
			float fTemp154 = ((6.123234e-17f * fTemp152) - fTemp153);
			fVec21[IOTA&16383] = fTemp154;
			fRec66[0] = (fSlow42 + (0.999f * ((iSlow41 * iTemp12) + fRec66[1])));
			float fTemp155 = (fRec66[0] + -1.49999f);
			int iTemp156 = int(min(8192, max(0, int(fTemp155))));
			float fTemp157 = floorf(fTemp155);
			float fTemp158 = (fTemp157 + (2.0f - fRec66[0]));
			float fTemp159 = (fRec66[0] - fTemp157);
			fRec17[0] = ((((fVec21[(IOTA-iTemp156)&16383] * fTemp158) / fTemp159) + fVec21[(IOTA-int((iTemp156 + 1)))&16383]) + (fRec17[1] * (0 - (fTemp158 / fTemp159))));
			fRec15[0] = fRec17[0];
			float fTemp160 = (fTemp152 + (6.123234e-17f * fTemp153));
			fVec22[IOTA&16383] = fTemp160;
			fRec68[0] = (fSlow44 + (0.999f * ((iSlow43 * iTemp12) + fRec68[1])));
			float fTemp161 = (fRec68[0] + -1.49999f);
			int iTemp162 = int(min(8192, max(0, int(fTemp161))));
			float fTemp163 = floorf(fTemp161);
			float fTemp164 = (fTemp163 + (2.0f - fRec68[0]));
			float fTemp165 = (fRec68[0] - fTemp163);
			fRec67[0] = (fVec22[(IOTA-int((iTemp162 + 1)))&16383] + ((fRec67[1] * (0 - (fTemp164 / fTemp165))) + ((fTemp164 * fVec22[(IOTA-iTemp162)&16383]) / fTemp165)));
			fRec16[0] = fRec67[0];
			float fTemp166 = ((fRec15[1] * fTemp5) + (fTemp3 * fTemp102));
			float fTemp167 = ((fTemp1 * fTemp166) + (fTemp7 * fRec3[1]));
			float fTemp168 = ((fTemp1 * fTemp167) + (fTemp7 * fRec6[1]));
			float fTemp169 = ((fTemp1 * fTemp168) + (fTemp7 * fRec9[1]));
			float fTemp170 = (fTemp1 * fTemp169);
			float fTemp171 = (fRec12[1] * fTemp7);
			float fTemp172 = ((fTemp5 * fRec16[1]) + (fTemp3 * fTemp107));
			float fTemp173 = ((fRec4[1] * fTemp7) + (fTemp1 * fTemp172));
			float fTemp174 = ((fRec7[1] * fTemp7) + (fTemp1 * fTemp173));
			float fTemp175 = ((fRec10[1] * fTemp7) + (fTemp1 * fTemp174));
			float fTemp176 = ((fRec13[1] * fTemp7) + (fTemp1 * fTemp175));
			float fTemp177 = ((6.123234e-17f * (fTemp170 + fTemp171)) - fTemp176);
			fVec23[IOTA&16383] = fTemp177;
			fRec69[0] = (fSlow47 + (0.9999f * ((iSlow46 * iTemp12) + fRec69[1])));
			float fTemp178 = (fRec69[0] + -1.49999f);
			int iTemp179 = int(min(8192, max(0, int(fTemp178))));
			float fTemp180 = floorf(fTemp178);
			float fTemp181 = (fTemp180 + (2.0f - fRec69[0]));
			float fTemp182 = (fRec69[0] - fTemp180);
			fRec14[0] = ((((fVec23[(IOTA-iTemp179)&16383] * fTemp181) / fTemp182) + fVec23[(IOTA-int((iTemp179 + 1)))&16383]) + (fRec14[1] * (0 - (fTemp181 / fTemp182))));
			fRec12[0] = fRec14[0];
			float fTemp183 = ((fTemp170 + (6.123234e-17f * fTemp176)) + fTemp171);
			fVec24[IOTA&16383] = fTemp183;
			fRec71[0] = (fSlow49 + (0.9999f * ((iSlow48 * iTemp12) + fRec71[1])));
			float fTemp184 = (fRec71[0] + -1.49999f);
			int iTemp185 = int(min(8192, max(0, int(fTemp184))));
			float fTemp186 = floorf(fTemp184);
			float fTemp187 = (fTemp186 + (2.0f - fRec71[0]));
			float fTemp188 = (fRec71[0] - fTemp186);
			fRec70[0] = (fVec24[(IOTA-int((iTemp185 + 1)))&16383] + ((fRec70[1] * (0 - (fTemp187 / fTemp188))) + ((fTemp187 * fVec24[(IOTA-iTemp185)&16383]) / fTemp188)));
			fRec13[0] = fRec70[0];
			float fTemp189 = ((fTemp1 * fRec12[1]) + (fTemp169 * fTemp6));
			float fTemp190 = ((fTemp1 * fRec13[1]) + (fTemp175 * fTemp6));
			float fTemp191 = ((6.123234e-17f * fTemp189) - fTemp190);
			fVec25[IOTA&16383] = fTemp191;
			fRec72[0] = (fSlow52 + (0.999f * ((iSlow51 * iTemp12) + fRec72[1])));
			float fTemp192 = (fRec72[0] + -1.49999f);
			int iTemp193 = int(min(8192, max(0, int(fTemp192))));
			float fTemp194 = floorf(fTemp192);
			float fTemp195 = (fTemp194 + (2.0f - fRec72[0]));
			float fTemp196 = (fRec72[0] - fTemp194);
			fRec11[0] = ((((fVec25[(IOTA-iTemp193)&16383] * fTemp195) / fTemp196) + fVec25[(IOTA-int((iTemp193 + 1)))&16383]) + (fRec11[1] * (0 - (fTemp195 / fTemp196))));
			fRec9[0] = fRec11[0];
			float fTemp197 = (fTemp189 + (6.123234e-17f * fTemp190));
			fVec26[IOTA&16383] = fTemp197;
			fRec74[0] = (fSlow54 + (0.999f * ((iSlow53 * iTemp12) + fRec74[1])));
			float fTemp198 = (fRec74[0] + -1.49999f);
			int iTemp199 = int(min(8192, max(0, int(fTemp198))));
			float fTemp200 = floorf(fTemp198);
			float fTemp201 = (fTemp200 + (2.0f - fRec74[0]));
			float fTemp202 = (fRec74[0] - fTemp200);
			fRec73[0] = (fVec26[(IOTA-int((iTemp199 + 1)))&16383] + ((fRec73[1] * (0 - (fTemp201 / fTemp202))) + ((fTemp201 * fVec26[(IOTA-iTemp199)&16383]) / fTemp202)));
			fRec10[0] = fRec73[0];
			float fTemp203 = ((fTemp1 * fRec9[1]) + (fTemp168 * fTemp6));
			float fTemp204 = ((fTemp1 * fRec10[1]) + (fTemp174 * fTemp6));
			float fTemp205 = ((6.123234e-17f * fTemp203) - fTemp204);
			fVec27[IOTA&16383] = fTemp205;
			fRec75[0] = (fSlow57 + (0.999f * ((iSlow56 * iTemp12) + fRec75[1])));
			float fTemp206 = (fRec75[0] + -1.49999f);
			int iTemp207 = int(min(8192, max(0, int(fTemp206))));
			float fTemp208 = floorf(fTemp206);
			float fTemp209 = (fTemp208 + (2.0f - fRec75[0]));
			float fTemp210 = (fRec75[0] - fTemp208);
			fRec8[0] = ((((fVec27[(IOTA-iTemp207)&16383] * fTemp209) / fTemp210) + fVec27[(IOTA-int((iTemp207 + 1)))&16383]) + (fRec8[1] * (0 - (fTemp209 / fTemp210))));
			fRec6[0] = fRec8[0];
			float fTemp211 = (fTemp203 + (6.123234e-17f * fTemp204));
			fVec28[IOTA&16383] = fTemp211;
			fRec77[0] = (fSlow59 + (0.999f * ((iSlow58 * iTemp12) + fRec77[1])));
			float fTemp212 = (fRec77[0] + -1.49999f);
			int iTemp213 = int(min(8192, max(0, int(fTemp212))));
			float fTemp214 = floorf(fTemp212);
			float fTemp215 = (fTemp214 + (2.0f - fRec77[0]));
			float fTemp216 = (fRec77[0] - fTemp214);
			fRec76[0] = (fVec28[(IOTA-int((iTemp213 + 1)))&16383] + ((fRec76[1] * (0 - (fTemp215 / fTemp216))) + ((fTemp215 * fVec28[(IOTA-iTemp213)&16383]) / fTemp216)));
			fRec7[0] = fRec76[0];
			float fTemp217 = ((fTemp1 * fRec6[1]) + (fTemp167 * fTemp6));
			float fTemp218 = ((fTemp1 * fRec7[1]) + (fTemp173 * fTemp6));
			float fTemp219 = ((6.123234e-17f * fTemp217) - fTemp218);
			fVec29[IOTA&16383] = fTemp219;
			fRec78[0] = (fSlow62 + (0.999f * ((iSlow61 * iTemp12) + fRec78[1])));
			float fTemp220 = (fRec78[0] + -1.49999f);
			int iTemp221 = int(min(8192, max(0, int(fTemp220))));
			float fTemp222 = floorf(fTemp220);
			float fTemp223 = (fTemp222 + (2.0f - fRec78[0]));
			float fTemp224 = (fRec78[0] - fTemp222);
			fRec5[0] = ((((fVec29[(IOTA-iTemp221)&16383] * fTemp223) / fTemp224) + fVec29[(IOTA-int((iTemp221 + 1)))&16383]) + (fRec5[1] * (0 - (fTemp223 / fTemp224))));
			fRec3[0] = fRec5[0];
			float fTemp225 = (fTemp217 + (6.123234e-17f * fTemp218));
			fVec30[IOTA&16383] = fTemp225;
			fRec80[0] = (fSlow64 + (0.999f * ((iSlow63 * iTemp12) + fRec80[1])));
			float fTemp226 = (fRec80[0] + -1.49999f);
			int iTemp227 = int(min(8192, max(0, int(fTemp226))));
			float fTemp228 = floorf(fTemp226);
			float fTemp229 = (fTemp228 + (2.0f - fRec80[0]));
			float fTemp230 = (fRec80[0] - fTemp228);
			fRec79[0] = (fVec30[(IOTA-int((iTemp227 + 1)))&16383] + ((fRec79[1] * (0 - (fTemp229 / fTemp230))) + ((fTemp229 * fVec30[(IOTA-iTemp227)&16383]) / fTemp230)));
			fRec4[0] = fRec79[0];
			fVec31[0] = fSlow65;
			float fTemp231 = (fSlow65 + fVec31[1]);
			float fTemp232 = (1.0f - (0.5f * fTemp231));
			fRec2[0] = ((((fTemp1 * fRec3[1]) + (fTemp166 * fTemp6)) * fTemp232) + (0.5f * (fTemp231 * fRec2[1])));
			fRec0[IOTA&1023] = fRec2[0];
			fRec81[0] = ((0.5f * (fTemp231 * fRec81[1])) + (fTemp232 * ((fTemp172 * fTemp6) + (fTemp1 * fRec4[1]))));
			fRec1[IOTA&1023] = fRec81[0];
			output0[i] = (FAUSTFLOAT)fRec0[(IOTA-0)&1023];
			output1[i] = (FAUSTFLOAT)fRec1[(IOTA-0)&1023];
			// post processing
			fRec81[1] = fRec81[0];
			fRec2[1] = fRec2[0];
			fVec31[1] = fVec31[0];
			fRec4[1] = fRec4[0];
			fRec79[1] = fRec79[0];
			fRec80[1] = fRec80[0];
			fRec3[1] = fRec3[0];
			fRec5[1] = fRec5[0];
			fRec78[1] = fRec78[0];
			fRec7[1] = fRec7[0];
			fRec76[1] = fRec76[0];
			fRec77[1] = fRec77[0];
			fRec6[1] = fRec6[0];
			fRec8[1] = fRec8[0];
			fRec75[1] = fRec75[0];
			fRec10[1] = fRec10[0];
			fRec73[1] = fRec73[0];
			fRec74[1] = fRec74[0];
			fRec9[1] = fRec9[0];
			fRec11[1] = fRec11[0];
			fRec72[1] = fRec72[0];
			fRec13[1] = fRec13[0];
			fRec70[1] = fRec70[0];
			fRec71[1] = fRec71[0];
			fRec12[1] = fRec12[0];
			fRec14[1] = fRec14[0];
			fRec69[1] = fRec69[0];
			fRec16[1] = fRec16[0];
			fRec67[1] = fRec67[0];
			fRec68[1] = fRec68[0];
			fRec15[1] = fRec15[0];
			fRec17[1] = fRec17[0];
			fRec66[1] = fRec66[0];
			fRec19[1] = fRec19[0];
			fRec64[1] = fRec64[0];
			fRec65[1] = fRec65[0];
			fRec18[1] = fRec18[0];
			fRec20[1] = fRec20[0];
			fRec63[1] = fRec63[0];
			fRec22[1] = fRec22[0];
			fRec61[1] = fRec61[0];
			fRec62[1] = fRec62[0];
			fRec21[1] = fRec21[0];
			fRec23[1] = fRec23[0];
			fRec60[1] = fRec60[0];
			fRec25[1] = fRec25[0];
			fRec58[1] = fRec58[0];
			fRec59[1] = fRec59[0];
			fRec24[1] = fRec24[0];
			fRec26[1] = fRec26[0];
			fRec57[1] = fRec57[0];
			fRec28[1] = fRec28[0];
			fRec55[1] = fRec55[0];
			fRec56[1] = fRec56[0];
			fRec27[1] = fRec27[0];
			fRec29[1] = fRec29[0];
			fRec54[1] = fRec54[0];
			fRec31[1] = fRec31[0];
			fRec52[1] = fRec52[0];
			fRec53[1] = fRec53[0];
			fRec30[1] = fRec30[0];
			fRec32[1] = fRec32[0];
			fRec51[1] = fRec51[0];
			fRec34[1] = fRec34[0];
			fRec49[1] = fRec49[0];
			fRec50[1] = fRec50[0];
			fRec33[1] = fRec33[0];
			fRec35[1] = fRec35[0];
			fRec48[1] = fRec48[0];
			fRec37[1] = fRec37[0];
			fRec46[1] = fRec46[0];
			fRec47[1] = fRec47[0];
			fRec36[1] = fRec36[0];
			fRec38[1] = fRec38[0];
			fRec45[1] = fRec45[0];
			fVec5[1] = fVec5[0];
			fRec44[1] = fRec44[0];
			fRec43[1] = fRec43[0];
			fRec42[1] = fRec42[0];
			fRec41[1] = fRec41[0];
			IOTA = IOTA+1;
			fRec40[1] = fRec40[0];
			fRec39[1] = fRec39[0];
			fVec3[1] = fVec3[0];
			fVec2[1] = fVec2[0];
			iVec1[1] = iVec1[0];
			fVec0[1] = fVec0[0];
		}
	}
};



//----------------------------------------------------------------------------
// SuperCollider/Faust interface
//----------------------------------------------------------------------------


#undef primes
// Thin SC-wrapper equivalent: set the 7 controls each block, then compute.
struct GreyholeRaw {
  void Init(double sr) {
    dsp_.instanceInit(static_cast<int>(sr));
  }
  void ProcessBlock(const float* inL, const float* inR, float* outL,
                    float* outR, int n, float damping, float delayTime,
                    float diffusion, float feedback, float modDepth,
                    float modFreq, float size) {
    dsp_.SetParams(damping, delayTime, diffusion, feedback, modDepth,
                   modFreq, size);
    FAUSTFLOAT* inputs[2] = {const_cast<FAUSTFLOAT*>(inL),
                             const_cast<FAUSTFLOAT*>(inR)};
    FAUSTFLOAT* outputs[2] = {outL, outR};
    dsp_.compute(n, inputs, outputs);
  }
  GreyholeRawDsp dsp_;
};
}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_GREYHOLE_H_
