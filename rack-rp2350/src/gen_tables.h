#ifndef GEN_TABLES_H
#define GEN_TABLES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 1. 20 CANONICAL 12-BIT PITCH-CLASS SCALE MASKS
 * Bit s (0..11) = (1u << s) indicates presence of semitone s above root.
 * Bit 0 (root) is always set.
 * ========================================================================== */
typedef enum {
    SCALE_CHROMATIC = 0,
    SCALE_IONIAN_MAJOR,
    SCALE_DORIAN,
    SCALE_PHRYGIAN,
    SCALE_LYDIAN,
    SCALE_MIXOLYDIAN,
    SCALE_AEOLIAN_MINOR,
    SCALE_LOCRIAN,
    SCALE_PENTATONIC_MAJ,
    SCALE_PENTATONIC_MIN,
    SCALE_BLUES,
    SCALE_HARMONIC_MINOR,
    SCALE_MELODIC_MINOR,
    SCALE_PHRYGIAN_DOM,
    SCALE_WHOLE_TONE,
    SCALE_OCTATONIC_HW,
    SCALE_HIRAJOSHI,
    SCALE_INSEN,
    SCALE_DOUBLE_HARMONIC,
    SCALE_ACID_DOM7_POOL,
    SCALE_COUNT
} scale_id_t;

static const uint16_t SCALE_BITMASKS[SCALE_COUNT] = {
    0x0FFFu, /*  0: Chromatic          [0,1,2,3,4,5,6,7,8,9,10,11] */
    0x0AB5u, /*  1: Ionian (Major)     [0,2,4,5,7,9,11]            */
    0x06ADu, /*  2: Dorian             [0,2,3,5,7,9,10]            */
    0x05ABu, /*  3: Phrygian           [0,1,3,5,7,8,10]            */
    0x0AD5u, /*  4: Lydian             [0,2,4,6,7,9,11]            */
    0x06B5u, /*  5: Mixolydian         [0,2,4,5,7,9,10]            */
    0x05ADu, /*  6: Aeolian (Nat Min)  [0,2,3,5,7,8,10]            */
    0x056Bu, /*  7: Locrian            [0,1,3,5,6,8,10]            */
    0x0295u, /*  8: Major Pentatonic   [0,2,4,7,9]                 */
    0x04A9u, /*  9: Minor Pentatonic   [0,3,5,7,10]                */
    0x04E9u, /* 10: Blues Scale        [0,3,5,6,7,10]              */
    0x09ADu, /* 11: Harmonic Minor     [0,2,3,5,7,8,11]            */
    0x0AADu, /* 12: Melodic Minor      [0,2,3,5,7,9,11]            */
    0x05B3u, /* 13: Phrygian Dominant  [0,1,4,5,7,8,10]            */
    0x0555u, /* 14: Whole Tone         [0,2,4,6,8,10]              */
    0x06DBu, /* 15: Octatonic (H-W)    [0,1,3,4,6,7,9,10]          */
    0x018Du, /* 16: Hirajoshi          [0,2,3,7,8]                 */
    0x04A3u, /* 17: In-Sen             [0,1,5,7,10]                */
    0x09B3u, /* 18: Double Harmonic    [0,1,4,5,7,8,11]            */
    0x0499u  /* 19: Acid Dom7 Pool     [0,3,4,7,10]                */
};

/* ==========================================================================
 * 2. HARMONIC GRAVITY & VOICE-LEADING TRANSITION TABLES (uint8_t [0..255])
 * ========================================================================== */

/* Krumhansl-Schmuckler Empirical Key Profiles normalized to uint8_t [0..255]
 * Index = Semitones above tonic (0 = Root, 7 = Fifth, etc.) */
static const uint8_t KS_GRAVITY_MAJOR[12] = {
    255,  90, 140,  94, 176, 164, 101, 208,  96, 147,  92, 116
};

static const uint8_t KS_GRAVITY_MINOR[12] = {
    255, 108, 142, 217, 105, 142, 102, 191, 160, 108, 135, 128
};

/* Metrical Beat-Hierarchy Scale-Degree Gravity Weights (for 7-note scales)
 * Row 0: Strong Beats (Steps 0, 4, 8, 12) -> Locks tightly to Root (0), 5th (4), 3rd (2)
 * Row 1: Medium Beats (Steps 2, 6, 10, 14) -> Balances Triad + 7th (6) & 4th/6th
 * Row 2: Weak 16th Beats (Odd steps)       -> Favors passing/tendency degrees (1, 3, 5, 6) */
static const uint8_t DEGREE_METRICAL_GRAVITY[3][7] = {
    /* Deg 0(1st)  Deg 1(2nd)  Deg 2(3rd)  Deg 3(4th)  Deg 4(5th)  Deg 5(6th)  Deg 6(7th) */
    {    255,         15,        180,         20,        210,         15,         40 }, /* Strong */
    {    180,         60,        170,         70,        190,         85,        120 }, /* Medium */
    {     60,        165,         95,        155,         90,        150,        175 }  /* Weak   */
};

/* First-Order Scale-Degree Voice-Leading Markov Matrix (7x7)
 * Row = Current Scale Degree (0..6), Col = Next Scale Degree (0..6).
 * Enforces classical/electronic tendency tones:
 *   - Degree 6 (Leading tone / b7) resolves strongly to Degree 0 (Root) or 4 (5th)
 *   - Degree 3 (Subdominant 4th) resolves strongly down to Degree 2 (3rd)
 *   - Degree 1 (Supertonic 2nd) moves to Degree 0 (Root) or Degree 2 (3rd)
 *   - Degree 5 (Submediant 6th) gravitates down to Degree 4 (5th) */
static const uint8_t DEGREE_MARKOV_7X7[7][7] = {
    /* To:  d0   d1   d2   d3   d4   d5   d6  */
    /* d0 */ { 30,  70,  65,  35,  60,  20,  40 },
    /* d1 */ { 95,  10,  85,  25,  40,  10,  25 },
    /* d2 */ { 55,  65,  20,  60,  55,  25,  15 },
    /* d3 */ { 35,  30, 110,  10,  65,  20,  15 },
    /* d4 */ { 80,  25,  45,  50,  25,  60,  45 },
    /* d5 */ { 30,  20,  30,  40, 110,  10,  55 },
    /* d6 */ {140,  20,  25,  15,  50,  25,  10 }
};

/* ==========================================================================
 * 3. DIATONIC & NEO-RIEMANNIAN CHORD PROGRESSION STATE MACHINES
 * ========================================================================== */

/* HYP-1 Empirical Diatonic Chord Drift Weights (Degrees I..VII) */
static const uint8_t DIATONIC_W7_WEIGHTS[7] = { 40, 20, 10, 30, 30, 30, 4 };

/* Functional Diatonic Chord Progression Markov Matrix (I..VII -> I..VII) */
static const uint8_t CHORD_MARKOV_7X7[7][7] = {
    /* From \ To:  I   II  III   IV    V   VI  VII */
    /* I   */ {   0,  35,  20,  65,  65,  55,  10 },
    /* II  */ {  15,   0,  10,  20, 110,  15,  25 },
    /* III */ {  15,  35,   0,  65,  20,  80,  10 },
    /* IV  */ {  55,  45,  10,   0,  85,  25,  20 },
    /* V   */ { 120,  15,  10,  35,   0,  65,  10 },
    /* VI  */ {  25,  65,  25,  70,  45,   0,  10 },
    /* VII */ { 130,  10,  20,  15,  40,  35,   0 }
};

/* Neo-Riemannian PLR & NSH Triad Transformations
 * Each transform flips triad quality (0=Major <-> 1=Minor) and adds a
 * semitone delta (mod 12) to the root pitch class depending on starting quality. */
typedef enum {
    NR_TRANSFORM_P = 0, /* Parallel:          C Maj <-> C Min  */
    NR_TRANSFORM_L,     /* Leittonwechsel:    C Maj <-> E Min  */
    NR_TRANSFORM_R,     /* Relative:          C Maj <-> A Min  */
    NR_TRANSFORM_N,     /* Nebenverwandt:     C Maj <-> F Min  */
    NR_TRANSFORM_S,     /* Slide (shared 3rd):C Maj <-> C# Min */
    NR_TRANSFORM_H,     /* Hexatonic Pole:    C Maj <-> Ab Min */
    NR_TRANSFORM_COUNT
} neo_riemannian_id_t;

/* Col 0: Semitone delta when current triad is Major (0 -> 1)
 * Col 1: Semitone delta when current triad is Minor (1 -> 0) */
static const uint8_t NEO_RIEMANNIAN_DELTAS[NR_TRANSFORM_COUNT][2] = {
    { 0,  0 }, /* P: Maj+0 -> Min,  Min+0  -> Maj */
    { 4,  8 }, /* L: Maj+4 -> Min,  Min+8  -> Maj */
    { 9,  3 }, /* R: Maj+9 -> Min,  Min+3  -> Maj */
    { 5,  7 }, /* N: Maj+5 -> Min,  Min+7  -> Maj */
    { 1, 11 }, /* S: Maj+1 -> Min,  Min+11 -> Maj */
    { 8,  4 }  /* H: Maj+8 -> Min,  Min+4  -> Maj */
};

static inline void apply_neo_riemannian(uint8_t *root_pc, uint8_t *is_minor, neo_riemannian_id_t op) {
    uint8_t q = (*is_minor) ? 1u : 0u;
    uint8_t delta = NEO_RIEMANNIAN_DELTAS[op % NR_TRANSFORM_COUNT][q];
    *root_pc = (uint8_t)((*root_pc + delta) % 12u);
    *is_minor = (uint8_t)(1u - q);
}

/* ==========================================================================
 * 4. 16 CANONICAL 1-BAR MELODIC CONTOUR & MOTIF ARCHETYPES (int8_t [16][16])
 * Expressed in signed scale-degree offsets relative to the active chord root
 * (where 0 = Root, 2 = 3rd, 4 = 5th, 7 = Octave in a 7-note diatonic scale).
 * ========================================================================== */
static const int8_t MELODIC_CONTOURS_DEG[16][16] = {
    /*  0: Arch (Smooth diatonic swell peaking at octave on step 7) */
    {  0,  1,  2,  3,  4,  5,  6,  7,  6,  5,  4,  3,  2,  1,  0,  0 },
    /*  1: Inverted Arch (Valley descent into lower register, rising back home) */
    {  0, -1, -2, -3, -4, -5, -6, -7, -6, -5, -4, -3, -2, -1,  0,  0 },
    /*  2: Pendulum (Converging oscillation between low root and high octave) */
    {  0,  7,  1,  6,  2,  5,  3,  4,  3,  5,  2,  6,  1,  7, -1,  0 },
    /*  3: Pedal-Point Ostinato (Grounded root on even steps, rising melody on odd steps) */
    {  0,  2,  0,  3,  0,  4,  0,  5,  0,  6,  0,  7,  0,  4,  0, -1 },
    /*  4: Acid Octave-Bounce (Syncopated octave pops and b7/3rd inflections) */
    {  0,  0,  7,  0,  0,  7,  0,  2,  0,  0,  7,  0,  7,  0,  4, -1 },
    /*  5: Rising Cadence (Two-octave diatonic climb from -7 to +7) */
    { -7, -6, -5, -4, -3, -2, -1,  0,  1,  2,  3,  4,  5,  6,  7,  7 },
    /*  6: Falling Resolution (Cascading descent from high octave to low root) */
    {  7,  6,  5,  4,  3,  2,  1,  0, -1, -2, -3, -4, -5, -6, -7, -7 },
    /*  7: Double Wave Hump (Two symmetrical phrase peaks on beats 1.4 and 3.4) */
    {  0,  2,  4,  7,  4,  2,  1,  0,  1,  3,  5,  7,  5,  3,  1,  0 },
    /*  8: High Plateau (Rapid arpeggiated ascent to sustained octave, late drop) */
    {  0,  4,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  4,  2,  0 },
    /*  9: Terraced Sequence Ascending (Four 4-step rising diatonic sequences) */
    {  0,  1,  2,  4,  1,  2,  3,  5,  2,  3,  4,  6,  3,  4,  5,  7 },
    /* 10: Terraced Sequence Descending (Four 4-step tumbling diatonic sequences) */
    {  7,  5,  4,  2,  6,  4,  3,  1,  5,  3,  2,  0,  4,  2,  1,  0 },
    /* 11: Triad + 7th Arpeggio Wave (Up-down 1-3-5-7-8 voice-led arpeggiation) */
    {  0,  2,  4,  6,  7,  6,  4,  2,  0,  2,  4,  6,  7,  6,  4,  1 },
    /* 12: Cresting Tension Wave (Slow chromatic/diatonic creep to step 12, fast drop) */
    {  0,  0,  1,  1,  2,  2,  3,  4,  5,  6,  7,  8,  9,  4,  2, -1 },
    /* 13: Grounded Techno Sub-Bass (Root anchor with syncopated 7th/octave stabs) */
    {  0,  0,  0,  6,  0,  0,  7,  0,  0,  0,  0,  6,  0,  7,  0,  4 },
    /* 14: Turnaround Fill Cascade (Root-fifth alternation followed by steps 10..15 run) */
    {  0,  4,  0,  4,  0,  4,  0,  4,  0,  4,  7,  6,  5,  4,  2,  1 },
    /* 15: Syncopated Call & Response (Question leap on beat 1-2, step answer on 3-4) */
    {  0,  4,  7,  0,  0,  0,  5,  4,  2,  0,  0,  0,  4,  3,  1,  0 }
};

/* Metrical Mutation Sensitivity (0..255) for Algo 4 (Deja-Vu):
 * Weak 16th steps (odd indices) have high sensitivity (220..255) and mutate first.
 * Downbeats (steps 0, 8) have low sensitivity (35..60) and stay anchored longest. */
static const uint8_t METRIC_MUTATE_SENSITIVITY[16] = {
     35, 240, 140, 255,  85, 230, 150, 255,
     60, 240, 140, 255,  95, 235, 160, 250
};

/* ==========================================================================
 * 5. FAST INTEGER SCALE & DEGREE CONVERSION UTILITIES
 * Negative-safe Euclidean floor division and modulo (C99 '%' truncates toward 0!)
 * ========================================================================== */
static inline int16_t floor_div_i16(int16_t a, int16_t b) {
    int16_t q = a / b;
    int16_t r = a % b;
    if ((r != 0) && ((r < 0) != (b < 0))) {
        q--;
    }
    return q;
}

static inline int16_t floor_mod_i16(int16_t a, int16_t b) {
    int16_t r = a % b;
    if ((r != 0) && ((r < 0) != (b < 0))) {
        r += b;
    }
    return r;
}

/* Expands a 12-bit scale mask into ascending semitone offsets (0..11), returns note count M */
static inline uint8_t expand_scale_mask(uint16_t mask, uint8_t steps_out[12]) {
    uint8_t count = 0;
    mask |= 0x0001u; /* Always force root bit 0 active */
    for (uint8_t s = 0; s < 12u; s++) {
        if ((mask >> s) & 0x01u) {
            steps_out[count++] = s;
        }
    }
    return count;
}

/* Converts a chord-relative scale degree offset into a folded MIDI note [28..96] */
static inline uint8_t degree_to_midi_folded(uint8_t root_pc,
                                            int8_t chord_deg,
                                            int16_t deg_offset,
                                            int8_t oct_offset,
                                            const uint8_t scale_steps[12],
                                            uint8_t num_steps)
{
    int16_t m = (num_steps > 0) ? (int16_t)num_steps : 7;
    int16_t total_deg = (int16_t)chord_deg + deg_offset;
    int16_t oct_shift = floor_div_i16(total_deg, m) + (int16_t)oct_offset;
    int16_t scale_idx = floor_mod_i16(total_deg, m);

    int16_t base = (root_pc >= 24) ? (int16_t)root_pc : (int16_t)(36 + (root_pc % 12u));
    int16_t midi_note = base
                      + (int16_t)scale_steps[scale_idx]
                      + (12 * oct_shift);

    while (midi_note > 96) midi_note -= 12;
    while (midi_note < 28) midi_note += 12;
    return (uint8_t)midi_note;
}

#ifdef __cplusplus
}
#endif

#endif /* GEN_TABLES_H */
