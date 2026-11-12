## Embedded Generative Melodic Sequencer Architectures and C99 Firmware Implementations for 16-Step Hardware Synthesizers

Monophonic hardware sequencers and Eurorack generative modules avoid the aimless, "white-noise" quality of unconstrained pseudo-random number generation by separating **rhythmic gate generation**, **scale-degree/chord-relative pitch indexing**, **phrase-level contour modulation**, and **stateful loop memory** into orthogonal, mathematically constrained layers. Instead of selecting raw MIDI notes from a uniform distribution, musical engines operate in **chord-degree-relative coordinate spaces** where patterns are stored as diatonic scale-step offsets (`ac_off`) and register shifts (`ac_oct`) relative to an active harmonic root and chord degree. When a chord changes—either via an internal harmonic state machine or external MIDI input—the underlying rhythmic and melodic motif automatically re-tiles across the new harmonic foundation without breaking voice-leading or metrical phrasing.

For embedded microcontroller targets such as the Raspberry Pi RP2350 (Dual ARM Cortex-M33 / Hazard3 RISC-V) and Espressif ESP32-C3 (single-core RISC-V), executing these generative architectures within deterministic, sub-microsecond clock interrupt budgets requires replacing floating-point probability distributions and dynamic heap allocations (`malloc`) with static state structures, 32-bit `Xorshift32` pseudo-random number generators (PRNGs), 12-bit pitch-class bitmasks, and pre-computed 8-bit cumulative distribution function (CDF) lookup tables. By encoding scales as 12-bit unsigned integers where bit `s` (`0..11`) represents `s` semitones above the root, scale validation and expansion execute in a few clock cycles via bitwise shifts.

This report details the exact algorithmic mechanics of four foundational hardware sequencing paradigms (Mutable Instruments Marbles, Intellijel Metropolix / Roland M-185, Torso T-1 / OXI One, and the Roland TB-303 / Berlin School architecture), provides bit-verified C99 lookup tables for scales, Krumhansl-Schmuckler tonal hierarchies, Neo-Riemannian/diatonic chord state machines, and 16-step contours, and delivers four complete, zero-malloc C99 generative melody engines unified under an 8-pot hardware control surface.

## 1\. Survey of Hardware & Plugin Generative Melody Architectures

Commercial generative sequencers achieve musical coherence by constraining where randomness is allowed to occur in time (metrical hierarchy) and pitch (tonal hierarchy). Across established Eurorack and desktop hardware, four primary architectural models dominate monophonic melody and bassline generation.

### Mutable Instruments Marbles (Deja Vu Shift Registers, Distribution Shaping, and STEPS Quantization)

Designed by Émilie Gillet (`pichenettes`), Mutable Instruments Marbles (`marbles/random/`) separates rhythmic trigger generation (`t_generator`) from clocked random voltage generation (`x_y_generator`, `output_channel`, `distributions`, `quantizer`), binding both sections through a shared loop-memory architecture called `DEJA VU`.

#### The DEJA VU Shift Register & Three-Zone Loop State Machine

Instead of storing quantized MIDI notes in its history buffer, Marbles stores normalized random realizations `u` in `[0.0, 1.0)` inside a ring buffer (`RandomSequence`) of variable length `N` (`1..16` steps). Storing raw random values upstream of the voltage waveshaper and scale quantizer is a critical architectural choice: it allows the musician to lock a repeating loop and subsequently alter the `SPREAD`, `BIAS`, or `STEPS` controls to dynamically re-voice, transpose, or re-quantize the locked phrase without destroying its underlying contour.

Rather than acting as a single linear probability fade, the `DEJA VU` parameter `D` in `[0.0, 1.0]` operates across three distinct mathematical zones. In **Zone 1 (7 o'clock to 12 o'clock, `D` in `[0.0, 0.5)`)**, the probability of recycling the historical value from `N` steps ago increases linearly from `p_loop = 0.0` (completely fresh random generation on every clock tick) to `p_loop = 1.0` at 12 o'clock. Setting the knob around 11 o'clock (`p_loop ≈ 0.85–0.95`) produces slow, organic melodic mutation where only 5% to 15% of steps in the `N` -step ring buffer are overwritten per pass. In **Zone 2 (12 o'clock virtual notch, `D ≈ 0.5`)**, a firmware deadband forces `p_loop = 1.0` and `p_jump = 0.0`, guaranteeing a deterministic repeating loop of length `N` despite potentiometer jitter. Furthermore, firmware revision 1.2 introduced a **"Super Lock"** state (triggered by a long press on the `t` or `X` Deja Vu button), which freezes the shift register contents completely and ignores changes to the `DEJA VU` knob on that section—enabling the user to permute or mutate pitch while keeping rhythm strictly locked, or vice versa. In **Zone 3 (12 o'clock to 5 o'clock, `D` in `(0.5, 1.0]`)**, no new random values are injected into the ring buffer, preserving the exact pitch pool of the `N` recorded steps. Instead, the probability of randomly jumping the read pointer to an index `k` in `[0, N - 1]` within the buffer increases linearly from `p_jump = 0.0` at 12 o'clock to `p_jump = 1.0` at 5 o'clock, generating melodic permutations of a fixed set of notes.

#### t Generator Rhythmic Division Models

The `t` section generates a master clock on `t2` (with adjustable timing jitter that models an instrumentalist lagging and catching up) and splits it into two complementary gate streams `t1` and `t3` using one of three selectable algorithms controlled by `BIAS`. In the **Coin Toss (Bernoulli Split)** model, at every pulse of `t2`, a biased coin toss routes the trigger to either `t1` (probability `b`) or `t3` (probability `1 - b`). In the **Random Ratio Divider/Multiplier** model, `t1` and `t3` are generated by multiplying and dividing the `t2` clock period by complementary random integer ratios, with `BIAS` scaling the extremity of the ratios. In the **Alternating Drum Pattern** model, triggers alternate deterministically and probabilistically between `t1` and `t3` following kick/snare backbeat skeletons. When the `X` voltage section is clocked internally by `t1`, `t2`, and `t3` rather than a common external clock, each output samples the `DEJA VU` ring buffer at its own rhythmic pace; consequently, a 3-note loop (`LENGTH = 3`) cycled across sparse triggers yields a 9-beat polyrhythmic phasing cycle rather than a static 3-beat loop.

#### Voltage Distribution Shaping (SPREAD & BIAS) and STEPS Salience Quantization

Before quantization, each uniform random number `u` in `[0.0, 1.0)` is shaped by a probability density function (PDF) governed by `SPREAD` and `BIAS`. At minimum `SPREAD` (fully CCW), variance collapses to zero, outputting a constant center voltage. Between CCW and 12 o'clock, voltages follow a bell-shaped distribution concentrated near the center; at 2 o'clock, the distribution flattens into a uniform PDF across the active voltage range; past 2 o'clock toward fully CW, the PDF inverts into a bimodal U-shaped distribution pushing voltages toward the upper and lower extremes, until fully CW it becomes a binary gate toggling strictly between the minimum and maximum rails. Rather than adding a post-hoc DC offset, `BIAS` applies a non-linear polynomial or beta-distribution skew prior to quantization, weighting the probability histogram toward lower or higher registers while keeping the overall scale boundaries fixed.

The `STEPS` knob controls horizontal slew when turned CCW from 12 o'clock and vertical scale quantization when turned CW. At 12 o'clock, `STEPS` outputs raw sample-and-hold (S&H) step voltages; turning CCW engages a `lag_processor` that slews step edges into linear segments and smooth random curves. Turning CW from 12 o'clock engages the scale quantizer and progressively strips notes out of the scale according to their **tonal salience (frequency weight)**. When a custom scale is programmed into Marbles by playing a short musical fragment—for example, the 18-note sequence `C D E F G A B C D F G A C F G C G C` —the firmware counts note occurrences to rank each pitch class's salience. Sweeping `STEPS` clockwise from 12 o'clock to 5 o'clock progressively masks out the least frequent pitch classes in five discrete salience tiers: first yielding the full 7-note diatonic scale (`C D E F G A B`), then stripping the least frequent notes (`E` and `B`) to leave a 5-note pentatonic subset (`C D F G A`), then eliminating `D` and `A` to isolate the primary harmonic pillars (`C F G`), then stripping `F` to leave the root-fifth dyad (`C G`), and finally locking exclusively to the tonic (`C`) and its octaves.

### Intellijel Metropolix / Roland M-185 (Multi-Stage Pulse Counts, Ratchets, and Accumulators)

Derived from RYK's Roland System-100m M-185 sequencer and expanded in the Intellijel Metropolis and Metropolix, this architecture replaces the fixed 1-clock-per-step linear grid with an **8-stage state machine** where each stage `s` (`0..7`) owns an independent duration measured in master clock pulses `P_s` (`1..8`).

#### Hierarchical Stage, Pulse, and Ratchet Timing

Time progresses through a three-level nested counter (`Stage -> Pulse -> Ratchet`). The sequencer remains on stage `s` for `P_s` master clock ticks before advancing to stage `s + 1`. During those `P_s` pulses, gate generation is governed by one of four stage play modes: **Single** (gate fires on pulse `0` at 50% duty cycle, rests on pulses `1..P_s - 1`), **Multi** (gate fires on every pulse `0..P_s - 1`), **Hold** (gate remains continuously high across all `P_s` pulses, creating a tied note), and **Rest** (all pulses remain low). Ratcheting subdivides an individual pulse gate into `R_s` rapid sub-bursts (`1..4` subdivisions per pulse) without altering the stage's total duration `P_s`. Each stage or pulse is further gated by a probability threshold `T_s` (`0–100%`); when a `PROB` check fails and suppresses a stage or pulse, the Metropolix engine treats that event identically to a `GATE MODE = REST`, meaning downstream pitch accumulators are also suppressed and **do not** increment on skipped notes.

#### Pitch Accumulator Mechanics (Mode, Trigger, Order, Polarity, and Limits)

Because an 8-stage pattern can quickly sound static, Metropolix incorporates a deterministic per-stage **Accumulator** that adds a signed scale-degree offset $\Delta_s$ to stage $s$ 's base pitch across successive passes. In **Stage Accumulator** mode (`Mode = Stage`), stage $s$ 's accumulation counter $A_s$ only transposes stage $s$ itself; in **Track Accumulator** mode (`Mode = Track`), an accumulation increment triggered on stage $s$ adds to a shared track accumulator that shifts every stage in the sequence simultaneously. The **Transposition Trigger** (`Stage`, `Pulse`, or `Ratchet`) defines when $A_s$ increments by $\Delta_s$, interacting strictly with the stage's Gate Play Mode:

| Gate Play Mode | `STAGE` Transpose Trigger | `PULSE` Transpose Trigger | `RATCHET` Transpose Trigger |
| --- | --- | --- | --- |
| **HOLD** | Transposes once per stage play | Transposes once per stage play | Transposes once per stage play |
| **MULTI** | Transposes once per stage play | Transposes on **every pulse** of the stage | Transposes on **each ratchet in every pulse** |
| **SINGLE** | Transposes once per stage play | Transposes on **first pulse** only | Transposes on **each ratchet in the first pulse** |
| **REST** (or failed `PROB`) | No transpose | No transpose | No transpose |

The vertical span of the accumulator is bounded by a **Positive Limit** `L_+` (`+1 to +28` scale degrees, up to 4 octaves in a 7-note scale, default `+7`) and a **Negative Limit** `L_-` (`-1 to -28` scale degrees, default `-7`), paired with a **Polarity** setting (`Unipolar` vs. `Bipolar`). When `Polarity = Unipolar`, the accumulator operates between `0` and `+L_+` (or `0` and `-L_-`); when `Polarity = Bipolar`, it sweeps across the full bipolar range `[-L_-, +L_+]` around the stage's starting pitch.

When an accumulation step pushes $A_s + \Delta_s$ past the active boundary $[L_{\min}, L_{\max}]$, the trajectory is governed by one of four **Accumulator Order** modes (`Wrap`, `Pendulum`, `Random`, or `Hold`). In **`Wrap`** mode (the default), the offset wraps modularly around the limits according to:

$$
A_s \leftarrow L_{\min} + \left((A_s + \Delta_s - L_{\min}) \bmod (L_{\max} - L_{\min} + 1)\right)
$$

which generates rising or falling sawtooth staircases across bars. In **`Pendulum`** mode, exceeding $L_{\max}$ or $L_{\min}$ reflects the value back inside the bounds and inverts the accumulation direction $d_s \in \{-1, +1\}$, creating smooth triangular arches across multiple sequence passes. For example, given an increment of `+2` scale degrees inside a bipolar range of `[-3, +5]`, 10 successive triggers under `Wrap` produce the scale-degree sequence `[0, 2, 4, -3, -1, 1, 3, 5, -2, 0]`, whereas `Pendulum` produces the reflected sequence `[0, 2, 4, 4, 2, 0, -2, -2, 0, 2]`. In **`Random`** mode, each trigger selects a random scale-degree offset within $[L_{\min}, L_{\max}]$, while **`Hold`** saturates (clamps) at $L_{\max}$ or $L_{\min}$ and stays pinned at that ceiling or floor until a manual or bar-quantized sequence `RESET` returns all accumulators to `0`.

### Torso T-1 & OXI One (Harmonizer Locking, Phrase Contours, and Stochastic Matrices)

Desktop algorithmic sequencers like the Torso Electronics T-1 and OXI Instruments OXI One decouple rhythmic pulse generation from pitch trajectories, treating a melodic phrase as the intersection of a rhythm generator, a harmonic filter, and a slow contour modulator.

#### Torso T-1: Euclidean Generators, Voicing/Style, Range/Phrase, and Harmony

On the Torso T-1, each track generates rhythmic triggers using Bjorklund's Euclidean algorithm `E(k, n, r)` (`PULSES`, `STEPS`, `ROTATE`), supplemented by `REPEATS` (ratchet/delay echoes) and `PACE` (accelerate/decelerate dynamics). Pitch generation is governed by four paired primary/secondary (`CTRL` + knob) controls. **`Pitch` / `Harmony`** sets the active note pool (root note or multi-note chord stack) and base transposition, while its secondary function `Harmony` (`CTRL + Pitch`) shifts the active chord pool along a harmonic walk, picking the nearest voice-led chord inversion up or down within the track's `Scale` so that patterns re-voice smoothly across progressions. **`Voicing` / `Style`** acts as an algorithmic arpeggiator that adjusts the pitch ordering of notes in the active pool, while `Style` (`CTRL + Voicing`) selects the directional traversal rule. **`Range` / `Phrase`** sets the vertical pitch span of the notes played and modulates that register using an internal low-frequency modulator (LFO), where `Phrase` (`CTRL + Range`) selects the modulation contour shape (such as upward/downward ramps, arches, sine waves, or stepped random walks) across a multi-bar division set by `Rate` (`CTRL + Random`). Holding **`Random`** and turning any parameter knob applies bipolar randomization to that specific parameter, whereas `Probability` (`CTRL + Velocity`) thins out rhythmic density.

#### OXI One: Stochastic Mode, Matriceal Decoupling, and the Harmonizer

The OXI One (and MkII) provides six sequencer modes: `Mono`, `Chord`, `Polyphonic`, `Multitrack`, `Stochastic`, and `Matriceal`. In **`Stochastic`** mode, the first 12 columns of the grid control the independent probability weight `W_i` (`0–100%`) of each pitch class (or active scale degree) being played, which also influences trigger density on that pitch. On every active gate trigger, the engine evaluates a cumulative distribution function (CDF) across all enabled pitch columns to select the output note. The last four columns on the right side of the grid adjust **Octave Spread** (likelihood of transposing by ±1 or ±2 octaves), **Gate Length** and **Clock Division** modifiers, and independent **Randomization / Lock** probabilities for **Pitch** and **Rhythm**. Setting Pitch or Rhythm randomization to `0%` locks the invisible evolving buffer into a deterministic repeating loop, which can also be copied and pasted into a `Mono` sequencer track as static step data.

In **`Matriceal`** mode, OXI One decouples step parameters into eight independent polymetric parameter pages— `Gate`, `Note`, `Interval`, `Velocity`, `Octave`, `Retrigger`, `Trigger Probability`, and `Glide` —allowing a short `Note` loop and a different-length `Interval` (diatonic transpose) loop to phase against a 16-step `Gate` pattern. Across `Mono`, `Polyphonic`, `Multitrack`, and `Stochastic` tracks, enabling the **`Harmonizer`** scale setting dynamically locks slave tracks to the master `Chord` track (or external MIDI `Follower Keyboard`), remapping generated scale degrees onto the active chord's tones in real time. While the OXI One manuals document `Stochastic`, `Matriceal`, and `Harmonizer` modes rather than formal Lindenmayer (L-system) grammars, explicit **Markov chain transition matrices**, **Morph phrase mutators**, **AABA Form arrangers**, and **Answer call-and-response generators** are implemented in open-source embedded groovebox firmwares such as **Tessera** (`Rastapopolix`) and **HYP-1** (`amb.c`) running on 32-bit microcontrollers.

### Classic 303 / Acid & Berlin School Sequencers (Degree Tiling, CPU Timing, and Turnarounds)

The propulsive groove of the Roland TB-303 and Berlin School sequencers (Tangerine Dream, Klaus Schulze) arises from the tight coupling between **chord-relative pitch offsets**, **rest placement**, and **analog envelope/slew hardware behavior**.

#### Chord-Relative Degree Offsets vs. Passing Tones (HYP-1 / TB-3PO Model)

In embedded acid generators such as the open-source HYP-1 engine (`amb.c`), a 16-step pattern is stored as five compact values per step: `gate` (`0/1`), `accent` (`0/1`), `slide` (`0/1`), `off` (scale steps above the active chord degree `deg`), and `oct` (octave offset `0..RANGE-1`, where `RANGE` is `1..3` octaves). When generating a step via `ac_roll()`, step 0 (the downbeat of the bar) receives a `+0.30` gate probability boost (`min(dens + 0.30, 1.0)`) so phrases reliably anchor on beat 1, while strategic rests on steps 2–3, 5, 9, 11, or 15 create syncopated breathing room against a four-on-the-floor kick drum. On 80% of steps, `off` is chosen as an **even scale-step offset** above the chord degree (`off = 2 * irand(chord_nt)`, yielding `0, 2, 4, 6` —the chord's root, 3rd, 5th, and 7th). On the remaining 20% of steps, `off` is drawn from any scale degree (`irand(nsteps)`), inserting passing tones that connect chord tones on weak 16th-note subdivisions. The base register (`oct = 0`) is selected 55% of the time, with upper octave jumps occurring 45% of the time, alongside a 30% `accent` probability and a `Slide * 0.007` (`0.0–0.70`) `slide` probability. In practice, placing octave jumps and accents on off-beat 16th steps (such as step 4 just before the beat-2 kick, or step 14 just after the beat-4 kick) creates syncopated forward momentum without clashing with the kick drum.

#### Hardware TB-303 CPU Timing, Slide Overlap, and Turnaround Fills

Logic-analyzer measurements of the original Roland TB-303's NEC μPD650C-133 CPU (clocked at `440 kHz` with an independent `1.8 ms` hardware interrupt timer beating against a `24 PPQN` DIN sync clock) explain why naive MIDI sequencers fail to capture authentic 303 articulation. In 4/4 mode, each 16th-note step spans 6 DIN sync pulses (3 pulses gate-on, 3 pulses gate-off); due to fixed interrupt latency (`0.8–3.3 ms` gate-on delay, mean `2.06 ms`; `2.25–5.45 ms` gate-off delay, mean `3.51 ms`), the actual hardware gate duty cycle measures between `49.96%` and `55.8%` (modeled as `0.55 * step_len` in HYP-1). On the TB-303 hardware, the `Slide` signal controls both the 6-bit R2R pitch DAC latch and the accent flip-flop (`IC13`). On a non-slid note, the slide signal activates briefly for `44 µs` at the start of the step to latch the new pitch and accent state. When a step's `slide` flag is set, the gate signal **remains continuously high** (`100%` duty cycle) across the step boundary into the next step, skipping envelope retriggering while engaging an analog RC portamento slew (`~50–60 ms`). When sliding across multiple different notes in sequence, the CPU pulls the slide line low for an `8.75 µs` pulse at the start of each new step strictly to latch the new DAC pitch value without interrupting the continuous slide. Finally, both Berlin School sequencers and HYP-1 (`dd_bar`) preserve multi-bar phrasing by triggering a **Turnaround Fill** on the back half of the final bar of a 4-, 8-, or 16-bar cycle (steps 8–15 or 12–15), temporarily overriding the loop with an ascending or cascading run of chord tones (`off = {0, 2, 4, 6, 8, 10, 12, 14}`) and forced accents on steps 12–15.

## 2\. Canonical Scale, Chord & Melodic Gravity Lookup Tables

To execute scale quantization, weighted pitch selection, and harmonic progression state machines in `O(1)` or `O(M)` time (`M ≤ 12`) without floating-point unit (FPU) overhead—critical on the integer-only RISC-V ESP32-C3 and freeing the Cortex-M33 FPU on the RP2350 for audio DSP—all musical structures are encoded as compact `uint16_t`, `uint8_t`, and `int8_t` lookup tables in flash (`const`) memory.

### 1\. Bit-Verified 12-Bit Pitch-Class Scale Masks

In embedded C99, a scale is represented as a 12-bit unsigned integer (`uint16_t`) where bit `s` (`0..11`, `(1u << s)`) is set to `1` if and only if the pitch class `s` semitones above the root belongs to the scale. Bit 0 (`0x0001`, the root) is always forced on.

| Index | Scale / Mode Name | Hex Mask (`uint16_t`) | 12-Bit Binary (`B11..B0`) | Semitone Offsets Above Root | Note Count (`M`) |
| --- | --- | --- | --- | --- | --- |
| **0** | Chromatic | `0x0FFF` | `111111111111` | `0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11` | 12 |
| **1** | Ionian (Major) | `0x0AB5` | `101010110101` | `0, 2, 4, 5, 7, 9, 11` | 7 |
| **2** | Dorian | `0x06AD` | `011010101101` | `0, 2, 3, 5, 7, 9, 10` | 7 |
| **3** | Phrygian | `0x05AB` | `010110101011` | `0, 1, 3, 5, 7, 8, 10` | 7 |
| **4** | Lydian | `0x0AD5` | `101011010101` | `0, 2, 4, 6, 7, 9, 11` | 7 |
| **5** | Mixolydian | `0x06B5` | `011010110101` | `0, 2, 4, 5, 7, 9, 10` | 7 |
| **6** | Aeolian (Natural Minor) | `0x05AD` | `010110101101` | `0, 2, 3, 5, 7, 8, 10` | 7 |
| **7** | Locrian | `0x056B` | `010101101011` | `0, 1, 3, 5, 6, 8, 10` | 7 |
| **8** | Major Pentatonic | `0x0295` | `001010010101` | `0, 2, 4, 7, 9` | 5 |
| **9** | Minor Pentatonic | `0x04A9` | `010010101001` | `0, 3, 5, 7, 10` | 5 |
| **10** | Blues Scale | `0x04E9` | `010011101001` | `0, 3, 5, 6, 7, 10` | 6 |
| **11** | Harmonic Minor | `0x09AD` | `100110101101` | `0, 2, 3, 5, 7, 8, 11` | 7 |
| **12** | Melodic Minor (Asc.) | `0x0AAD` | `101010101101` | `0, 2, 3, 5, 7, 9, 11` | 7 |
| **13** | Phrygian Dominant | `0x05B3` | `010110110011` | `0, 1, 4, 5, 7, 8, 10` | 7 |
| **14** | Whole Tone | `0x0555` | `010101010101` | `0, 2, 4, 6, 8, 10` | 6 |
| **15** | Octatonic (Half-Whole) | `0x06DB` | `011011011011` | `0, 1, 3, 4, 6, 7, 9, 10` | 8 |
| **16** | Hirajoshi | `0x018D` | `000110001101` | `0, 2, 3, 7, 8` | 5 |
| **17** | In-Sen | `0x04A3` | `010010100011` | `0, 1, 5, 7, 10` | 5 |
| **18** | Double Harmonic (Byzantine) | `0x09B3` | `100110110011` | `0, 1, 4, 5, 7, 8, 11` | 7 |
| **19** | Acid Dom7 / Minor3 Pool | `0x0499` | `010010011001` | `0, 3, 4, 7, 10` | 5 |

(Note: The bitmasks in the table above are verified directly from $\sum_{s} 2^s$. Exercise caution when porting legacy tables from online repositories where bit orderings or modal accidentals—such as Ionian vs. Lydian or Phrygian vs. Aeolian—are occasionally transposed.)

### 2\. Harmonic Gravity Weights & Voice-Leading State Machines

To prevent generated melodies from wandering aimlessly, pitch selection combines two complementary gravity models: **empirical pitch-class stability profiles** (Krumhansl-Schmuckler / Krumhansl-Kessler probe-tone hierarchies) and **scale-degree transition matrices** (Li & Huron / Madsen & Widmer interval transition profiles).

<iframe allow="xr-spatial-tracking; web-share" title="Preview" sandbox="allow-pointer-lock allow-popups allow-forms allow-popups-to-escape-sandbox allow-downloads allow-same-origin allow-scripts" src="https://1t5nkiqtp78xf23brzsu6vo91u47gz7sb7xi5n0f69twlur1dw-h1018257101.scf.usercontent.goog/gemini-code-immersive/shim.html?origin=https%3A%2F%2Fgemini.google.com&amp;cache=1"></iframe>

#### Krumhansl-Schmuckler Empirical Gravity Profiles & Metrical Beat Hierarchy

In Carol Krumhansl and Edward Kessler's probe-tone experiments, listeners rated how well each of the 12 chromatic pitch classes fit a major or minor tonal context. In C Major, the raw empirical ratings are `C: 6.35, C#: 2.23, D: 3.48, D#: 2.33, E: 4.38, F: 4.09, F#: 2.52, G: 5.19, G#: 2.39, A: 3.66, A#: 2.29, B: 2.88`. In C Minor, the raw ratings are `C: 6.33, C#: 2.68, D: 3.52, Eb: 5.38, E: 2.60, F: 3.53, F#: 2.54, G: 4.75, Ab: 3.98, A: 2.69, Bb: 3.34, B: 3.17`. Normalizing these ratings to 8-bit unsigned integers via:

$$
W_{\text{u8}}[p] = \text{round}\left(255 \times \frac{R[p]}{\max(R)}\right)
$$

yields exact `uint8_t` lookup tables where the tonic equals `255`, the dominant equals `208` (Major) or `191` (Minor), and the characteristic mediant equals `176` (Major 3rd) or `217` (Minor b3rd).

Because the empirical Krumhansl-Schmuckler profiles assign baseline non-zero ratings (`90–108` in normalized 8-bit units) to chromatic non-scale tones—reflecting perceptual tolerance for chromaticism in listening tests—sampling raw K-S weights directly without scale masking or metrical conditioning produces too many unstable notes on downbeats. Production sequencers therefore stratify scale-degree gravity across three **metrical beat tiers** (`DEGREE_METRICAL_GRAVITY[3][7]`): on **Strong Beats (Quarter-note downbeats: Steps 0, 4, 8, 12)**, weights lock tightly to the active triad (`Root = 255`, `5th = 210`, `3rd = 180`, `7th = 40`, other diatonic tones `15–20`); on **Medium Beats (Offbeat 8ths: Steps 2, 6, 10, 14)**, weights balance triad stability (`170–190`) with 7th and 6th color extensions (`85–120`); and on **Weak Subdivisions (Odd 16th steps: 1, 3, 5, 7, 9, 11, 13, 15)**, weights invert to favor passing and tendency degrees (`2nd, 4th, 6th, 7th = 150–175`) that resolve stepwise into the next strong beat.

#### Voice-Leading & Neo-Riemannian (PLR / NSH) + Diatonic (W7) Chord State Machines

To drive harmonic movement underneath a monophonic sequence, two state-machine models are provided in the lookup header below. First, in the **Diatonic Scale-Degree Drift (`W7` & Functional Markov)** model from HYP-1 (`amb.c`), diatonic chords are built on scale degree `deg` (`0..M - 1`) using stacked diatonic intervals (`TRIAD` `{0, 2, 4}`, `7TH` `{0, 2, 4, 6}`, `9TH` `{0, 2, 6, 1}`, `SUS` `{0, 3, 4, 6}`) or fixed parallel shapes (`DUB` minor-9th `{0, 3, 10, 14}` and `4THS` `{0, 5, 10, 15}`). When a chord change triggers on a bar line, the engine selects a new degree `i != deg` using the empirical 7-note diatonic weight vector `W7[7] = {40, 20, 10, 30, 30, 30, 4}` (favoring degree `I` with weight 40, `IV/V/VI` with weight 30 each, `II` with 20, `III` with 10, and `VII` with 4) or a directed `7×7` functional Markov matrix (`CHORD_MARKOV_7X7`).

Second, in the **Neo-Riemannian Tonnetz (`PLR` and `NSH`) Triad State Machine** —implemented in Eurorack modules such as Ornament & Crime's *Harrington 1200* and *Automatonnetz* —transformations operate directly on any major (`qual = 0`, intervals `{0, 4, 7}`) or minor (`qual = 1`, intervals `{0, 3, 7}`) triad by flipping its modality (`qual' = 1 - qual`) and shifting its root pitch class `root_pc` (`0..11`) by a deterministic semitone delta:

| Transform | Name & Musical Definition | Composition | Major $\to$ Minor ($\Delta r$, new qual) | Minor $\to$ Major ($\Delta r$, new qual) | Example from C Maj `{C, E, G}` |
| --- | --- | --- | --- | --- | --- |
| **`P`** | **Parallel**: Shifts third by ±1 semitone; preserves root & fifth. | Atomic `P` | `+0` semitones, Min | `+0` semitones, Maj | `C Maj` $\leftrightarrow$ `C Min {C, Eb, G}` |
| **`L`** | **Leittonwechsel** (Leading-Tone Exchange): Preserves minor third; moves remaining note by 1 semitone. | Atomic `L` | `+4` semitones, Min | `+8` semitones, Maj | `C Maj` $\leftrightarrow$ `E Min {E, G, B}` |
| **`R`** | **Relative**: Preserves major third; moves remaining note by 2 semitones. | Atomic `R` | `+9` semitones, Min | `+3` semitones, Maj | `C Maj` $\leftrightarrow$ `A Min {A, C, E}` |
| **`N`** | **Nebenverwandt**: Exchanges a major triad with its minor subdominant (or minor triad with major dominant). | `RLP` | `+5` semitones, Min | `+7` semitones, Maj | `C Maj` $\leftrightarrow$ `F Min {F, Ab, C}` |
| **`S`** | **Slide**: Exchanges two triads of opposite quality that share the exact same third. | `LPR` | `+1` semitone, Min | `+11` semitones, Maj | `C Maj` $\leftrightarrow$ `C# Min {C#, E, G#}` |
| **`H`** | **Hexatonic Pole**: Exchanges a triad with its hexatonic pole via 3 simultaneous semitone moves. | `LPL` | `+8` semitones, Min | `+4` semitones, Maj | `C Maj` $\leftrightarrow$ `Ab Min {Ab, B, Eb}` |

### 3\. Complete C99 Lookup Header (gen\_tables.h)

The following C99 header encapsulates all 20 scale bitmasks, Krumhansl-Schmuckler and chord-tone gravity tables, a `7×7` scale-degree tendency/voice-leading transition matrix, diatonic and Neo-Riemannian chord progression state machines, and 16 canonical 1-bar melodic contour archetypes.

```markdown
#ifndef GEN_TABLES_H
#define GEN_TABLES_H

#include <stdint.h>
#include <stdbool.h>

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

    int16_t midi_note = 36 + (int16_t)(root_pc % 12u)
                      + (int16_t)scale_steps[scale_idx]
                      + (12 * oct_shift);

    while (midi_note > 96) midi_note -= 12;
    while (midi_note < 28) midi_note += 12;
    return (uint8_t)midi_note;
}

/* Fast deterministic Xorshift32 PRNG (< 100 ns on Cortex-M33 / RISC-V) */
static inline uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    if (x == 0u) x = 0x9E3779B9u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

#endif /* GEN_TABLES_H */
```

## 3\. Four Distinct Generative Melody Algorithms with Embedded C99 Implementations

To support a switchable multi-algorithm hardware module on RP2350 or ESP32-C3, each of the four requested engines below is implemented in strict, zero-malloc, integer-only C99 using `gen_tables.h`. All four engines output a standardized per-16th-step articulation struct (`seq_step_out_t`) containing the target MIDI note (`28..96` or `-1` for rest), velocity (`0..127`), `gate`, `accent`, `slide` (legato continuation), and `ratchet_count` (`1..4`).

```markdown
/* Shared Step Output Structure for All 4 Engines */
typedef struct {
    int16_t midi_note;      /* Target MIDI pitch [28..96], or -1 on rest */
    uint8_t velocity;       /* 120 if accented, 90 if normal, 0 on rest */
    bool    gate;           /* True if step is active */
    bool    accent;         /* True if TB-303 / VCF accent is active */
    bool    slide;          /* True if this step holds gate & slews into next step */
    uint8_t ratchet_count;  /* Sub-triggers within this 16th step (1 = normal, 2..4 = ratchet) */
} seq_step_out_t;
```

### Algorithm 1: Phrase-Grammar & Motif Transformer (ABAC / AABC)

Rather than rolling 16 independent random steps—which lacks phrase-level memory—Algorithm 1 generates a 4-step (1-beat) **seed motif `A`** anchored on chord tones and derives the remaining three beats (`B` and `C`) via deterministic **musicological transformations**: **Diatonic Transposition** (shifting by `+2` or `-1` scale degrees), **Diatonic Inversion** ($d' = 2 \cdot d_{\text{pivot}} - d$ around a scale-degree axis), **Retrograde** (reversing the 4-step order `i <-> 3 - i`), and **Rhythmic Displacement** (rotating the 4-step beat grid by 1 sixteenth note with an octave accent pop). Furthermore, the final step of Motif `C` (step 15 of the bar) is grammatically constrained to act as a **cadential pickup** (selecting scale degree `-1` \[leading tone\] or `+4` \[dominant fifth\] with `slide` and `accent`) that resolves back into step 0 of Motif `A`.

```markdown
#include "gen_tables.h"

typedef enum {
    FORM_ABAC = 0,  /* Beat 1: A, Beat 2: B, Beat 3: A, Beat 4: C (Call-Response) */
    FORM_AABC,      /* Beat 1: A, Beat 2: A, Beat 3: B, Beat 4: C (Sentence Form) */
    FORM_ABBC,      /* Beat 1: A, Beat 2: B, Beat 3: B, Beat 4: C (Bridge Expansion) */
    FORM_AABA       /* Beat 1: A, Beat 2: A, Beat 3: B, Beat 4: A (Song Form) */
} phrase_form_t;

typedef enum {
    XFORM_DIATONIC_TRANSPOSE = 0, /* Shift motif degrees by +1 or +2 scale steps */
    XFORM_DIATONIC_INVERT,        /* Invert scale degrees around pivot: d' = 2*pivot - d */
    XFORM_RETROGRADE,             /* Reverse 4-step order: step[i] = seed[3 - i] */
    XFORM_RHYTHMIC_DISPLACE       /* Rotate beat right by 1 step + octave pop */
} motif_transform_t;

typedef struct {
    bool    gate;
    int8_t  deg_off;    /* Scale steps relative to active chord degree */
    int8_t  oct_off;    /* Octave offset (-1, 0, +1) */
    bool    accent;
    bool    slide;
} motif_cell_t;

typedef struct {
    uint32_t          prng;
    motif_cell_t      seed_A[4];      /* 1-beat (4-step) seed motif */
    motif_cell_t      bar_steps[16];  /* Expanded 16-step phrase grammar */
    phrase_form_t     form;           /* ABAC, AABC, ABBC, AABA */
    motif_transform_t xform_B;        /* Transformation applied to derive B from A */
    motif_transform_t xform_C;        /* Transformation applied to derive C from A */
    uint8_t           density;        /* 0..255 gate probability */
    uint8_t           mutate_rate;    /* 0..255 probability of mutating seed_A on bar wrap */
} algo1_motif_engine_t;

static void algo1_apply_transform(const motif_cell_t src[4],
                                  motif_cell_t dst[4],
                                  motif_transform_t xform,
                                  int8_t transpose_amt,
                                  bool force_cadential_tail)
{
    for (uint8_t i = 0; i < 4u; i++) {
        dst[i] = src[i];
    }

    switch (xform) {
        case XFORM_DIATONIC_TRANSPOSE:
            for (uint8_t i = 0; i < 4u; i++) {
                dst[i].deg_off = (int8_t)(src[i].deg_off + transpose_amt);
            }
            break;

        case XFORM_DIATONIC_INVERT: {
            int8_t pivot = src[0].deg_off + 2; /* Invert around mediant/3rd axis */
            for (uint8_t i = 0; i < 4u; i++) {
                dst[i].deg_off = (int8_t)((2 * pivot) - src[i].deg_off);
            }
            break;
        }

        case XFORM_RETROGRADE:
            for (uint8_t i = 0; i < 4u; i++) {
                dst[i] = src[3u - i];
            }
            break;

        case XFORM_RHYTHMIC_DISPLACE:
            for (uint8_t i = 0; i < 4u; i++) {
                dst[i] = src[(i + 3u) & 3u]; /* Rotate right by 1 sixteenth step */
            }
            if (dst[2].gate) {
                dst[2].oct_off = 1;          /* Add syncopated upper-octave pop */
                dst[2].accent  = true;
            }
            break;
    }

    /* Enforce cadential resolution on step 3 of Motif C (Step 15 of the 16-step bar) */
    if (force_cadential_tail) {
        dst[3].gate    = true;
        dst[3].deg_off = (transpose_amt & 1) ? -1 : 4; /* Leading tone (-1) or 5th (+4) */
        dst[3].oct_off = 0;
        dst[3].accent  = true;
        dst[3].slide   = true; /* Legato slide into beat 1 root of next bar */
    }
}

static void algo1_compile_bar(algo1_motif_engine_t *eng) {
    motif_cell_t motif_B[4];
    motif_cell_t motif_C[4];

    algo1_apply_transform(eng->seed_A, motif_B, eng->xform_B, 2, false);
    algo1_apply_transform(eng->seed_A, motif_C, eng->xform_C, -1, true);

    const motif_cell_t *beats[4];
    switch (eng->form) {
        case FORM_ABAC:
            beats[0] = eng->seed_A; beats[1] = motif_B;
            beats[2] = eng->seed_A; beats[3] = motif_C;
            break;
        case FORM_AABC:
            beats[0] = eng->seed_A; beats[1] = eng->seed_A;
            beats[2] = motif_B;     beats[3] = motif_C;
            break;
        case FORM_ABBC:
            beats[0] = eng->seed_A; beats[1] = motif_B;
            beats[2] = motif_B;     beats[3] = motif_C;
            break;
        case FORM_AABA:
        default:
            beats[0] = eng->seed_A; beats[1] = eng->seed_A;
            beats[2] = motif_B;     beats[3] = eng->seed_A;
            break;
    }

    for (uint8_t b = 0; b < 4u; b++) {
        for (uint8_t s = 0; s < 4u; s++) {
            eng->bar_steps[(b * 4u) + s] = beats[b][s];
        }
    }
}

static void algo1_roll_seed_cell(algo1_motif_engine_t *eng, uint8_t idx) {
    uint32_t r = xorshift32(&eng->prng);
    uint8_t gate_thresh = (idx == 0u)
        ? (uint8_t)((eng->density > 180u) ? 255u : (eng->density + 75u))
        : eng->density;

    eng->seed_A[idx].gate = ((r & 0xFFu) < gate_thresh);

    /* Step 0 anchors on root (0) or 5th (4); other steps pick chord tones (80%) or passing (20%) */
    static const int8_t CHORD_DEGS[4] = { 0, 2, 4, 6 };
    if (idx == 0u) {
        eng->seed_A[idx].deg_off = ((r >> 8) & 1u) ? 0 : 4;
    } else if (((r >> 8) & 0xFFu) < 204u) {
        eng->seed_A[idx].deg_off = CHORD_DEGS[(r >> 16) & 3u];
    } else {
        eng->seed_A[idx].deg_off = (int8_t)((r >> 16) % 7u);
    }

    eng->seed_A[idx].oct_off = (((r >> 20) & 0xFFu) > 200u) ? 1 : 0;
    eng->seed_A[idx].accent  = (((r >> 24) & 0x0Fu) < 4u);
    eng->seed_A[idx].slide   = (((r >> 28) & 0x0Fu) < 3u);
}

void algo1_init(algo1_motif_engine_t *eng, uint32_t seed) {
    eng->prng        = seed ? seed : 0x61C88647u;
    eng->form        = FORM_ABAC;
    eng->xform_B     = XFORM_DIATONIC_TRANSPOSE;
    eng->xform_C     = XFORM_DIATONIC_INVERT;
    eng->density     = 170u;
    eng->mutate_rate = 32u;

    for (uint8_t i = 0; i < 4u; i++) {
        algo1_roll_seed_cell(eng, i);
    }
    eng->seed_A[0].gate    = true;
    eng->seed_A[0].deg_off = 0; /* Guarantee tonic anchor on beat 1.1 */
    algo1_compile_bar(eng);
}

void algo1_step(algo1_motif_engine_t *eng,
                uint8_t step_16,
                uint8_t root_pc,
                int8_t chord_deg,
                const uint8_t scale_steps[12],
                uint8_t num_steps,
                seq_step_out_t *out)
{
    uint8_t idx = step_16 & 15u;
    const motif_cell_t *cell = &eng->bar_steps[idx];

    if (cell->gate) {
        out->midi_note = (int16_t)degree_to_midi_folded(
            root_pc, chord_deg, cell->deg_off, cell->oct_off, scale_steps, num_steps
        );
        out->velocity      = cell->accent ? 120u : 90u;
        out->gate          = true;
        out->accent        = cell->accent;
        out->slide         = cell->slide;
        out->ratchet_count = 1u;
    } else {
        out->midi_note     = -1;
        out->velocity      = 0u;
        out->gate          = false;
        out->accent        = false;
        out->slide         = false;
        out->ratchet_count = 1u;
    }

    /* At end of bar, optionally mutate 1 cell of seed_A and re-derive B and C */
    if (idx == 15u && eng->mutate_rate > 0u) {
        uint32_t r = xorshift32(&eng->prng);
        if ((r & 0xFFu) < eng->mutate_rate) {
            uint8_t target_cell = (uint8_t)((r >> 8) & 3u);
            algo1_roll_seed_cell(eng, target_cell);
            algo1_compile_bar(eng);
        }
    }
}
```

### Algorithm 2: Tonal-Gravity Markov / Constrained Random Walk (Pendulum & Pedal-Point)

Algorithm 2 combines two classic electronic bass and lead techniques: a **Pedal-Point Root Anchor** on strong metrical steps (`0, 4, 8, 12`, or even steps when pedal intensity is high) and a **Constrained First-Order Markov Pitch Walker** on weak steps. On every non-pedal step, the walker computes the composite transition weight for each candidate scale degree `d` (`0..6`) by multiplying the **Voice-Leading Tendency Weight** `DEGREE_MARKOV_7X7[prev_deg][d]` (which pulls the 7th to the root, the 4th to the 3rd, and the 6th to the 5th) by the **Metrical Beat-Hierarchy Gravity** `DEGREE_METRICAL_GRAVITY[beat_tier][d]`, and adding a **Contour Target Pull** that boosts candidates closest to the active `MELODIC_CONTOURS_DEG[contour_id][step_16]` trajectory.

```markdown
#include "gen_tables.h"

typedef struct {
    uint32_t prng;
    uint8_t  prev_deg;          /* Last melodic scale degree (0..6) for Markov chain */
    int8_t   prev_oct;          /* Last melodic octave (-1..+1) */
    uint8_t  density;           /* 0..255 overall gate density */
    uint8_t  pedal_anchor_prob; /* 0..255 chance strong/even steps lock to pedal root */
    uint8_t  contour_id;        /* 0..15 index into MELODIC_CONTOURS_DEG */
    uint8_t  contour_pull;      /* 0..255 strength of attraction to contour target */
} algo2_markov_engine_t;

void algo2_init(algo2_markov_engine_t *eng, uint32_t seed) {
    eng->prng              = seed ? seed : 0xCAFEBABEu;
    eng->prev_deg          = 0u;
    eng->prev_oct          = 0;
    eng->density           = 190u;
    eng->pedal_anchor_prob = 180u;
    eng->contour_id        = 3u;  /* Default: Pedal-Point Ostinato contour */
    eng->contour_pull      = 160u;
}

void algo2_step(algo2_markov_engine_t *eng,
                uint8_t step_16,
                uint8_t root_pc,
                int8_t chord_deg,
                const uint8_t scale_steps[12],
                uint8_t num_steps,
                seq_step_out_t *out)
{
    uint8_t s = step_16 & 15u;
    uint32_t r = xorshift32(&eng->prng);

    /* Determine metrical tier: 0 = Quarter downbeat, 1 = 8th offbeat, 2 = 16th subdivision */
    uint8_t beat_tier = ((s & 3u) == 0u) ? 0u : (((s & 1u) == 0u) ? 1u : 2u);

    /* Gate probability biased by metrical tier */
    uint16_t eff_dens = eng->density;
    if (beat_tier == 0u) eff_dens += 55u;
    else if (beat_tier == 1u) eff_dens += 20u;
    if (eff_dens > 255u) eff_dens = 255u;

    if ((r & 0xFFu) >= eff_dens) {
        out->midi_note     = -1;
        out->velocity      = 0u;
        out->gate          = false;
        out->accent        = false;
        out->slide         = false;
        out->ratchet_count = 1u;
        return;
    }

    /* Evaluate Pedal-Point Root Anchor on strong/medium beats */
    bool is_pedal_step = false;
    uint8_t pedal_roll = (uint8_t)((r >> 8) & 0xFFu);
    if (beat_tier == 0u && pedal_roll < eng->pedal_anchor_prob) {
        is_pedal_step = true;
    } else if (beat_tier == 1u && (pedal_roll < (eng->pedal_anchor_prob >> 1))) {
        is_pedal_step = true;
    }

    int16_t chosen_deg_off = 0;
    int8_t  chosen_oct_off = 0;
    bool    accent         = false;
    bool    slide          = false;

    if (is_pedal_step) {
        chosen_deg_off = 0; /* Anchor directly on active chord root */
        chosen_oct_off = 0;
        accent         = (s == 0u);
        slide          = false;
    } else {
        /* Markov + Metrical Gravity + Contour Target CDF Evaluation */
        int8_t target_deg = MELODIC_CONTOURS_DEG[eng->contour_id & 15u][s];
        int8_t target_mod = (int8_t)floor_mod_i16(target_deg, 7);

        uint32_t cdf[7];
        uint32_t total_w = 0u;
        uint8_t  p_deg   = eng->prev_deg % 7u;

        for (uint8_t d = 0; d < 7u; d++) {
            uint32_t w_markov = DEGREE_MARKOV_7X7[p_deg][d];
            uint32_t w_metric = DEGREE_METRICAL_GRAVITY[beat_tier][d];
            uint32_t w        = (w_markov * w_metric) >> 7; /* Scale to ~0..512 */

            /* Apply melodic contour proximity bonus */
            int8_t dist = (int8_t)d - target_mod;
            if (dist < 0) dist = (int8_t)(-dist);
            if (dist > 3) dist = (int8_t)(7 - dist); /* Circular distance in 7-step scale */

            uint32_t bonus = (uint32_t)(4 - dist) * (uint32_t)eng->contour_pull;
            w += (bonus >> 2);
            if (w == 0u) w = 1u;

            total_w += w;
            cdf[d]   = total_w;
        }

        uint32_t pick = (xorshift32(&eng->prng) >> 8) % total_w;
        uint8_t next_d = 0u;
        for (uint8_t d = 0; d < 7u; d++) {
            if (pick < cdf[d]) {
                next_d = d;
                break;
            }
        }

        eng->prev_deg  = next_d;
        chosen_deg_off = (int16_t)next_d;
        chosen_oct_off = (target_deg >= 7) ? 1 : ((target_deg < 0) ? -1 : 0);
        accent         = (beat_tier == 2u) && (((r >> 16) & 0xFFu) < 70u);
        slide          = (beat_tier == 2u) && (((r >> 24) & 0xFFu) < 65u);
    }

    out->midi_note = (int16_t)degree_to_midi_folded(
        root_pc, chord_deg, chosen_deg_off, chosen_oct_off, scale_steps, num_steps
    );
    out->velocity      = accent ? 120u : 90u;
    out->gate          = true;
    out->accent        = accent;
    out->slide         = slide;
    out->ratchet_count = 1u;
}
```

### Algorithm 3: Accumulator & Stage-Ratchet Sequencer (Metropolix / M-185 Style)

Algorithm 3 implements the Intellijel Metropolix / Roland M-185 multi-stage architecture: 8 stages with independent pulse counts (`1..8`), play modes (`SINGLE`, `MULTI`, `HOLD`, `REST`), ratchets (`1..4`), stage probabilities (`0..255`), and per-stage pitch accumulators supporting all four hardware accumulation orders (`WRAP`, `PENDULUM`, `RANDOM`, `HOLD`), polarities (`UNIPOLAR`, `BIPOLAR`), and transposition triggers (`STAGE`, `PULSE`, `RATCHET`).

```markdown
#include "gen_tables.h"

typedef enum {
    METRO_GATE_SINGLE = 0, /* Gate on pulse 0 only, rest on pulses 1..P-1 */
    METRO_GATE_MULTI,      /* Gate on every pulse 0..P-1 */
    METRO_GATE_HOLD,       /* Continuous tied gate across all P pulses */
    METRO_GATE_REST        /* Silent stage */
} metro_gate_mode_t;

typedef enum {
    ACCUM_TRIG_STAGE = 0,  /* Increment once per stage visit */
    ACCUM_TRIG_PULSE,      /* Increment on each active pulse */
    ACCUM_TRIG_RATCHET     /* Increment on each active ratchet sub-trigger */
} accum_trig_mode_t;

typedef enum {
    ACCUM_ORDER_WRAP = 0,  /* Modulo wrap between limits */
    ACCUM_ORDER_PENDULUM,  /* Bounce back and forth between limits */
    ACCUM_ORDER_RANDOM,    /* Jump to random degree within limits */
    ACCUM_ORDER_HOLD       /* Clamp/saturate at limit until reset */
} accum_order_t;

typedef struct {
    int8_t            base_deg;      /* Stage base scale degree (e.g. 0, 2, 4, 6) */
    int8_t            oct_off;       /* Stage base octave offset (-1, 0, +1) */
    uint8_t           pulse_count;   /* Duration in 16th clock pulses (1..8) */
    metro_gate_mode_t gate_mode;     /* SINGLE, MULTI, HOLD, REST */
    uint8_t           ratchet_count; /* Sub-triggers per active pulse (1..4) */
    uint8_t           probability;   /* 0..255 probability of stage/pulse firing */
    int8_t            accum_step;    /* Scale degree increment per trigger (-7..+7) */
    int8_t            accum_val;     /* Current accumulated scale-degree offset */
    int8_t            accum_dir;     /* +1 or -1 for PENDULUM order */
    bool              accent;
    bool              slide;
} metro_stage_t;

typedef struct {
    uint32_t          prng;
    metro_stage_t     stages[8];
    uint8_t           num_stages;    /* Active stages (1..8) */
    uint8_t           cur_stage;
    uint8_t           cur_pulse;
    bool              stage_prob_latch;
    accum_trig_mode_t trig_mode;
    accum_order_t     order_mode;
    bool              bipolar;       /* True: [-neg_lim, +pos_lim], False: [0, +pos_lim] */
    int8_t            pos_limit;     /* +1..+28 scale degrees (default 7) */
    int8_t            neg_limit;     /* +1..+28 scale degrees (default 7) */
    uint16_t          reset_bars;    /* Auto-reset accumulators every N bars (0 = never) */
    uint32_t          step_counter;
} algo3_metro_engine_t;

static void algo3_advance_accumulator(algo3_metro_engine_t *eng, metro_stage_t *st) {
    if (st->accum_step == 0) return;

    int16_t low  = eng->bipolar ? (int16_t)(-eng->neg_limit) : 0;
    int16_t high = (int16_t)eng->pos_limit;
    if (low > high) { int16_t t = low; low = high; high = t; }
    int16_t span = high - low + 1;

    switch (eng->order_mode) {
        case ACCUM_ORDER_WRAP: {
            int16_t next = (int16_t)st->accum_val + (int16_t)st->accum_step;
            st->accum_val = (int8_t)(low + floor_mod_i16((int16_t)(next - low), span));
            break;
        }
        case ACCUM_ORDER_PENDULUM: {
            if (st->accum_dir == 0) st->accum_dir = 1;
            int16_t delta = (int16_t)st->accum_step * (int16_t)st->accum_dir;
            int16_t next  = (int16_t)st->accum_val + delta;
            if (next > high) {
                next = high - (next - high);
                st->accum_dir = (int8_t)(-st->accum_dir);
            }
            if (next < low) {
                next = low + (low - next);
                st->accum_dir = (int8_t)(-st->accum_dir);
            }
            if (next > high) next = high;
            if (next < low)  next = low;
            st->accum_val = (int8_t)next;
            break;
        }
        case ACCUM_ORDER_RANDOM: {
            uint32_t r = xorshift32(&eng->prng);
            st->accum_val = (int8_t)(low + (int16_t)((r >> 8) % (uint32_t)span));
            break;
        }
        case ACCUM_ORDER_HOLD:
        default: {
            int16_t next = (int16_t)st->accum_val + (int16_t)st->accum_step;
            if (next > high) next = high;
            if (next < low)  next = low;
            st->accum_val = (int8_t)next;
            break;
        }
    }
}

void algo3_reset_accumulators(algo3_metro_engine_t *eng) {
    for (uint8_t i = 0; i < 8u; i++) {
        eng->stages[i].accum_val = 0;
        eng->stages[i].accum_dir = 1;
    }
}

void algo3_init(algo3_metro_engine_t *eng, uint32_t seed) {
    eng->prng             = seed ? seed : 0x13579BDFu;
    eng->num_stages       = 8u;
    eng->cur_stage        = 0u;
    eng->cur_pulse        = 0u;
    eng->stage_prob_latch = true;
    eng->trig_mode        = ACCUM_TRIG_STAGE;
    eng->order_mode       = ACCUM_ORDER_PENDULUM;
    eng->bipolar          = true;
    eng->pos_limit        = 7;
    eng->neg_limit        = 7;
    eng->reset_bars       = 8u; /* Auto-reset every 8 bars (128 steps) */
    eng->step_counter     = 0u;

    static const int8_t  INIT_DEGS[8]   = { 0, 2, 4, 6, 4, 2, 1, 4 };
    static const uint8_t INIT_PULSES[8] = { 2, 2, 3, 1, 2, 2, 2, 2 }; /* Sums to 16 steps */
    static const int8_t  INIT_ACCUM[8]  = { 0, 1, 0, 2, 0, -1, 1, 2 };

    for (uint8_t i = 0; i < 8u; i++) {
        eng->stages[i].base_deg      = INIT_DEGS[i];
        eng->stages[i].oct_off       = (i == 3u |

| i == 7u) ? 1 : 0;
        eng->stages[i].pulse_count   = INIT_PULSES[i];
        eng->stages[i].gate_mode     = (i == 0u) ? METRO_GATE_HOLD
                                     : ((i == 4u) ? METRO_GATE_SINGLE : METRO_GATE_MULTI);
        eng->stages[i].ratchet_count = (i == 3u |

| i == 7u) ? 2u : 1u;
        eng->stages[i].probability   = (i == 0u) ? 255u : 220u;
        eng->stages[i].accum_step    = INIT_ACCUM[i];
        eng->stages[i].accum_val     = 0;
        eng->stages[i].accum_dir     = 1;
        eng->stages[i].accent        = (i == 0u |

| i == 3u |
| i == 6u);
        eng->stages[i].slide         = (i == 0u |

| i == 2u |
| i == 5u);
    }
}

void algo3_step(algo3_metro_engine_t *eng,
                uint8_t root_pc,
                int8_t chord_deg,
                const uint8_t scale_steps[12],
                uint8_t num_steps,
                seq_step_out_t *out)
{
    /* Automatic accumulator reset on phrase boundary */
    if (eng->reset_bars > 0u && eng->step_counter > 0u) {
        if ((eng->step_counter % ((uint32_t)eng->reset_bars * 16u)) == 0u) {
            algo3_reset_accumulators(eng);
        }
    }

    metro_stage_t *st = &eng->stages[eng->cur_stage % eng->num_stages];

    /* Evaluate PROB at the start of the stage (or per pulse for MULTI) */
    if (eng->cur_pulse == 0u |

| st->gate_mode == METRO_GATE_MULTI) {
        uint8_t roll = (uint8_t)(xorshift32(&eng->prng) & 0xFFu);
        eng->stage_prob_latch = (roll < st->probability);
    }

    bool gate_fires = false;
    if (eng->stage_prob_latch && st->gate_mode != METRO_GATE_REST) {
        switch (st->gate_mode) {
            case METRO_GATE_SINGLE:
                gate_fires = (eng->cur_pulse == 0u);
                break;
            case METRO_GATE_MULTI:
            case METRO_GATE_HOLD:
                gate_fires = true;
                break;
            default:
                gate_fires = false;
                break;
        }
    }

    if (gate_fires) {
        int16_t eff_deg = (int16_t)st->base_deg + (int16_t)st->accum_val;
        out->midi_note = (int16_t)degree_to_midi_folded(
            root_pc, chord_deg, eff_deg, st->oct_off, scale_steps, num_steps
        );
        out->velocity      = st->accent ? 120u : 90u;
        out->gate          = true;
        out->accent        = st->accent;
        /* HOLD mode ties pulses together via slide/hold until the final pulse of the stage */
        bool is_hold_tie   = (st->gate_mode == METRO_GATE_HOLD)
                          && ((eng->cur_pulse + 1u) < st->pulse_count);
        out->slide         = is_hold_tie |

| st->slide;
        out->ratchet_count = (st->gate_mode == METRO_GATE_HOLD) ? 1u : st->ratchet_count;

        /* Hardware-accurate Metropolix Accumulator Trigger Matrix:
         * Failed PROB or REST never increments accumulator! */
        if (eng->trig_mode == ACCUM_TRIG_STAGE && eng->cur_pulse == 0u) {
            algo3_advance_accumulator(eng, st);
        } else if (eng->trig_mode == ACCUM_TRIG_PULSE) {
            if (st->gate_mode == METRO_GATE_MULTI |

| eng->cur_pulse == 0u) {
                algo3_advance_accumulator(eng, st);
            }
        } else if (eng->trig_mode == ACCUM_TRIG_RATCHET) {
            uint8_t n_ratch = out->ratchet_count;
            for (uint8_t r = 0; r < n_ratch; r++) {
                algo3_advance_accumulator(eng, st);
            }
        }
    } else {
        out->midi_note     = -1;
        out->velocity      = 0u;
        out->gate          = false;
        out->accent        = false;
        out->slide         = false;
        out->ratchet_count = 1u;
    }

    /* Advance Pulse and Stage counters */
    eng->cur_pulse++;
    if (eng->cur_pulse >= st->pulse_count) {
        eng->cur_pulse = 0u;
        eng->cur_stage = (uint8_t)((eng->cur_stage + 1u) % eng->num_stages);
    }
    eng->step_counter++;
}
```

### Algorithm 4: Deja-Vu Loop & Mutate Shift Register (Marbles / Turing Style with Weak-Beat Mutation Bias)

Algorithm 4 combines Mutable Instruments Marbles' 16-step normalized ring buffer (`u_pitch`, `u_gate`), `SPREAD` / `BIAS` distribution shaping, and `STEPS` progressive salience masking with a **Metrically Biased Deja-Vu Mutator**. Unlike a standard Turing Machine that flips bits with uniform probability on any step—which frequently destroys the downbeat root anchor—Algorithm 4 weights the mutation threshold by `METRIC_MUTATE_SENSITIVITY[16]` so that **weak 16th-note subdivisions mutate first** as the `deja_vu` control moves counter-clockwise away from the 12 o'clock lock zone (`128`), while beat 1 (`step 0`) and beat 3 (`step 8`) remain locked unless `deja_vu` approaches pure randomness (`0`). Turning `deja_vu` clockwise above `143` keeps the 16 stored voltages fixed while probabilistically **permuting (jumping)** the read index within the active loop.

```markdown
#include "gen_tables.h"

/* Metrical Mutation Sensitivity (0..255):
 * Weak 16th steps (odd indices) have high sensitivity (220..255) and mutate first.
 * Downbeats (steps 0, 8) have low sensitivity (35..60) and stay anchored longest. */
static const uint8_t METRIC_MUTATE_SENSITIVITY[16] = {
     35, 240, 140, 255,  85, 230, 150, 255,
     60, 240, 140, 255,  95, 235, 160, 250
};

typedef struct {
    uint32_t prng;
    uint8_t  u_pitch[16];    /* Normalized raw pitch register [0..255] */
    uint8_t  u_gate[16];     /* Normalized raw gate/articulation register [0..255] */
    uint8_t  loop_len;       /* Active shift register length (1..16) */
    uint8_t  deja_vu;        /* 0..112: Mutate->Lock, 113..143: Locked Notch, 144..255: Permute */
    uint8_t  spread;         /* 0..255 PDF width around center */
    uint8_t  bias;           /* 0..255 register skew (0=Low, 128=Center, 255=High) */
    uint8_t  steps_salience; /* 0..255: progressive salience filter (0=Full Scale, 255=Root Only) */
    uint8_t  density;        /* 0..255 gate threshold */
    bool     super_lock;     /* Freezes register mutations when true */
} algo4_dejavu_engine_t;

void algo4_init(algo4_dejavu_engine_t *eng, uint32_t seed) {
    eng->prng           = seed ? seed : 0x2468ACE1u;
    eng->loop_len       = 16u;
    eng->deja_vu        = 128u; /* 12 o'clock virtual notch (locked loop) */
    eng->spread         = 140u;
    eng->bias           = 128u;
    eng->steps_salience = 64u;  /* Full diatonic/triad salience */
    eng->density        = 175u;
    eng->super_lock     = false;

    for (uint8_t i = 0; i < 16u; i++) {
        uint32_t r = xorshift32(&eng->prng);
        eng->u_pitch[i] = (uint8_t)(r & 0xFFu);
        eng->u_gate[i]  = (uint8_t)((r >> 8) & 0xFFu);
    }
    eng->u_pitch[0] = 128u; /* Center root bias on step 0 */
    eng->u_gate[0]  = 250u; /* High gate energy on step 0 */
}

void algo4_step(algo4_dejavu_engine_t *eng,
                uint8_t step_16,
                uint8_t root_pc,
                int8_t chord_deg,
                const uint8_t scale_steps[12],
                uint8_t num_steps,
                seq_step_out_t *out)
{
    uint8_t len      = (eng->loop_len >= 1u && eng->loop_len <= 16u) ? eng->loop_len : 16u;
    uint8_t write_ix = step_16 % len;
    uint8_t read_ix  = write_ix;
    uint32_t r       = xorshift32(&eng->prng);

    if (!eng->super_lock) {
        if (eng->deja_vu < 113u) {
            /* Zone 1 (CCW of 12 o'clock): Metrically Biased Mutation
             * Global mutation drive (0..226) scaled by step's metrical weakness */
            uint16_t mut_drive = (uint16_t)(113u - eng->deja_vu) * 2u;
            uint16_t step_sens = METRIC_MUTATE_SENSITIVITY[step_16 & 15u];
            uint16_t eff_prob  = (mut_drive * step_sens) >> 8;

            if ((r & 0xFFu) < eff_prob) {
                uint32_t r_new = xorshift32(&eng->prng);
                eng->u_pitch[write_ix] = (uint8_t)(r_new & 0xFFu);
                eng->u_gate[write_ix]  = (uint8_t)((r_new >> 8) & 0xFFu);
            }
        } else if (eng->deja_vu > 143u) {
            /* Zone 3 (CW of 12 o'clock): Loop Permutation (Random Read Jump) */
            uint8_t jump_prob = (uint8_t)((eng->deja_vu - 143u) * 2u);
            if ((r & 0xFFu) < jump_prob) {
                read_ix = (uint8_t)(((r >> 8) & 0xFFu) % len);
            }
        }
        /* Zone 2 (113..143): 12 o'clock Virtual Notch -> 100% locked loop */
    }

    uint8_t raw_g = eng->u_gate[read_ix];
    uint8_t raw_p = eng->u_pitch[read_ix];

    /* Gate evaluation with downbeat boost on step 0 */
    uint16_t g_thresh = (uint16_t)(255u - eng->density);
    if ((step_16 & 15u) == 0u && g_thresh > 40u) g_thresh -= 40u;

    if (raw_g < g_thresh) {
        out->midi_note     = -1;
        out->velocity      = 0u;
        out->gate          = false;
        out->accent        = false;
        out->slide         = false;
        out->ratchet_count = 1u;
        return;
    }

    /* Apply SPREAD (variance scaling around 128) and BIAS (register shift) */
    int32_t centered = (int32_t)raw_p - 128;
    int32_t shaped   = (centered * (int32_t)eng->spread) >> 7; /* -256..+256 */
    int32_t biased   = shaped + (((int32_t)eng->bias - 128) * 2);

    /* Map shaped value to a scale degree offset in [-10..+10] */
    int16_t deg_span   = (int16_t)((biased * 10) / 256);
    int16_t base_d_mod = floor_mod_i16(deg_span, 7);
    int16_t oct_shift  = floor_div_i16(deg_span, 7);

    /* Apply Marbles STEPS Progressive Salience Quantizer:
     * As steps_salience increases (0 -> 255), strip weak scale degrees into strong ones */
    if (eng->steps_salience > 210u) {
        /* Root only */
        base_d_mod = 0;
    } else if (eng->steps_salience > 160u) {
        /* Root and Fifth (0, 4) only */
        base_d_mod = (base_d_mod >= 3 && base_d_mod <= 5) ? 4 : 0;
    } else if (eng->steps_salience > 110u) {
        /* Triad only (0, 2, 4) */
        static const int8_t TRIAD_SNAP[7] = { 0, 0, 2, 2, 4, 4, 0 };
        base_d_mod = TRIAD_SNAP[base_d_mod];
    } else if (eng->steps_salience > 60u) {
        /* Pentatonic subset (strip 4th [deg 3] and 7th [deg 6]) */
        if (base_d_mod == 3) base_d_mod = 2;
        if (base_d_mod == 6) base_d_mod = 4;
    }

    int16_t final_deg_off = base_d_mod + (oct_shift * (int16_t)num_steps);
    bool accent = (raw_g > 210u);
    bool slide  = ((raw_g & 0x07u) == 0x07u);

    out->midi_note = (int16_t)degree_to_midi_folded(
        root_pc, chord_deg, final_deg_off, 0, scale_steps, num_steps
    );
    out->velocity      = accent ? 120u : 90u;
    out->gate          = true;
    out->accent        = accent;
    out->slide         = slide;
    out->ratchet_count = 1u;
}
```

## 4\. Hardware Control Compression (Max 8 Controls) & MIDI Root/Chord Follow

When designing a physical synthesizer or Eurorack module with a maximum of 6 to 8 hardware potentiometers or encoders, exposing dozens of algorithm-specific parameters via deep menu diving degrades live playability. Instead, all four engines can be normalized under a **unified 8-pot control abstraction** where each physical control maps to an isomorphic musical dimension across every algorithm.

<iframe allow="xr-spatial-tracking; web-share" title="Preview" sandbox="allow-pointer-lock allow-popups allow-forms allow-popups-to-escape-sandbox allow-downloads allow-same-origin allow-scripts" src="https://22j3a0y656y1686eq5gwzvhhez1w33pstmi3iueipig49dbul2-h1018257101.scf.usercontent.goog/gemini-code-immersive/shim.html?origin=https%3A%2F%2Fgemini.google.com&amp;cache=1"></iframe>

### 1\. The 8-Pot Unified Hardware Control Matrix

By normalizing every potentiometer's ADC reading to an 8-bit unsigned integer (`0..255`), a single 8-knob control surface governs all four generative paradigms without modal confusion:

| Pot # | Control Label | Global / Shared Function | Algorithm 1 (`ABAC` Motif) | Algorithm 2 (Markov / Pedal) | Algorithm 3 (Metropolix Accum) | Algorithm 4 (Marbles `DejaVu`) |
| --- | --- | --- | --- | --- | --- | --- |
| **Pot 1** | **`ALGO / MODE`** | Selects active algorithm (`0..3`) & sub-variant (`0..3`) | Selects Form: `ABAC`, `AABC`, `ABBC`, `AABA` | Selects Pedal Mode: Quarter, 8th, Syncopated, Off | Selects Accum Order: `Wrap`, `Pendulum`, `Random`, `Hold` | Selects Salience Tier (`STEPS`: Full, Pent, Triad, Root) |
| **Pot 2** | **`ROOT / FOLLOW`** | Base Root (`0..11`: `C..B`) + MIDI / Chord-Drift Follow mode | `0..119`: Fixed Root `C..B`; `120..199`: Internal W7/PLR Chord Drift; `200..255`: External MIDI Root/Chord Follow | Same as Global | Same as Global | Same as Global |
| **Pot 3** | **`SCALE / CHORD`** | Selects active Scale (`0..19`) and Chord Voicing (`TRIAD`, `7TH`, `9TH`, `SUS`) | Quantizes Motif `A/B/C` scale steps & chord triad | Sets Scale Mask & Markov gravity mode (Maj/Min) | Sets Scale Mask & Accumulator Limit span (`±7` to `±14`) | Sets Scale Mask & `BIAS` register center |
| **Pot 4** | **`DENSITY`** | Controls gate probability (`0..255`) & downbeat anchoring | Gate probability of Seed `A` cells (`0..255`) | Overall note density vs. rests (`0..255`) | Stage `PROB` threshold & `REST` vs. `MULTI` gate balance | Gate threshold (`u_gate` cutoff) |
| **Pot 5** | **`CONTOUR / GROOVE`** | Macro-shaping of melodic arch, pedal weight, or stage ratchets | Selects `(xform_B, xform_C)` pair (Transpose, Invert, Retro, Displace) | Sweeps `contour_id` (`0..15`) & `pedal_anchor_prob` | Controls Stage `accum_step` depth & `ratchet_count` density | Controls `SPREAD` (PDF width from flat center to bimodal) |
| **Pot 6** | **`MUTATE / DEJAVU`** | Controls pattern evolution vs. deterministic loop locking | Seed `A` bar-wrap mutation probability (`0..255`) | Markov transition entropy (strict voice-leading vs. wide leaps) | Accumulator Trigger Mode (`Stage`, `Pulse`, `Ratchet`) & Auto-Reset | Marbles 3-Zone `DEJA VU`: CCW=Mutate, 12h=Lock, CW=Permute |
| **Pot 7** | **`DIV / LENGTH`** | Clock division (`/4`, `/2`, `1x`, `2x`, `3x`) & Loop Length (`1..16`) | Pattern loop length (`1..16`) & clock divider | Pattern loop length (`1..16`) & clock divider | Active stage count (`1..8`) & stage pulse multiplier | Shift register `loop_len` (`1..16`) & clock divider |
| **Pot 8** | **`DEST / RANGE`** | Octave fold range (`1..3` octaves), Slide probability & Turnaround Fill | Octave jump probability & 4/8-bar Turnaround Fill enable | Octave range (`1..3`) & Turnaround Fill enable | Bipolar vs. Unipolar range & Slide probability | `BIAS` register offset & `super_lock` toggle at extreme |

### 2\. Dynamic Pattern Re-Tiling Over Incoming MIDI Notes or Internal Chord Progressions

As demonstrated in the HYP-1 architecture (`amb.c`) and OXI One's `Harmonizer`, the key to making a monophonic sequence follow a chord progression effortlessly is **decoupling the pattern's internal storage from absolute MIDI pitches**. Because all four algorithms store and compute step pitches as **diatonic scale-degree offsets (`deg_off`) relative to `chord_deg`**, dynamic re-tiling happens automatically at the moment a step is converted to MIDI via `degree_to_midi_folded()`.

Three operational modes allow external MIDI or internal state machines to drive `(root_pc, chord_deg, scale_steps)` seamlessly. In **Parallel Chromatic Transpose (Berlin School Style)**, when a user plays a single MIDI note `midi_note` on a designated MIDI "Follower" channel, the controller updates `root_pc = midi_note % 12` while keeping `chord_deg = 0`. The entire scale mask shifts in parallel to the new root pitch class, producing the classic 1970s Tangerine Dream / Klaus Schulze sequence transposition effect. In **Diatonic Scale-Degree Re-Tiling (Harmonizer Style)**, the controller keeps the global key `root_pc` and `scale_mask` fixed, and maps the incoming MIDI note `midi_note` to the nearest scale degree `chord_deg` (`0..M - 1`) within the active scale. When the incoming bass note moves from `C` (`chord_deg = 0`, degree `I`) to `F` (`chord_deg = 3`, degree `IV`) in C Major, a motif step playing the chord's 3rd (`deg_off = +2`) automatically shifts from `E` (major 3rd above `C`) to `A` (major 3rd above `F`), maintaining 100% diatonic lock without out-of-key notes. Finally, in **Polyphonic MIDI Chord Detection & Bar-Quantized State-Machine Follow**, when receiving a 3- or 4-note MIDI chord buffer (or advancing the internal `W7` / Neo-Riemannian progression on a bar boundary), the engine extracts the lowest held note as the bass degree `chord_deg` (`ha_set_chord()` in `amb.c`) and optionally updates the active pitch-class pool with the held chord tones so non-diatonic borrowed chords quantize cleanly.

```markdown
/* Unified 8-Pot Hardware Dispatcher & MIDI Chord-Follow Controller */
typedef struct {
    uint8_t               active_algo;       /* 0..3 selected by Pot 1 */
    uint8_t               root_pc;           /* 0..11 global key root */
    int8_t                chord_deg;         /* 0..M-1 active diatonic chord degree */
    scale_id_t            scale_id;          /* Active scale index (0..19) */
    uint8_t               scale_steps[12];
    uint8_t               num_steps;
    uint8_t               follow_mode;       /* 0=Fixed, 1=Internal W7 Drift, 2=MIDI Scale-Deg, 3=MIDI Parallel */
    uint8_t               turnaround_period; /* 0=Off, 4=Every 4th bar, 8=Every 8th bar */
    uint16_t              bar_counter;
    algo1_motif_engine_t  algo1;
    algo2_markov_engine_t algo2;
    algo3_metro_engine_t  algo3;
    algo4_dejavu_engine_t algo4;
} unified_seq_controller_t;

/* Called when a MIDI Note-On arrives on the Follower Channel */
void seq_on_midi_follow_note(unified_seq_controller_t *ctrl, uint8_t midi_note) {
    uint8_t pc = midi_note % 12u;
    if (ctrl->follow_mode == 3u) {
        /* Parallel Berlin-School Transpose: shift global root directly */
        ctrl->root_pc   = pc;
        ctrl->chord_deg = 0;
    } else if (ctrl->follow_mode == 2u) {
        /* Diatonic Harmonizer Re-Tiling: find nearest scale degree in active key */
        uint8_t rel_semi = (uint8_t)((pc + 12u - ctrl->root_pc) % 12u);
        uint8_t best_deg = 0u;
        uint8_t min_err  = 12u;
        for (uint8_t i = 0; i < ctrl->num_steps; i++) {
            uint8_t err = (rel_semi >= ctrl->scale_steps[i])
                        ? (rel_semi - ctrl->scale_steps[i])
                        : (ctrl->scale_steps[i] - rel_semi);
            if (err < min_err) {
                min_err  = err;
                best_deg = i;
            }
        }
        ctrl->chord_deg = (int8_t)best_deg;
    }
}
```

## 5\. Articulation & DSP Nuances

Generating musical MIDI note numbers is only half of a hardware synthesizer's sequencer pipeline; the tactile groove of an embedded bassline or lead depends on how **legato slides**, **filter/amplitude accents**, **octave folding**, and **multi-bar turnaround fills** drive the downstream DSP voice.

### 1\. 303-Style Legato Slides (Envelope Retrigger Suppression & Constant-Time Pitch Slew)

In a hardware TB-303 (and authentic DSP ports such as `Open303` and `HYP-1`), a `slide` flag is a **forward-looking attribute**: setting `slide = true` on step `s` means step `s` 's gate is held continuously (`100%` duty cycle) across the step boundary into step `s + 1`. Consequently, when step `s + 1` arrives, the voice engine checks whether **step `s` was both gated and had `slide = true`** (`slid = prev_step.gate && prev_step.slide && voice_held`).

When `slid == false` (a normal unslid attack), on step `s` 's rising edge the engine fires a fresh envelope trigger (`env_trig = 1`), snaps the oscillator's pitch target immediately to `midi_note` without portamento, and schedules `gate_off` at `55%` of the 16th-note step duration (`gate_samples = (samples_per_step * 55) / 100`). Conversely, when `slid == true` and step `s + 1` is gated (a legato slide continuation), the engine **skips retriggering the VCA and VCF envelopes**, and if outputting external MIDI, transmits the new `Note On` **before** sending the previous note's `Note Off` (activating mono legato on external hardware) while slewing internal pitch toward the new note. If `slid == true` but step `s + 1` is a rest, the held gate releases immediately.

As circuit analyses by Robin Whittle and Tim Stinchcombe demonstrate, the TB-303 pitch glide is a single-pole RC filter (`~60 ms` time constant) charging toward the new DAC voltage—meaning it is **constant-time** (taking the same ~50–60 ms to reach 90–95% of the target regardless of whether the interval is 1 semitone or 2 octaves) rather than constant-rate (`ms/octave`). At a `44.1–48 kHz` audio sample rate, a first-order exponential IIR filter replicates this trajectory:

$$
P[n] = P[n-1] + \alpha_{\text{slide}} \cdot \left(P_{\text{target}} - P[n-1]\right)
$$

where $\alpha_{\text{slide}} = 1 - e^{-1 / (f_s \cdot \tau_{\text{slide}})} \approx 0.0012$ per sample (`amb.c:2734`), achieving ~95% settling in `50–60 ms`.

### 2\. Accent-to-Filter / Velocity Coupling & Consecutive Accent Capacitor Stacking

On a basic MIDI sequencer, `accent` merely switches MIDI velocity from `90` (unaccented) to `120` (accented). Inside an integrated hardware synthesizer DSP voice on RP2350 or ESP32-C3, however, an active `accent` flag must simultaneously modify four synthesis mechanisms to recreate the bark and squelch of an analog acid voice:

| Accent Mechanism | Hardware / DSP Behavior | Musical & Timbral Effect |
| --- | --- | --- |
| **1\. VCA Amplitude Boost** | Scales VCA velocity/gain from `90` (normal) to `120` (accented). | Emphasizes syncopated off-beat notes above the unaccented groove. |
| **2\. VCF Cutoff Depth Boost** | Injects an additional envelope modulation signal into the resonant low-pass filter cutoff. | Produces a sharp, resonant vocal sweep on accented steps. |
| **3\. Shortened VCF Decay** | Switches the filter envelope decay time to a fast, fixed `~200 ms` regardless of the main `Decay` knob position. | Turns wide filter sweeps into tight, percussive acid barks; when paired with `slide`, yields a sweeping resonant glide. |
| **4\. C13 Capacitor Stacking** | Accumulates residual charge across consecutive accented steps because the hardware accent sweep capacitor does not fully discharge in one 16th step. | Chained accents (`step s` and `step s + 1` both accented) ramp the filter cutoff progressively higher on each successive step. |

### 3\. Octave Range Folding (28..96 MIDI Window)

When combining chord degree offsets (`0..6`), accumulator transpositions (`-28..+28` scale steps), and octave modifiers (`-2..+2` octaves), raw calculated MIDI notes can drop into inaudible sub-bass (`< 28`, below `E1 = 41.2 Hz`) or pierce above `C7` (`> 96`). Rather than hard-clamping (`if (n > 96) n = 96`), which flattens melodic contours into repeated ceiling notes and destroys scale-degree identity, **octave folding** shifts the pitch by multiples of 12 semitones (`while (n > 96) n -= 12; while (n < 28) n += 12;`). This preserves the exact pitch class (`n mod 12`) and harmonic function while keeping the synthesizer voice inside its optimal `28..96` (`E1..C7`) playable register.

### 4\. Automatic 4 / 8 / 16-Bar Cadential Turnaround Fills & Complete Voice Articulation Processor

To give generative sequences long-form structural phrasing, the master clock tracks `bar_counter` (`sixteenth_counter >> 4`) and triggers an automatic **Cadential Turnaround Fill** during the second half (`steps 8..15` or `12..15`) of every 4th, 8th, or 16th bar (`(bar_counter % turnaround_period) == (turnaround_period - 1)`). Rather than introducing random chaos, the turnaround interceptor executes one of four deterministic cadential gestures selected by the phrase cycle index: an **Ascending Chord-Tone Run (`HYP-1` Fill)** that forces gates on steps `8..15` with ascending even scale-degree offsets `{0, 2, 4, 6, 8, 10, 12, 14}` and rising accents on steps `12..15`; a **Descending Scale Cascade (`Acid` Fill)** on steps `12..15` using scale degrees `{7, 5, 4, 1}` with `slide = true` on steps `12..14`; an **Octave Pedal Ratchet Build** on steps `12..15` alternating between root (`0`) and octave (`+7`) with accelerating ratchets (`1, 2, 3, 4`); or a **Dominant Tension Hold** that ties the fifth (`+4`) across steps `12..14` before dropping to a leading-tone pickup (`-1`) on step `15`.

The following C99 implementation provides the complete **Turnaround Fill Interceptor** and **303 DSP Articulation & Accent-Stacking Slew Processor**:

```markdown
#include "gen_tables.h"

typedef struct {
    /* Pitch Slew & Gate State (Fixed-Point Q16.16 MIDI Note for zero-float DSP) */
    int32_t  current_pitch_q16;  /* Current slewed pitch in Q16.16 MIDI semitones */
    int32_t  target_pitch_q16;   /* Target pitch in Q16.16 MIDI semitones */
    int32_t  slew_coeff_q16;     /* Exponential slew alpha in Q16.16 (~79 for 1.2e-3) */
    bool     voice_gate_high;    /* True while hardware/DSP gate is held high */
    bool     prev_step_slid;     /* True if previous 16th step was gated AND had slide=1 */
    uint32_t gate_remain_samples;/* Remaining audio samples before staccato gate-off */

    /* TB-303 Accent Capacitor Stacking State (Q16.16 normalized 0..65536+) */
    int32_t  accent_cap_q16;     /* Models C13 residual charge across chained accents */
    int32_t  vcf_env_q16;        /* Current VCF envelope level (Q16.16) */
    int32_t  vcf_decay_mul_q16;  /* Normal vs. Shortened (~200ms) accent decay multiplier */
} acid_dsp_articulator_t;

void articulator_init(acid_dsp_articulator_t *art) {
    art->current_pitch_q16   = 48 << 16; /* Default C3 */
    art->target_pitch_q16    = 48 << 16;
    art->slew_coeff_q16      = 79;       /* 79 / 65536 ~= 0.001205 per sample @ 48kHz (~55ms glide) */
    art->voice_gate_high     = false;
    art->prev_step_slid      = false;
    art->gate_remain_samples = 0u;
    art->accent_cap_q16      = 0;
    art->vcf_env_q16         = 0;
    art->vcf_decay_mul_q16   = 65470;    /* Normal decay multiplier */
}

/* Intercepts step output on the final bar of a 4/8/16-bar phrase to inject cadential fills */
void apply_turnaround_fill(uint8_t step_16,
                           uint16_t bar_idx,
                           uint8_t turnaround_period, /* e.g., 4, 8, or 16 bars */
                           uint8_t root_pc,
                           int8_t chord_deg,
                           const uint8_t scale_steps[12],
                           uint8_t num_steps,
                           seq_step_out_t *io_step)
{
    if (turnaround_period == 0u) return;
    if ((bar_idx % turnaround_period) != (uint16_t)(turnaround_period - 1u)) return;

    uint8_t s = step_16 & 15u;
    uint8_t fill_type = (uint8_t)((bar_idx / turnaround_period) & 3u);

    if (fill_type == 0u && s >= 8u) {
        /* Type 0: HYP-1 8-step Ascending Chord-Tone Run (Steps 8..15) */
        static const int8_t RUN_DEGS[8] = { 0, 2, 4, 6, 8, 10, 12, 14 };
        io_step->gate          = true;
        io_step->accent        = (s >= 12u);
        io_step->slide         = (s == 11u |

| s == 14u);
        io_step->ratchet_count = 1u;
        io_step->velocity      = io_step->accent ? 120u : 90u;
        io_step->midi_note     = (int16_t)degree_to_midi_folded(
            root_pc, chord_deg, RUN_DEGS[s - 8u], 0, scale_steps, num_steps
        );
    } else if (fill_type == 1u && s >= 12u) {
        /* Type 1: Classic 303 Descending Slid Cascade (Steps 12..15) */
        static const int8_t DESC_DEGS[4] = { 7, 5, 4, 1 };
        io_step->gate          = true;
        io_step->accent        = true;
        io_step->slide         = (s != 15u); /* Slide steps 12->13->14, release on 15 */
        io_step->ratchet_count = 1u;
        io_step->velocity      = 120u;
        io_step->midi_note     = (int16_t)degree_to_midi_folded(
            root_pc, chord_deg, DESC_DEGS[s - 12u], 0, scale_steps, num_steps
        );
    } else if (fill_type == 2u && s >= 12u) {
        /* Type 2: Accelerating Octave Ratchet Build (Steps 12..15) */
        int16_t deg = ((s & 1u) == 0u) ? 0 : 7;
        io_step->gate          = true;
        io_step->accent        = (s >= 14u);
        io_step->slide         = false;
        io_step->ratchet_count = (uint8_t)(s - 11u); /* Ratchets: 1, 2, 3, 4 */
        io_step->velocity      = io_step->accent ? 120u : 95u;
        io_step->midi_note     = (int16_t)degree_to_midi_folded(
            root_pc, chord_deg, deg, 0, scale_steps, num_steps
        );
    } else if (fill_type == 3u && s >= 12u) {
        /* Type 3: Dominant Tension Tie -> Leading-Tone Pickup */
        int16_t deg = (s < 15u) ? 4 : -1;
        io_step->gate          = true;
        io_step->accent        = (s == 12u |

| s == 15u);
        io_step->slide         = true;
        io_step->ratchet_count = 1u;
        io_step->velocity      = io_step->accent ? 120u : 90u;
        io_step->midi_note     = (int16_t)degree_to_midi_folded(
            root_pc, chord_deg, deg, 0, scale_steps, num_steps
        );
    }
}

/* Called at the start of every 16th-note step (or ratchet sub-step) */
void articulator_on_step_edge(acid_dsp_articulator_t *art,
                              const seq_step_out_t *step,
                              uint32_t samples_per_step,
                              bool *out_retrigger_env)
{
    if (!step->gate |

| step->midi_note < 0) {
        /* Step is a Rest: release any held slide immediately */
        art->voice_gate_high     = false;
        art->prev_step_slid      = false;
        art->gate_remain_samples = 0u;
        *out_retrigger_env       = false;
        return;
    }

    int32_t new_pitch_q16 = ((int32_t)step->midi_note) << 16;
    bool is_legato_slide  = art->prev_step_slid && art->voice_gate_high;

    if (is_legato_slide) {
        /* Legato Slide Continuation: DO NOT retrigger envelope; slew pitch toward target */
        art->target_pitch_q16 = new_pitch_q16;
        *out_retrigger_env    = false;
    } else {
        /* Fresh Note Attack: snap pitch immediately and fire envelope trigger */
        art->current_pitch_q16 = new_pitch_q16;
        art->target_pitch_q16  = new_pitch_q16;
        art->vcf_env_q16       = 65536; /* Reset VCF envelope to 1.0 in Q16.16 */
        *out_retrigger_env     = true;
    }

    /* Update TB-303 Accent Capacitor Stacking (C13 charge accumulation) */
    if (step->accent) {
        /* Add 0.75 (49152) to residual capacitor charge (stacks on consecutive accents!) */
        art->accent_cap_q16 += 49152;
        if (art->accent_cap_q16 > 131072) art->accent_cap_q16 = 131072; /* Clamp at 2.0x */
        art->vcf_decay_mul_q16 = 65200; /* Shortened punchy ~200ms VCF decay on accent */
    } else {
        art->vcf_decay_mul_q16 = 65470; /* Normal VCF decay */
    }

    /* Set Gate Duration: 100% hold if slide is set on THIS step, else 55% duty cycle */
    art->voice_gate_high = true;
    art->prev_step_slid  = step->slide;
    if (step->slide) {
        art->gate_remain_samples = 0xFFFFFFFFu; /* Held continuously across step boundary */
    } else {
        art->gate_remain_samples = (samples_per_step * 55u) / 100u; /* 55% 303 gate length */
    }
}

/* Executed per audio sample (or control block) in the DSP ISR */
static inline void articulator_process_sample(acid_dsp_articulator_t *art,
                                              int32_t *out_pitch_q16,
                                              int32_t *out_vcf_mod_q16,
                                              bool *out_gate_active)
{
    /* 1. Constant-time exponential pitch portamento slew */
    int32_t diff = art->target_pitch_q16 - art->current_pitch_q16;
    if (diff != 0) {
        art->current_pitch_q16 += (int32_t)(((int64_t)diff * art->slew_coeff_q16) >> 16);
    }

    /* 2. VCF Envelope & Stacked Accent Capacitor Discharge */
    art->vcf_env_q16    = (int32_t)(((int64_t)art->vcf_env_q16 * art->vcf_decay_mul_q16) >> 16);
    art->accent_cap_q16 = (int32_t)(((int64_t)art->accent_cap_q16 * 65380) >> 16);

    /* 3. Gate duty-cycle countdown (when not tied by slide) */
    if (art->gate_remain_samples > 0u && art->gate_remain_samples != 0xFFFFFFFFu) {
        if (--art->gate_remain_samples == 0u) {
            art->voice_gate_high = false;
        }
    }

    *out_pitch_q16   = art->current_pitch_q16;
    *out_vcf_mod_q16 = art->vcf_env_q16 + art->accent_cap_q16;
    *out_gate_active = art->voice_gate_high;
}
```