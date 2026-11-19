// gen_ugen_ref.scd — per-UGen canonical NRT reference renders (T-D2).
// One 1.0 s float32 WAV per UGen in ugen_ref/, rendered through the SAME
// wrapper discipline as the drone goldens: RandSeed.kr(1, 16rC0FFEE) inside
// the SynthDef pins the synth's RGen, so every random draw is reproducible.
//
// Canonical params are part of the contract: the C++ kit test constructs
// each UGen with the params listed here (see the comment above each entry).
// .kr outputs are rendered through K2A (block-held) so the C++ side compares
// ProcessKr() replicated across the block.
//
// Headless-sclang constraints (see gen_goldens.scd): body = called Function,
// vars first, no top-level ( ... ) group.
{ |scriptPath|
    var seed = 16rC0FFEE;
    var dir = PathName(scriptPath).pathOnly;
    var outDir = dir ++ "ugen_ref";
    var dur = 1.0;
    var scsynthFlags = ("DC_SCSYNTH_FLAGS").getenv ?? "";
    var force = ("DC_FORCE").getenv == "1";
    var entries;

    entries = [
        // name, body ({ |h| ... } — h = 1/sr for kr-rate helpers is unused here)
        ["WhiteNoise", { Out.ar(0, WhiteNoise.ar.dup) }],
        ["ClipNoise",  { Out.ar(0, ClipNoise.ar.dup) }],
        ["GrayNoise",  { Out.ar(0, GrayNoise.ar.dup) }],
        ["PinkNoise",  { Out.ar(0, PinkNoise.ar.dup) }],
        ["BrownNoise", { Out.ar(0, BrownNoise.ar.dup) }],
        ["Crackle",    { Out.ar(0, Crackle.ar(1.95).dup) }],
        ["Dust",       { Out.ar(0, Dust.ar(1000).dup) }],
        ["Dust2",      { Out.ar(0, Dust2.ar(1000).dup) }],
        // LF noise: .kr(10) through K2A (block-held) and .ar(400) native.
        ["LFNoise0kr", { Out.ar(0, K2A.ar(LFNoise0.kr(10)).dup) }],
        ["LFNoise1kr", { Out.ar(0, K2A.ar(LFNoise1.kr(10)).dup) }],
        ["LFNoise2kr", { Out.ar(0, K2A.ar(LFNoise2.kr(10)).dup) }],
        ["LFNoise0ar", { Out.ar(0, LFNoise0.ar(400).dup) }],
        ["LFNoise1ar", { Out.ar(0, LFNoise1.ar(400).dup) }],
        ["LFNoise2ar", { Out.ar(0, LFNoise2.ar(400).dup) }],
        ["LFClipNoise",{ Out.ar(0, LFClipNoise.ar(400).dup) }],
        ["LFDNoise0kr",{ Out.ar(0, K2A.ar(LFDNoise0.kr(3)).dup) }],
        ["LFDNoise0ar",{ Out.ar(0, LFDNoise0.ar(400).dup) }],
        ["LFDNoise3kr",{ Out.ar(0, K2A.ar(LFDNoise3.kr(3)).dup) }],
        ["LFDNoise3ar",{ Out.ar(0, LFDNoise3.ar(400).dup) }],
        // init-time random: Rand(0.25, 0.75) etc. render as constants; the
        // C++ test compares one value from the same seed.
        ["Rand",       { Out.ar(0, K2A.ar(Rand(0.25, 0.75)).dup) }],
        ["IRand",      { Out.ar(0, K2A.ar(IRand(0, 100)).dup) }],
        ["NRand",      { Out.ar(0, K2A.ar(NRand(0, 1, 4)).dup) }],
        ["LinRand",    { Out.ar(0, K2A.ar(LinRand(0, 1, 0)).dup) }],
        ["ExpRand",    { Out.ar(0, K2A.ar(ExpRand(0.1, 10)).dup) }],

        // ---- T-D2 lfo family (canonical params: 5 Hz / 440 Hz / duty 0.3) ----
        ["LFSawar",    { Out.ar(0, LFSaw.ar(440).dup) }],
        ["LFSawkr",    { Out.ar(0, K2A.ar(LFSaw.kr(5)).dup) }],
        ["LFParar",    { Out.ar(0, LFPar.ar(440).dup) }],
        ["LFParkr",    { Out.ar(0, K2A.ar(LFPar.kr(5)).dup) }],
        ["LFCubar",    { Out.ar(0, LFCub.ar(440).dup) }],
        ["LFPulsear",  { Out.ar(0, LFPulse.ar(440, 0, 0.3).dup) }],
        ["LFPulsekr",  { Out.ar(0, K2A.ar(LFPulse.kr(5, 0, 0.3)).dup) }],
        ["VarSawar",   { Out.ar(0, VarSaw.ar(440, 0, 0.3).dup) }],
        ["Impulsekr",  { Out.ar(0, K2A.ar(Impulse.kr(10)).dup) }],
        ["Impulsear",  { Out.ar(0, Impulse.ar(440).dup) }],
        ["Phasorkr",   { Out.ar(0, K2A.ar(Phasor.kr(0, 0.001, 0, 1, 0)).dup) }],
        ["Clipkr",     { Out.ar(0, K2A.ar(Clip.kr(SinOsc.kr(5), -0.5, 0.5)).dup) }],
        ["Foldkr",     { Out.ar(0, K2A.ar(Fold.kr(SinOsc.kr(5) * 3, -0.5, 0.5)).dup) }],
        ["DCkr",       { Out.ar(0, K2A.ar(DC.kr(0.25)).dup) }],
        ["A2Kar",      { Out.ar(0, K2A.ar(A2K.kr(SinOsc.ar(440))).dup) }],

        // ---- T-D2 trig family (kr forms; trig inputs from Impulse.kr) ----
        ["Trig1kr",    { Out.ar(0, K2A.ar(Trig1.kr(Impulse.kr(2), 0.1)).dup) }],
        ["Trigkr",     { Out.ar(0, K2A.ar(Trig.kr(Impulse.kr(2), 0.1)).dup) }],
        ["TDelaykr",   { Out.ar(0, K2A.ar(TDelay.kr(Impulse.kr(2), 0.1)).dup) }],
        ["Latchkr",    { Out.ar(0, K2A.ar(Latch.kr(SinOsc.kr(5), Impulse.kr(3))).dup) }],
        ["SetResetkr", { Out.ar(0, K2A.ar(SetResetFF.kr(Impulse.kr(2), Impulse.kr(0.7))).dup) }],
        ["Peakkr",     { Out.ar(0, K2A.ar(Peak.kr(SinOsc.kr(7), Impulse.kr(1))).dup) }],
        ["Gatear",     { Out.ar(0, Gate.ar(SinOsc.ar(440), LFPulse.kr(2))) }],
        ["Changedkr",  { Out.ar(0, K2A.ar(Changed.kr(LFPulse.kr(5))).dup) }],

        // ---- T-D2 osc2 family ----
        ["Sawar",      { Out.ar(0, Saw.ar(440).dup) }],
        ["SinOscFBar", { Out.ar(0, SinOscFB.ar(440, 0.5).dup) }],
        ["Selectkr",   { Out.ar(0, K2A.ar(Select.kr(LFPulse.kr(2), [0.25, 0.5, 0.75])).dup) }],
        // Klank renders identically in scsynth (rc 139 at shutdown) and
        // supernova (rc 0) — supernova avoids the shutdown crash.
        ["Klankar",    { Out.ar(0, Klank.ar(`[[220,440,660], [1,0.5,0.25], [1,1,1]], Impulse.ar(5))) }, "supernova"],
        ["SawDPWar",   { Out.ar(0, SawDPW.ar(440).dup) }],
        ["PulseDPWar", { Out.ar(0, PulseDPW.ar(440, 0.3).dup) }],

        // ---- T-D2 filter family (core 11) ----
        ["LPFar",      { Out.ar(0, LPF.ar(Saw.ar(220), 2000)) }],
        ["HPFar",      { Out.ar(0, HPF.ar(Saw.ar(220), 500)) }],
        ["BPFar",      { Out.ar(0, BPF.ar(Saw.ar(220), 1000, 0.25)) }],
        ["RLPFar",     { Out.ar(0, RLPF.ar(Saw.ar(220), 1000, 0.5)) }],
        ["RHPFar",     { Out.ar(0, RHPF.ar(Saw.ar(220), 1000, 0.5)) }],
        ["OnePolear",  { Out.ar(0, OnePole.ar(Saw.ar(220), 0.9)) }],
        ["Integratorar",{ Out.ar(0, Integrator.ar(Saw.ar(220), 0.9)) }],
        ["LeakDCar",   { Out.ar(0, LeakDC.ar(Saw.ar(220), 0.995)) }],
        ["Slewar",     { Out.ar(0, Slew.ar(Saw.ar(220), 1, 1)) }],
        ["Lagar",      { Out.ar(0, Lag.ar(Saw.ar(220), 0.01)) }],
        ["LagUDar",    { Out.ar(0, LagUD.ar(Saw.ar(220), 0.01, 0.05)) }],
        ["MoogFFar",   { Out.ar(0, MoogFF.ar(Saw.ar(220), 100, 2)) }],
        // BEQ suite: constant scalar params (the _ii/_kkk paths).
        ["BLowPassar", { Out.ar(0, BLowPass.ar(Saw.ar(220), 1000, 1)) }],
        ["BHiPassar",  { Out.ar(0, BHiPass.ar(Saw.ar(220), 1000, 1)) }],
        ["BPeakEQar",  { Out.ar(0, BPeakEQ.ar(Saw.ar(220), 1000, 1, 6)) }],
        ["BAllPassar", { Out.ar(0, BAllPass.ar(Saw.ar(220), 1000, 1)) }],
        ["BLowShelfar",{ Out.ar(0, BLowShelf.ar(Saw.ar(220), 500, 1, 6)) }],
        ["BHiShelfar", { Out.ar(0, BHiShelf.ar(Saw.ar(220), 5000, 1, 6)) }],

        // ---- T-D2 Task 7: SC3P filters ----
        ["SVFar",      { Out.ar(0, SVF.ar(Saw.ar(220), 2200, 0.1, 1, 0, 0, 0, 0)) }],
        ["MoogLadderar",{ Out.ar(0, MoogLadder.ar(Saw.ar(220), 440, 0.3)) }],
        // DFM1's upstream NoiseGen seeds xorshift from time(NULL) at ctor,
        // so the render epoch is captured into DFM1.seed.txt just before the
        // scsynth launch; the C++ port re-seeds from that same value.
        ["DFM1",       { Out.ar(0, DFM1.ar(Saw.ar(220), 1000, 0.1, 1, 0, 0.0003)) }],

        // ---- T-D2 Task 8: core delay family (shared ring buffer) ----
        ["DelayNar",   { Out.ar(0, DelayN.ar(Saw.ar(220), 0.01, 0.005)) }, "supernova"],
        ["DelayLar",   { Out.ar(0, DelayL.ar(Saw.ar(220), 0.01, 0.005)) }, "supernova"],
        ["DelayCar",   { Out.ar(0, DelayC.ar(Saw.ar(220), 0.01, 0.005)) }, "supernova"],
        ["CombNar",    { Out.ar(0, CombN.ar(Saw.ar(220), 0.01, 0.005, 0.1)) }, "supernova"],
        ["CombLar",    { Out.ar(0, CombL.ar(Saw.ar(220), 0.01, 0.005, 0.1)) }, "supernova"],
        ["CombCar",    { Out.ar(0, CombC.ar(Saw.ar(220), 0.01, 0.005, 0.1)) }, "supernova"],
        ["AllpassNar", { Out.ar(0, AllpassN.ar(Saw.ar(220), 0.01, 0.005, 0.1)) }, "supernova"],
        ["AllpassLar", { Out.ar(0, AllpassL.ar(Saw.ar(220), 0.01, 0.005, 0.1)) }, "supernova"],
        ["AllpassCar", { Out.ar(0, AllpassC.ar(Saw.ar(220), 0.01, 0.005, 0.1)) }, "supernova"],
        ["Pluckar",    { Out.ar(0, Pluck.ar(Saw.ar(220), Impulse.ar(4), 0.01, 0.005, 0.1, 0.5)) }, "supernova"],
        // Buf* delays read a 1024-frame LocalBuf (power-of-two SndBuf) instead
        // of owning a private ring; dsamp is clipped to bufSamples-1.
        ["BufDelayNar", { var b = LocalBuf(1024); Out.ar(0, BufDelayN.ar(b, Saw.ar(220), 0.005)) }, "supernova"],
        ["BufDelayLar", { var b = LocalBuf(1024); Out.ar(0, BufDelayL.ar(b, Saw.ar(220), 0.005)) }, "supernova"],
        ["BufDelayCar", { var b = LocalBuf(1024); Out.ar(0, BufDelayC.ar(b, Saw.ar(220), 0.005)) }, "supernova"],
        ["BufAllpassNar", { var b = LocalBuf(1024); Out.ar(0, BufAllpassN.ar(b, Saw.ar(220), 0.005, 0.1)) }, "supernova"],
        ["BufAllpassLar", { var b = LocalBuf(1024); Out.ar(0, BufAllpassL.ar(b, Saw.ar(220), 0.005, 0.1)) }, "supernova"],
        ["BufAllpassCar", { var b = LocalBuf(1024); Out.ar(0, BufAllpassC.ar(b, Saw.ar(220), 0.005, 0.1)) }, "supernova"],
        ["PitchShiftar", { Out.ar(0, PitchShift.ar(Saw.ar(220), 0.01, 1.5, 0.1, 0.001)) }, "supernova"],
        ["LocalIO",    { var fb, sig;
                         fb = LocalIn.ar(1);
                         sig = Saw.ar(220) + (fb * 0.5);
                         LocalOut.ar(sig * 0.5);
                         Out.ar(0, sig) }, "supernova"],

        // ---- T-D2 Task 9: reverbs ----
        ["FreeVerbar", { Out.ar(0, FreeVerb.ar(Saw.ar(220), 0.5, 0.5, 0.2)) }, "supernova"],
        ["GVerbar",    { Out.ar(0, GVerb.ar(Saw.ar(220))) }, "supernova"],

        // ---- T-D2 Task 10: pan family ----
        ["Pan2ar",     { Out.ar(0, Pan2.ar(Saw.ar(220), 0.3, 0.8)) }],
        ["Balance2ar", { Out.ar(0, Balance2.ar(Saw.ar(220), Saw.ar(330), 0.3, 0.8)) }],
        ["Rotate2ar",  { Out.ar(0, Rotate2.ar(Saw.ar(220), Saw.ar(330), 0.3)) }],
        ["Splayar",    { Out.ar(0, Splay.ar([Saw.ar(220), Saw.ar(330), Saw.ar(440)], 1, 0.8, 0, true)) }],
        ["PanAzar",    { Out.ar(0, PanAz.ar(4, Saw.ar(220), 0.3, 0.8, 2, 0.5)) }],
        ["SplayAzar",  { Out.ar(0, SplayAz.ar(4, [Saw.ar(220), Saw.ar(330), Saw.ar(440)], 1, 0.8, 2, 0, 0.5, true)) }],

        // ---- T-D2 Task 11: demand UGens (pulled by an impulse) ----
        ["DemandDseq",   { Out.ar(0, Demand.ar(Impulse.ar(10), 0, Dseq([1,2,3], inf))) }, "supernova"],
        ["DemandSeries", { Out.ar(0, Demand.ar(Impulse.ar(10), 0, Dseries(5, 2))) }, "supernova"],
        ["DemandMixed",  { Out.ar(0, Demand.ar(Impulse.ar(10), 0, Dseq([1,2,3],inf), Dseries(5,2))) }, "supernova"],
        ["DemandDrand",  { Out.ar(0, Demand.ar(Impulse.ar(10), 0, Drand([10,20,30], inf))) }, "supernova"],
        ["DemandDxrand", { Out.ar(0, Demand.ar(Impulse.ar(10), 0, Dxrand([1,2,3,4], inf))) }, "supernova"],
        ["DemandDbrown", { Out.ar(0, Demand.ar(Impulse.ar(10), 0, Dbrown(0, 1, 0.1))) }, "supernova"],

        // ---- T-D2 Task 12: buffer UGens ----
        ["SampleRate",  { Out.ar(0, K2A.ar(SampleRate.ir)) }],
        ["ControlRate", { Out.ar(0, K2A.ar(ControlRate.ir)) }],
        ["BufFrames",   { var b = LocalBuf(128); Out.ar(0, K2A.ar(BufFrames.kr(b))) }],
        ["BufRateScale",{ var b = LocalBuf(128); Out.ar(0, K2A.ar(BufRateScale.kr(b))) }],
        ["BufSampleRate",{ var b = LocalBuf(128); Out.ar(0, K2A.ar(BufSampleRate.kr(b))) }],
        ["BufWrRd1",    { var b = LocalBuf(128);
                          BufWr.ar(DC.ar(0.42), b, DC.ar(5), 0);
                          Out.ar(0, BufRd.ar(1, b, DC.ar(5), 0, 1)) }],
        // Pre-write every neighbour frame so interpolation never touches the
        // uninitialized LocalBuf memory (the actual NRT blocker).
        ["BufWrRd2",    { var b = LocalBuf(128);
                          [4,5,6].do { |k, i| BufWr.ar(DC.ar(0.1*(i+1)), b, DC.ar(k), 0) };
                          Out.ar(0, BufRd.ar(1, b, DC.ar(5), 0, 2)) }],
        ["BufWrRd4",    { var b = LocalBuf(128);
                          [4,5,6,7].do { |k, i| BufWr.ar(DC.ar(0.1*(i+1)), b, DC.ar(k), 0) };
                          Out.ar(0, BufRd.ar(1, b, DC.ar(5), 0, 4)) }],
        ["DemandBufRd", { var b = LocalBuf(8);
                          BufWr.ar(DC.ar(0.42), b, DC.ar(5), 0);
                          Out.ar(0, Demand.ar(Impulse.ar(10), 0, Dbufrd(b, 5, 0))) }],

        // ---- T-D2 Task 16: hardware constants (NRT global = 0) ----
        ["MouseXkr",   { Out.ar(0, K2A.ar(MouseX.kr(0, 1, 0, 0.2))) }],
        ["MouseYkr",   { Out.ar(0, K2A.ar(MouseY.kr(0, 1, 0, 0.2))) }],

        // ---- T-D2 Task 15: sugar expansion helpers ----
        ["SelectXar",  { Out.ar(0, SelectX.ar(0.3, [Saw.ar(220), Saw.ar(330)])) }],
        ["TChoosear",  { Out.ar(0, TChoose.ar(Impulse.ar(10), [DC.ar(1), DC.ar(2), DC.ar(3)])) }],

        // ---- T-D2: EnvGen / VarLag ----
        ["EnvGenkr",    { Out.ar(0, K2A.ar(EnvGen.kr(Env([0,1,0],[0.01,0.02]), Impulse.kr(2)))) }],

        // ---- T-D2 Task 14: SC3P dynamics ----
        ["Decimator",    { Out.ar(0, Decimator.ar(Saw.ar(220), 12000, 8)) }],
        ["CrossoverDis", { Out.ar(0, CrossoverDistortion.ar(Saw.ar(220), 0.5, 0.9)) }],
        ["SineShaper",   { Out.ar(0, SineShaper.ar(Saw.ar(220), 0.5)) }],
        ["DiodeRingMod", { Out.ar(0, DiodeRingMod.ar(Saw.ar(220), SinOsc.ar(330))) }],
        ["InsideOut",    { Out.ar(0, InsideOut.ar(Saw.ar(220))) }],

        // ---- Task 9 cont.: GreyholeRaw (faust, separate header) ----
        // GreyholeRaw needs scsynth (supernova cannot load the UGen at all);
        // the Faust plugin segfaults at SHUTDOWN after writing a complete wav.
        ["GreyholeRaw", { Out.ar(0, GreyholeRaw.ar(Saw.ar(220), Saw.ar(330), 0, 2, 0.5, 0.9, 0.1, 2, 1)) }],

        // ---- T-D2 dynamics family (core: Amplitude/Compander/Limiter) ----
        ["Amplitude",  { Out.ar(0, Amplitude.ar(Saw.ar(220), 0.01, 0.1)) }],
        ["Compander",  { Out.ar(0, Compander.ar(Saw.ar(220), Saw.ar(1), 0.5, 2, 1.5, 0.01, 0.1)) }],
        // Limiter/Normalizer segfault scsynth 3.14.1-6 (rc 139, any graph);
        // supernova renders the same FilterUGens.cpp law cleanly and
        // deterministically. Third tuple element overrides the engine.
        ["Limiter",    { Out.ar(0, Limiter.ar(Saw.ar(220) * 2, 1, 0.01)) }, "supernova"],

        // ---- T-D2: mapping UGens (LinExp core, LinLin classlib mul-add) ----
        // The drones' filter-sweep argument shapes (Hecker's LinExp.kr(lfo,-1,1,
        // 20,20000); brief-canonical LinLin.kr(in,0,1,0.8,0.33)). The UGen
        // classes take NO clip argument — clip=\minmax is the language-side
        // SimpleNumber default and does not apply to these graphs.
        ["LinExpkr",   { Out.ar(0, K2A.ar(LinExp.kr(SinOsc.kr(5), -1, 1, 20, 20000)).dup) }],
        ["LinExpar",   { Out.ar(0, LinExp.ar(SinOsc.ar(440), -1, 1, 20, 20000)).dup }],
        ["LinLinkr",   { Out.ar(0, K2A.ar(LinLin.kr(SinOsc.kr(5), 0, 1, 0.8, 0.33)).dup) }],
        ["LinLinar",   { Out.ar(0, LinLin.ar(SinOsc.ar(440), 0, 1, 0.8, 0.33)).dup }],
    ];

    "ugen_ref: begin".postln;
    if (File.exists(outDir).not) { File.mkdir(outDir) };

    entries.do { |pair|
        var name, body, engine, def, score, opts, oscPath, wavPath, cmd, rc;
        name = pair[0];
        body = pair[1];
        engine = if (pair.size > 2, { pair[2] }, { Score.program });
        def = SynthDef("ref_" ++ name, {
            RandSeed.kr(1, 16rC0FFEE);
            body.value();
        });
        wavPath = outDir ++ "/" ++ name ++ ".expected.wav";
        score = Score.new;
        opts = ServerOptions.new.numOutputBusChannels_(2).numInputBusChannels_(0);
        oscPath = outDir ++ "/" ++ name ++ ".osc";
        if (File.exists(wavPath).and { force.not }) {
            nil
        } {
            score.add([0, [\d_recv, def.asBytes,
                           [\s_new, "ref_" ++ name, 1000, 0, 0]]]);
            score.add([dur, [0]]);
            score.writeOSCFile(oscPath, 0, dur);
            cmd = engine ++ " -N " ++ oscPath.quote ++ " _ " ++ wavPath.quote
                ++ " 48000 WAV float" ++ opts.asOptionsString
                ++ (if (scsynthFlags != "", { " " ++ scsynthFlags }, { "" }));
            if (name == "DFM1") {
                // Keep retrying until the whole render stays inside one epoch
                // second: then the captured value must equal scsynth's
                // time(NULL) at DFM1_Ctor (no boundary crossed mid-launch).
                var before, after, stable = false, okrc, same;
                5.do {
                    if (stable.not) {
                        var f;
                        before = Date.getDate.rawSeconds.asInteger;
                        f = File(outDir ++ "/DFM1.seed.txt", "w");
                        f.write(before.asString); f.close;
                        rc = cmd.systemCmd;
                        after = Date.getDate.rawSeconds.asInteger;
                        okrc = (rc == 0);
                        same = (before == after);
                        stable = okrc.and(same);
                        if (stable.not) {
                            "DFM1: render crossed an epoch boundary, retrying".postln;
                        };
                    };
                };
                if (stable.not) { Error("DFM1 epoch never stable").throw };
            } {
                rc = cmd.systemCmd;
                if (name == "GreyholeRaw" && rc != 0) {
                    ("GreyholeRaw: scsynth exit rc=" ++ rc ++ " after writing wav").postln;
                    rc = 0;
                };
                if (rc != 0) { Error("scsynth failed for " ++ name).throw };
            };
            File.delete(oscPath);
        };
    };
    "ugen_ref: done".postln;
}.value(thisProcess.nowExecutingPath);
0.exit
