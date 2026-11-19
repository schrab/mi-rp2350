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
    ];

    "ugen_ref: begin".postln;
    if (File.exists(outDir).not) { File.mkdir(outDir) };

    entries.do { |pair|
        var name, body, def, score, opts, oscPath, wavPath, cmd, rc;
        name = pair[0];
        body = pair[1];
        def = SynthDef("ref_" ++ name, {
            RandSeed.kr(1, 16rC0FFEE);
            body.value();
        });
        score = Score.new;
        opts = ServerOptions.new.numOutputBusChannels_(2).numInputBusChannels_(0);
        oscPath = outDir ++ "/" ++ name ++ ".osc";
        wavPath = outDir ++ "/" ++ name ++ ".expected.wav";
        score.add([0, [\d_recv, def.asBytes,
                       [\s_new, "ref_" ++ name, 1000, 0, 0]]]);
        score.add([dur, [0]]);
        score.writeOSCFile(oscPath, 0, dur);
        cmd = Score.program ++ " -N " ++ oscPath.quote ++ " _ " ++ wavPath.quote
            ++ " 48000 WAV float" ++ opts.asOptionsString;
        rc = cmd.systemCmd;
        if (rc != 0) { Error("scsynth failed for " ++ name).throw };
        File.delete(oscPath);
    };
    "ugen_ref: done".postln;
}.value(thisProcess.nowExecutingPath);
0.exit
