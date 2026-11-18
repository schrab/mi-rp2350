// gen_goldens.scd — render the T-D1 trio through the SynthSocket wrap to
// float32 WAVs via NRT scsynth. Schedule matches the C++ harness exactly:
// voice at t = 0 (block 0), gate off at t = 3.0 s (block 2250), 4.0 s total.
// Rendering is SYNCHRONOUS (Score:recordNRT uses unixCmd and would race the
// file copy): write the OSC score, then run scsynth -N with systemCmd.
//
// Headless-sclang constraints (3.14.1, plain sclang, no IDE):
// - the body is a Function called with .value; a top-level ( ... ) group is
//   NOT executed by plain sclang (it just constructs the function and hangs).
// - every `var` must be declared before any statement (a late var is a
//   parse error, and headless sclang hangs instead of reporting it).
{ |scriptPath|
    var seed = 16rC0FFEE;
    var drones = ["Sine", "Square", "Triangle"];
    var dur = 4.0, gateOff = 3.0;
    var attack = 1.0, release = 1.0, hz = 55.0, amp = 0.4;
    var dir = PathName(scriptPath).pathOnly;
    var outDir = dir ++ "goldens";
    // scsynth -N loads UGen plugins from its compile-time prefix; a
    // user-local SC root (see run_goldens.sh) must pass -U <plugindir>.
    // Unset on a standard install -> no extra flag.
    var scsynthFlags = ("DC_SCSYNTH_FLAGS").getenv ?? "";

    "renderer: begin".postln;
    thisThread.randSeed_(seed);           // sclang-side RNG (graph construction)
    if (File.exists(outDir).not) { File.mkdir(outDir) };

    drones.do { |name|
        var fn, def, score, opts, oscPath, wavPath, cmd, rc;
        fn = (dir ++ "drones/" ++ name ++ ".scd").load;   // a Function
        def = SynthDef("dc_" ++ name, {
            |hz = 55.0, amp = 1.0, out = 0, gate = 1, attack = 1.0, release = 1.0|
            var snd, aenv;
            // Server RGen seed (the world RGen is time-seeded; see SC_World.cpp).
            // RandSeed's ctor triggers once, re-seeding the RGen at synth start.
            // The C++ engine calls sc::RGen::init(sc::kGoldenSeed) with this value.
            RandSeed.kr(1, 16rC0FFEE);
            snd = fn.value(K2A.ar(hz), K2A.ar(amp), 0);       // bufnum 0: no tape buffer
            aenv = EnvGen.kr(Env.asr(attack, 1, release), gate, doneAction: 2);
            Out.ar(out, snd * aenv);
        });
        score = Score.new;
        opts = ServerOptions.new.numOutputBusChannels_(2).numInputBusChannels_(0);
        oscPath = outDir ++ "/" ++ name ++ ".osc";
        wavPath = outDir ++ "/" ++ name ++ ".expected.wav";

        // ONE bundle at t = 0. The synth starts via d_recv's COMPLETION
        // message: a bare [\d_recv, bytes] followed by [\s_new, ...] in the
        // same bundle races in scsynth (the def registers on another thread
        // — "SynthDef not found" + a silent render). The completion message
        // runs after the def is registered, in order, still in block 0.
        score.add([0, [\d_recv, def.asBytes,
                       [\s_new, "dc_" ++ name, 1000, 0, 0, \hz, hz, \amp, amp]]]);
        score.add([gateOff, [\n_set, 1000, \gate, 0]]);
        score.add([dur, [0]]);                     // score-length marker
        score.writeOSCFile(oscPath, 0, dur);

        // Synchronous render (recordNRT's unixCmd is async — do not use it here).
        // Command fields mirror Score:recordNRT exactly.
        cmd = Score.program ++ " -N " ++ oscPath.quote ++ " _ " ++ wavPath.quote
            ++ " 48000 WAV float" ++ opts.asOptionsString
            ++ (if (scsynthFlags != "", { " " ++ scsynthFlags }, { "" }));
        rc = cmd.systemCmd;
        ("rendered " ++ name ++ " rc=" ++ rc).postln;
        if (rc != 0) { Error("scsynth failed for " ++ name).throw };
        File.delete(oscPath);
    };
    "renderer: done".postln;
}.value(thisProcess.nowExecutingPath);
0.exit
