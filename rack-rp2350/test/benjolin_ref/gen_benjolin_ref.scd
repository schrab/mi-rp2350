// gen_benjolin_ref.scd — full-\benjolis-SynthDef NRT golden renders.
// The SynthDef body is COPIED VERBATIM from the upstream norns engine
// (scazan/benjolis, lib/Engine_Benjolis.sc:25-89) — do not edit, rename, or
// "fix" anything in it (including the degenerate sh0 comparator and the
// comment-adjacent code; the golden encodes the shipped graph).
//
// NO RandSeed pin and NO DFM1 epoch sidecar are needed: the graph contains
// zero random UGens (LFTri/PulseDPW/DelayN/Latch/LocalIn are all
// deterministic), and DFM1 — whose ctor seeds xorshift from time(NULL) — is
// instantiated but its output is discarded by Select (filterType=0 selects
// RLPF in both scenarios), so its noise never reaches the output.
//
// Headless-sclang constraints (see gen_ugen_ref.scd): body = called Function,
// vars first, no top-level ( ... ) group.
{ |scriptPath|
    var dir = PathName(scriptPath).pathOnly;
    var outDir = dir ++ "goldens";
    var dur = 1.0;
    var force = ("DC_FORCE").getenv == "1";
    var scsynthFlags = ("DC_SCSYNTH_FLAGS").getenv ?? "";
    var def, scenarios;

    scenarios = [
        // name, s_new argument pairs (SynthDef defaults otherwise)
        ["default", [\amp, 0.5]],
        ["loop1", [\amp, 0.5, \loop, 0.9, \rungler1, 0.3, \freq2, 8]],
        ["loop1sh0", [\amp, 0.5, \loop, 0.9, \rungler1, 0.3, \freq2, 8, \outSignal, 5]],
        // bisect taps (Select only changes the monitored branch)
        ["tri1",  [\amp, 0.5, \outSignal, 0]],
        ["osc1m", [\amp, 0.5, \outSignal, 1]],
        ["osc2s", [\amp, 0.5, \outSignal, 3]],
        ["pwmS",  [\amp, 0.5, \outSignal, 4]],
        ["sh0S",  [\amp, 0.5, \outSignal, 5]],
    ];

    "benjolin_ref: begin".postln;
    if (File.exists(outDir).not) { File.mkdir(outDir) };

    def = SynthDef.new(\benjolis, {
        | out, freq1= 40, freq2=4, scale=1, rungler1=0.16, rungler2=0.0, runglerFilt=9, loop=0, filtFreq=40, q=0.82, gain=1, filterType=0, outSignal=6, amp=0, pan=0|
        var osc1, osc2, tri1, tri2, sh0, sh1, sh2, sh3, sh4, sh5, sh6, sh7, sh8=1, rungler, pwm, filt, output;
        var sr;
        var osc2freq, buf, bufR;

        // RENDERER FIX (deviation from verbatim, documented): NRT does not
        // zero the synth's local audio bus, so this block-0 read returns
        // garbage -> midicps(garbage) drives the FM insane for one block and
        // the chaotic loop decorrelates permanently. The Select gate (NOT a
        // kr multiply: ar BinaryOps CALCSLOPE-ramp kr inputs) zeroes exactly
        // block 0, matching the kit's zeroed LocalBus / the hardware's
        // power-on state. Everything else stays verbatim.
        bufR = LocalIn.ar(2,0) * Select.ar(Sweep.kr > 0, [DC.ar(0), DC.ar(1)]);
        rungler = bufR.at(0);
        buf = bufR.at(1);

        sr = SampleDur.ir;
        //sr = ControlDur.ir;
        tri1 = LFTri.ar((rungler*rungler1)+freq1);
        tri2 = LFTri.ar((rungler*rungler2)+freq2);
        osc1 = PulseDPW.ar((rungler*rungler1)+freq1);
        osc2 = PulseDPW.ar((rungler*rungler2)+freq2);

        //pwm = tri1 > tri2;
        pwm = BinaryOpUGen('>', (tri1 + tri2),(0));

        osc1 = ((buf*loop)+(osc1* (loop* -1 +1)));
        sh0 = BinaryOpUGen('>', osc1, 0.5);
        sh0 = BinaryOpUGen('==', (sh8 > sh0), (sh8 < sh0));
        sh0 = (sh0 * -1) + 1;

        sh1 = DelayN.ar(Latch.ar(sh0,osc2),0.01,sr);
        sh2 = DelayN.ar(Latch.ar(sh1,osc2),0.01,sr*2);
        sh3 = DelayN.ar(Latch.ar(sh2,osc2),0.01,sr*3);
        sh4 = DelayN.ar(Latch.ar(sh3,osc2),0.01,sr*4);
        sh5 = DelayN.ar(Latch.ar(sh4,osc2),0.01,sr*5);
        sh6 = DelayN.ar(Latch.ar(sh5,osc2),0.01,sr*6);
        sh7 = DelayN.ar(Latch.ar(sh6,osc2),0.01,sr*7);
        sh8 = DelayN.ar(Latch.ar(sh7,osc2),0.01,sr*8);

        //rungler = ((sh6/8)+(sh7/4)+(sh8/2)); //original circuit
        //rungler = ((sh5/16)+(sh6/8)+(sh7/4)+(sh8/2));

        rungler = ((sh1/2.pow(8))+(sh2/2.pow(7))+(sh3/2.pow(6))+(sh4/2.pow(5))+(sh5/2.pow(4))+(sh6/2.pow(3))+(sh7/2.pow(2))+(sh8/2.pow(1)));

        buf = rungler;
        rungler = (rungler * scale.linlin(0,1,0,127));
        rungler = rungler.midicps;

        LocalOut.ar([rungler,buf]);



        filt = Select.ar(filterType, [
            RLPF.ar(pwm,(rungler*runglerFilt)+filtFreq, q* -1 +1,gain),
            //BMoog.ar(pwm,(rungler*runglerFilt)+filtFreq, q,0,gain),
            RHPF.ar(pwm,(rungler*runglerFilt)+filtFreq, q* -1 +1,gain),
            SVF.ar(pwm,(rungler*runglerFilt)+filtFreq, q, 1,0,0,0,0,gain),
            DFM1.ar(pwm,(rungler*runglerFilt)+filtFreq, q, gain,1)
        ]);


        output = Select.ar(outSignal, [
            tri1, osc1, tri2, osc2, pwm, sh0, filt
        ]);


        output = (output * amp).tanh;
        output = [DelayL.ar(output, delaytime: pan.clip(0,1).lag(0.1)), DelayL.ar(output, delaytime: (pan.clip(-1,0) * -1).lag(0.1))];
        Out.ar(out, LeakDC.ar(output));
    });

    scenarios.do { |pair|
        var name, args, score, opts, oscPath, wavPath, cmd, rc;
        name = pair[0];
        args = pair[1];
        wavPath = outDir ++ "/" ++ name ++ ".expected.wav";
        score = Score.new;
        opts = ServerOptions.new.numOutputBusChannels_(2).numInputBusChannels_(0);
        oscPath = outDir ++ "/" ++ name ++ ".osc";
        if (File.exists(wavPath).and { force.not }) {
            nil
        } {
            score.add([0, [\d_recv, def.asBytes,
                           [\s_new, \benjolis, 1000, 0, 0] ++ args]]);
            score.add([dur, [0]]);
            score.writeOSCFile(oscPath, 0, dur);
            cmd = Score.program ++ " -N " ++ oscPath.quote ++ " _ "
                ++ wavPath.quote ++ " 48000 WAV float"
                ++ opts.asOptionsString
                ++ (if (scsynthFlags != "", { " " ++ scsynthFlags }, { "" }));
            rc = cmd.systemCmd;
            if (rc != 0) {
                // scsynth 3.14.1-6 can exit nonzero AFTER writing a complete
                // wav (the known shutdown crash) — only fail if no wav.
                if (File.exists(wavPath).not) {
                    Error("scsynth failed for " ++ name).throw;
                };
                ("benjolin_ref: " ++ name ++ " rc=" ++ rc
                    ++ " after writing wav").postln;
            };
            File.delete(oscPath);
        };
    };
    "benjolin_ref: done".postln;
}.value(thisProcess.nowExecutingPath);
0.exit
