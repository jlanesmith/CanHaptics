NetAddr.langPort;


(
OSCdef.new(
	\anything,
	{
		arg msg, time, addr, port;
		~x.do{arg synth; synth.set(\freq, msg[1]);};
	},
	'/hapstrument'
);
)]

s.boot;

SynthDef.new(\tone, {|freq = 220, out| Out.ar(out, SinOsc.ar(freq, 0, 0.2)) }).add;

~x = Synth.new(\tone);


x.free;
s.freeAll;
