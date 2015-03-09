# EuclideanSampler
Arduino sketch for the nw2s:b that does sampling, playback, and sequencing using euclidean rhythms.

Currently it can sample two short (0.7sec, enough for drum hits) clips using the Arduino ADC, sampled at about 12-bit 30KHz.  You can load/save them to the SD card so they persist across resets.  The are loaded from the SD card at startup.

![alt tag](https://raw.github.com/gattis/EuclideanSampler/master/euclidean_sampler.png)

You can patch the euclidean trig outs into the playback trig ins, or use the euclidean trig outs for external modules, or use the playback trig ins from an external module.
