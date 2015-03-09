# EuclideanSampler
Arduino sketch for the nw2s:b that does sampling, playback, and sequencing using euclidean rhythms.

Currently it can sample two short (0.7sec, enough for drum hits) clips using the Arduino ADC, sampled at about 12-bit 30KHz.  You can load/save them to the SD card so they persist across resets.  The are loaded from the SD card at startup.

Since the quality of the ADC sampling is a little noisy, you can put your own samples on the SD card.  Currently it's stored in one file called out.wav that has no header, then 20,000 16-bit unsigned samples for sample A, then 20,000 16-bit unsigned samples for sample B.  You can load from Audacity using Import -> Raw Data.

![alt tag](https://raw.github.com/gattis/EuclideanSampler/master/euclidean_sampler.png)

You can patch the euclidean trig outs into the playback trig ins, or use the euclidean trig outs for external modules, or use the playback trig ins from an external module.
