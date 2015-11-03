## Sound Reactive LED Strips

I set this up for my buddies at the Embassy, a camp at Burning Man 2015. We strung 5 sets of [5 meter LED strips](http://www.amazon.com/gp/product/B00B2F3KDQ) off of PVC ribs on our largest quonset.

## Parts and Hardware

### Power Board
I used [this tutorial](https://learn.adafruit.com/rgb-led-strips/usage) to develop a power board. I was able to run about 15A of average current through 3 NPN transistors. I then strung 5 strands of 14 gauge wire from each transistor as well as 5 strands of wire from the power output of [a "30A" AC to DC power supply](http://www.amazon.com/SUPERNIGHT-Switch-Power-Supply-Switching/dp/B00ADGMGEO) running from a generator.

### Sound Detection
I used the [Sparkfun Sound Detector](https://www.sparkfun.com/products/12642) to pickup the signal from which I used the straight audio signal as the analog input to the Arduino.

### Signal Processing and Color Computation

An Arduino ATMega2560 was used as the development platform. I used a 9V battery as the power source in order to reduce the chatter in the signal that would result from using the power output from a DC power supply.

## Algorithm Design

The signal is run through an [FHT](http://wiki.openmusiclabs.com/wiki/ArduinoFHT), a cousin of the FFT, and then low pass filtered and used along with user input to create a color in hue, saturation, and value color space.

## Thanks

The original code was modified from a [tutorial](https://bochovj.wordpress.com/2013/09/14/lightbox-a-vj-oriented-leds-controller/) by Sario Salvi. This code had an Artistic License 2.0, which is referenced below.

I combined this code with some original work from myself and some embedded wizardry from [this tutorial](http://vtchl.uiuc.edu/node/557) by Ven Te Chow Hydrosystems Lab at the University of Illinois.

## License

This code is free for use by anyone that so desires to use it for anything that they would like as long as it follows the original restrictions set forth in the [Artistic License 2.0](http://opensource.org/licenses/artistic-license-2.0) that I modified the original code from. 

In addition, below is the copyright included with the use of the FHT for frequency analysis from which I based most of the Arduino ATMega2560 code on.

This code uses the [Arduino FHT library from OpenMusicLabs](http://wiki.openmusiclabs.com/wiki/ArduinoFHT).  That library is copyright by it's respective owner.  The code here is also includes portions from the [AnalogIsrLogger program](http://code.google.com/p/beta-lib/downloads/list). These libraries are copyright by their respective owners.  No license was provided.

For portions of the code that are not obtained from the aforementioned sources, the following University of Illinois/NCSA license (a permissive license based on the MIT/X11 license and the 3-clause BSD license).

Copyright (c) 2012 University of Illinois

All rights reserved.
 
Developed by: [Ven Te Chow Hydrosystems Lab, University of Illinois](http://vtchl.illinois.edu)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal with the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 
- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimers.

- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimers in the documentation and/or other materials provided with the distribution.

- Neither the names of Ven Te Chow Hydrosystems Lab or the University of Illinois, nor the names of its contributors may be used to endorse or promote products derived from this Software without specific prior written permission. 

 
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE SOFTWARE.
