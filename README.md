# Channel DSP
Channel DSP is a modular digital mixer, allowing for multiple 48Khz 24-bit stero audio channels to be summed into one or more output channels.  Each channel has a digital dynamic compressor, gate, and ability to route signals to the main trunk, and to two sends.  Each channel is managed by two RP2400s - one for performing the digital signal processing with the CODEC and VCAs, and one to drive the user interface elements: buttons, motor controlled slider, rotary controller, display and communications with the master controller.

The audio_processor directory contains the firmware for the digital signal processing, and the controller contains the non-audio based I/O elements.

