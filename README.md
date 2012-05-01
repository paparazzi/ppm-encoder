ppm-encoder
===========

PPM Encoder board: http://paparazzi.enac.fr/wiki/ATmega_PPM_Encoder_Board

Author: hendrix (Chris Efstathiou)

Comments:  This software is FREE. Use it at YOUR OWN RISK.

PPM Encoder Hardware
--------------------
  - There are currently three revisions of the PPM Encoder board based around the ATMega168 mcu
  - The DIYDrones/3DRobotics version is an additional and different fourth hardware version based on this work
  - The code is/should be compatible with other ATMega8 series mcus, (known to work with the ATMega328)
  - The first board revision uses the internal RC oscillator of the mcu; this can cause timing accuracy issues
  - Later versions use an external 16MHz crystal/oscillator and provide cleaner, more stable output
  - Setting the fuses correctly for your hardware design is very important!

PPM Encoder Software
--------------------
  - The software is user configurable in the servo2ppm_settings.h file.
  - The software must be configured for the correct clock frequency and mcu in the makefile (or possibly the settings file).
  - The compiler for the software is avr-gcc.
  - Ideally, the software should be compiled using the version of avr-gcc included with WinAVR 2007 build 1221, available for download on sourceforge: http://sourceforge.net/projects/winavr/files/WinAVR/20071221/
  - Since this is windows software, you can run it on a windows box, a windows virtual machine or in wine (which seems to work fine). If using wine, call make with "wine" as a prefix: $ wine make (arguments).
  - The software can be easily loaded with avrdude using a supported programmer. Depending on which version you have, you may need to update avrdude.conf to include your mcu (i.e. if you have a 328).
  - There is a bootloader available if no programmer is available (but the bootloader will need to be loaded once with a programmer prior to use).
  - The fuses on your mcu must be carefully set. This can be done with avrdude. See below for correct fuse settings.

Fuse Settings
-------------
  - These settings may not work in all cases, depending on your board.
  - VERIFY BEFORE CHANGING FUSES ON YOUR MCU, OR YOU MAY BRICK IT! (especially the low fuse)
  - An online avr fuse calculator may help: http://www.engbedded.com/fusecalc
  - Low fuse has the same functions for the mega 328 and mega 168
  - High fuse sets sets the bootsize on 328 and the bodlevel in 168
  - Extended fuse sets the bodlevel only on 328 but it sets the bootsize on 168i
  - If you aren't using the bootloader, the fuses should still work fine, though feel free to disable the bootloader reset vector

**Low Fuse (same for 168 and 328, with or without bootloader):**
  - **0xDF** : 16Mhz External Oscillator (typically 0xFF on DIYDrones board)
  - **0xC2** : 8Mhz Internal RC Oscillator

**High Fuse:**
  - **0xDC** : for 168
  - **0xD8** : for 328

**Extended Fuse:**
  - **0xF8** : for 168
  - **0xFC** : for 328

**Note:** some numerical values refer to fuses containing undefined bits (set to '1' here). Depending on the target device these fuse bits will be read either as '0' or '1'. Verification errors will occur if the values are read back with undefined bits set to '0'.
 - For the 168: Everything is fine if the values read from the device are either the same as programmed, or the following values (undefined set to '0'): Extended: 0x00.
 - For the 328: Everything is fine if the values read from the device are either the same as programmed, or the following values (undefined set to '0'): Extended: 0x04

(Above note from http://www.engbedded.com/fusecalc)

Bootloader
----------
 - The bootloader is found in sw/bootloader.
 - To compile the bootloader, edit the MCU and F_CPU part of the makefile to match your hardware.
 - The memory offset will be automatically selected and assumes the fuses are set as described above
 - To compile the bootloader after editing the makefile:

    $ make


PPM Encoder
-----------
 - The ppm encoder source is found in sw/ppm_encoder.
 - Before compiling, edit the makefile for the appropriate clock frequency to match your hardware. You may also need to specify the path to the compiled bootloader .hex if you are trying to load the bootloader at the same time as the ppm encoder program.
 - To compile for the atmega328p (the makefile defaults to the 328):

    $ make

 - To compile for the atmega168:

    $ make mcu=atmega168


Uploading the Code
------------------

 - To upload the code to your board with a programmer, either directly call avrdude, or adjust the parameters in the makefile and call make program:

    $ avrdude -C /PATH/TO/ALTERNATE/avrdude.conf -c YOUR_PROGRAMMER_NAME -P YOUR_PROG_PORT -p YOUR_MCU -U flash:w:ppm_encoder_v4_3.hex:i

 - Check the avrdude help for more details and for how to check and write fuses (be careful). The makefile in sw/ppm_encoder can also be used to set fuses with make fuse_read to check and make fuse_write to set (make sure MCU and F_CPU are set correctly). The alternate avrdude.conf file might be needed if your version of avrdude does not have your mcu type defined (i.e. macports avrdude can direct to the Arduino 1.0 avrdude.conf if needed)

Some more example commands would be nice!
