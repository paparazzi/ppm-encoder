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
  - The software must be configured for the correct clock frequency in the makefile (or possibly the settings file).
  - The software should be compiled using the version of avr-gcc included with WinAVR 2007 build 1221, available for download on sourceforge: http://sourceforge.net/projects/winavr/files/WinAVR/20071221/
  - Since this is windows software, you can run it on a windows box, a windows virtual machine or in wine (which seems to work fine). If using wine, call make with: $ wine make (arguments).
  - The software can be easily loaded with avrdude using a supported programmer. Depending on which version you have, you may need to update avrdude.conf to include your mcu (i.e. if you have a 328).
  - There is a bootloader available if no programmer is available (but the bootloader will need to be loaded once with a programmer prior to use).
  - Even though you can change the target mcu in the makefile, it should be left at atmega168 even if you have a 328.
  - Exception: Make sure your bootloader gets put into the right part of memory to correspond to the bootloader reset vector (if using a bootloader)
  - The fuses on your mcu must be carefully set. This can be done with avrdude. See below for correct fuse settings.

Fuse Settings
-------------
  - These settings may not work in all cases, depending on your board.
  - VERIFY BEFORE CHANGING FUSES ON YOUR MCU, OR YOU MAY BRICK IT! (especially the low fuse)
  - An online avr fuse calculator may help: http://www.engbedded.com/fusecalc
  - Low fuse has the same functions for the mega 328 and mega 168
  - High fuse sets sets the bootsize on 328 and the bodlevel in 168
  - Extended fuse sets the bodlevel only on 328 but it sets the bootsize on 168

 Low Fuse (same for 168 and 328, with or without bootloader):
  - **0xDF** : 16Mhz External Oscillator (typically 0xFF on DIYDrones board)
  - **0xC2** : 8Mhz Internal RC Oscillator

 High Fuse:
  - **0xDC** : for 168
  - **0xD8** : for 328 WITH bootloader
  - **0xDF** : for 328 NO bootloader

 Extended Fuse:
  - **0xF8** : for 168 WITH bootloader
  - **0xFF** : for 168 NO bootloader
  - **0xFC** : for 328

**Note** that some numerical values refer to fuses containing undefined bits (set to '1' here). Depending on the target device these fuse bits will be read either as '0' or '1'. Verification errors will occur if the values are read back with undefined bits set to '0'.
For the 168: Everything is fine if the values read from the device are either the same as programmed, or the following values (undefined set to '0'): Extended: 0x00.
For the 328: Everything is fine if the values read from the device are either the same as programmed, or the following values (undefined set to '0'): Extended: 0x04
(Above note from http://www.engbedded.com/fusecalc)

Sample Commands
---------------
-To compile the code for an ATMega168 or ATMega328 board that has a 16MHz crystal with default settings:
    $ cd your_ppm-encoder_directory/sw/ppm_encoder
    $ wine make --file=makefile.16Mhz
-If working on windows, drop wine
-To upload the code to your board with a programmer, either directly call avrdude, or adjust the parameters in the makefile and call make program:
    $ avrdude -C /PATH/TO/ALTERNATE/avrdude.conf -c YOUR_PROGRAMMER_NAME -P YOUR_PROG_PORT -p YOUR_MCU -U flash:w:ppm_encoder_v4_3.hex:i
-Check the avrdude help for more details and for how to check and write fuses (be careful). The alternate avrdude.conf file might be needed if your version of avrdude does not have your mcu type defined (i.e. macports avrdude can direct to the Arduino 1.0 avrdude.conf if needed)
