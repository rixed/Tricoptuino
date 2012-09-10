Dumbest Tricopter Ever
======================

The idea is to build an autonomous flying tricopter build around
cheap and easily available parts (arduino + friends).

The software stack is cracked from scratch on top of GNU's AVR libc
(because I find it much easier to work with standard GNU tools than
with a custom IDE).

make -C lib && make tricopter.up should build and load all the required
software (once tricopter.c is written :-p)

