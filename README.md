The hardware used for this project (LocoNet_MarklinGndDetection) is a 'LocoNet board v2.3 from Geert Giebens'.

This project is build on the LocoNet driver software for the PIC18F24/25/26/27/45/46/47Q10 microcontroller family.
The code is written in C and is compatible to the "[LocoNet Personal Use Edition 1.0 SPECIFICATION](https://www.digitrax.com/static/apps/cms/media/documents/loconet/loconetpersonaledition.pdf)" from DigiTrax Inc.

The LocoNet driver uses the EUSART 1 and the Timer 1, both with a low priority interrupt.

The following hardware pins on the microcontroller are used:
 - RA3: comparator 1, non-inverting input (C1IN+)
 - RA4: comparator 1, output (C1OUT)
 - RC6: LocoNet transmitter (EUSART 1, TXD)
 - RC7: LocoNet receiver (EUSART 1, RXD)

The pins RA5 is used as indication LED: data trafic on LocoNet

The locoNet driver is built in the files: ln.h, ln.c, circular_queue.h and circular_queue.c
Include this library (files) into your (LocoNet) project.
 - To transmit a LocoNet message, the function lnTxMessageHandler(lnMessage*) can be invoked.
 - To receive a LocoNet message, a lnRxMessageHandler(lnMessage*) callback function must be included.
