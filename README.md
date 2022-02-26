# mitsubishi-encoder
 STM32 program to read position from mitsubishi servomotor from J2/J2S family.
 J2 series motors have 13-bit resolution (8192/rotation), J2S have 17-bit (131072/revolution).
 Written and tested on STM32F401CCU (blackpill board) and CubeIDE.
 On start, motor model is identified, but my method may not be accurate because I have no documentation of protocol used.
 Position is read and processed inside timer4 ISR (18kHz) together with error detection (communication/excess speed)
 Main function loop draws info on Nokia 5110 LCD module with SPI. Graphics library is not my work.
