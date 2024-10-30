# ECSE-444-Labs

Lab1:
  - Built single-variable Kalman-Filter using ARM Assembly Language, C and CMSIS-DSP library.
  - Checked the derived results using functions like subtraction, mean, standard-deviation, convolution and correlation.
  - Checked time efficiency and optimization using SWV trace log implemented in STM32CubeIDE.

Lab2:
  - Built a system that switches the board from reading the voltage to reading the temperature using the ADC value, and identifying the switched channel with LED lights on the board with GPIO pins.
  - Created helper functions like READ_TEMP() and READ_VREF() that switches the board to read temperature and read the temperature using ADC values and manually doing the conversion, and vice versa for reference Voltage.

Lab3:
  - Part 1:
     - Generate triangle wave, sawtooth wave, and sine wave to see it on SWV Data Trace Timeline Graph on STM32CubeIDE.
     - Use 'HAL_DAC_SetValue()' to hear the wave sounds from the speaker attached on the board (sine wave should make the clearest sound).
