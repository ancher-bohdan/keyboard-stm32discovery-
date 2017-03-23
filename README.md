# keyboard-stm32discovery-
The driver for operationg with matrix keyboard 3x4 using FreeRTOS. Also adding a driver for reading iButton key from UART reader.

Each row connected to some pin. This pins was configurated as simple GPIO output. At any particular time the only one of these pin
must be set to 1. For every update event for some timer this value changed. The frequency of this timer directly define the frequency
of reading keyboard.

The columns are connecting to timer. This timer are configurating for trigering from input. Therefore the value of counter for particular
timer increment, when input is pullet up.
