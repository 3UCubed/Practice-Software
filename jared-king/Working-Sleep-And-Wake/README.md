### These are the steps to use stop mode:
  1. Connect an STM32F051 to your machine with a USB cable. Also plug in a UART cable to your machine.
  2. Plug the yellow cable (RX) from UART to PA9. Plug the orange cable (TX) from UART to PA10. Plug in the black cable (ground) to GND.
  3. Open "Firmware" with the STM32CubeIDE, and run main.c.
  4. Use `screen -L /dev/cu.usbserial-FT61T5FW 115200` to see output and be able to input. (You may need to change characters after "usbserial-". If the FTDI is plugged in, tab to autocomplete the name)
  5. If you would like to put it in Stop Mode, type "s" in the terminal. To then wake it, type "£" in the terminal.
