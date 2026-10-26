From this directory, with esptool installed:
python -m esptool --chip esp32s3 --port YOUR_SERIAL_PORT write-flash @flash_args

The merged image is an alternative at address 0x0, not an additional segment.
Built with m5stack:esp32:m5stack_atoms3, core 3.3.8.
