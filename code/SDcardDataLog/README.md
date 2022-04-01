Things to look at:

https://arduino.stackexchange.com/questions/28540/how-to-increase-sd-card-write-speed-in-arduino - "Handshake" every file.write - unsure what that means

https://github.com/greiman/SdFat/issues/129 - Using interrupts to buffer it in the background.

- Try using exFAT instead of FAT32
- 
