## Bootloader
BerryBot’s BOOTSEL mode lives in read-only memory inside the RP2040 chip, and can’t be overwritten accidentally. No matter what, if you hold down the BOOTSEL button when you plug in your BerryBot, it will appear as a drive onto which you can drag a new UF2 file. There is no way to brick the board through software. However, there are some circumstances where you might want to make sure your Flash memory is empty. You can do this by dragging and dropping a special UF2 binary onto your Pico when it is in mass storage mode.

##### Let's start!

- Download the firmware.uf2 file.
- Hold down the BOOTSEL button on your BerryBot and plug it into your computer's USB port.
- Open Explorer, and open the BerryBot directory like you would any other hard drive
- Drag and drop the UF2 file into the BerryBot directory

- ![](https://www.robotistan.com/Data/EditorFiles/22814-15-1.png)
