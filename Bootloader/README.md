## Bootloader
BerryBot’s BOOTSEL mode lives in read-only memory inside the RP2040 chip, and can’t be overwritten accidentally. No matter what, if you hold down the BOOTSEL button when you plug in your BerryBot, it will appear as a drive onto which you can drag a new UF2 file. There is no way to brick the board through software. However, there are some circumstances where you might want to make sure your Flash memory is empty. You can do this by dragging and dropping a special UF2 binary onto your RP2040 when it is in mass storage mode.

##### Let's start!

![bootsel1](https://github.com/user-attachments/assets/5a36ce71-707e-4c76-bca7-326a4ef93eae)
![bootsel2](https://github.com/user-attachments/assets/459f13d8-2fe8-410c-91f0-ffa46ee9c641)
![bootsel3](https://github.com/user-attachments/assets/1f2ed450-e032-417f-b3f0-e0429cc8353a)
