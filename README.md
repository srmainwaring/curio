# Curio - a Sawppy clone

ROS software packages to control a version of [Roger Chen's Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover).

## Onboard computer - Raspberry Pi 4

We decided to use a Raspberry Pi 4 (4GB) for the onboard computer as we
built and configured ROS for the Raspberry Pi 4 when developing our wild1 rover.

To create an image for curio we clone the existing image from wild1's SD card:

1. Remove the SD card from the Raspberry Pi and place it in the card reader.

2. Find the device name for the card:

```bash
$ diskutil list
/dev/disk0 (internal, physical):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:      GUID_partition_scheme                        *240.1 GB   disk0
   1:                        EFI EFI                     209.7 MB   disk0s1
   2:                  Apple_HFS MacPro1 El Capitan      239.2 GB   disk0s2
   3:                 Apple_Boot Recovery HD             650.0 MB   disk0s3

...

/dev/disk6 (external, physical):
   #:                       TYPE NAME                    SIZE       IDENTIFIER
   0:     FDisk_partition_scheme                        *31.9 GB    disk6
   1:             Windows_FAT_32 boot                    268.4 MB   disk6s1
   2:                      Linux                         31.6 GB    disk6s2

```

The last entry for `/dev/disk6` is the rasbian image.

3. Clone the image using `dd`

```bash
sudo dd if=/dev/rdisk6 of=/Volumes/G-RAID/backup/Pi/wild1.img bs=1m
```

4. When complete remove the orginal SD card and place it back in wild1's Raspberry Pi.

5. Place the SD card for the cloned image in the card reader and use `balenaEtcher`
to flash and verify the card.

6. Start up curio's Pi and change the hostname to `curio`.

## Resources

- [Clone SD card on Mac](http://pi.bek.no/cloneSDcard/)
