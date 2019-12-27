# Curio - a Sawppy clone

ROS software packages to control a version of [Roger Chen's Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover).

## Onboard computer - Raspberry Pi 4

We decided to use a Raspberry Pi 4 (4GB) for the onboard computer as we
built and configured ROS for the Raspberry Pi 4 when developing our rover wild1
which is based on the Wild Thumper chassis.

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

## Mesh files

This section describes where the mesh files for the URDF / SDF model were sourced
and outlines any additional processing needed to prepare the mesh files for the robot model.

### Sawppy STL files

Roger Chen's CAD model for Sawppy is available online here: [Sawppy the Rover](https://cad.onshape.com/documents/43678ef564a43281c83e1aef/w/392bbf8745395bc24367a35c/e/9bd6bbb7aba50a97523d14f2).

To generate the mesh files for the robot description we exported the Sawppy model from OnShape
as a STEP file and imported it to FreeCAD. In FreeCAD we exported the various assemblies for the URDF
model as STL files and used Blender to complete the processing. Typically this involves scaling (by 0.001),
a rotation of 90 degrees about the z-axis and then translation of the part in order to centre
it's joint axis at the world origin. The reason for centering assemblies at the origin is that in
URDF child link poses are defined relative to the joint, when the visual's joint axis is centred
at the origin no further pose adjustments are usually required in the URDF `<visual>` element which
simplifies defining and maintaining the model.

### Turnbuckle STL file

The turnbuckle CAD file was authored by Carlos Rey and retrieved from GrabCAD [https://grabcad.com/library/tensor-turnbuckle-1](https://grabcad.com/library/tensor-turnbuckle-1) on 27 Dec 2019.

We modified the original STL file in Blender. The model has been rescaled and rotated around the x-axis.

## URDF / SDF modelling

Most of the steps required to construct the URDF / SDF model are standard and well described in
the ROS and Gazebo documentation and tutorials. The main exception is modelling the differential.
URDF expects the model to have a tree structure with a single root node
(which according to [ROS REP 105](https://www.ros.org/reps/rep-0105.html) for mobile robots
should be denoted `base_link`). The differential introduces a closed-loop kinematic chain which
cannot be described in URDF. SDF on the other hand is capable of describing closed kinematic chains
and we would like the simulated model to capture this feature of the robot's mechanics correctly.

This Gazebo tutorial ([Model a 4-bar linkage in SDFormat and URDF](http://gazebosim.org/tutorials?tut=kinematic_loop&cat=)) describes the issue and an approach for specifying the linkage
using a `<gazebo>` extension element. Our solution for the Sawppy Rover differential is to
define UDRF trees for the left and right rocker assemblies and another for the differential.
The turnbuckle linkages connected to the differential are modelled using a combination of
revolute joints. The final two revolute joints connecting the turnbuckles to the rockers
are left out of the URDF but included in a `<gazebo>` element.

With this approach we have a single description of the robot for visualisation in Rviz and
simulation in Gazebo.

