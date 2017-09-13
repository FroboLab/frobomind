Updated 2017-09-11 Kjeld Jensen kjen@mmmi.sdu.dk

GT-Position is a local positioning system from http://gamesontrack.dk

The positioning is based on ultrasound loudspeakers mounted on robots and
drones that emit a coded signal. This signal is picked up by microphones
installed at the LPS covered area, processed by a GT controller and Windows
software.

The output is a series of NMEA strings with the $GTPOS containing the
loudspeaker id and x,y,z coordinates. This output is read by the FroboMind
 gt_serial_server_node.py and published using the configured format:

  'nmea' Pseudo $GPGGA strings
  'slip' binary formatted with CRC-16 checksum and SLIP encoded


