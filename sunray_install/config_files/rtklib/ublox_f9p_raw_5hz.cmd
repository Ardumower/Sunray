# 
# str2str configuration command file to enable raw measurements
# and suppress NMEA messages.
#
# http://allegrobotics.com/rtklib/UBLOX_CMD_GUIDE.txt
# https://portal.u-blox.com/s/question/0D52p00008HKDn7/ubx-message-documentation
# https://github.com/jdesbonnet/GNSS_RTK/blob/master/ublox_m8p/cfg_m8p_basestation.cmd

# Set sample rate low while configuring receiver
# Sample rate 1 Hz (measRate=1000, navRate=1, timeRef=1 (GSPTime))
!UBX CFG-RATE 1000 1 1

#
# disable NMEA out for USB 
# https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgPRT.msg
# !UBX CFG-PRT 3 0 0 0 0 3 1 0 0
#
# enable NMEA+UBX out for USB 
# https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgPRT.msg
!UBX CFG-PRT 3 0 0 0 0 3 3 0 0


# enable RTCM3 in for serial2 
# https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgPRT.msg
!UBX CFG-PRT 2 0 0 2240 115200 32 0 0 0


#
# Enable RAW messages
#

# turn on UBX RXM-RAWX messages on USB
!UBX CFG-MSG  2 21 0 0 0 1 0 0

# turn on UBX RXM-SFRBX messages on USB
!UBX CFG-MSG  2 19 0 0 0 1 0 0

# turn on UBX TIM TM2 messages on USB
!UBX CFG-MSG 13 3 0 0 0 1 0 0

# turn on UBX NAV-SVIN (survey in data) on USB
!UBX CFG-MSG  1 59  0 0 0 1 0 0

# turn on UBX NAV-SVINFO on USB
!UBX CFG-MSG  1 48  0 0 0 1 0 0

# turn non UBX NAV-SOL on USB
!UBX CFG-MSG  1  6  0 0 0 1 0 0


# Turn off all NMEA


# Turn off NMEA GGA
# !UBX CFG-MSG 240 0 0 0 0 0 0 0
# Turn on NMEA GGA
!UBX CFG-MSG 240 0 0 0 0 1 0 0


# Turn off NMEA GLL
#!UBX CFG-MSG 240 1 0 0 0 0 0 0
# Turn on NMEA GLL
!UBX CFG-MSG 240 1 0 0 0 1 0 0


# Turn off NMEA GSA
#!UBX CFG-MSG 240 2 0 0 0 0 0 0
# Turn on NMEA GSA
!UBX CFG-MSG 240 2 0 0 0 1 0 0


# Turn off NMEA GSV
#!UBX CFG-MSG 240 3 0 0 0 0 0 0
# Turn on NMEA GSV
!UBX CFG-MSG 240 3 0 0 0 1 0 0


# Turn off NMEA RMC
#!UBX CFG-MSG 240 4 0 0 0 0 0 0
# Turn on NMEA RMC
!UBX CFG-MSG 240 4 0 0 0 1 0 0


# Turn off NMEA VTG
#!UBX CFG-MSG 240 5 0 0 0 0 0 0
# Turn on NMEA VTG
!UBX CFG-MSG 240 5 0 0 0 1 0 0


# Turn off NMEA ZDA
#!UBX CFG-MSG 240 8 0 0 0 0 0 0
# Turn on NMEA ZDA
!UBX CFG-MSG 240 8 0 0 0 1 0 0




# Set sample rate to 5 Hz
!UBX CFG-RATE 200 1 1


