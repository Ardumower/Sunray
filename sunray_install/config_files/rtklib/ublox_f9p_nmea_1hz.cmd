# 
# str2str configuration command file to disable raw measurements
# and enable NMEA messages.
#
# http://allegrobotics.com/rtklib/UBLOX_CMD_GUIDE.txt
# https://portal.u-blox.com/s/question/0D52p00008HKDn7/ubx-message-documentation
# https://github.com/jdesbonnet/GNSS_RTK/blob/master/ublox_m8p/cfg_m8p_basestation.cmd

# Set sample rate low while configuring receiver
# Sample rate 1 Hz (measRate=1000, navRate=1, timeRef=1 (GSPTime))
!UBX CFG-RATE 1000 1 1

#
# enable NMEA+UBX out for USB 
# https://github.com/KumarRobotics/ublox/blob/master/ublox_msgs/msg/CfgPRT.msg
!UBX CFG-PRT 3 0 0 0 0 3 3 0 0


#
# disable UBX RAW messages
#

# turn off UBX RXM-RAWX messages on USB
!UBX CFG-MSG  2 21 0 0 0 0 0 0

# turn off UBX RXM-SFRBX messages on USB
!UBX CFG-MSG  2 19 0 0 0 0 0 0


#
# disable UBX other messages
#

# turn off UBX RXM-RTCM messages on USB
!UBX CFG-MSG 2 50 0 0 0 0 0 0

# turn off UBX TIM TM2 messages on USB
!UBX CFG-MSG 13 3 0 0 0 0 0 0

# turn off UBX NAV-POSECEF messages on USB
!UBX CFG-MSG 1 1 0 0 0 0 0 0

# turn off UBX NAV-DOP messages on USB
!UBX CFG-MSG 1 4 0 0 0 0 0 0

# turn off UBX NAV-PVT messages on USB
!UBX CFG-MSG 1 7 0 0 0 0 0 0

# turn off UBX NAV-VELNED messages on USB
!UBX CFG-MSG 1 18 0 0 0 0 0 0

# turn off UBX NAV-HPPOSLLH messages on USB
!UBX CFG-MSG 1 20 0 0 0 0 0 0

# turn off UBX NAV-RELPOSNED messages on USB
!UBX CFG-MSG 1 60 0 0 0 0 0 0

# turn off UBX NAV-SAT messages on USB
!UBX CFG-MSG 1 53 0 0 0 0 0 0

# turn off UBX NAV-SIG messages on USB
!UBX CFG-MSG 1 67 0 0 0 0 0 0

# turn off UBX NAV-SVIN (survey in data) on USB
!UBX CFG-MSG  1 59  0 0 0 0 0 0

# turn off UBX NAV-SVINFO on USB
!UBX CFG-MSG  1 48  0 0 0 0 0 0

# turn off UBX NAV-SOL on USB
!UBX CFG-MSG  1  6  0 0 0 0 0 0

# turn off UBX MON-COMMS messages on USB
!UBX CFG-MSG 10 54 0 0 0 0 0 0

# turn off UBX MON-RF messages on USB
!UBX CFG-MSG 10 56 0 0 0 0 0 0


# Turn on all NMEA

# Turn on NMEA GGA
!UBX CFG-MSG 240 0 0 0 0 1 0 0

# Turn on NMEA GLL
!UBX CFG-MSG 240 1 0 0 0 1 0 0

# Turn on NMEA GSA
!UBX CFG-MSG 240 2 0 0 0 1 0 0

# Turn on NMEA GSV
!UBX CFG-MSG 240 3 0 0 0 1 0 0

# Turn on NMEA RMC
!UBX CFG-MSG 240 4 0 0 0 1 0 0

# Turn on NMEA VTG
!UBX CFG-MSG 240 5 0 0 0 1 0 0

# Turn on NMEA ZDA
!UBX CFG-MSG 240 8 0 0 0 1 0 0



# Set sample rate to 1 Hz
!UBX CFG-RATE 1000 1 1


