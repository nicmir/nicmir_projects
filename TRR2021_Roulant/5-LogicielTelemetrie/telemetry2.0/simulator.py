#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Simulator main entry for telemetry2.0, main features are :
- read and send data to serial interface

Mandaroy:
pip3 install pyserial

Usefull:
for mac user : socat -d -d pty,raw,echo=0 pty,raw,echo=0
"""

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Import
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
import sys
from argparse import ArgumentParser
from lib import banner
from lib import msgType
import serial
import io
import time
from datetime import datetime
from random   import randrange

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Main function
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
def main():
  nbMsgSent = 0
    
  # Display context
  print(banner.Banner("Simulator").show())
  
  # Check parameters
  parser = ArgumentParser()
  parser.add_argument("-s", "--sport", dest="serial_port", required=True,
                      help="port com for serial connection", type=str)
  parser.add_argument("-e", "--speed", dest="serial_speed",
                      help="serial speed for serial connection", type=int, default=115200)
  args = parser.parse_args()

  # Open serial connection
  try:
    ser = serial.Serial(port=args.serial_port, baudrate=args.serial_speed, timeout = 0)
    sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
  except Exception as e:
    print (f"#> Unable to open serial port {args.serial_port} with speed {args.serial_speed}")
    print(e)
    return(1)
  print(f" > Connected to serial port {args.serial_port} with speed {args.serial_speed}")

  # Start/Stop telemetry
  sendTelemetry = False

  # Infinite loop, till the end
  goOn = 1
  while goOn:

    # Breath, read command & Check exit signal ?
    try:
      time.sleep(.001)
      rx_buffer = sio.readline()
    except KeyboardInterrupt:
      goOn = 0
      print(" > Exit by user (CTRL+C)")

    # manage received command
    if len(rx_buffer) != 0:
      print(" > command receive: %s", rx_buffer)

      # START_TELEMETRY
      if (rx_buffer == str(msgType.Id.start_telemetry_serial)):
        sendTelemetry = True

      # STOP_TELEMETRY
      if (rx_buffer == str(msgType.Id.stop_telemetry_serial)):
          sendTelemetry = False
      
      # REQ_GET_PARAM
      if (rx_buffer == str(msgType.Id.get_param_serial)):
        txt = 'p;1.898;099;GNV'
        print('%s# Send serial response: %s' % (datetime.now().strftime('%M:%S.%f'), txt))
        sio.flush()
        sio.write(txt)
        sio.flush()
        time.sleep(0.05)

    # Build and publish string following the example :
    #
    #   printf("T;%f;%f;%f;%f;%f;%f;%d;%d;%d;%d;%d;%d\r\n",
    #   pTeleElement->consigne_vitesse,
    #   pTeleElement->consigne_direction,
    #   pTeleElement->mesure_vitesse,
    #   pTeleElement->heading,
    #   pTeleElement->gyro_dps,
    #   pTeleElement->mesure_distance,
    #   (int)pTeleElement->lidar_droit,
    #   (int)pTeleElement->lidar_gauche,
    #   (int)pTeleElement->lidar_avant,
    #   (int)pTeleElement->lidar_haut,
    #   (int)pTeleElement->etat_automate_principal,
    #   (int)pTeleElement->etat_automate_auto);
    #
    if sendTelemetry == True:
      nbMsgSent += 1
      txt = 'T;%f;%f;%f;%f;%f;%f;%d;%d;%d;%d;%d;%d;' % (1.387 * randrange(-100, 100),\
                                                       2.387 * randrange(-100, 100),\
                                                       3.387 * randrange(-100, 100),\
                                                       4.387 * randrange(-100, 100),\
                                                       5.387 * randrange(-100, 100),\
                                                       6.387 * randrange(-100, 100),\
                                                       randrange(0, 10),\
                                                       randrange(0, 20),\
                                                       randrange(0, 30),\
                                                       randrange(0, 100),\
                                                       randrange(0, 3000),\
                                                       randrange(0, 500))
      #print('%s# Send serial telemetry (%d): %s' % (datetime.now().strftime('%M:%S.%f'), \
      #                                              nbMsgSent, \
      #                                              txt))

      sio.write(txt)
      sio.flush()
          
    # End of while

  return(0)

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Here we are...
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
if __name__ == '__main__':
  sys.exit(main())

# Eof
