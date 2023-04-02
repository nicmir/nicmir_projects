#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Backend main entry for telemetry2.0, main features are :
- open serial connection to embeded device (i.e. STM32)
- accept connection from remote client (frontend telemetry)
- get telemetry data from serial interface

Mandaroy:
pip3 install pyserial

Usefull for test purpose:
for mac/linux user : socat -d -d pty,raw,echo=0 pty,raw,echo=0
"""

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Import
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
import sys
from argparse import ArgumentParser
from lib import banner
from lib import zipped_pickle
from lib import msgType
import serial
import io
import zmq
import re

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Main function
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
def main():

  linkToSerial = False

  maxSample    = 10000
  nbSample     = 0
  all_samples  = ["" for i in range(maxSample)]
  
  # Display context
  print(banner.Banner("Backend").show())
  
  # Check parameters
  parser = ArgumentParser()
  parser.add_argument("-f", "--frontend", dest="frontend_ip", required=True,
                      help="Frontend IP address", type=str)
  parser.add_argument("-b", "--backend", dest="backend_ip", required=True,
                      help="Backend IP address", type=str)
  parser.add_argument("-s", "--sport", dest="serial_port", required=True,
                      help="port com for serial connection", type=str)
  parser.add_argument("-e", "--speed", dest="serial_speed",
                      help="serial speed for serial connection", type=int, default=115200)
  args = parser.parse_args()

  # ZMQ context
  context = zmq.Context()

  ####################
  ### Publish part ###
  ####################
  # Socket in PUBlisher mode to forward serial telemetry
  socket = context.socket(zmq.PUB)
  socket.bind('tcp://' + args.backend_ip + ':5558')

  ######################
  ### Subscribe part ###
  ######################
  socket_control = context.socket(zmq.SUB)
  socket_control.connect('tcp://' + args.frontend_ip + ':5555')
  # SUBscribe to all topics i.e. the subscriber want to process all the message from the publisher
  socket_control.setsockopt(zmq.SUBSCRIBE, b'')

  # Create poller context
  poller = zmq.Poller()
  poller.register(socket_control, zmq.POLLIN)

  # Main loop
  #-=-=-=-=-=
  while True:
      
    # Read serial link if connected
    #-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    if linkToSerial == True:
      rx_buffer = sio.readline()
      # Check for new message
      if len(rx_buffer) != 0:
        
        # Telemetry message
        if ('T' == rx_buffer[0]):
          if (nbSample < maxSample):
            # Try to fix badly formated serial sample
            z = re.match("(.*)\;T(.*)", rx_buffer)
            if (z) and (len(z.groups())>=2):
              all_samples[nbSample] = z.groups()[0] + ';'
              nbSample += 1
              all_samples[nbSample] = 'T' + z.groups()[1]
              nbSample += 1
            else:
              all_samples[nbSample] = rx_buffer
              nbSample += 1
            # Max limit reached
            if (nbSample >= maxSample):
              print('*** MAX samples reached: ', maxSample)
        else:
          # Forward message to frontend
          #-=-=-=-=-=-=-=-=-=-=-=-=-=-=
          try:
            zipped_pickle.send(socket, rx_buffer)
          except zmq.ZMQError as e:
            print('ZMQ error (%s), stopping...' % e.strerror)
            print(e)

    # Read frontend request, if any
    #-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    try:
      socks = dict(poller.poll(0))
    except zmq.ZMQError as e:
      print(' > ZMQ error (%s), stopping...' % e.strerror)
      print(e)
      socket_control.close()
      context.term()
      return(-1)
    except KeyboardInterrupt:
      print(' > Backend server stopped by user, stopping...')
      socket_control.close()
      context.term()
      return(0)

    # Parse message if any
    #-=-=-=-=-=-=-=-=-=-=-
    if socks.get(socket_control) == zmq.POLLIN:
      full_msg = socket_control.recv(0)
      msg = str(full_msg).split(';')
      print(" > Receive message type: ", str(msg[0]))
      ######################
      # DISCONNECT TO SERIAL
      ######################
      if (msg[0] == str(msgType.Id.disconnect_serial)):
          if (linkToSerial == False):
              print("#> Error, connection to serial already disable")
          else:
              ser.close()
              linkToSerial = False
              print(f" > Disconnected to serial port {args.serial_port} with speed {args.serial_speed}")
      ###################
      # CONNECT TO SERIAL
      ###################
      elif (msg[0] == str(msgType.Id.connect_serial)):
        if (linkToSerial == True):
          print("#> Error, connection to serial already enable")
        else:
          # Open serial connection
          try:
            ser = serial.Serial(port=args.serial_port,\
                                baudrate=args.serial_speed,\
                                timeout = 0)
            sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
          except Exception as e:
            print (f"#> Unable to open serial port {args.serial_port} with speed {args.serial_speed}")
            print(e)
          else:
            linkToSerial = True
            print(f" > Connected to serial port {args.serial_port} with speed {args.serial_speed}")
      ######################
      # START/STOP TELEMETRY
      # GET PARAM
      ######################
      elif ((msg[0] == str(msgType.Id.start_telemetry_serial)) or \
            (msg[0] == str(msgType.Id.stop_telemetry_serial))  or \
            (msg[0] == str(msgType.Id.get_param_serial))):
        if (linkToSerial == False):
          print("#> Error, connection to serial not set")
        else:
            
          if (msg[0] == str(msgType.Id.start_telemetry_serial)):
            all_samples  = ["" for i in range(maxSample)]
            nbSample     = 0
          
          if (msg[0] == str(msgType.Id.stop_telemetry_serial)):
            nbSample = maxSample

          sio.write(msg[0])
          sio.flush()
              
      ######################
      # UPLOAD TELEMETRY
      ######################
      elif ((msg[0] == str(msgType.Id.upload_telemetry))):
        if (linkToSerial == False):
          print("#> Error, connection to serial not set")
        else:
          print('*** Start upload ***')
          for s in range(0, nbSample):
            if len(all_samples[s]) != 0:
              try:
                zipped_pickle.send(socket, all_samples[s])
              except zmq.ZMQError as e:
                print('ZMQ error (%s), stopping...' % e.strerror)
                print(e)
          # Send Final sample message
          try:
            zipped_pickle.send(socket, 'F;')
          except zmq.ZMQError as e:
            print('ZMQ error (%s), stopping...' % e.strerror)
            print(e)
          print('*** End upload ***')

      ###########
      # SET PARAM
      ###########
      elif ((msg[0] == str(msgType.Id.set_param_serial))):
        if (linkToSerial == False):
          print("#> Error, connection to serial not set")
        else:
          sio.write(full_msg)
          sio.flush()

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Here we are...
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
if __name__ == '__main__':
  sys.exit(main())

# Eof
