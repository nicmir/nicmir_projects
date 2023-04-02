#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Frontend main entry for telemetry2.0, main features are :
- GUI with URGENT STOP command
- push data to ES for funcky display and history/backup
"""

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Import
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
import sys
from argparse import ArgumentParser
from lib import banner
from lib import zipped_pickle
from lib import msgType
import zmq
from tkinter import *
import time
from datetime import datetime
from elasticsearch import Elasticsearch
from elasticsearch import ElasticsearchException
from elasticsearch import helpers
import traceback

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Global
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
global socket_control
global runNumber
global esInd
global esMap
global frame
global frame_param_list
global file_stream
global docs

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Functions linked to GUI action
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
def CONNECT_TO_SERIAL():
  global socket_control
  socket_control.send(msgType.Id.connect_serial)
def DISCONNECT_TO_SERIAL():
  global socket_control
  socket_control.send(msgType.Id.disconnect_serial)
def START_TELEMETRY():
  global socket_control
  socket_control.send(msgType.Id.start_telemetry_serial)
def STOP_TELEMETRY():
  global socket_control
  socket_control.send(msgType.Id.stop_telemetry_serial)
def UPLOAD_TELEMETRY():
  global socket_control
  global file_stream
  global docs
  socket_control.send(msgType.Id.upload_telemetry)
  if file_stream != None:
    file_stream.close()
  file_stream = open(time.strftime("%Y_%m_%d-%H%M_%S.txt"), "a")
  docs.clear()
  print('*** Start upload ***')
def NEW_ES_INDEX():
  global runNumber
  global es
  global esInd
  global esMap
  runNumber += 1
  esInd = 'es_frontend' + '_run_' + '{0}'.format(str(runNumber).zfill(4)) + datetime.now().strftime('_%d_%m_%Y')
  print(' > New index: ', esInd)
  res = es.indices.create(index=esInd, body=esMap, ignore=400)
def GET_PARAMETERS():
  global socket_control
  socket_control.send(msgType.Id.get_param_serial)
def SET_PARAMETERS():
  global socket_control
  global frame_param_list
  txt = ';s'
  for g in range(0, len(frame_param_list)):
    ctx = frame_param_list[str(g)]
    txt += ';' + ctx.get()
  print(txt)
  socket_control.send(msgType.Id.set_param_serial + str.encode(txt))

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Main function
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
def main():
  global socket_control
  global runNumber
  global es
  global esInd
  global esMap
  global frame
  global frame_param_list
  global file_stream
  global docs
  
  nbMsgSerial = 0
  runNumber   = 0
  docs        = []
  file_stream = None
  docType     = 'serial_telemetry'
  esInd       = 'es_frontend' + '_run_' +\
                '{0}'.format(str(runNumber).zfill(4)) +\
                datetime.now().strftime('_%d_%m_%Y')
  esMap       = {
      "mappings" : {
      "serial_telemetry" : {
      "properties" : {
            "@timestamp" : {
            "type" : "date"
          },
          "c_vitesse" : {
            "type" : "float"
          },
          "c_direction" : {
            "type" : "float"
          },
          "m_vitesse" : {
            "type" : "float"
          },
          "heading" : {
            "type" : "float"
          },
          "gyro_dps" : {
            "type" : "float"
          },
          "m_distance" : {
            "type" : "float"
          },
          "l_droit" : {
            "type" : "long"
          },
          "l_gauche" : {
            "type" : "long"
          },
          "l_avant" : {
            "type" : "long"
          },
          "l_haut" : {
            "type" : "long"
          },
          "e_auto_principal" : {
            "type" : "long"
          },
          "e_auto_auto" : {
            "type" : "long"
          }
        }
      }
    }
  }

  # Display context
  print(banner.Banner("Frontend").show())
  
  # Check parameters
  parser = ArgumentParser()
  parser.add_argument("-f", "--frontend", dest="frontend_ip", required=True,
                      help="Frontend IP address", type=str)
  parser.add_argument("-b", "--backend", dest="backend_ip", required=True,
                      help="Backend IP address", type=str)
  parser.add_argument("-e", "--es", dest="es_ip", required=True,
                      help="Elastiseach IP address", type=str)
  args = parser.parse_args()

  #  ZMQ context
  context = zmq.Context()

  ####################
  ### Publish part ###
  ####################
  # Socket in PUBlish mode to control the telemetry producers
  socket_control = context.socket(zmq.PUB)
  socket_control.bind('tcp://' + args.frontend_ip + ':5555')

  ######################
  ### Subscribe part ###
  ######################
  # Socket in SUBscribe mode for serial telemetry reception
  socket_serial = context.socket(zmq.SUB)
  socket_serial.connect('tcp://' + args.backend_ip + ':5558')
  # SUBscribe to serial topic
  socket_serial.setsockopt(zmq.SUBSCRIBE, b'')

  # Create poller context
  poller = zmq.Poller()
  poller.register(socket_serial, zmq.POLLIN)

  #####################
  # Main frontend frame
  #####################
  frame = Tk()
  frame.title('Frontend')

  # Fill the frame
  rowId = 0
  Button(frame, text="Connect to serial",    command=CONNECT_TO_SERIAL).grid(row=rowId, column=0)
  rowId +=1
  Button(frame, text="Disconnect to serial", command=DISCONNECT_TO_SERIAL).grid(row=rowId, column=0)
  rowId +=1
  Button(frame, text="New ES index",         command=NEW_ES_INDEX).grid(row=rowId, column=0)
  rowId +=1
  Button(frame, text="Start telemetry",      command=START_TELEMETRY).grid(row=rowId, column=0)
  rowId +=1
  Button(frame, text="Stop telemetry",       command=STOP_TELEMETRY).grid(row=rowId, column=0)
  rowId +=1
  Button(frame, text="Upload telemetry",     command=UPLOAD_TELEMETRY).grid(row=rowId, column=0)
  rowId +=1
  Button(frame, text="Get parameters",       command=GET_PARAMETERS).grid(row=rowId, column=0)
  rowId +=1
  Label(frame, text="Kp").grid(row=rowId, column=1)
  frame_kp = Entry(frame)
  frame_kp.grid(row=rowId, column=2)
  rowId +=1
  Label(frame, text="v").grid(row=rowId, column=1)
  frame_v = Entry(frame)
  frame_v.grid(row=rowId, column=2)
  rowId +=1
  Label(frame, text="site").grid(row=rowId, column=1)
  frame_site = Entry(frame)
  frame_site.grid(row=rowId, column=2)
  rowId +=1
  Button(frame, text="Set parameters", command=SET_PARAMETERS).grid(row=rowId, column=0)
  rowId +=1

  # Build frame parameter list
  frame_param_list = {'0':frame_kp, '1':frame_v, '2':frame_site}

  # First update display
  frame.update_idletasks()
  frame.update()

  ###############
  # ES connection
  ###############
  es_connected = False
  try:
    es = Elasticsearch([args.es_ip])
    if es.ping():
      print(' > Frontend connected to ES. Current index is: %s', esInd)
      es_connected = True
      # Create index and set mapping
      res = es.indices.create(index=esInd, body=esMap, ignore=400)
    else:
      print('#> Error unable to connect to ES')
  except Exception:
    print("Exception in user code:")
    print("-"*60)
    traceback.print_exc(file=sys.stdout)
    print("-"*60)

  # Infinite loop
  while True:
      
    # Update main frame
    try:
      frame.update_idletasks()
      frame.update()
    except:
      socket_control.close()
      socket_serial.close()
      context.term()
      return(0)

    # Get info from publisher
    try:
      # Non-blocking wait for message from client
      socks = dict(poller.poll(0))
    except zmq.ZMQError as e:
      print('ZMQ error (%s), stopping...' % e.strerror)
      print(e)

    # Process serial telemetry
    if socks.get(socket_serial) == zmq.POLLIN:
      nbMsgSerial += 1
      serialString = zipped_pickle.recv(socket_serial)
      if (serialString[0] == 'p'):
        print('%s# Receive SERIAL response (%d): %s' % (datetime.now().strftime('%M:%S.%f'), \
                                                        nbMsgSerial, \
                                                        serialString))
        txt = serialString.split(';')
        for g in range(1, len(txt)):
          ctx = frame_param_list[str(g-1)]
          ctx.delete(0,END)
          ctx.insert(0,txt[g])

      # Check of Telemetry samples and for Final sample message
      elif es_connected and ((serialString[0] == 'T') or (serialString[0] == 'F')):
          
        # Telemetry sample
        if (serialString[0] == 'T'):
          # Forward to ES, date must be stamped in UTC
          res = serialString.split(';')
          # Filter bad format serial message
          if (len(res)>14):
            print("# Receive bad formated message: ", len(res), res)
          else:
            t = datetime.utcnow()
            one_doc = {  '_index'          : esInd,\
                         '_doc_type'       : docType,\
                         '@timestamp'      : t.strftime('%Y-%m-%dT%H:%M:%S.%fZ'),\
                         'c_vitesse'       : float(res[1]),\
                         'c_direction'     : float(res[2]),\
                         'm_vitesse'       : float(res[3]),\
                         'heading'         : float(res[4]),\
                         'gyro_dps'        : float(res[5]),\
                         'm_distance'      : float(res[6]),\
                         'l_droit'         : int(res[7]),\
                         'l_gauche'        : int(res[8]),\
                         'l_avant'         : int(res[9]),\
                         'l_haut'          : int(res[10]),\
                         'e_auto_principal': int(res[11]),\
                         'e_auto_auto'     : int(res[12]) }
            docs.append(one_doc)
            file_stream.write(serialString + "\n")

        # Final sample
        if (serialString[0] == 'F'):
          # Close the current file
          if (file_stream != None):
            file_stream.close()
            file_stream = None
          # Insert data in bulk mode
          try:
            print("Number sample inserted in ES: ", len(docs))
            res = helpers.bulk(es, docs)
          except Exception as e:
            print(e)
          docs.clear()
          print('*** End upload ***')

  # Never reached the end
  return(0)

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Here we are...
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
if __name__ == '__main__':
  sys.exit(main())

# Eof
