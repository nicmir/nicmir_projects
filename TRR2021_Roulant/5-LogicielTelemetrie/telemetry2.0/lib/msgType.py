#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Simple banner class
"""

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Import
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
# Main class
#-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
class Id:
  connect_serial           = b'REQ_CONNECT'
  disconnect_serial        = b'REQ_DISCONNECT'
  start_telemetry_serial   = b'REQ_STARTTELEMETRY'
  stop_telemetry_serial    = b'REQ_STOPTELEMETRY'
  upload_telemetry         = b'REQ_UPLOADTELEMETRY'
  get_param_serial         = b'REQ_GETPARAM'
  set_param_serial         = b'REQ_SETPARAM'

# Eof
