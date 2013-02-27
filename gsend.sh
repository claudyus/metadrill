#!/bin/bash
# Tiny test script for sending G-codes to the CNC machine
teletype /dev/tts/USB0 19200 "echo '$*'; read"
