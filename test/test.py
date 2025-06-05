#!/usr/bin/env python3

import socket
import os

s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect('/tmp/pcat-manager.sock')

s.send(b"{'command':'pmu-fw-version-get'}\0")
b = s.recv(256)
print(b)
