#!/usr/bin/env python
#-*- coding: utf-8 -*-
import sys
import threading
import atexit
import time
import Queue as queue
import Tkinter as tk

import signal

_terminate_thread = threading.Event()

TK_AFTER_INTERVAL = 10

perror = sys.stderr.write
def _tk_loop():
	global _tk_root
	try:
		_tk_root = tk.Tk() # Ventana root
		_tk_root.withdraw() # Oculta
		_tk_root.after(TK_AFTER_INTERVAL, _tk_attend_requests)
		_tk_root.mainloop()
	except (Exception,) as e:
		print e
		_tk_root = None
		perror("Error inicializando los gráficos\n")
# Que se ejecute al salir:
#@atexit.register
def _tk_loop_cleanup():
	"Indica al thread que se detenga y lo espera"
	_terminate_thread.set()
	_tk_loop_thread.join(5)
	if _tk_loop_thread.is_alive():
		print "Timeout :("
# También para el Ctrl+C:
signal.signal(signal.SIGINT, lambda signal, frame: _tk_loop_cleanup)

_tk_loop_thread = threading.Thread(target=_tk_loop)
#print _tk_loop_thread.daemon
_tk_loop_thread.daemon = True
_tk_loop_thread.start()

_request_queue = queue.Queue()

def _tk_attend_requests():
	"""Atiende requests desde adentro del thread,
	recibe métodos desde _request_queue"""
	if _terminate_thread.is_set():
		# Terminar el thread desde adentro
		_tk_root.quit()
		exit()
	try:
		request = _request_queue.get(block=False)
	except queue.Empty:
		pass
	else:
		_tk_root.after(0, request)
	finally:
		_tk_root.after(TK_AFTER_INTERVAL, _tk_attend_requests)

class Senses(object):
	def __init__(self, robot):
		_request_queue.put(self.run)
	def run(self):
		toplevel = tk.Toplevel()
		toplevel.grid()
		label = tk.Label(toplevel, text="Hola")
		label.pack()

a = raw_input()
s = Senses(2)
b = raw_input()
	
