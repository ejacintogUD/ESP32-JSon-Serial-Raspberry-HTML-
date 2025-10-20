#!/usr/bin/env python3
# Leer JSON solo desde el puerto serial /dev/ttyUSB0 y mostrar la temperatura.

import sys
import json
import time

def iter_serial_lines(port, baud, timeout):
    try:
        import serial
    except ImportError:
        print("Falta el paquete 'pyserial'. Instálalo: pip install pyserial", file=sys.stderr)
        sys.exit(5)

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=timeout)
    except Exception as e:
        print(f"Error abriendo puerto serial {port}: {e}", file=sys.stderr)
        sys.exit(4)

    try:
        while True:
            try:
                line = ser.readline()
            except Exception as e:
                print(f"Error leyendo serial: {e}", file=sys.stderr)
                break
            if not line:
                continue
            try:
                yield line.decode("utf-8", errors="replace").strip()
            except Exception:
                yield line.decode("latin-1", errors="replace").strip()
    finally:
        ser.close()

def print_temp_from_json_obj(obj):
    temp = obj.get("temp") if isinstance(obj, dict) else None
    ts = obj.get("ts") if isinstance(obj, dict) else None

    if temp is None:
        print("temp: null")
    else:
        try:
            print(f"{float(temp):.4f} °C")
        except Exception:
            print(f"temp: {temp}")

    if ts is not None:
        # opcional: mostrar timestamp
        # print(f"ts: {ts}")
        pass

def main():
    port = "/dev/ttyUSB0"   # puerto fijo solicitado
    baud = 115200
    timeout = 2

    try:
        for line in iter_serial_lines(port, baud, timeout):
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as e:
                print(f"JSON inválido recibido: {e}", file=sys.stderr)
                continue
            print_temp_from_json_obj(obj)
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario.", file=sys.stderr)
        sys.exit(0)

if __name__ == "__main__":
    main()
