"""
Over-The-Air (OTA) File Upload Server for MicroPython
======================================================

Runs on Pico, allows uploading files and restarting via WiFi.

Protocol (UDP port 5007):
  UPLOAD:<filename>:<size>\n<data>  - Upload file
  DELETE:<filename>                  - Delete file
  LIST                               - List files
  RESTART                            - Soft reset
  EXEC:<code>                        - Execute Python code

Usage on Pico:
    from ota import OTAServer
    ota = OTAServer()
    # In main loop or as background task:
    ota.check()  # Non-blocking check for commands

Usage from PC:
    python ota_client.py upload main.py
    python ota_client.py restart
    python ota_client.py list
"""

import socket
import os
import machine


class OTAServer:
    """Non-blocking OTA file server."""

    def __init__(self, port=5007):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)

        # For multi-packet uploads
        self._upload_file = None
        self._upload_size = 0
        self._upload_data = b''
        self._upload_addr = None

        print(f"[OTA] Server listening on port {port}")

    def check(self):
        """Check for incoming commands (non-blocking)."""
        try:
            data, addr = self.sock.recvfrom(4096)
        except OSError:
            return None  # No data available

        return self._handle(data, addr)

    def _handle(self, data, addr):
        """Handle incoming command."""
        try:
            # Check for continuation of multi-packet upload
            if self._upload_file and addr == self._upload_addr:
                return self._continue_upload(data, addr)

            # Parse command
            if data.startswith(b'UPLOAD:'):
                return self._start_upload(data, addr)
            elif data.startswith(b'DELETE:'):
                return self._delete(data, addr)
            elif data == b'LIST':
                return self._list(addr)
            elif data == b'RESTART':
                return self._restart(addr)
            elif data.startswith(b'EXEC:'):
                return self._exec(data, addr)
            elif data == b'PING':
                self._send(addr, b'PONG')
                return 'PONG'
            else:
                self._send(addr, b'ERR:Unknown command')
                return None

        except Exception as e:
            self._send(addr, f'ERR:{e}'.encode())
            return None

    def _start_upload(self, data, addr):
        """Start file upload: UPLOAD:<filename>:<size>\n<data>"""
        try:
            header_end = data.find(b'\n')
            if header_end < 0:
                self._send(addr, b'ERR:Invalid upload format')
                return None

            header = data[:header_end].decode()
            parts = header.split(':')
            if len(parts) != 3:
                self._send(addr, b'ERR:Invalid header')
                return None

            _, filename, size_str = parts
            size = int(size_str)
            file_data = data[header_end + 1:]

            # Sanitize filename
            filename = filename.strip().replace('..', '').lstrip('/')

            if len(file_data) >= size:
                # Complete in one packet
                with open(filename, 'wb') as f:
                    f.write(file_data[:size])
                self._send(addr, f'OK:Uploaded {filename} ({size} bytes)'.encode())
                print(f"[OTA] Uploaded {filename} ({size} bytes)")
                return ('UPLOAD', filename, size)
            else:
                # Multi-packet upload
                self._upload_file = filename
                self._upload_size = size
                self._upload_data = file_data
                self._upload_addr = addr
                self._send(addr, f'CONTINUE:{len(file_data)}/{size}'.encode())
                return None

        except Exception as e:
            self._send(addr, f'ERR:{e}'.encode())
            return None

    def _continue_upload(self, data, addr):
        """Continue multi-packet upload."""
        self._upload_data += data
        received = len(self._upload_data)

        if received >= self._upload_size:
            # Complete
            with open(self._upload_file, 'wb') as f:
                f.write(self._upload_data[:self._upload_size])
            msg = f'OK:Uploaded {self._upload_file} ({self._upload_size} bytes)'
            self._send(addr, msg.encode())
            print(f"[OTA] Uploaded {self._upload_file} ({self._upload_size} bytes)")

            result = ('UPLOAD', self._upload_file, self._upload_size)
            self._upload_file = None
            self._upload_data = b''
            return result
        else:
            self._send(addr, f'CONTINUE:{received}/{self._upload_size}'.encode())
            return None

    def _delete(self, data, addr):
        """Delete file: DELETE:<filename>"""
        filename = data[7:].decode().strip()
        try:
            os.remove(filename)
            self._send(addr, f'OK:Deleted {filename}'.encode())
            print(f"[OTA] Deleted {filename}")
            return ('DELETE', filename)
        except OSError as e:
            self._send(addr, f'ERR:Cannot delete {filename}: {e}'.encode())
            return None

    def _list(self, addr):
        """List files."""
        try:
            files = os.listdir()
            file_list = '\n'.join(files)
            self._send(addr, f'OK:\n{file_list}'.encode())
            return ('LIST', files)
        except Exception as e:
            self._send(addr, f'ERR:{e}'.encode())
            return None

    def _restart(self, addr):
        """Soft reset."""
        self._send(addr, b'OK:Restarting...')
        print("[OTA] Restarting...")
        import time
        time.sleep_ms(100)
        machine.soft_reset()

    def _exec(self, data, addr):
        """Execute Python code: EXEC:<code>"""
        code = data[5:].decode()
        try:
            result = eval(code)
            self._send(addr, f'OK:{result}'.encode())
            return ('EXEC', result)
        except:
            try:
                exec(code)
                self._send(addr, b'OK:executed')
                return ('EXEC', None)
            except Exception as e:
                self._send(addr, f'ERR:{e}'.encode())
                return None

    def _send(self, addr, data):
        """Send response."""
        try:
            self.sock.sendto(data, addr)
        except:
            pass
