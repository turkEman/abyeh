#!/usr/bin/env python3
"""
FC WiFi Bridge
──────────────────────────────────────────────────────────────
Sits between the ESP32 (TCP binary telemetry) and the browser (WebSocket JSON).
Also relays control JSON from the browser back to the ESP32.

Quick start:
    pip install websockets
    python bridge.py

Then open index.html in a browser.
Do NOT navigate to localhost:8765 directly — that port only speaks WebSocket.
Open index.html as a file (or via a local HTTP server) and the JS inside
will connect to ws://localhost:8765 automatically.

Windows firewall note:
    On first run Windows will ask to allow Python through the firewall.
    Click "Allow".  If it doesn't ask, add a manual rule for TCP port 5000
    and TCP port 8765 (or just disable the private-network firewall briefly
    while testing).
──────────────────────────────────────────────────────────────
"""

import asyncio
import struct
import json
import logging
import csv
import os
from datetime import datetime
from typing import Optional, Set

import websockets

# ═══════════════════════════════════════════════════════════════════════
#   User Configuration  —  edit this section
# ═══════════════════════════════════════════════════════════════════════

TCP_HOST = '0.0.0.0'   # listen on all interfaces so ESP32 can reach us
TCP_PORT = 5000         # must match WB_SERVER_PORT in wifi_bridge_config.h

WS_HOST  = 'localhost'  # browser is on same machine
WS_PORT  = 8765         # browser WebSocket connects here: ws://localhost:8765

# Telemetry variable names
# !! ORDER MUST MATCH the order of wifiBridge_addTelemetry() calls in setup() !!
# !! If you add or remove a variable, update both sides.                       !!
TELEM_NAMES: list[str] = [
    'gx', 'gy', 'gz',
    'ax', 'ay', 'az',
    'angle_ax', 'angle_ay',
    'angle_kx', 'angle_ky'
    # add more as needed
]

# Set True to write every received packet to a timestamped CSV in ./logs/
LOG_CSV = False
LOG_DIR = 'logs'

# ═══════════════════════════════════════════════════════════════════════
#   Wire protocol  (must match wifi_bridge.cpp)
# ═══════════════════════════════════════════════════════════════════════

MAGIC_0    = 0xFC
MAGIC_1    = 0x1A
HDR_SIZE   = 16       # bytes before float payload
PKT_TELEM  = 0x01
PKT_CTRL_RESP = 0x02

# ── Logging setup ────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s.%(msecs)03d  %(levelname)-7s  %(message)s',
    datefmt='%H:%M:%S',
)
log = logging.getLogger('bridge')

# Suppress internal websockets tracebacks.
# These fire when a browser navigates to localhost:8765 directly (plain HTTP
# instead of a WebSocket upgrade).  That is harmless and expected — the port
# only speaks WebSocket.  The noise would otherwise flood the console.
logging.getLogger('websockets').setLevel(logging.CRITICAL)

# ═══════════════════════════════════════════════════════════════════════
#   Shared state
# ═══════════════════════════════════════════════════════════════════════

esp32_writer: Optional[asyncio.StreamWriter] = None
ws_clients:   Set = set()   # Set[websocket] — type dropped; WebSocketServerProtocol is deprecated in websockets 14+

# ═══════════════════════════════════════════════════════════════════════
#   CSV logger (optional)
# ═══════════════════════════════════════════════════════════════════════

csv_writer = None
csv_file   = None

def open_csv():
    global csv_writer, csv_file
    os.makedirs(LOG_DIR, exist_ok=True)
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(LOG_DIR, f'telem_{ts}.csv')
    csv_file   = open(path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    # Header row: seq + ts_us + all variable names
    csv_writer.writerow(['seq', 'ts_us'] + TELEM_NAMES)
    log.info(f'CSV logging to {path}')

def log_csv_row(seq: int, ts_us: int, values: list):
    if csv_writer:
        csv_writer.writerow([seq, ts_us] + [f'{v:.6f}' for v in values])

# ═══════════════════════════════════════════════════════════════════════
#   Binary packet parser
#
#   Wire format (HDR_SIZE = 16 bytes):
#     [0]     0xFC  magic
#     [1]     0x1A  magic
#     [2]     version  uint8
#     [3]     type     uint8  (0x01 = telemetry)
#     [4..7]  seq       uint32 LE
#     [8..11] timestamp uint32 LE (µs since boot)
#     [12..13] var_count uint16 LE
#     [14..15] payload_len uint16 LE (= var_count × 4)
#     [16..]  floats    IEEE 754 LE
#
#   We sync by searching for the two magic bytes.  The chance of the
#   bytes 0xFC 0x1A appearing inside a float payload is very low, and
#   a misaligned parse is caught immediately by the var_count check.
# ═══════════════════════════════════════════════════════════════════════

# async def read_packet(reader: asyncio.StreamReader) -> Optional[dict]:
#     """
#     Read exactly one telemetry packet from the TCP stream.
#     Returns a dict with 'seq', 'ts_us', 'var_count', 'values' on success.
#     Returns None on connection loss or parse error.
#     """
#     try:
#         # Sync to magic bytes
#         while True:
#             b0 = await reader.readexactly(1)
#             if b0[0] != MAGIC_0:
#                 continue
#             b1 = await reader.readexactly(1)
#             if b1[0] != MAGIC_1:
#                 continue
#             break  # found 0xFC 0x1A

#         # Read the remaining 14 bytes to complete the 16-byte header
#         hdr = await reader.readexactly(14)

#         # Unpack header fields
#         # Format: BB = version(u8) + type(u8)
#         #         II = seq(u32) + timestamp(u32)
#         #         HH = var_count(u16) + payload_len(u16)
#         version, pkt_type, seq, ts_us, var_count, payload_len = \
#             struct.unpack('<BBIIHH', hdr)

#         if pkt_type != PKT_TELEM:
#             # Unknown packet type — skip payload and resync
#             await reader.readexactly(payload_len)
#             return None

#         # Read float payload
#         payload = await reader.readexactly(payload_len)
#         n_floats = payload_len // 4
#         values   = list(struct.unpack(f'<{n_floats}f', payload))

#         return {
#             'seq':       seq,
#             'ts_us':     ts_us,
#             'var_count': var_count,
#             'values':    values,
#         }

#     except asyncio.IncompleteReadError:
#         return None   # connection closed
#     except Exception as e:
#         log.warning(f'Packet parse error: {e}')
#         return None

async def read_packet(reader: asyncio.StreamReader) -> Optional[dict]:
    """
    Read one framed packet. Returns:
      {'type': 'telem', 'seq', 'ts_us', 'var_count', 'values'}
      {'type': 'ctrl_resp', 'ctrl': {name: value, ...}}
    Returns None on connection loss or parse error.
    """
    try:
        # Sync to magic bytes
        while True:
            b0 = await reader.readexactly(1)
            if b0[0] != MAGIC_0:
                continue
            b1 = await reader.readexactly(1)
            if b1[0] != MAGIC_1:
                continue
            break

        hdr = await reader.readexactly(14)
        version, pkt_type, seq, ts_us, var_count, payload_len = \
            struct.unpack('<BBIIHH', hdr)

        payload = await reader.readexactly(payload_len)

        if pkt_type == PKT_TELEM:
            n_floats = payload_len // 4
            values   = list(struct.unpack(f'<{n_floats}f', payload))
            return {
                'type':      'telem',
                'seq':       seq,
                'ts_us':     ts_us,
                'var_count': var_count,
                'values':    values,
            }

        elif pkt_type == PKT_CTRL_RESP:
            try:
                ctrl_dict = json.loads(payload.decode('utf-8'))
            except Exception as e:
                log.warning(f'ctrl_resp JSON parse error: {e}')
                return None
            return {'type': 'ctrl_resp', 'ctrl': ctrl_dict}

        else:
            log.warning(f'Unknown packet type 0x{pkt_type:02X} — skipped')
            return None

    except asyncio.IncompleteReadError:
        return None
    except Exception as e:
        log.warning(f'Packet parse error: {e}')
        return None

# ═══════════════════════════════════════════════════════════════════════
#   Broadcast helper
# ═══════════════════════════════════════════════════════════════════════

async def ws_broadcast(msg: str):
    """Send msg to all connected browsers; prune dead connections."""
    dead = set()
    for client in ws_clients.copy():
        try:
            await client.send(msg)
        except Exception:
            dead.add(client)
    ws_clients.difference_update(dead)

# ═══════════════════════════════════════════════════════════════════════
#   TCP handler  (one instance per ESP32 connection)
# ═══════════════════════════════════════════════════════════════════════

async def tcp_handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    global esp32_writer

    peer = writer.get_extra_info('peername')

    # ── Guard: warn if a previous ESP32 connection is still active ────────
    if esp32_writer is not None and not esp32_writer.is_closing():
        old_peer = esp32_writer.get_extra_info('peername')
        log.warning(
            f'┌─ NEW ESP32 connection from {peer} while {old_peer} still active!')
        log.warning(
            f'└─ Dropping old connection. (ESP32 may have reset or re-connected.)')
        try:
            esp32_writer.close()
            await esp32_writer.wait_closed()
        except Exception:
            pass

    log.info(f'━━━  ESP32 CONNECTED    peer={peer}  ━━━')
    esp32_writer = writer

    last_seq:   int = -1
    drop_total: int = 0
    pkt_count:  int = 0
    t_start = asyncio.get_event_loop().time()

    try:
        while True:
            pkt = await read_packet(reader)
            if pkt is None:
                break  # connection closed or unrecoverable parse error
            
            # ── Ctrl-response: forward to browser and continue ────────────
            if pkt['type'] == 'ctrl_resp':
                ctrl = pkt['ctrl']
                log.info(f'ESP32 → browser (ctrl values): {ctrl}')
                await ws_broadcast(json.dumps({'_ctrl_values': ctrl}))
                continue

            # ── Telemetry ─────────────────────────────────────────────────
            seq       = pkt['seq']
            ts_us     = pkt['ts_us']
            var_count = pkt['var_count']
            values    = pkt['values']

            # Sequence check (telemetry only)
            if last_seq >= 0:
                expected = (last_seq + 1) & 0xFFFFFFFF
                if seq != expected:
                    dropped = (seq - last_seq - 1) & 0xFFFFFFFF
                    drop_total += dropped
                    log.warning(f'Dropped {dropped} packets  '
                                f'(total {drop_total}, seq {last_seq}→{seq})')
            last_seq = seq

            if var_count != len(TELEM_NAMES):
                log.warning(
                    f'var_count mismatch: ESP32 sent {var_count}, '
                    f'Python expects {len(TELEM_NAMES)}. '
                    f'Check TELEM_NAMES matches addTelemetry() order.'
                )

            data: dict = {'seq': seq, 'ts_us': ts_us}
            for i, v in enumerate(values):
                key = TELEM_NAMES[i] if i < len(TELEM_NAMES) else f'v{i}'
                data[key] = round(float(v), 6)

            await ws_broadcast(json.dumps(data))

            if LOG_CSV:
                log_csv_row(seq, ts_us, values)

            # ── Optional CSV logging ──────────────────────────────────────
            if LOG_CSV:
                log_csv_row(seq, ts_us, values)

            # ── Rate log every 500 packets ────────────────────────────────
            pkt_count += 1
            if pkt_count % 500 == 0:
                elapsed = asyncio.get_event_loop().time() - t_start
                rate    = pkt_count / elapsed if elapsed > 0 else 0
                log.info(f'{pkt_count} packets  avg {rate:.1f} Hz  '
                         f'drops {drop_total}')

    except Exception as e:
        log.error(f'TCP handler exception: {e}')

    finally:
        elapsed  = asyncio.get_event_loop().time() - t_start
        avg_rate = pkt_count / elapsed if elapsed > 0 else 0
        log.info(
            f'━━━  ESP32 DISCONNECTED  peer={peer}  '
            f'{pkt_count} pkts  {avg_rate:.1f} Hz avg  '
            f'{drop_total} drops  ━━━'
        )
        # Only clear the global if this handler still owns it
        # (a new connection may have already replaced it)
        if esp32_writer is writer:
            esp32_writer = None
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass

# ═══════════════════════════════════════════════════════════════════════
#   WebSocket handler  (one instance per browser tab)
# ═══════════════════════════════════════════════════════════════════════

async def ws_handler(websocket):   # type annotation removed — WebSocketServerProtocol is deprecated in websockets 14+
    ws_clients.add(websocket)
    peer = websocket.remote_address
    log.info(f'Browser connected  {peer}  ({len(ws_clients)} total)')

    try:
        async for msg in websocket:
            # Validate it's JSON before forwarding
            try:
                ctrl = json.loads(msg)
            except json.JSONDecodeError:
                log.warning(f'Non-JSON from browser: {msg!r}')
                continue

            if esp32_writer is None:
                log.warning('Control received but ESP32 is not connected')
                continue

            # Forward as newline-delimited JSON to ESP32
            line = json.dumps(ctrl) + '\n'
            try:
                esp32_writer.write(line.encode())
                await esp32_writer.drain()
                log.info(f'Control → ESP32: {ctrl}')
            except Exception as e:
                log.error(f'Failed to forward control: {e}')

    except websockets.exceptions.ConnectionClosedOK:
        pass
    except Exception as e:
        log.error(f'WebSocket error: {e}')
    finally:
        ws_clients.discard(websocket)
        log.info(f'Browser disconnected  {peer}  ({len(ws_clients)} remaining)')

# ═══════════════════════════════════════════════════════════════════════
#   Entry point
# ═══════════════════════════════════════════════════════════════════════

async def main():
    if LOG_CSV:
        open_csv()

    tcp_server = await asyncio.start_server(tcp_handler, TCP_HOST, TCP_PORT)
    ws_server  = await websockets.serve(ws_handler, WS_HOST, WS_PORT)

    log.info('─' * 60)
    log.info(f'TCP  listening on {TCP_HOST}:{TCP_PORT}  (ESP32 connects here)')
    log.info(f'WS   listening on ws://{WS_HOST}:{WS_PORT}  (browser connects here)')
    log.info(f'Expecting {len(TELEM_NAMES)} telemetry vars: {TELEM_NAMES}')
    log.info(f'CSV logging: {"ON → " + LOG_DIR if LOG_CSV else "OFF"}')
    log.info('─' * 60)
    log.info('Tip: open index.html — do NOT navigate to localhost:8765 directly.')
    log.info('Waiting for ESP32 connection...')

    async with tcp_server, ws_server:
        await asyncio.gather(
            tcp_server.serve_forever(),
            ws_server.serve_forever(),
        )

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log.info('Bridge stopped.')
        if csv_file:
            csv_file.close()