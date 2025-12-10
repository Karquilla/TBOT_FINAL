import socket
import threading
import sys
from collections import defaultdict

import numpy as np

import matplotlib
matplotlib.use("TkAgg")  # good backend for Windows

# Disable matplotlib key bindings that conflict with your controls
matplotlib.rcParams['keymap.save'] = []    # 's' normally = save figure
matplotlib.rcParams['keymap.quit'] = []    # 'q' normally = quit
matplotlib.rcParams['keymap.yscale'] = []  # 'y'
matplotlib.rcParams['keymap.grid'] = []    # 'g'

import matplotlib.pyplot as plt
from pynput import keyboard


# ==========================
# CONFIG
# ==========================

ROBOT_IP = "192.168.0.83"  # <-- change if robot IP changes
ROBOT_PORT = 9000

WORLD_RADIUS = 100   # cm
GRID_RES = 5      # cm


# ==========================
# UDP socket
# ==========================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.1)

# ==========================
# Shared state
# ==========================

path_x = []
path_y = []
heatmap_counts = defaultdict(int)

running = True


# ==========================
# Helpers
# ==========================

def world_to_grid(x, y):
    """Map world coordinate (x,y) into heatmap grid indices (i,j)."""
    half = WORLD_RADIUS
    if not (-half <= x <= half and -half <= y <= half):
        return None
    size = int((2 * WORLD_RADIUS) / GRID_RES)
    gx = int((x + half) / (2 * half) * size)
    gy = int((y + half) / (2 * half) * size)
    return gx, gy


def send_command(cmd: str):
    """Send a single command string over UDP to the robot."""
    if not cmd:
        return
    data = cmd.encode("utf-8")
    try:
        sock.sendto(data, (ROBOT_IP, ROBOT_PORT))
        print("Sent command:", cmd)
        sys.stdout.flush()
    except OSError:
        # socket may be closed during shutdown
        pass


# ==========================
# Telemetry thread
# ==========================

def telemetry_thread():
    global running, path_x, path_y, heatmap_counts

    while running:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError:
            # socket closed
            break

        try:
            line = data.decode("utf-8").strip()
        except UnicodeDecodeError:
            continue

        if not line:
            continue

        parts = line.split(",")
        if len(parts) < 1:
            continue

        tag = parts[0]

        if tag == "P" and len(parts) == 4:
            # Pose telemetry: P,x,y,theta
            try:
                x = float(parts[1])
                y = float(parts[2])
            except ValueError:
                continue
            path_x.append(x)
            path_y.append(y)

        elif tag == "L" and len(parts) == 3:
            # Lidar point: L,x,y
            try:
                lx = float(parts[1])
                ly = float(parts[2])
            except ValueError:
                continue
            cell = world_to_grid(lx, ly)
            if cell is not None:
                heatmap_counts[cell] += 1


# ==========================
# Keyboard listener (pynput)
# ==========================

def on_press(key):
    """
    Global key handler using pynput.

    Controls:
      1-7 : select Task 1..7 on robot
      W   : forward
      S   : stop
      B   : backward / reverse
      A   : turn left
      D   : turn right
      Space : stop
      Q or Esc : quit client
    """
    global running

    # Letter / number keys
    try:
        k = key.char
        if k is None:
            return
        k = k.upper()
    except AttributeError:
        # Special keys (space, esc, etc.)
        if key == keyboard.Key.space:
            send_command("S")   # stop
        elif key == keyboard.Key.esc:
            print("ESC pressed, quitting...")
            running = False
        return

    if k in {"1", "2", "3", "4", "5", "6", "7"}:
        send_command(k)
    elif k == "W":
        send_command("F")       # forward
    elif k == "A":
        send_command("L")       # left
    elif k == "D":
        send_command("R")       # right
    elif k == "B":
        send_command("B")       # backward / reverse
    elif k == "S":
        send_command("S")       # stop
    elif k == "Q":
        print("Q pressed, quitting...")
        running = False


def start_keyboard_listener():
    listener = keyboard.Listener(on_press=on_press)
    listener.daemon = True
    listener.start()
    return listener


# ==========================
# Main (GUI in main thread)
# ==========================

def main():
    global running, path_x, path_y, heatmap_counts

    print("TB Final Project Client")
    print("========================")
    print(f"Robot IP   : {ROBOT_IP}")
    print(f"Robot Port : {ROBOT_PORT}")
    print()
    print("Controls (global via pynput):")
    print("  1-7 : select Task 1..7 on robot")
    print("  W   : forward")
    print("  B   : backward / reverse")
    print("  A   : turn left")
    print("  D   : turn right")
    print("  S   : stop")
    print("  Space : stop")
    print("  Q or ESC : quit client")
    print()

    # Start background threads
    t_tele = threading.Thread(target=telemetry_thread, daemon=True)
    t_tele.start()

    kb_listener = start_keyboard_listener()

    # ---- Plot setup (must be in main thread) ----
    plt.ion()
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))

    ax_path = axes[0]
    ax_map = axes[1]

    # Path plot
    path_line, = ax_path.plot([], [], "-")
    ax_path.set_title("Robot Path")
    ax_path.set_xlabel("X (m)")
    ax_path.set_ylabel("Y (m)")
    ax_path.set_aspect("equal", "box")
    ax_path.set_xlim(-WORLD_RADIUS, WORLD_RADIUS)
    ax_path.set_ylim(-WORLD_RADIUS, WORLD_RADIUS)
    ax_path.grid(True)

    # Heatmap plot
    size = int((2 * WORLD_RADIUS) / GRID_RES)
    grid = np.zeros((size, size), dtype=float)
    img = ax_map.imshow(
        grid,
        origin="lower",
        extent=[-WORLD_RADIUS, WORLD_RADIUS, -WORLD_RADIUS, WORLD_RADIUS],
        interpolation="nearest",
        aspect="equal"
    )
    ax_map.set_title("Lidar Heatmap")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")

    plt.tight_layout()

    try:
        while running:
            # Update path
            if path_x and path_y:
                path_line.set_data(path_x, path_y)

            # Update heatmap
            grid.fill(0.0)
            for (gx, gy), c in heatmap_counts.items():
                if 0 <= gx < size and 0 <= gy < size:
                    grid[gy, gx] = c
            img.set_data(grid)
            if grid.max() > 0:
                img.set_clim(0, grid.max())

            plt.pause(0.05)

    except KeyboardInterrupt:
        running = False
    finally:
        running = False
        sock.close()
        plt.ioff()
        try:
            plt.show(block=False)
        except Exception:
            pass
        print("Exiting client...")


if __name__ == "__main__":
    main()
