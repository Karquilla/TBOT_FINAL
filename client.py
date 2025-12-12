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
GRID_RES = 5         # cm

# Manual drive PWM settings (tune these to taste)
FWD_FAST = 500   # forward full speed
FWD_SLOW = 250   # forward slower wheel for arcs
REV_FAST = -500  # reverse full speed
REV_SLOW = -250  # reverse slower wheel for arcs
TURN_SPEED = 500 # pivot speed (left/right in place)


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

# control state
manual_mode = False           # True only when Task 6 is selected
pressed_move_keys = set()     # current set of held movement keys: {"W","A","S","D","B"}
last_pwm = (0, 0)             # last (left, right) PWM we sent


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


def send_pwm(left: int, right: int):
    """Send wheel PWM command as M,left,right (for manual drive)."""
    global last_pwm
    if (left, right) == last_pwm:
        return
    last_pwm = (left, right)
    cmd = f"M,{left},{right}"
    send_command(cmd)


def stop_robot():
    """Send a stop command (zero PWM + optional 'S')."""
    global last_pwm
    last_pwm = (0, 0)
    # main stop: zero wheel command
    send_pwm(0, 0)
    # also send legacy 'S' in case robot still listens for it
    send_command("S")


def compute_pwm_from_keys(keys: set[str]) -> tuple[int, int]:
    """
    Decide (leftPWM, rightPWM) based on which movement keys are held.

    Movement keys (only active when Task 6 selected):
      W = forward
      B = backward
      A = pivot/arc left
      D = pivot/arc right

    Combos:
      W + D -> forward-right arc
      W + A -> forward-left arc
      B + D -> backward-right arc
      B + A -> backward-left arc

    Singles:
      {W}       -> straight forward
      {B}       -> straight backward
      {A}       -> pivot left (in place)
      {D}       -> pivot right (in place)

    S is handled separately as an immediate stop, not as a held movement key.
    """
    # Normalize to just the movement keys we care about
    forward = "W" in keys
    back    = "B" in keys
    left    = "A" in keys
    right   = "D" in keys

    # No meaningful movement keys -> stop
    if not (forward or back or left or right):
        return 0, 0

    # If both forward and back somehow held, neutral
    if forward and back:
        return 0, 0

    # Forward cases
    if forward:
        # Arcs
        if right and not left:
            # forward-right arc: left faster than right
            return FWD_FAST, FWD_SLOW
        if left and not right:
            # forward-left arc: right faster than left
            return FWD_SLOW, FWD_FAST
        # Straight forward (W, or W+A+D → treat as straight)
        return FWD_FAST, FWD_FAST

    # Backward cases
    if back:
        if right and not left:
            # backward-right arc
            return REV_FAST, REV_SLOW
        if left and not right:
            # backward-left arc
            return REV_SLOW, REV_FAST
        # Straight backward
        return REV_FAST, REV_FAST

    # No forward/back, maybe pivots
    if left and not right:
        # pivot left: left backward, right forward
        return -TURN_SPEED, TURN_SPEED
    if right and not left:
        # pivot right: left forward, right backward
        return TURN_SPEED, -TURN_SPEED

    # Both A and D but no W/B: treat as stop
    return 0, 0


def update_motion():
    """Recompute PWM from pressed_move_keys and send it (only in manual mode)."""
    if not manual_mode:
        return
    left, right = compute_pwm_from_keys(pressed_move_keys)
    if left == 0 and right == 0:
        stop_robot()
    else:
        send_pwm(left, right)


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

MOVE_KEYS = {"W", "A", "D", "B"}  # movement keys; S is stop only


def on_press(key):
    """
    Global key handler using pynput.

    MENU:
      1-7 : select Task 1..7 on robot
             (Task 6 = Manual Drive mode)

    MANUAL DRIVE (only when Task 6 selected):
      W   : forward   (hold)
      B   : backward  (hold)
      A   : left pivot / arc (hold)
      D   : right pivot / arc (hold)

      Combos:
        W + D -> forward-right arc
        W + A -> forward-left arc
        B + D -> backward-right arc
        B + A -> backward-left arc

      Release all: STOP.

    OTHER:
      S     : immediate stop (any time)
      Space : immediate stop (any time)
      Q or Esc : quit client
    """
    global running, manual_mode, pressed_move_keys

    # Letter / number keys
    try:
        k = key.char
        if k is None:
            return
        k = k.upper()
    except AttributeError:
        # Special keys (space, esc, etc.)
        if key == keyboard.Key.space:
            pressed_move_keys.clear()
            stop_robot()
        elif key == keyboard.Key.esc:
            print("ESC pressed, quitting...")
            running = False
        return

    # ---- Task selection menu ----
    if k in {"1", "2", "3", "4", "5", "6", "7"}:
        send_command(k)  # tell robot which task to run

        # Only Task 6 is manual mode
        manual_mode = (k == "6" or k == "7")
        pressed_move_keys.clear()
        stop_robot()  # ensure robot is stopped on task switch

        if manual_mode:
            print("Manual Drive MODE (Task 6) ENABLED")
        else:
            print(f"Task {k} selected, Manual Drive DISABLED")
        return

    # Stop key (always available)
    if k == "S":
        pressed_move_keys.clear()
        stop_robot()
        return

    # Quit shortcut
    if k == "Q":
        print("Q pressed, quitting...")
        running = False
        return

    # ---- Manual drive keys only if Task 6 is selected ----
    if not manual_mode:
        if k in MOVE_KEYS:
            print("Ignoring drive key (not in Task 6 / Manual Drive).")
        return

    # From here on: manual_mode == True
    if k in MOVE_KEYS:
        # If already held, ignore auto-repeat
        if k not in pressed_move_keys:
            pressed_move_keys.add(k)
            update_motion()


def on_release(key):
    """Update motion / send stop when movement keys are released (only in manual mode)."""
    global manual_mode, pressed_move_keys

    if not manual_mode:
        return

    # Try to get character for letter keys
    try:
        k = key.char
        if k is None:
            return
        k = k.upper()
    except AttributeError:
        return

    if k in MOVE_KEYS:
        if k in pressed_move_keys:
            pressed_move_keys.discard(k)
            update_motion()


def start_keyboard_listener():
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
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
    print("MENU:")
    print("  1-7 : select Task 1..7 on robot")
    print("        (Task 6 = Manual Drive mode)")
    print()
    print("MANUAL DRIVE CONTROLS (only active in Task 6):")
    print("  W   : forward   (hold)")
    print("  B   : backward  (hold)")
    print("  A   : pivot left / arc left (hold)")
    print("  D   : pivot right / arc right (hold)")
    print("  Combos: W+D, W+A, B+D, B+A for arcs")
    print("  Release all move keys: STOP")
    print()
    print("OTHER:")
    print("  S     : STOP immediately")
    print("  Space : STOP immediately")
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
    ax_path.set_xlabel("X")
    ax_path.set_ylabel("Y")
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
    ax_map.set_xlabel("X")
    ax_map.set_ylabel("Y")

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
