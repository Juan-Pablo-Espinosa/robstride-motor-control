#!/usr/bin/env python3
"""
GR-00 LED Commander — Jetson Terminal UI
Fancy curses-based TUI to control LEDs on the PolarFire via ROS2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
import time
import curses
import sys


# ── State ─────────────────────────────────────────────────────────────────────

led_states = {i: 0 for i in range(1, 9)}   # 0=off 1=on 2=blink
log_lines   = []
ros_node    = None
running     = True


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class LEDCommander(Node):
    def __init__(self):
        super().__init__('led_commander')
        self.pub = self.create_publisher(Int32MultiArray, 'led_command', 10)

    def send(self, led_id: int, mode: int):
        msg = Int32MultiArray()
        msg.data = [led_id, mode]
        self.pub.publish(msg)
        names = {0: 'OFF', 1: 'ON ', 2: 'BLK'}
        add_log(f'LED {led_id}  ->  {names[mode]}', 'action')

    def send_all(self, mode: int, delay=0.0):
        for i in range(1, 9):
            led_states[i] = mode
            self.send(i, mode)
            if delay:
                time.sleep(delay)

    def rainbow(self):
        add_log('Rainbow sweep...', 'info')
        for _ in range(2):
            for i in range(1, 9):
                for j in range(1, 9):
                    led_states[j] = 0
                led_states[i] = 1
                self.send_all_state()
                time.sleep(0.07)
            for i in range(8, 0, -1):
                for j in range(1, 9):
                    led_states[j] = 0
                led_states[i] = 1
                self.send_all_state()
                time.sleep(0.07)
        for i in range(1, 9):
            led_states[i] = 0
        self.send_all_state()
        add_log('Rainbow done.', 'info')

    def send_all_state(self):
        for i in range(1, 9):
            msg = Int32MultiArray()
            msg.data = [i, led_states[i]]
            self.pub.publish(msg)

    def demo(self):
        add_log('Demo sequence starting...', 'info')
        self.send_all(1);       time.sleep(0.8)
        self.send_all(0);       time.sleep(0.3)
        self.rainbow()
        self.send_all(2);       time.sleep(2.0)
        self.send_all(0)
        add_log('Demo complete.', 'info')


# ── Log ───────────────────────────────────────────────────────────────────────

def add_log(msg: str, kind: str = ''):
    ts = time.strftime('%H:%M:%S')
    log_lines.append((f'[{ts}] {msg}', kind))
    if len(log_lines) > 200:
        log_lines.pop(0)


# ── Command parser ────────────────────────────────────────────────────────────

def handle_command(cmd: str) -> bool:
    global running
    parts = cmd.strip().lower().split()
    if not parts:
        return True

    verb = parts[0]

    if verb in ('q', 'quit', 'exit'):
        running = False
        return False

    elif verb == 'clear':
        ros_node.send_all(0)
        for i in range(1, 9):
            led_states[i] = 0

    elif verb == 'rainbow':
        threading.Thread(target=ros_node.rainbow, daemon=True).start()

    elif verb == 'demo':
        threading.Thread(target=ros_node.demo, daemon=True).start()

    elif verb == 'party':
        ros_node.send_all(2)
        add_log('Party mode! (runs 4s)', 'info')
        def stop():
            time.sleep(4)
            ros_node.send_all(0)
            for i in range(1, 9):
                led_states[i] = 0
        threading.Thread(target=stop, daemon=True).start()

    elif verb in ('on', 'off', 'blink'):
        mode = {'on': 1, 'off': 0, 'blink': 2}[verb]
        if len(parts) < 2:
            add_log(f'Usage: {verb} <1-8 | all>', 'warn')
            return True
        target = parts[1]
        if target == 'all':
            ros_node.send_all(mode)
            for i in range(1, 9):
                led_states[i] = mode
        else:
            try:
                lid = int(target)
                if 1 <= lid <= 8:
                    led_states[lid] = mode
                    ros_node.send(lid, mode)
                else:
                    add_log('LED ID must be 1-8', 'warn')
            except ValueError:
                add_log(f'Bad LED ID: {target}', 'warn')
    else:
        add_log(f'Unknown command: {verb}', 'warn')

    return True


# ── TUI ───────────────────────────────────────────────────────────────────────

def draw(stdscr, input_buf: list):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()

    curses.init_pair(1, curses.COLOR_CYAN,    -1)
    curses.init_pair(2, curses.COLOR_GREEN,   -1)
    curses.init_pair(3, curses.COLOR_YELLOW,  -1)
    curses.init_pair(4, curses.COLOR_RED,     -1)
    curses.init_pair(5, curses.COLOR_WHITE,   -1)
    curses.init_pair(6, curses.COLOR_MAGENTA, -1)   # Changed from BLACK to MAGENTA (bright & cool)
    curses.init_pair(7, curses.COLOR_MAGENTA, -1)

    CYAN    = curses.color_pair(1)
    GREEN   = curses.color_pair(2)
    YELLOW  = curses.color_pair(3)
    RED     = curses.color_pair(4)
    NORMAL  = curses.color_pair(5)
    MUTED   = curses.color_pair(6)          # Now bright Magenta
    MAGENTA = curses.color_pair(7)
    BOLD    = curses.A_BOLD

    stdscr.nodelay(True)
    stdscr.keypad(True)

    while running:
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        # Header
        header = ' GR-00  LED COMMANDER '
        sub    = ' jetson -> polarfire via ros2 '
        stdscr.attron(CYAN | BOLD)
        stdscr.addstr(0, 0, '-' * w)
        stdscr.addstr(1, max(0, (w - len(header)) // 2), header)
        stdscr.attroff(CYAN | BOLD)
        stdscr.attron(MUTED)
        try: stdscr.addstr(2, max(0, (w - len(sub)) // 2), sub)
        except: pass
        stdscr.attroff(MUTED)
        stdscr.attron(CYAN | BOLD)
        stdscr.addstr(3, 0, '-' * w)
        stdscr.attroff(CYAN | BOLD)

        # LED Strip - "LED ARRAY" now in bright Red + Bold (very visible)
        stdscr.attron(RED | BOLD)
        stdscr.addstr(5, 2, '[ LED ARRAY ]')
        stdscr.attroff(RED | BOLD)

        cell_w = min(9, (w - 4) // 8)
        for i in range(1, 9):
            x = 2 + (i - 1) * cell_w
            state = led_states[i]

            if state == 1:
                bulb  = '( O )'
                label = f' L{i:02d} '
                color = GREEN | BOLD
            elif state == 2:
                tick = int(time.time() * 2) % 2
                bulb  = '( * )' if tick else '(   )'
                label = f' L{i:02d} '
                color = YELLOW | BOLD
            else:
                bulb  = '(   )'
                label = f' L{i:02d} '
                color = MUTED | BOLD          # Off LEDs now also use bright magenta

            if x + cell_w < w:
                stdscr.attron(color)
                try: stdscr.addstr(6, x, bulb[:cell_w])
                except: pass
                try: stdscr.addstr(7, x, label[:cell_w])
                except: pass
                stdscr.attroff(color)

        # State row
        for i in range(1, 9):
            s = led_states[i]
            tag = 'ON ' if s == 1 else 'BLK' if s == 2 else 'off'
            col = GREEN if s == 1 else YELLOW if s == 2 else MUTED
            stdscr.attron(col)
            try: stdscr.addstr(9, 2 + (i-1)*cell_w, tag[:cell_w])
            except: pass
            stdscr.attroff(col)

        # Divider
        stdscr.attron(CYAN)
        try: stdscr.addstr(11, 0, '-' * w)
        except: pass
        stdscr.attroff(CYAN)

        # Help
        help_row = 12
        stdscr.attron(MUTED | BOLD)
        stdscr.addstr(help_row, 2, '[ COMMANDS ]')
        stdscr.attroff(MUTED | BOLD)

        # ... (rest of help and log stays the same)

        cmds = [
            ('on <id|all>',    'turn on'),
            ('off <id|all>',   'turn off'),
            ('blink <id|all>', 'blink'),
            ('clear',          'all off'),
            ('rainbow',        'sweep animation'),
            ('party',          'all blink 4s'),
            ('demo',           'full sequence'),
            ('q',              'quit'),
        ]

        col_w = w // 2 - 2
        for idx, (c, desc) in enumerate(cmds):
            row = help_row + 1 + (idx % 4)
            col = 2 if idx < 4 else col_w + 2
            stdscr.attron(CYAN)
            try: stdscr.addstr(row, col, c.ljust(18))
            except: pass
            stdscr.attroff(CYAN)
            stdscr.attron(MUTED)
            try: stdscr.addstr(row, col + 18, desc)
            except: pass
            stdscr.attroff(MUTED)

        # Log
        log_top = help_row + 6
        stdscr.attron(CYAN)
        try: stdscr.addstr(log_top, 0, '-' * w)
        except: pass
        stdscr.attroff(CYAN)
        stdscr.attron(MUTED)
        try: stdscr.addstr(log_top + 1, 2, '[ LOG ]')
        except: pass
        stdscr.attroff(MUTED)

        log_area_h = h - log_top - 5
        visible = log_lines[-log_area_h:] if log_area_h > 0 else []
        for li, (line, kind) in enumerate(visible):
            row = log_top + 2 + li
            if row >= h - 3:
                break
            color = GREEN if kind == 'action' else YELLOW if kind == 'warn' else CYAN if kind == 'info' else NORMAL
            try: stdscr.addstr(row, 2, line[:w-4], color)
            except: pass

        # Input bar
        stdscr.attron(CYAN)
        try: stdscr.addstr(h - 2, 0, '-' * w)
        except: pass
        stdscr.attroff(CYAN)
        prompt = 'gr00> '
        stdscr.attron(MAGENTA | BOLD)
        stdscr.addstr(h - 1, 0, prompt)
        stdscr.attroff(MAGENTA | BOLD)
        buf_str = ''.join(input_buf)
        try: stdscr.addstr(h - 1, len(prompt), buf_str + '_')
        except: pass

        stdscr.refresh()

        # Input handling (unchanged)
        try:
            ch = stdscr.getch()
        except:
            ch = -1

        if ch == -1:
            time.sleep(0.05)
            continue
        elif ch in (curses.KEY_ENTER, 10, 13):
            cmd = ''.join(input_buf).strip()
            input_buf.clear()
            if cmd:
                add_log(f'> {cmd}', '')
                handle_command(cmd)
        elif ch in (curses.KEY_BACKSPACE, 127, 8):
            if input_buf:
                input_buf.pop()
        elif 32 <= ch <= 126:
            input_buf.append(chr(ch))


def ros_spin(node):
    rclpy.spin(node)


def main():
    global ros_node

    rclpy.init()
    ros_node = LEDCommander()

    spin_thread = threading.Thread(target=ros_spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    add_log('GR-00 LED Commander ready', 'info')
    add_log('Publishing to /led_command', 'info')

    input_buf = []
    try:
        curses.wrapper(lambda s: draw(s, input_buf))
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        rclpy.shutdown()
        ros_node.destroy_node()


if __name__ == '__main__':
    main()