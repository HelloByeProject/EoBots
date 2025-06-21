#LulsSquad
import pydirectinput
import time
import pymem
import threading
import frida
import pyautogui
import tkinter as tk
from tkinter import ttk
import json
import random

from queue import PriorityQueue

import bresenham


# --- Default Configuration ---
config = {
    "expAddress": 0x000000,  # TODO: set actual EXP offset
    "walkAddress": 0xE6C23,
    "npcAddress": 0x1E65C4,
    "directionalAddress": 0x67DEE,
    "varOffset": 0x92,
    "pickupLoops": 5,
    "pickupPause": 0.05,
    "pickupMoveDuration": 0.05
}

# --- GUI to configure addresses and ranges ---
def show_config_window():
    def save():
        try:
            config["expAddress"] = int(exp_addr.get(), 16)
            config["walkAddress"] = int(walk_addr.get(), 16)
            config["npcAddress"] = int(npc_addr.get(), 16)
            config["directionalAddress"] = int(dir_addr.get(), 16)
            config["varOffset"] = int(var_off.get(), 16)
            config["pickupLoops"] = int(loops.get())
            config["pickupPause"] = float(pause.get())
            config["pickupMoveDuration"] = float(move_dur.get())
            window.destroy()
        except ValueError:
            messagebox.showerror("Invalid Input", "Please check your values and formats.")

    window = tk.Tk()
    window.title("CeraBot Configuration")

    labels = ["EXP Address (hex)", "Walk Address (hex)", "NPC Address (hex)",
              "Directional Address (hex)", "Var Offset (hex)",
              "Pickup Loops", "Pause Between Clicks (s)", "Move Duration (s)"]
    entries = []
    defaults = [hex(config["expAddress"]), hex(config["walkAddress"]), hex(config["npcAddress"]),
                hex(config["directionalAddress"]), hex(config["varOffset"]),
                str(config["pickupLoops"]), str(config["pickupPause"]), str(config["pickupMoveDuration"])]

    for i, text in enumerate(labels):
        ttk.Label(window, text=text).grid(row=i, column=0, sticky="w", padx=5, pady=2)
        ent = ttk.Entry(window)
        ent.insert(0, defaults[i])
        ent.grid(row=i, column=1, padx=5, pady=2)
        entries.append(ent)

    exp_addr, walk_addr, npc_addr, dir_addr, var_off, loops, pause, move_dur = entries

    ttk.Button(window, text="Save", command=save).grid(row=len(labels), column=0, columnspan=2, pady=10)
    window.mainloop()

# --- Pickup-on-EXP Variables & Setup ---
from pynput import mouse, keyboard as pkeyboard

click_positions = []
pickup_active = False
pyautogui.PAUSE = 0

def setup_click_points():
    def on_click(x, y, button, pressed):
        if pressed and button == mouse.Button.left:
            click_positions.append((x, y))
            print(f"Point {len(click_positions)} saved: {x}, {y}")
            if len(click_positions) == 4:
                return False

    print("Click Left Mouse Button on 4 points to mark pickup spots...")
    with mouse.Listener(on_click=on_click) as listener:
        listener.join()
    print(f"Pickup spots saved: {click_positions}")


def run_pickup_loop():
    global pickup_active
    if pickup_active:
        return
    pickup_active = True

    def loop():
        global pickup_active
        duration_per_point = 0.05
        pause_clicks = 0.05
        pause_loops = 0.05
        loops = 0
        while loops < 5:
            for x, y in click_positions:
                pyautogui.moveTo(x, y, duration=duration_per_point)
                pyautogui.mouseDown()
                pyautogui.mouseUp()
                time.sleep(pause_clicks)
            loops += 1
            time.sleep(pause_loops)
        pickup_active = False

    threading.Thread(target=loop, daemon=True).start()


def get_current_exp():
    """
    Read the current EXP value from game memory.
    """
    # Ensure pymem is initialized
    initialize_pymem()
    try:
        # Read 4-byte int at expAddress (adjust type/size as needed)
        exp_value = pm.read_int(expAddress)
        return exp_value
    except Exception as e:
        print(f"Error reading EXP: {e}")
        return 0

# Monitor EXP gains and trigger pickup
def monitor_exp_and_combat():
    prev = get_current_exp()
    while True:
        time.sleep(0.5)
        curr = get_current_exp()
        if curr > prev:
            print("EXP gained! Running pickup loop...")
            run_pickup_loop()
            prev = curr

#memory addresses (update these values as needed)
walkAddress = "0xE6C23"
npcAddress = "0x1E65C4"
directionalAddress = "0x67DEE"
varOffset = 0x92

x_address = None
y_address = None
directional_address = None
xstarted = 0
debug = 0

last_direction = None
wandering_target = None
stuck_timer_start = None
last_position = None
pm = None  # Global pymem instance

def initialize_pymem():
    global pm
    if pm is None:
        pm = pymem.Pymem("Endless.exe")

def press_key(key, presses=2, delay=0.1):
    """
    Presses a key and optionally waits for a specified delay.
    """
    pydirectinput.press(key, presses)
    time.sleep(delay)

def hold_key(key):
    pydirectinput.keyDown(key)

def release_key(key):
    pydirectinput.keyUp(key)

def on_message_xy(message, data):
    global xstarted, x_address, y_address
    if message['type'] == 'send':
        addresses = message['payload']
        x_address = int(addresses['x_address'], 16)
        y_address = int(addresses['y_address'], 16)
        if debug == 1:
            print(f"X Address: {hex(x_address)}, Y Address: {hex(y_address)}")
        # Mark as completed and detach session
        xstarted = 1
        session.detach()
    else:
        print(f"Error: {message}")

def start_frida_session_xy(walk_address):
    global session
    session = frida.attach("Endless.exe")
    print("XY Started - Waiting for you to move to begin")
    script_code = f"""
    var baseAddress = Module.findBaseAddress("Endless.exe").add(ptr({walk_address}));
    Interceptor.attach(baseAddress, {{
        onEnter: function(args) {{
            var xAddress = this.context.ecx.add(0x08);
            var yAddress = xAddress.add(0x04);
            send({{x_address: xAddress.toString(), y_address: yAddress.toString()}});
        }}
    }});
    """
    script = session.create_script(script_code)
    script.on('message', on_message_xy)
    script.load()

    while xstarted == 0:
        continue

    print("Session completed and detached.")

def on_message_directional(message, data):
    global directional_address, xstarted
    if message['type'] == 'send':
        payload = message['payload']
        directional_address = int(payload.get('directional_address'), 16)
        character_direction = payload.get('character_direction')
        if debug == 1:
            print(f"Character Direction Address: {directional_address}")
            print(f"Character Direction Value: {character_direction}")
        xstarted = 2
        session.detach()
    elif message['type'] == 'error':
        print(f"Error: {message['stack']}")

def start_frida_session_directional(target_address):
    global session
    session = frida.attach("Endless.exe")
    print("Directional Started - Waiting for mov [ebx+55],dl to execute")
    script_code = f"""
    var baseAddress = Module.findBaseAddress("Endless.exe").add(ptr("{target_address}"));
    Interceptor.attach(baseAddress, {{
        onEnter: function(args) {{
            var ebxValue = this.context.ebx;
            var characterDirectionAddress = ebxValue.add(0x55);
            var characterDirection = characterDirectionAddress.readU8();
            send({{directional_address: characterDirectionAddress.toString(), character_direction: characterDirection.toString()}});
        }}
    }});
    """
    script = session.create_script(script_code)
    script.on('message', on_message_directional)
    script.load()

    while xstarted == 1:
        continue

    print("Directional Session Completed.")

class PlayerDataManager:
    def __init__(self):
        self.data = {
            "x": 0,
            "y": 0,
            "direction": 0
        }

    def update(self, x, y, direction):
        self.data["x"] = x
        self.data["y"] = y
        self.data["direction"] = direction

    def get_data(self):
        return self.data

class AddressManager:
    def __init__(self):
        self.addresses = {}

    def add_address(self, address):
        address1 = int(address, 16)
        address2 = address1 + 2
        address1_hex = hex(address1).upper()
        address2_hex = hex(address2).upper()
        if address1_hex not in self.addresses:
            self.addresses[address1_hex] = {
                "paired_address": address2_hex,
                "last_x": None,
                "last_y": None,
                "last_moved": time.time(),
                "is_dead_counter": 0,
                "last_is_dead_value": None
            }
            return True
        return False

    def remove_address(self, address):
        address1 = int(address, 16)
        address1_hex = hex(address1).upper()
        if address1_hex in self.addresses:
            del self.addresses[address1_hex]
            return True
        return False

    def list_addresses(self):
        return [{"X": x, "Y": data["paired_address"]} for x, data in self.addresses.items()]

manager = AddressManager()
player_data_manager = PlayerDataManager()
map_data = []

class PlayerDataPopup:
    def __init__(self, player_data_manager):
        self.player_data_manager = player_data_manager
        self.root = tk.Tk()
        self.root.title("Player Data")
        self.labels = {}
        self.create_widgets()
        self.create_styles()
        self.update_ui()

    def create_widgets(self):
        self.frame = ttk.Frame(self.root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        for idx, text in enumerate(["X", "Y", "Direction"], start=2):
            ttk.Label(self.frame, text=text).grid(row=idx, column=0, sticky=tk.W)
            value_label = ttk.Label(self.frame, text="0")
            value_label.grid(row=idx, column=1, sticky=tk.E)
            self.labels[text] = value_label
        self.canvas = tk.Canvas(self.frame, width=200, height=200, bg="white")
        self.canvas.grid(row=0, column=2, rowspan=6, padx=10)

    def create_styles(self):
        style = ttk.Style(self.root)
        style.theme_use('default')
        style.configure("red.Horizontal.TProgressbar", troughcolor='white', background='red')
        style.configure("blue.Horizontal.TProgressbar", troughcolor='white', background='blue')

    def update_ui(self):
        data = self.player_data_manager.get_data()
        self.labels["X"].config(text=data["x"])
        self.labels["Y"].config(text=data["y"])
        self.labels["Direction"].config(text=data["direction"])
        self.draw_map()
        self.root.after(100, self.update_ui)

    def draw_map(self):
        self.canvas.delete("all")
        player_x = self.player_data_manager.get_data()["x"]
        player_y = self.player_data_manager.get_data()["y"]
        canvas_size = 200
        max_x = 40
        max_y = 40
        center_x = canvas_size / 2
        center_y = canvas_size / 2
        player_radius = 5
        self.canvas.create_oval(center_x - player_radius, center_y - player_radius,
                                center_x + player_radius, center_y + player_radius,
                                fill="blue", outline="black")
        for item in map_data:
            if item["type"] == "npc":
                npc_x = item["X"]
                npc_y = item["Y"]
                canvas_x = center_x + (npc_x - player_x) * (canvas_size / max_x)
                canvas_y = center_y + (npc_y - player_y) * (canvas_size / max_y)
                self.canvas.create_oval(canvas_x - 3, canvas_y - 3,
                                        canvas_x + 3, canvas_y + 3,
                                        fill="red", outline="black")

    def run(self):
        self.root.mainloop()

def check_player_data(x_address, y_address, directional_address):
    global pm
    initialize_pymem()
    try:
        temp_map_data = []
        while True:
            try:
                x = pm.read_short(x_address)
                y = pm.read_short(y_address)
                direction = pm.read_bytes(directional_address, 1)[0]
                player_data_manager.update(x, y, direction)
                temp_map_data = [{
                    "type": "player",
                    "X": x,
                    "Y": y,
                    "direction": direction
                }]
                for addr, data in list(manager.addresses.items()):
                    address_x = int(addr, 16)
                    address_y = int(data["paired_address"], 16)
                    try:
                        value_x = pm.read_short(address_x)
                        value_y = pm.read_short(address_y)
                        temp_map_data.append({
                            "type": "npc",
                            "X": value_x,
                            "Y": value_y,
                            "address_x": addr,
                            "address_y": data["paired_address"]
                        })
                    except:
                        pass
                map_data.clear()
                map_data.extend(temp_map_data)
            except Exception as e:
                print(f"Error reading player data: {e}")
            time.sleep(0.1)
    except Exception as e:
        print(f"Failed to initialize memory reading: {e}")

def on_message(message, data):
    if message['type'] == 'send':
        payload = message['payload']
        action = payload.get('action')
        address = payload.get('address')
        if action == 'add' and address is not None:
            manager.add_address(address)

def start_frida(npc_address):
    print("Npc Started")
    frida_script = f"""
Interceptor.attach(Module.findBaseAddress("Endless.exe").add({npc_address}), {{
    onEnter: function(args) {{
        var eax = this.context.eax.toInt32();
        var offset = {varOffset};
        var address = eax + offset;
        var addressHex = '0x' + address.toString(16).toUpperCase();
        send({{action: 'add', address: addressHex}});
    }}
}});
"""
    session = frida.attach("Endless.exe")
    script = session.create_script(frida_script)
    script.on('message', on_message)
    script.load()

def check_values():
    global pm
    initialize_pymem()
    while True:
        try:
            addresses_to_remove = []
            current_time = time.time()
            player_x = player_data_manager.data["x"]
            player_y = player_data_manager.data["y"]
            for x in list(manager.addresses.keys()):
                data = manager.addresses[x]
                address_x = int(x, 16)
                address_y = int(data["paired_address"], 16)
                try:
                    value_x = pm.read_short(address_x)
                    value_y = pm.read_short(address_y)
                    is_dead_address = address_x - 0x58
                    is_dead = pm.read_bool(is_dead_address)
                    if is_dead:
                        addresses_to_remove.append(x)
                        continue
                    last_x = data["last_x"]
                    last_y = data["last_y"]
                    last_moved = data["last_moved"]
                    if value_x != last_x or value_y != last_y:
                        manager.addresses[x]["last_x"] = value_x
                        manager.addresses[x]["last_y"] = value_y
                        manager.addresses[x]["last_moved"] = current_time
                    else:
                        if current_time - last_moved > 10:
                            addresses_to_remove.append(x)
                    if value_x == 0 or value_x > 100 or value_y == 0 or value_y > 100:
                        addresses_to_remove.append(x)
                except:
                    addresses_to_remove.append(x)
            for address in addresses_to_remove:
                manager.remove_address(address)
        except Exception as e:
            print(f"Error checking values: {e}")
        time.sleep(0.05)

def load_walkable_tiles(file_path='walkable.json'):
    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
            if 'safe_tiles' in data and data['safe_tiles']:
                return {(tile['X'], tile['Y']) for tile in data['safe_tiles']}
            else:
                print("No valid tiles found in file, using default walkable tiles.")
                return {(x, y) for x in range(101) for y in range(101)}
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error loading walkable tiles: {e}. Using default walkable tiles.")
        return {(x, y) for x in range(101) for y in range(101)}

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar_pathfinding(start, goal, walkable_tiles):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    while not open_set.empty():
        _, current = open_set.get()
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in walkable_tiles:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))
    return None

def find_closest_npc(player_x, player_y, npcs):
    closest_npc = None
    min_distance = float('inf')
    for npc in npcs:
        distance = abs(player_x - npc['X']) + abs(player_y - npc['Y'])
        if distance < min_distance:
            min_distance = distance
            closest_npc = npc
    return closest_npc

pause_flag = False

def key_listener():
    global pause_flag
    import keyboard
    while True:
        if keyboard.is_pressed('0'):
            pause_flag = True
            print("Pausing for 1 minute...")
            time.sleep(60)
            pause_flag = False
            print("Resuming combat...")
        time.sleep(0.1)

last_combat_time = time.time()
current_target_npc = None
sitting = False

def combat_thread():
    global last_combat_time, sitting, pause_flag
    walkable_tiles = set(load_walkable_tiles()) or {(x, y) for x in range(101) for y in range(101)}
    last_direction = None
    wandering_target = None
    stuck_timer_start = None
    last_position = None
    ctrl_pressed = False
    blocked_tiles = {}
    movement_start_time = None
    target_tile = None

    while True:
        if pause_flag:
            time.sleep(1)
            continue

        current_time = time.time()
        blocked_tiles = {tile: expire_time for tile, expire_time in blocked_tiles.items() if expire_time > current_time}
        adjusted_walkable_tiles = walkable_tiles.difference(blocked_tiles.keys())

        if map_data:
            player = next(item for item in map_data if item["type"] == "player")
            npcs = [item for item in map_data if item["type"] == "npc"]
            valid_npcs = [npc for npc in npcs if (npc['X'], npc['Y']) in adjusted_walkable_tiles]

            if not valid_npcs:
                if ctrl_pressed:
                    release_key('ctrl')
                    ctrl_pressed = False
                else:
                    if last_position and (player['X'], player['Y']) == last_position:
                        if not stuck_timer_start:
                            stuck_timer_start = time.time()
                        elif time.time() - stuck_timer_start > 10:
                            print("Detected being stuck for 10 seconds, regenerating wandering target.")
                            wandering_target = random.choice(list(adjusted_walkable_tiles))
                            stuck_timer_start = None
                    else:
                        stuck_timer_start = None
                        last_position = (player['X'], player['Y'])
                    if not wandering_target or (player['X'], player['Y']) == wandering_target:
                        wandering_target = random.choice(list(adjusted_walkable_tiles))
                    path_to_wander = astar_pathfinding((player['X'], player['Y']), wandering_target, adjusted_walkable_tiles)
                    if path_to_wander and len(path_to_wander) > 1:
                        move_to_x, move_to_y = path_to_wander[1]
                        current_direction = None
                        if player['X'] < move_to_x:
                            current_direction = 'right'
                        elif player['X'] > move_to_x:
                            current_direction = 'left'
                        if player['Y'] < move_to_y:
                            current_direction = 'down'
                        elif player['Y'] > move_to_y:
                            current_direction = 'up'
                        if current_direction:
                            if current_direction != last_direction:
                                press_key(current_direction, 2, 0.05)
                            else:
                                press_key(current_direction, 1, 0.05)
                            last_direction = current_direction
                        player['X'], player['Y'] = move_to_x, move_to_y
                        print(f"Moving to tile: ({move_to_x}, {move_to_y})")
                continue

            if not sitting:
                npc_positions = {(npc['X'], npc['Y']) for npc in valid_npcs}
                extended_walkable_tiles = adjusted_walkable_tiles.difference(npc_positions)
                if valid_npcs:
                    closest_npc = find_closest_npc(player['X'], player['Y'], valid_npcs)
                    npc_x, npc_y = closest_npc['X'], closest_npc['Y']
                    player_x, player_y = player['X'], player['Y']
                    distance_x = abs(player_x - npc_x)
                    distance_y = abs(player_y - npc_y)
                    if (distance_x == 2 and player_y == npc_y) or (distance_y == 2 and player_x == npc_x):
                        line_of_sight = list(bresenham.bresenham(player_x, player_y, npc_x, npc_y))
                        line_of_sight_safe = all((x, y) in walkable_tiles for x, y in line_of_sight)
                        if line_of_sight_safe:
                            target_direction = None
                            if npc_x > player_x:
                                target_direction = 3  # East
                            elif npc_x < player_x:
                                target_direction = 1  # West
                            elif npc_y > player_y:
                                target_direction = 0  # South
                            elif npc_y < player_y:
                                target_direction = 2  # North
                            if player['direction'] != target_direction:
                                time.sleep(0.2)
                                press_key(['down', 'left', 'up', 'right'][target_direction], 1, 0.05)
                            if not ctrl_pressed:
                                hold_key('ctrl')
                                ctrl_pressed = True
                            last_combat_time = time.time()
                            continue
                    else:
                        if ctrl_pressed:
                            release_key('ctrl')
                            ctrl_pressed = False
                        potential_positions = [
                            (npc_x - 2, npc_y), (npc_x + 2, npc_y),
                            (npc_x, npc_y - 2), (npc_x, npc_y + 2)
                        ]
                        valid_positions = [pos for pos in potential_positions if pos in extended_walkable_tiles]
                        if valid_positions:
                            paths = [(astar_pathfinding((player_x, player_y), target, extended_walkable_tiles), target) for target in valid_positions]
                            paths = [path for path in paths if path[0]]
                            if paths:
                                best_path, best_target = min(paths, key=lambda x: len(x[0]))
                                move_to_x, move_to_y = best_path[1]
                                if (move_to_x, move_to_y) != target_tile:
                                    movement_start_time = current_time
                                    target_tile = (move_to_x, move_to_y)
                                current_direction = None
                                if player_x < move_to_x:
                                    current_direction = 'right'
                                elif player_x > move_to_x:
                                    current_direction = 'left'
                                if player_y < move_to_y:
                                    current_direction = 'down'
                                elif player_y > move_to_y:
                                    current_direction = 'up'
                                if current_direction:
                                    if current_direction != last_direction:
                                        press_key(current_direction, 2, 0.05)
                                    else:
                                        press_key(current_direction, 1, 0.05)
                                    last_direction = current_direction
                                player['X'], player['Y'] = move_to_x, move_to_y
                                print(f"Moving to tile: ({move_to_x}, {move_to_y})")
                                if movement_start_time and (current_time - movement_start_time) > 2:
                                    if (player['X'], player['Y']) != move_to_x or (player['X'], player['Y']) != move_to_y:
                                        blocked_tiles[(move_to_x, move_to_y)] = current_time + 3
                                        print(f"Marking tile as blocked: ({move_to_x}, {move_to_y}) for 3 seconds")
                                    movement_start_time = None
                            continue
                        else:
                            paths = [(astar_pathfinding((player_x, player_y), target, walkable_tiles), target) for target in potential_positions]
                            paths = [path for path in paths if path[0]]
                            if paths:
                                best_path, best_target = min(paths, key=lambda x: len(x[0]))
                                move_to_x, move_to_y = best_path[1]
                                if (move_to_x, move_to_y) != target_tile:
                                    movement_start_time = current_time
                                    target_tile = (move_to_x, move_to_y)
                                current_direction = None
                                if player_x < move_to_x:
                                    current_direction = 'right'
                                elif player_x > move_to_x:
                                    current_direction = 'left'
                                if player_y < move_to_y:
                                    current_direction = 'down'
                                elif player_y > move_to_y:
                                    current_direction = 'up'
                                if current_direction:
                                    if current_direction != last_direction:
                                        press_key(current_direction, 2, 0.05)
                                    else:
                                        press_key(current_direction, 1, 0.05)
                                    last_direction = current_direction
                                player['X'], player['Y'] = move_to_x, move_to_y
                                print(f"Moving to tile: ({move_to_x}, {move_to_y})")
                                if movement_start_time and (current_time - movement_start_time) > 2:
                                    if (player['X'], player['Y']) != move_to_x or (player['X'], player['Y']) != move_to_y:
                                        blocked_tiles[(move_to_x, move_to_y)] = current_time + 3
                                        print(f"Marking tile as blocked: ({move_to_x}, {move_to_y}) for 3 seconds")
                                    movement_start_time = None
                            continue
        if ctrl_pressed:
            release_key('ctrl')
            ctrl_pressed = False

def main():
    show_config_window()
    # Pickup initialization
    setup_click_points()
    threading.Thread(target=monitor_exp_and_combat, daemon=True).start()

    # Hardcoded configuration values
    attacknumber = 8

    # Hardcoded memory addresses
    walk_address = walkAddress
    npc_address = npcAddress
    directional_offset = directionalAddress

    print("Using hardcoded offsets:")
    print(f"Walk Address: {walk_address}")
    print(f"NPC Address: {npc_address}")
    print(f"Directional Address: {directional_offset}")

    # Start Frida sessions and hooks
    threading.Thread(target=start_frida, args=(npc_address,)).start()
    start_frida_session_xy(walk_address)
    start_frida_session_directional(directional_offset)

    # Start background threads
    threading.Thread(target=check_player_data, args=(x_address, y_address, directional_address)).start()
    threading.Thread(target=check_values).start()
    threading.Thread(target=combat_thread).start()
    key_listener_thread = threading.Thread(target=key_listener, daemon=True)
    key_listener_thread.start()

    # Launch GUI
    popup = PlayerDataPopup(player_data_manager)
    popup.run()

if __name__ == "__main__":
    main()
