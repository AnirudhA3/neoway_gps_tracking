import serial
import time
import re
from geopy.distance import geodesic

# ---------------- CONFIG ----------------
SERIAL_PORT = 'COM15'        # Replace with your Neoway serial port
BAUD_RATE = 9600             # Neoway default
FIXED_LOCATION = (12.284391, 76.615086)  # Reference location (lat, lon)
FETCH_INTERVAL = 1           # seconds
# ---------------------------------------

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)

def send_at_command(command, delay=1):
    ser.write((command + '\r').encode())
    time.sleep(delay)
    response = ser.read_all().decode(errors='ignore')
    return response

# ---------- GPS ----------
def initialize_gps():
    print("Powering on GPS...")
    send_at_command('AT$MYGPSPWR=1', delay=2)
    time.sleep(2)

def enable_periodic_output_gps():
    print("Enabling periodic GPS output (GPGGA format)...")
    send_at_command('AT$MYGPSPOS=0,1', delay=1)

def parse_gpgga(sentence):
    try:
        parts = sentence.split(',')
        if parts[0] != '$GPGGA':
            return None
        return parse_common(parts)
    except (ValueError, IndexError):
        return None

def get_gps_position(type=0, mode=0):
    cmd = f'AT$MYGPSPOS={type},{mode}'
    raw_resp = send_at_command(cmd, delay=2)
    nmea_match = re.search(r'\$MYGPSPOS:\s*(\$(GPGGA|GPGSA|GPGSV|GPRMC|GPVTG|GPGLL)[^\r\n]*)', raw_resp)
    if not nmea_match:
        return None
    nmea_sentence = nmea_match.group(1).strip()
    if type == 0:
        return parse_gpgga(nmea_sentence)
    return None

# ---------- GLONASS ----------
def initialize_glonass():
    print("Powering on GLONASS...")
    send_at_command('AT$MYGLONASSPWR=1', delay=2)
    time.sleep(2)

def enable_periodic_output_glonass():
    print("Enabling periodic GLONASS output (GLGGA format)...")
    send_at_command('AT$MYGLONASSPOS=0,1', delay=1)

def parse_glgga(sentence):
    try:
        parts = sentence.split(',')
        if parts[0] != '$GLGGA':
            return None
        return parse_common(parts)
    except (ValueError, IndexError):
        return None

def get_glonass_position(type=0, mode=0):
    cmd = f'AT$MYGLONASSPOS={type},{mode}'
    raw_resp = send_at_command(cmd, delay=2)
    nmea_match = re.search(r'\$MYGLONASSPOS:\s*(\$(GLGGA|GLGSA|GLGSV|GLRMC)[^\r\n]*)', raw_resp)
    if not nmea_match:
        return None
    nmea_sentence = nmea_match.group(1).strip()
    if type == 0:
        return parse_glgga(nmea_sentence)
    return None

# ---------- Common Parsing ----------
def parse_common(parts):
    utc_time = parts[1]
    lat_raw = parts[2]
    lat_dir = parts[3]
    lon_raw = parts[4]
    lon_dir = parts[5]
    fix_quality = int(parts[6])
    num_sats = int(parts[7])
    hdop = float(parts[8])
    altitude = float(parts[9])
    altitude_units = parts[10]
    geoid_height = float(parts[11])
    geoid_units = parts[12]

    lat = convert_to_decimal_degrees(lat_raw, lat_dir)
    lon = convert_to_decimal_degrees(lon_raw, lon_dir)

    return {
        'utc_time': utc_time,
        'latitude': lat,
        'longitude': lon,
        'fix_quality': fix_quality,
        'num_sats': num_sats,
        'hdop': hdop,
        'altitude': altitude,
        'altitude_units': altitude_units,
        'geoid_height': geoid_height,
        'geoid_units': geoid_units
    }

def convert_to_decimal_degrees(raw_value, direction):
    if not raw_value or raw_value == '':
        return None
    d, m = divmod(float(raw_value), 100)
    decimal = d + m / 60
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

def calculate_accuracy(current_loc, fixed_loc):
    return geodesic(current_loc, fixed_loc).meters

# ---------- Module Info ----------
def get_basic_module_info():
    print("Getting module info and signal quality...")
    info = send_at_command('ATI', delay=1)
    print("ATI response:\n", info)
    signal = send_at_command('AT+CSQ', delay=1)
    print("Signal quality (AT+CSQ):\n", signal)

# ---------- Main ----------
def main():
    print("Starting Neoway N58 GPS + GLONASS tracking...")

    get_basic_module_info()
    initialize_gps()
    initialize_glonass()
    enable_periodic_output_gps()
    enable_periodic_output_glonass()

    try:
        while True:
            gps_pos = get_gps_position(type=0, mode=0)
            glonass_pos = get_glonass_position(type=0, mode=0)

            if gps_pos and gps_pos['fix_quality'] > 0:
                acc_gps = calculate_accuracy((gps_pos['latitude'], gps_pos['longitude']), FIXED_LOCATION)
                print("\n--- GPS Data ---")
                print(f"UTC Time: {gps_pos['utc_time']}")
                print(f"Latitude: {gps_pos['latitude']:.6f}, Longitude: {gps_pos['longitude']:.6f}")
                print(f"Fix Quality: {gps_pos['fix_quality']} Satellites: {gps_pos['num_sats']} HDOP: {gps_pos['hdop']}")
                print(f"Altitude: {gps_pos['altitude']} {gps_pos['altitude_units']} Geoid Height: {gps_pos['geoid_height']} {gps_pos['geoid_units']}")
                print(f"Distance to fixed point: {acc_gps:.2f} meters")

            if glonass_pos and glonass_pos['fix_quality'] > 0:
                acc_glonass = calculate_accuracy((glonass_pos['latitude'], glonass_pos['longitude']), FIXED_LOCATION)
                print("\n--- GLONASS Data ---")
                print(f"UTC Time: {glonass_pos['utc_time']}")
                print(f"Latitude: {glonass_pos['latitude']:.6f}, Longitude: {glonass_pos['longitude']:.6f}")
                print(f"Fix Quality: {glonass_pos['fix_quality']} Satellites: {glonass_pos['num_sats']} HDOP: {glonass_pos['hdop']}")
                print(f"Altitude: {glonass_pos['altitude']} {glonass_pos['altitude_units']} Geoid Height: {glonass_pos['geoid_height']} {glonass_pos['geoid_units']}")
                print(f"Distance to fixed point: {acc_glonass:.2f} meters")

            if not (gps_pos or glonass_pos):
                print("No valid GPS/GLONASS fix.\n")

            time.sleep(FETCH_INTERVAL)

    except KeyboardInterrupt:
        print("Stopping tracking...")
        ser.close()

if __name__ == "__main__":
    main()
