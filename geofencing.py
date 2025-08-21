import serial
import time
import re
from geopy.distance import geodesic

# ---------------- CONFIG ----------------
SERIAL_PORT = 'COM15'        # Replace with your Neoway serial port
BAUD_RATE = 9600             # Neoway default
FIXED_LOCATION = (12.284391, 76.615086)  # Replace with your reference location (lat, lon)
FETCH_INTERVAL = 1         # seconds
# ---------------------------------------

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)

def send_at_command(command, delay=1):
    """Send AT command and return raw response"""
    ser.write((command + '\r').encode())
    time.sleep(delay)
    response = ser.read_all().decode(errors='ignore')
    return response

def initialize_gps():
    """Power on GPS and basic initialization"""
    print("Powering on GPS...")
    send_at_command('AT$MYGPSPWR=1', delay=2)
    # You can add other initialization commands here if needed
    time.sleep(2)  # Give some time after power on

def enable_periodic_output():
    """Enable periodic GPS data output (GPGGA format, periodic)"""
    print("Enabling periodic GPS output (GPGGA format)...")
    send_at_command('AT$MYGPSPOS=0,1', delay=1)  # Type=0(GPGGA), Mode=1(periodic)

def parse_gpgga(sentence):
    """
    Parse $GPGGA sentence
    Example: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    Returns dict with relevant fields or None if invalid
    """
    try:
        parts = sentence.split(',')
        if parts[0] != '$GPGGA':
            return None

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
    except (ValueError, IndexError):
        return None

def convert_to_decimal_degrees(raw_value, direction):
    """
    Convert ddmm.mmmm to decimal degrees
    """
    if not raw_value or raw_value == '':
        return None
    d, m = divmod(float(raw_value), 100)
    decimal = d + m / 60
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

def get_gps_position(type=0, mode=0):
    """
    Get GPS position from AT$MYGPSPOS command
    type: 0=GPGGA, 1=GPGSA, 2=GPGSV, 3=GPRMC, 4=GPVTG, 5=GPGLL, 6=all
    mode: 0=once, 1=periodic, 2=disable periodic output
    Returns parsed data dict or None
    """
    cmd = f'AT$MYGPSPOS={type},{mode}'
    raw_resp = send_at_command(cmd, delay=2)
    # The response lines contain something like:
    # $MYGPSPOS: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    # OK

    # Extract the NMEA sentence from the response
    nmea_match = re.search(r'\$MYGPSPOS:\s*(\$(GPGGA|GPGSA|GPGSV|GPRMC|GPVTG|GPGLL)[^\r\n]*)', raw_resp)
    if not nmea_match:
        return None

    nmea_sentence = nmea_match.group(1).strip()

    if type == 0:  # GPGGA
        return parse_gpgga(nmea_sentence)

    # Add parsers for other NMEA types if needed
    return None

def calculate_accuracy(current_loc, fixed_loc):
    """Calculate distance in meters between two coordinates"""
    return geodesic(current_loc, fixed_loc).meters

def wait_for_fix(timeout=120, check_interval=1):
    """
    Wait until GPS fix is obtained or timeout reached.
    Checks every `check_interval` seconds up to `timeout` seconds.
    """
    print("Waiting for GPS fix...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        pos = get_gps_position(type=0, mode=0)  # Get GPGGA once
        if pos and pos['fix_quality'] > 0:
            print("GPS fix acquired!")
            return pos
        else:
            print("No fix yet, retrying...")
        time.sleep(check_interval)
    print("Timeout reached without GPS fix.")
    return None

def get_basic_module_info():
    """Send basic AT commands to get device info and signal quality"""
    print("Getting module info and signal quality...")
    info = send_at_command('ATI', delay=1)
    print("ATI response:\n", info)

    signal = send_at_command('AT+CSQ', delay=1)
    print("Signal quality (AT+CSQ):\n", signal)

def main():
    print("Starting Neoway N58 GPS tracking...")

    get_basic_module_info()
    initialize_gps()
    enable_periodic_output()

    # Wait for GPS fix with timeout of 2 minutes
    pos = wait_for_fix(timeout=120, check_interval=5)
    if not pos:
        print("Failed to get GPS fix, exiting...")
        ser.close()
        return

    try:
        while True:
            pos = get_gps_position(type=0, mode=0)  # GPGGA once

            if pos and pos['fix_quality'] > 0:
                accuracy = calculate_accuracy((pos['latitude'], pos['longitude']), FIXED_LOCATION)
                print(f"UTC Time: {pos['utc_time']}")
                print(f"Latitude: {pos['latitude']:.6f}, Longitude: {pos['longitude']:.6f}")
                print(f"Fix Quality: {pos['fix_quality']} Satellites: {pos['num_sats']} HDOP: {pos['hdop']}")
                print(f"Altitude: {pos['altitude']} {pos['altitude_units']} Geoid Height: {pos['geoid_height']} {pos['geoid_units']}")
                print(f"Distance to fixed point: {accuracy:.2f} meters\n")
            else:
                print("Lost GPS fix or invalid data.\n")

            time.sleep(FETCH_INTERVAL)

    except KeyboardInterrupt:
        print("Stopping GPS tracking...")
        ser.close()

if __name__ == "__main__":
    main()
