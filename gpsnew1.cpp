#include <windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <chrono>
#include <iomanip>

// ======================== Namespace Shortcuts ======================
using std::string;
using std::vector;
using std::map;
using std::stringstream;

using std::cout;
using std::endl;
using std::setw;
using std::left;

// =================================================================
// Utility: split string by delimiter
vector<string> split(const string& s, char delim) {
    vector<string> elems;
    string item;
    stringstream ss(s);
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

// =================================================================
// Serial helpers (no error checks)
HANDLE openSerial(const string& port, DWORD baud=115200) {
    HANDLE h = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, 0, NULL);

    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    SetCommState(h, &dcb);

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    SetCommTimeouts(h, &timeouts);

    return h;
}

string readSerial(HANDLE h) {
    char buf[512];
    DWORD n;
    string data;
    if (ReadFile(h, buf, sizeof(buf) - 1, &n, NULL) && n > 0) {
        buf[n] = 0;
        data.assign(buf, n);
    }
    return data;
}

// =================================================================
// NMEA Parsing for RMC (still keeps size check to avoid crash)
map<string,string> n58_parse(const string& cmd, const string& response) {
    map<string,string> out;
    out["cmd"] = cmd;

    vector<string> lines = split(response, '\n');
    for (string line : lines) {
        if (line.find("$") != string::npos) {
            vector<string> fields = split(line, ',');
            if (fields.size() > 9 && fields[0].find("RMC") != string::npos) {
                out["nmea_type"] = "RMC";
                out["time_utc"]  = fields[1];
                out["valid"]     = fields[2];

                string lat  = fields[3];
                string latd = fields[4];
                string lon  = fields[5];
                string lond = fields[6];

                if (!lat.empty()) {
                    double v = std::stod(lat.substr(0,2))
                             + std::stod(lat.substr(2))/60.0;
                    if (latd == "S") v = -v;
                    out["lat"] = std::to_string(v);
                }
                if (!lon.empty()) {
                    double v = std::stod(lon.substr(0,3))
                             + std::stod(lon.substr(3))/60.0;
                    if (lond == "W") v = -v;
                    out["lon"] = std::to_string(v);
                }
            }
        }
    }
    return out;
}

// =================================================================
// Pretty print a table of parsed values
void printTable(const map<string,string>& kv) {
    cout << "\n+----------------+--------------------------+\n";
    for (auto &p : kv) {
        cout << "| " << setw(14) << left << p.first
             << " | " << setw(24) << left << p.second << " |\n";
    }
    cout << "+----------------+--------------------------+\n";
}

// =================================================================
// Main loop
int main() {
    string port = "\\\\.\\COM3";  // adjust as needed
    HANDLE h = openSerial(port);

    cout << "Streaming GPS data (CTRL+C to quit)...\n";
    string buffer;

    auto lastPrint = std::chrono::steady_clock::now();

    while (true) {
        string data = readSerial(h);
        if (!data.empty()) {
            buffer += data;
            size_t pos;
            while ((pos = buffer.find("\r\n")) != string::npos) {
                string line = buffer.substr(0,pos);
                buffer.erase(0,pos+2);

                if (line.find("$GNRMC") != string::npos || line.find("$GPRMC") != string::npos) {
                    auto parsed = n58_parse("RMC", line);
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now-lastPrint).count() >= 1) {
                        printTable(parsed);
                        lastPrint = now;
                    }
                }
            }
        }
        Sleep(50);
    }

    CloseHandle(h);
    return 0;
}
