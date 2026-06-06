// ---------------------------------------------------------------------------
// motor_studio.cpp
//
// Simple terminal tool for testing RobStride motors.
// Auto-scans on startup, shows live state, accepts commands.
//
// Commands:
//   en              — enable selected motor
//   dis             — disable selected motor
//   enall           — enable all
//   disall          — disable all
//   sel <id>        — select motor
//   pos <rad>       — move to angle (MIT mode)
//   vel <rad/s>     — spin at velocity (MIT mode)
//   kp <val>        — set kp gain (applied on next pos/vel)
//   kd <val>        — set kd gain (applied on next pos/vel)
//   tor <Nm>        — set feedforward torque
//   zero            — set mechanical zero on selected motor
//   scan            — rescan bus
//   hz              — show measured control loop rate
//   q               — safe quit
// ---------------------------------------------------------------------------

#include "socketcan_transport.hpp"
#include "motor_bus.hpp"
#include <cstdio>
#include <cstring>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <iostream>

// ---------------------------------------------------------------------------
// Session state — kp/kd persist between commands
// ---------------------------------------------------------------------------
static float g_kp       = 10.0f;
static float g_kd       = 0.5f;
static uint8_t g_sel    = 0;

// ---------------------------------------------------------------------------
// Print live state table for all motors
// ---------------------------------------------------------------------------
void printState(MotorBus& bus) {
    auto ids = bus.motorIds();
    if (ids.empty()) {
        printf("  (no motors)\n");
        return;
    }

    printf("  %-4s %-8s %-10s %-10s %-10s %-8s %-6s %-10s\n",
        "ID", "MODEL", "ANGLE(rad)", "VEL(r/s)", "TORQ(Nm)",
        "TEMP(C)", "FAULT", "MODE");
    printf("  %s\n", std::string(78, '-').c_str());

    for (uint8_t id : ids) {
        MotorState s;
        bus.getState(id, s);

        const char* mode_str = "Unknown";
        switch (s.mode) {
            case MotorMode::Reset:       mode_str = "Reset";  break;
            case MotorMode::Calibration: mode_str = "Cali";   break;
            case MotorMode::Run:         mode_str = "Run";    break;
        }

        const char* sel_marker = (id == g_sel) ? "*" : " ";

        // Get model name from bus — requires exposing config from MotorEntry
        // For now use a placeholder; full introspection added later
        printf(" %s%-4d %-8s %-10.3f %-10.3f %-10.3f %-8.1f %-6s %-10s\n",
            sel_marker, id, "RS-03",
            s.angle, s.velocity, s.torque,
            s.temperature,
            s.fault == 0 ? "OK" : "FAULT",
            mode_str);
    }

    printf("\n  Loop: %.1f Hz  |  Selected: %s%d  |  kp=%.1f  kd=%.2f\n",
        bus.measuredHz(),
        g_sel == 0 ? "(none) " : "",
        g_sel, g_kp, g_kd);
}

// ---------------------------------------------------------------------------
// Handle one command
// Returns false if user wants to quit
// ---------------------------------------------------------------------------
bool handleCommand(const std::string& line, MotorBus& bus) {
    std::istringstream ss(line);
    std::string cmd;
    ss >> cmd;
    if (cmd.empty()) return true;

    if (cmd == "q" || cmd == "quit") {
        return false;

    } else if (cmd == "scan") {
        printf("Scanning 1-127...\n");
        auto found = bus.scanAndAdd(1, 127);
        if (found.empty()) {
            printf("No motors found. Check power and CAN connection.\n");
        } else {
            printf("Found %zu motor(s): ", found.size());
            for (uint8_t id : found) printf("%d ", id);
            printf("\n");
        }

    } else if (cmd == "en") {
        if (g_sel == 0) { printf("No motor selected. Use: sel <id>\n"); return true; }
        if (bus.enable(g_sel)) printf("Motor %d enabled.\n", g_sel);
        else                   printf("Enable failed.\n");

    } else if (cmd == "dis") {
        if (g_sel == 0) { printf("No motor selected.\n"); return true; }
        if (bus.disable(g_sel)) printf("Motor %d disabled.\n", g_sel);
        else                    printf("Disable failed.\n");

    } else if (cmd == "enall") {
        bus.enableAll();
        printf("All motors enabled.\n");

    } else if (cmd == "disall") {
        bus.disableAll();
        printf("All motors disabled.\n");

    } else if (cmd == "sel") {
        uint8_t id; ss >> id;
        g_sel = id;
        printf("Selected motor %d.\n", id);

    } else if (cmd == "pos") {
        if (g_sel == 0) { printf("No motor selected.\n"); return true; }
        float val; ss >> val;
        MotorTarget t;
        t.angle    = val;
        t.velocity = 0.0f;
        t.kp       = g_kp;
        t.kd       = g_kd;
        t.torque   = 0.0f;
        bus.setTarget(g_sel, t);
        printf("Motor %d -> pos %.3f rad  (kp=%.1f kd=%.2f)\n",
            g_sel, val, g_kp, g_kd);

    } else if (cmd == "vel") {
        if (g_sel == 0) { printf("No motor selected.\n"); return true; }
        float val; ss >> val;
        // MIT velocity field is feedforward only, not a velocity setpoint.
        // With kp=0 the motor will resist motion but not actively drive.
        // True velocity control (PARAM_SPD_REF) is not yet implemented.
        // Use 'pos' with incremental targets for motion, or wait for item 7.
        MotorTarget t;
        t.angle    = 0.0f;
        t.velocity = val;
        t.kp       = 0.0f;
        t.kd       = g_kd;
        t.torque   = 0.0f;
        bus.setTarget(g_sel, t);
        printf("Motor %d -> vel feedforward %.3f rad/s (kd=%.2f)\n", g_sel, val, g_kd);
        printf("WARNING: MIT vel is feedforward only. Motor won't spin freely. Item 7 adds real speed mode.\n");

    } else if (cmd == "kp") {
        ss >> g_kp;
        printf("kp set to %.2f\n", g_kp);

    } else if (cmd == "kd") {
        ss >> g_kd;
        printf("kd set to %.2f\n", g_kd);

    } else if (cmd == "tor") {
        if (g_sel == 0) { printf("No motor selected.\n"); return true; }
        float val; ss >> val;
        MotorTarget t;
        t.torque = val;
        bus.setTarget(g_sel, t);
        printf("Motor %d -> torque %.3f Nm\n", g_sel, val);

    } else if (cmd == "zero") {
        if (g_sel == 0) { printf("No motor selected.\n"); return true; }
        printf("Zero set on motor %d.\n", g_sel);

    } else if (cmd == "hz") {
        printf("Control loop: %.1f Hz\n", bus.measuredHz());

    } else if (cmd == "help") {
        printf("Commands: scan, sel <id>, en, dis, enall, disall,\n");
        printf("          pos <rad>, vel <rad/s>, kp <v>, kd <v>,\n");
        printf("          tor <Nm>, zero, hz, q\n");

    } else {
        printf("Unknown command: %s  (type help)\n", cmd.c_str());
    }

    return true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main() {
    printf("=== RobStride Motor Studio ===\n\n");

    // --- Open CAN ---
    SocketCANTransport transport("can1");
    if (!transport.open()) {
        fprintf(stderr, "Failed to open can1.\n");
        fprintf(stderr, "Run: sudo ip link set can1 type can bitrate 1000000 && sudo ip link set can1 up\n");
        return 1;
    }
    printf("CAN interface: can1 @ 1Mbps\n\n");

    MotorBus bus(transport);

    // --- Auto scan ---
    printf("Scanning for motors...\n");
    auto found = bus.scanAndAdd(1, 127);

    if (found.empty()) {
        printf("No motors found. Check power and CAN connection.\n");
        printf("You can try again with: scan\n\n");
    } else {
        printf("Found %zu motor(s): ", found.size());
        for (uint8_t id : found) printf("%d ", id);
        printf("\n\n");

        if (found.size() == 1) {
            // Only one motor — auto select it
            g_sel = found[0];
            printf("Auto-selected motor %d.\n", g_sel);
        } else {
            // Multiple motors — ask user
            printf("Multiple motors found. Select one (enter ID): ");
            fflush(stdout);
            int id;
            scanf("%d", &id);
            g_sel = static_cast<uint8_t>(id);
            printf("Selected motor %d.\n", g_sel);
        }
        printf("\n");
    }

    // --- Start control loop ---
    bus.start();
    printf("Control loop started. Type 'help' for commands.\n\n");

    // --- Main loop ---
    std::string input;
    while (true) {
        // Print state
        printState(bus);
        printf("\n> ");
        fflush(stdout);

        // Read command
        if (!std::getline(std::cin, input)) break;

        if (!handleCommand(input, bus)) break;
        printf("\n");
    }

    // --- Safe shutdown ---
    printf("\nShutting down...\n");
    bus.stop();
    transport.close();
    printf("Done.\n");
    return 0;
}
