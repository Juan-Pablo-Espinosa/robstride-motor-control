// motion_test.cpp
// Moves both motors independently in different patterns.
// Motor 42 (RS-03): slow sine wave
// Motor 127 (RS-04): faster triangle wave, opposite phase
// If one motor disconnects, the other keeps moving.
// Press 'q' to quit, 'e' to re-enable faulted motors.

#include "motor_bus.hpp"
#include "socketcan_transport.hpp"
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>

static int kbhit() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_cc[VMIN]  = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main() {
    SocketCANTransport transport("can1");
    if (!transport.open()) {
        fprintf(stderr, "Failed to open can1\n");
        return 1;
    }

    MotorBus bus(transport, 200);

    MotorConfig cfg03;
    cfg03.model            = MotorModel::RS03;
    cfg03.joint_name       = "rs03_42";
    cfg03.max_acceleration = 5.0f;

    MotorConfig cfg04;
    cfg04.model            = MotorModel::RS04;
    cfg04.joint_name       = "rs04_127";
    cfg04.max_acceleration = 5.0f;

    bus.addMotor(42,  cfg03);
    bus.addMotor(127, cfg04);

    printf("Enabling motors...\n");
    bool ok42  = bus.enable(42);
    bool ok127 = bus.enable(127);
    printf("Motor 42  (RS-03): %s\n", ok42  ? "OK" : "FAILED");
    printf("Motor 127 (RS-04): %s\n", ok127 ? "OK" : "FAILED");

    if (!ok42 && !ok127) {
        fprintf(stderr, "No motors enabled\n");
        return 1;
    }

    bus.start();
    printf("\nMoving motors. 'e' = re-enable faulted, 'q' = quit\n\n");

    auto t0 = std::chrono::steady_clock::now();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        // Motor 42: slow sine, 0.5 rad amplitude, 0.3 Hz
        MotorTarget tgt42;
        tgt42.angle  = 0.5f * sinf(2.0f * M_PI * 0.3f * t);
        tgt42.kp     = 100.0f;
        tgt42.kd     = 10.0f;

        // Motor 127: triangle wave, 0.3 rad amplitude, 0.5 Hz, opposite phase
        double tri_phase = fmod(0.5 * t + 0.5, 1.0);
        float tri = (tri_phase < 0.5) ? (4.0f * tri_phase - 1.0f)
                                       : (3.0f - 4.0f * tri_phase);
        MotorTarget tgt127;
        tgt127.angle = 0.3f * tri;
        tgt127.kp    = 100.0f;
        tgt127.kd    = 10.0f;

        bus.setTarget(42,  tgt42);
        bus.setTarget(127, tgt127);

        // Read state
        MotorState s42, s127;
        bus.getState(42,  s42);
        bus.getState(127, s127);

        auto now = std::chrono::steady_clock::now();
        double age42  = std::chrono::duration<double, std::milli>(now - s42.last_update).count();
        double age127 = std::chrono::duration<double, std::milli>(now - s127.last_update).count();

        const char* status42  = bus.isFaulted(42)  ? "FAULT" : (age42  > 50.0 ? "LOST " : "ok   ");
        const char* status127 = bus.isFaulted(127) ? "FAULT" : (age127 > 50.0 ? "LOST " : "ok   ");
        printf("[%5.1fs] M42  cmd=%+5.3f actual=%+5.3f [%s] age=%4.1fms | "
               "M127 cmd=%+5.3f actual=%+5.3f [%s] age=%4.1fms\n",
            t,
            tgt42.angle,  s42.angle,  status42,  age42,
            tgt127.angle, s127.angle, status127, age127);

        int ch = kbhit();
        if (ch == 'q' || ch == 'Q') break;
        if (ch == 'e' || ch == 'E') {
            printf(">>> Re-enabling faulted motors...\n");
            if (bus.isFaulted(42))  { bus.clearFault(42);  bus.enable(42); }
            if (bus.isFaulted(127)) { bus.clearFault(127); bus.enable(127); }
        }
    }

    printf("Stopping...\n");
    bus.stop();
    transport.close();
    return 0;
}
