#include <iostream>
#include <chrono>
#include <thread>

#include <cstdio>
#include <cerrno>
#include <cstring>
#include <csignal>

#include "rplidar.h"

using namespace rp::standalone::rplidar;
enum : _u16 {
    SCAN_MODE_STANDARD = 0,     // Scanning Frequency: 11.502; Measurements: 345; Errors: 36 10%; Unknowns: 41 11%
    SCAN_MODE_EXPRESS = 1,      // Scanning Frequency: 12.025; Measurements: 660; Errors: 98 14%; Unknowns: 0 0%
    SCAN_MODE_BOOST = 2,        // Scanning Frequency: 12.2477; Measurements: 1296; Errors: 159 12%; Unknowns: 0 0%
    SCAN_MODE_SENSITIVITY = 3,  // Scanning Frequency: 12.2856; Measurements: 1292; Errors: 125 9%; Unknowns: 0 0%
    SCAN_MODE_STABILITY = 4     // Scanning Frequency: 12.6582; Measurements: 790; Errors: 134 16%; Unknowns: 0 0%
} SCAN_MODES;


void on_finish(RPlidarDriver *lidar) {
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);
}

bool ctrl_c_pressed;

void ctrlc(int) {
    ctrl_c_pressed = true;
}

bool check_health(RPlidarDriver *lidar) {
    rplidar_response_device_health_t info;
    u_result op_result = lidar->getHealth(info);
    if (IS_OK(op_result)) {
        std::cout << "[INFO] Health status: " << info.status << std::endl
                  << "-----------------" << std::endl;
        if (info.status == RPLIDAR_STATUS_ERROR) {
            std::cerr << "[ERROR] Internal error detected. Please reboot the device to retry" << std::endl;
            return false;
        }
        return true;
    }

    std::cerr << "[ERROR] Cannot retrieve health code. Result: " << std::strerror(op_result) << std::endl;
    return false;
}

int main(int argc, char **argv) {
    const _u32 baud_rate{256000}; // 115200 or 256000
    const char *port_path{"/dev/ttyUSB0"};

    RPlidarDriver *lidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (lidar == nullptr) {
        std::cerr << "[ERROR] Failed to create serial driver" << std::endl
                  << "  Rate:" << baud_rate << std::endl
                  << "  Port:" << port_path << std::endl;
        return -1;
    }

    // connect to lidar
    u_result res = lidar->connect(port_path, baud_rate);
    if (IS_FAIL(res)) {
        RPlidarDriver::DisposeDriver(lidar);
        std::cerr << "[ERROR] Failed to connect to device" << std::endl
                  << "  Rate: " << baud_rate << std::endl
                  << "  Port: " << port_path << std::endl
                  << "  Result: " << std::strerror(res) << std::endl;
        return -1;
    }

    // fetch lidar data
    rplidar_response_device_info_t info;
    res = lidar->getDeviceInfo(info);
    if (IS_FAIL(res)) {
        std::cerr << "[ERROR] Failed to fetch device info" << std::endl
                  << "  Rate: " << baud_rate << std::endl
                  << "  Port: " << port_path << std::endl
                  << "  Result: " << std::strerror(res) << std::endl;
        on_finish(lidar);
        return -1;
    }
    std::cout << "[INFO] Device Info" << std::endl
              << "  Firmware Ver: " << (info.firmware_version >> 8) << "." << (info.firmware_version & 0xFF)
              << std::endl
              << "  Hardware Rev: " << info.hardware_version << std::endl
              << "-----------------" << std::endl;

    std::vector<RplidarScanMode> modes;
    res = lidar->getAllSupportedScanModes(modes);
    if (IS_FAIL(res)) {
        std::cerr << "[ERROR] Failed to fetch device supported scan modes" << std::endl
                  << "  Rate: " << baud_rate << std::endl
                  << "  Port: " << port_path << std::endl
                  << "  Result: " << std::strerror(res) << std::endl;
        on_finish(lidar);
        return -1;
    }
    std::cout << "[INFO] Scan Modes" << std::endl;
    for (auto mode: modes) {
        std::cout << "  ID: " << mode.id << std::endl
                  << "  Name: " << mode.scan_mode << std::endl
                  << "  Microseconds per sample: " << mode.us_per_sample << std::endl
                  << "  Max Distance: " << mode.max_distance << std::endl
                  << "-----------------" << std::endl;
    }

    if (!check_health(lidar)) {
        on_finish(lidar);
        return -1;
    }

    res = lidar->startMotor();
    if (IS_FAIL(res)) {
        std::cerr << "[ERROR] Failed to start motor" << std::endl
                  << "  Rate: " << baud_rate << std::endl
                  << "  Port: " << port_path << std::endl
                  << "  Result: " << std::strerror(res) << std::endl;
        on_finish(lidar);
        return -1;
    }

    // motor heat-up
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // start scan
    RplidarScanMode used_scan_mode;
    res = lidar->startScanExpress(false, SCAN_MODE_SENSITIVITY, 0, &used_scan_mode);
    if (IS_FAIL(res)) {
        std::cerr << "[ERROR] Failed to start scan" << std::endl
                  << "  Rate: " << baud_rate << std::endl
                  << "  Port: " << port_path << std::endl
                  << "  Result: " << std::strerror(res) << std::endl;
        on_finish(lidar);
        return -1;
    }
    std::cout << "[INFO] Used Scan Mode" << std::endl;
    std::cout << "  ID: " << used_scan_mode.id << std::endl
              << "  Name: " << used_scan_mode.scan_mode << std::endl
              << "  Microseconds per sample: " << used_scan_mode.us_per_sample << std::endl
              << "  Max Distance: " << used_scan_mode.max_distance << std::endl
              << "-----------------" << std::endl;

    signal(SIGINT, ctrlc);
    int scan_idx = 0;
    while (true) {
        std::cout << "------------- Scan " << ++scan_idx << " -------------" << std::endl;

        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t count{8192};
        std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
        start = std::chrono::high_resolution_clock::now();
        res = lidar->grabScanDataHq(nodes, count);
        end = std::chrono::high_resolution_clock::now();
        if (IS_FAIL(res)) {
            std::cerr << "[ERROR] Failed to grab scan data" << std::endl
                      << "  Rate: " << baud_rate << std::endl
                      << "  Port: " << port_path << std::endl
                      << "  Nodes count: " << count << std::endl
                      << "  Result: " << std::strerror(res) << std::endl;
            on_finish(lidar);
            return -1;
        }

        res = lidar->ascendScanData(nodes, count);
        if (IS_FAIL(res)) {
            std::cerr << "[ERROR] Failed to ascend scan data" << std::endl
                      << "  Rate: " << baud_rate << std::endl
                      << "  Port: " << port_path << std::endl
                      << "  Nodes count: " << count << std::endl
                      << "  Result: " << std::strerror(res) << std::endl;
            on_finish(lidar);
            return -1;
        }

        int errors{0};
        int unknowns{0};
        for (size_t pos = 0; pos < count; ++pos) {
            printf("Flag: %s; Angle: %03.2f; Distance: %8.2f; Quality: %d; Timestamp: %lu\n",
                   (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S" : "",
                   (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
                   nodes[pos].dist_mm_q2 / 4.0f,
                   nodes[pos].quality
            );

            if (nodes[pos].quality == 0) {
                errors++;
            } else if (nodes[pos].dist_mm_q2 == 0) {
                unknowns++;
            }
        }

        float frequency{0.0};
        res = lidar->getFrequency(used_scan_mode, count, frequency);
        if (IS_FAIL(res)) {
            std::cerr << "[ERROR] Failed to fetch current scanning frequency" << std::endl
                      << "  Rate: " << baud_rate << std::endl
                      << "  Port: " << port_path << std::endl
                      << "  Nodes count: " << count << std::endl
                      << "  Result: " << std::strerror(res) << std::endl;
            on_finish(lidar);
            return -1;
        }

        int64_t scan_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        std::cout << "Scan Time (us): " << scan_time << std::endl
                  << "Time per measurement (us): " << (scan_time / count) << std::endl
                  << "Scanning Frequency: " << frequency << std::endl
                  << "Measurements: " << count << std::endl
                  << "Errors: " << errors << " " << errors * 100 / count << "%" << std::endl
                  << "Unknowns: " << unknowns << " " << unknowns * 100 / count << "%" << std::endl;

        // TODO: remove break
        break;
        if (ctrl_c_pressed) {
            break;
        }
    }

    on_finish(lidar);
    return 0;
}