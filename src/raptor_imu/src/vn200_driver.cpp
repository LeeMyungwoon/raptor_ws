// Copyright (c) 2026 RAPTOR Project
// VN-200 IMU Driver Implementation

#include "raptor_imu/vn200_driver.hpp"

// ─── 생성자 ─── /
VN200Driver::VN200Driver() : Node("vn200_driver") {
    using namespace VN::Registers::System;

    // ── 파라미터 선언 ── //
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("BaudRate", static_cast<int>(VN::Sensor::BaudRate::Baud115200));

    // ASCII 비동기 출력 설정
    declare_parameter<int>("AsyncDataOutputType", static_cast<int>(AsyncOutputType::Ador::OFF));
    declare_parameter<int>("AsyncDataOutputTypeSerialPort", static_cast<int>(AsyncOutputType::SerialPort::Serial1));
    declare_parameter<int>("AsyncDataOutputFrequency", static_cast<int>(AsyncOutputFreq::Adof::Rate20Hz));
    declare_parameter<int>("AsyncDataOutputFrequencySerialPort", static_cast<int>(AsyncOutputFreq::SerialPort::Serial1));

    // 동기화 설정 (SyncIn: 카메라 트리거 연동용)
    declare_parameter<int>("syncInMode", static_cast<int>(SyncControl::SyncInMode::Count));
    declare_parameter<int>("syncInEdge", static_cast<int>(SyncControl::SyncInEdge::RisingEdge));
    declare_parameter<int>("syncInSkipFactor", 0);
    declare_parameter<int>("syncOutMode", static_cast<int>(SyncControl::SyncOutMode::None));
    declare_parameter<int>("syncOutPolarity", static_cast<int>(SyncControl::SyncOutPolarity::NegativePulse));
    declare_parameter<int>("syncOutSkipFactor", 0);
    declare_parameter<int>("syncOutPulseWidth_ns", 100000000);

    // 통신 프로토콜
    declare_parameter<int>("asciiAppendCount", static_cast<int>(ProtocolControl::AsciiAppendCount::None));
    declare_parameter<int>("asciiAppendStatus", static_cast<int>(ProtocolControl::AsciiAppendStatus::None));
    declare_parameter<int>("spiAppendCount", static_cast<int>(ProtocolControl::SpiAppendCount::None));
    declare_parameter<int>("spiAppendStatus", static_cast<int>(ProtocolControl::SpiAppendStatus::None));
    declare_parameter<int>("asciiChecksum", static_cast<int>(ProtocolControl::AsciiChecksum::Checksum8bit));
    declare_parameter<int>("spiChecksum", static_cast<int>(ProtocolControl::SpiChecksum::Off));
    declare_parameter<int>("errorMode", static_cast<int>(ProtocolControl::ErrorMode::SendError));

    // Binary Output 1 (VIO용 IMU 고속 출력)
    declare_parameter<int>("BO1.asyncMode", 3);        // ASYNCMODE_BOTH
    declare_parameter<int>("BO1.rateDivisor", 2);      // 800Hz / 2 = 400Hz
    declare_parameter<int>("BO1.commonField", 0x0000);
    declare_parameter<int>("BO1.timeField", 0x0001);   // TimeStartup
    declare_parameter<int>("BO1.imuField", 0x001E);    // UncompAccel + UncompGyro + DeltaTheta + DeltaVel
    declare_parameter<int>("BO1.gpsField", 0x0000);    // GNSS 비활성화
    declare_parameter<int>("BO1.attitudeField", 0x0000); // YPR + Quaternion 비활성화
    declare_parameter<int>("BO1.insField", 0x0000);
    declare_parameter<int>("BO1.gps2Field", 0x0000);

    // Binary Output 2 (비활성화)
    declare_parameter<int>("BO2.asyncMode", 0);
    declare_parameter<int>("BO2.rateDivisor", 0);
    declare_parameter<int>("BO2.commonField", 0x0000);
    declare_parameter<int>("BO2.timeField", 0x0000);
    declare_parameter<int>("BO2.imuField", 0x0000);
    declare_parameter<int>("BO2.gpsField", 0x0000);
    declare_parameter<int>("BO2.attitudeField", 0x0000);
    declare_parameter<int>("BO2.insField", 0x0000);
    declare_parameter<int>("BO2.gps2Field", 0x0000);

    // Binary Output 3 (비활성화)
    declare_parameter<int>("BO3.asyncMode", 0);
    declare_parameter<int>("BO3.rateDivisor", 0);
    declare_parameter<int>("BO3.commonField", 0x0000);
    declare_parameter<int>("BO3.timeField", 0x0000);
    declare_parameter<int>("BO3.imuField", 0x0000);
    declare_parameter<int>("BO3.gpsField", 0x0000);
    declare_parameter<int>("BO3.attitudeField", 0x0000);
    declare_parameter<int>("BO3.insField", 0x0000);
    declare_parameter<int>("BO3.gps2Field", 0x0000);

    declare_parameter<std::string>("frame_id", "imu_link");

    // ── Publisher 생성 ── //
    pubStdImu_ = this->create_publisher<sensor_msgs::msg::Imu>("raptor/imu/data", 10);
    // pubImuGroup_ = this->create_publisher<raptor_msgs::msg::ImuGroup>("raptor/imu/raw", 10);
    // pubAttitudeGroup_ = this->create_publisher<raptor_msgs::msg::AttitudeGroup>("raptor/imu/attitude", 10);
    pubTimeGroup_ = this->create_publisher<raptor_msgs::msg::TimeGroup>("raptor/imu/time", 10);

    // ── Service 생성 ── //
    srvTransaction_ = this->create_service<raptor_msgs::srv::Transaction>(
        "raptor/imu/transaction",
        std::bind(&VN200Driver::vnTransaction, this, std::placeholders::_1, std::placeholders::_2));
}

// ─── 초기화 ─── //
bool VN200Driver::init() {
    if (connect(get_parameter("port").as_string(), get_parameter("BaudRate").as_int())) { return true; }
    if (configureSensor()) { return true; }
    
    return false;
}

// ─── 센서 연결 ─── //
bool VN200Driver::connect(const std::string port, const int requestedBaudRate) {
    RCLCPP_INFO(get_logger(), "[IMU-INFO] Connecting to VN-200...");

    VN::Error vnError = vs_.autoConnect(port);
    if (vnError != VN::Error::None) {
        RCLCPP_ERROR(get_logger(), "[IMU-ERROR] Unable to connect to %s: %s", port.c_str(), VN::errorCodeToString(vnError).data());
        
        return true;
    }

    RCLCPP_INFO(get_logger(), "[IMU-INFO] AutoConnect complete.");

    // 요청된 baud rate로 변경
    if (requestedBaudRate != static_cast<int>(vs_.connectedBaudRate().value())) {
        vnError = vs_.changeBaudRate(static_cast<VN::Sensor::BaudRate>(requestedBaudRate));
        if (VN::Error::None != vnError) {
            RCLCPP_ERROR(get_logger(), "[IMU-ERROR] Failed to change baud rate: %s", VN::errorCodeToString(vnError).data());
            
            return true;
        }
    }

    // 센서 정보 출력
    VN::Registers::System::Model modelRegister;
    vs_.readRegister(&modelRegister);

    VN::Registers::System::FwVer firmwareRegister;
    vs_.readRegister(&firmwareRegister);

    VN::Registers::System::Serial serialRegister;
    vs_.readRegister(&serialRegister);

    RCLCPP_INFO(get_logger(), "[IMU-INFO] Connected to %s @ %d baud",
                port.c_str(), static_cast<uint32_t>(vs_.connectedBaudRate().value()));
    RCLCPP_INFO(get_logger(), "[IMU-INFO] Model: %s | FW: %s | S/N: %d",
                modelRegister.model.c_str(), firmwareRegister.fwVer.c_str(), serialRegister.serialNum);

    return false;
}

// ─── 센서 설정 ─── //
bool VN200Driver::configureSensor() {
    std::vector<VN::ConfigurationRegister*> configRegisters;
    using namespace VN::Registers::System;

    // ASCII 비동기 출력 설정
    AsyncOutputType asyncDataOutputType;
    asyncDataOutputType.ador = static_cast<AsyncOutputType::Ador>(get_parameter("AsyncDataOutputType").as_int());
    asyncDataOutputType.serialPort = static_cast<AsyncOutputType::SerialPort>(get_parameter("AsyncDataOutputTypeSerialPort").as_int());
    configRegisters.push_back(&asyncDataOutputType);

    AsyncOutputFreq asyncDataOutputFreq;
    asyncDataOutputFreq.adof = static_cast<AsyncOutputFreq::Adof>(get_parameter("AsyncDataOutputFrequency").as_int());
    asyncDataOutputFreq.serialPort = static_cast<AsyncOutputFreq::SerialPort>(get_parameter("AsyncDataOutputFrequencySerialPort").as_int());
    configRegisters.push_back(&asyncDataOutputFreq);

    // 동기화 설정
    SyncControl syncControl;
    syncControl.syncInMode = static_cast<SyncControl::SyncInMode>(get_parameter("syncInMode").as_int());
    syncControl.syncInEdge = static_cast<SyncControl::SyncInEdge>(get_parameter("syncInEdge").as_int());
    syncControl.syncInSkipFactor = get_parameter("syncInSkipFactor").as_int();
    syncControl.syncOutMode = static_cast<SyncControl::SyncOutMode>(get_parameter("syncOutMode").as_int());
    syncControl.syncOutPolarity = static_cast<SyncControl::SyncOutPolarity>(get_parameter("syncOutPolarity").as_int());
    syncControl.syncOutSkipFactor = get_parameter("syncOutSkipFactor").as_int();
    syncControl.syncOutPulseWidth = get_parameter("syncOutPulseWidth_ns").as_int();
    syncControl.resv1 = 0;
    syncControl.resv2 = 0;
    configRegisters.push_back(&syncControl);

    // 통신 프로토콜 설정
    ProtocolControl protocolControl;
    protocolControl.asciiAppendCount = static_cast<ProtocolControl::AsciiAppendCount>(get_parameter("asciiAppendCount").as_int());
    protocolControl.asciiAppendStatus = static_cast<ProtocolControl::AsciiAppendStatus>(get_parameter("asciiAppendStatus").as_int());
    protocolControl.spiAppendCount = static_cast<ProtocolControl::SpiAppendCount>(get_parameter("spiAppendCount").as_int());
    protocolControl.spiAppendStatus = static_cast<ProtocolControl::SpiAppendStatus>(get_parameter("spiAppendStatus").as_int());
    protocolControl.asciiChecksum = static_cast<ProtocolControl::AsciiChecksum>(get_parameter("asciiChecksum").as_int());
    protocolControl.spiChecksum = static_cast<ProtocolControl::SpiChecksum>(get_parameter("spiChecksum").as_int());
    protocolControl.errorMode = static_cast<ProtocolControl::ErrorMode>(get_parameter("errorMode").as_int());
    configRegisters.push_back(&protocolControl);

    // Binary Output 1 (VIO 고속 출력)
    BinaryOutput1 bo1;
    bo1.asyncMode = get_parameter("BO1.asyncMode").as_int();
    bo1.rateDivisor = get_parameter("BO1.rateDivisor").as_int();
    bo1.common = get_parameter("BO1.commonField").as_int();
    bo1.time = get_parameter("BO1.timeField").as_int();
    bo1.imu = get_parameter("BO1.imuField").as_int();
    bo1.gnss = get_parameter("BO1.gpsField").as_int();
    bo1.attitude = get_parameter("BO1.attitudeField").as_int();
    bo1.ins = get_parameter("BO1.insField").as_int();
    bo1.gnss2 = get_parameter("BO1.gps2Field").as_int();
    configRegisters.push_back(&bo1);

    // Binary Output 2
    BinaryOutput2 bo2;
    bo2.asyncMode = get_parameter("BO2.asyncMode").as_int();
    bo2.rateDivisor = get_parameter("BO2.rateDivisor").as_int();
    bo2.common = get_parameter("BO2.commonField").as_int();
    bo2.time = get_parameter("BO2.timeField").as_int();
    bo2.imu = get_parameter("BO2.imuField").as_int();
    bo2.gnss = get_parameter("BO2.gpsField").as_int();
    bo2.attitude = get_parameter("BO2.attitudeField").as_int();
    bo2.ins = get_parameter("BO2.insField").as_int();
    bo2.gnss2 = get_parameter("BO2.gps2Field").as_int();
    configRegisters.push_back(&bo2);

    // Binary Output 3
    BinaryOutput3 bo3;
    bo3.asyncMode = get_parameter("BO3.asyncMode").as_int();
    bo3.rateDivisor = get_parameter("BO3.rateDivisor").as_int();
    bo3.common = get_parameter("BO3.commonField").as_int();
    bo3.time = get_parameter("BO3.timeField").as_int();
    bo3.imu = get_parameter("BO3.imuField").as_int();
    bo3.gnss = get_parameter("BO3.gpsField").as_int();
    bo3.attitude = get_parameter("BO3.attitudeField").as_int();
    bo3.ins = get_parameter("BO3.insField").as_int();
    bo3.gnss2 = get_parameter("BO3.gps2Field").as_int();
    configRegisters.push_back(&bo3);

    // 레지스터 일괄 쓰기
    for (auto& reg : configRegisters) {
        VN::Error vnError = vs_.writeRegister(reg);
        if (vnError != VN::Error::None) {
            RCLCPP_ERROR(get_logger(), "[IMU-ERROR] Unable to configure Register %d -> %s",
                         reg->id(), VN::errorCodeToString(vnError).data());

            return true;
        }
    }

    RCLCPP_INFO(get_logger(), "[IMU-INFO] Sensor configured for VIO (GNSS disabled, IMU high-rate).");

    return false;
}

// ─── Transaction 서비스 ─── //
void VN200Driver::vnTransaction(
    const std::shared_ptr<raptor_msgs::srv::Transaction::Request> request,
    const std::shared_ptr<raptor_msgs::srv::Transaction::Response> response) {
    RCLCPP_INFO(get_logger(), "[IMU-INFO] (TX) $VN%s*XX", request->send.c_str());
    VN::GenericCommand cmd(request->send.c_str());
    VN::Error vnError = vs_.sendCommand(&cmd, VN::Sensor::SendCommandBlockMode::Block);
    if (VN::Error::None != vnError) {
        response->recv = "Error: " + std::string(VN::errorCodeToString(vnError));
        RCLCPP_ERROR(get_logger(), "[IMU-ERROR] (RX) %s", VN::errorCodeToString(vnError).data());
    }
    else {
        response->recv = std::string(cmd.getResponse().c_str());
        RCLCPP_INFO(get_logger(), "[IMU-INFO] (RX) %s", cmd.getResponse().c_str());
    }
}

// ─── 메시지 Publish ─── //
void VN200Driver::publishMessages() {
    const auto cd = vs_.getNextMeasurement();
    if (!cd) { return; }

    // ASCII 메시지는 무시, Binary만 처리
    if (std::holds_alternative<VN::AsciiHeader>(cd->header())) { return; }

    const rclcpp::Time rosTime = now();
    const VN::BinaryHeader header = std::get<VN::BinaryHeader>(cd->header());

    // 표준 sensor_msgs/Imu 퍼블리시 (VIO 알고리즘 호환용)
    publishStdImu(cd.get(), rosTime);

    // // IMU 그룹
    // if (header.isOutputGroupEnabled(IMU_BIT)) {
    //     publishImuGroup(cd.get(), rosTime);
    // }

    // // Attitude 그룹
    // if (header.isOutputGroupEnabled(ATTITUDE_BIT)) {
    //     publishAttitudeGroup(cd.get(), rosTime);
    // }

    // Time 그룹
    if (header.isOutputGroupEnabled(TIME_BIT)) {
        publishTimeGroup(cd.get(), rosTime);
    }
}

// ─── sensor_msgs/Imu 표준 메시지 ─── //
void VN200Driver::publishStdImu(const VN::CompositeData* cd, const rclcpp::Time& rosTime) {
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = rosTime;
    imu_msg.header.frame_id = this->get_parameter("frame_id").as_string();

    // VIO에서 Orientation 사용안함
    // Orientation (quaternion)
    // if (cd->attitude.quaternion.has_value()) {
    //     imu_msg.orientation = toQuaternion(cd->attitude.quaternion.value());
    // }

    // Angular velocity (rad/s)
    if (cd->imu.uncompGyro.has_value()) imu_msg.angular_velocity = toVector3(cd->imu.uncompGyro.value());
    else RCLCPP_ERROR(get_logger(), "[IMU-ERROR] Angular velocity not available");
    
    // Linear acceleration (m/s^2)
    if (cd->imu.uncompAccel.has_value()) imu_msg.linear_acceleration = toVector3(cd->imu.uncompAccel.value());
    else RCLCPP_ERROR(get_logger(), "[IMU-ERROR] Linear acceleration not available");

    pubStdImu_->publish(imu_msg);
}

// 지금 사용안함
// ─── IMU 그룹 상세 메시지 ─── //
void VN200Driver::publishImuGroup(const VN::CompositeData* cd, const rclcpp::Time& rosTime) {
    auto msg = raptor_msgs::msg::ImuGroup();
    msg.header.stamp = rosTime;
    msg.header.frame_id = this->get_parameter("frame_id").as_string();

    if (cd->imu.uncompAccel.has_value()) {
        msg.uncompaccel = toVector3(cd->imu.uncompAccel.value());
    }
    if (cd->imu.uncompGyro.has_value()) {
        msg.uncompgyro = toVector3(cd->imu.uncompGyro.value());
    }
    if (cd->imu.accel.has_value()) {
        msg.accel = toVector3(cd->imu.accel.value());
    }
    if (cd->imu.angularRate.has_value()) {
        msg.angularrate = toVector3(cd->imu.angularRate.value());
    }
    if (cd->imu.temperature.has_value()) {
        msg.temperature = cd->imu.temperature.value();
    }
    if (cd->imu.pressure.has_value()) {
        msg.pressure = cd->imu.pressure.value();
    }
    if (cd->imu.deltaTheta.has_value()) {
        msg.deltatheta.deltatheta = toVector3(cd->imu.deltaTheta->deltaTheta);
        msg.deltatheta.deltatime = cd->imu.deltaTheta->deltaTime;
    }
    if (cd->imu.deltaVel.has_value()) {
        msg.deltavel = toVector3(cd->imu.deltaVel.value());
    }
    if (cd->imu.mag.has_value()) {
        msg.mag = toVector3(cd->imu.mag.value());
    }

    pubImuGroup_->publish(msg);
}

// ─── Attitude 그룹 메시지 ─── //
void VN200Driver::publishAttitudeGroup(const VN::CompositeData* cd, const rclcpp::Time& rosTime) {
    auto msg = raptor_msgs::msg::AttitudeGroup();
    msg.header.stamp = rosTime;
    msg.header.frame_id = this->get_parameter("frame_id").as_string();

    if (cd->attitude.ypr.has_value()) {
        msg.ypr = toVector3(cd->attitude.ypr.value());
    }
    if (cd->attitude.quaternion.has_value()) {
        msg.quaternion = toQuaternion(cd->attitude.quaternion.value());
    }
    if (cd->attitude.dcm.has_value()) {
        msg.dcm.dcm00 = cd->attitude.dcm.value()(0, 0);
        msg.dcm.dcm01 = cd->attitude.dcm.value()(0, 1);
        msg.dcm.dcm02 = cd->attitude.dcm.value()(0, 2);
        msg.dcm.dcm10 = cd->attitude.dcm.value()(1, 0);
        msg.dcm.dcm11 = cd->attitude.dcm.value()(1, 1);
        msg.dcm.dcm12 = cd->attitude.dcm.value()(1, 2);
        msg.dcm.dcm20 = cd->attitude.dcm.value()(2, 0);
        msg.dcm.dcm21 = cd->attitude.dcm.value()(2, 1);
        msg.dcm.dcm22 = cd->attitude.dcm.value()(2, 2);
    }
    if (cd->attitude.linBodyAcc.has_value()) {
        msg.linbodyacc = toVector3(cd->attitude.linBodyAcc.value());
    }
    if (cd->attitude.yprU.has_value()) {
        msg.ypru = toVector3(cd->attitude.yprU.value());
    }

    pubAttitudeGroup_->publish(msg);
}

// ─── Time 그룹 메시지 ─── //
void VN200Driver::publishTimeGroup(const VN::CompositeData* cd, const rclcpp::Time& rosTime) {
    auto msg = raptor_msgs::msg::TimeGroup();
    msg.header.stamp = rosTime;
    msg.header.frame_id = this->get_parameter("frame_id").as_string();

    if (cd->time.timeStartup.has_value()) {
        msg.timestartup = cd->time.timeStartup->nanoseconds();
    }
    if (cd->time.timeSyncIn.has_value()) {
        msg.timesyncin = cd->time.timeSyncIn->nanoseconds();
    }
    if (cd->time.syncInCnt.has_value()) {
        msg.syncincnt = cd->time.syncInCnt.value();
    }
    if (cd->time.syncOutCnt.has_value()) {
        msg.syncoutcnt = cd->time.syncOutCnt.value();
    }
    if (cd->time.timeUtc.has_value()) {
        msg.timeutc.year = cd->time.timeUtc->year;
        msg.timeutc.month = cd->time.timeUtc->month;
        msg.timeutc.day = cd->time.timeUtc->day;
        msg.timeutc.hour = cd->time.timeUtc->hour;
        msg.timeutc.minute = cd->time.timeUtc->minute;
        msg.timeutc.second = cd->time.timeUtc->second;
        msg.timeutc.fracsec = cd->time.timeUtc->fracSec;
    }
    if (cd->time.timeStatus.has_value()) {
        msg.timestatus.bitfield = uint8_t(cd->time.timeStatus.value());
        msg.timestatus.towvalid = (msg.timestatus.bitfield & (0x01 << 0));
        msg.timestatus.datevalid = (msg.timestatus.bitfield & (0x01 << 1));
        msg.timestatus.utcvalid = (msg.timestatus.bitfield & (0x01 << 2));
    }

    pubTimeGroup_->publish(msg);
}
