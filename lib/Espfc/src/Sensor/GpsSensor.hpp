#pragma once

#include "Sensor/BaseSensor.hpp"
#include "../../Device/SerialDevice.hpp"
#include "Gps.hpp"

namespace Espfc::Sensor {

class GpsSensor : public BaseSensor
{
public:
  GpsSensor(Model& model);

  int begin(Device::SerialDevice* port, int baud) override;
  int update() override;

  void calculateHomeVector() const;

private:
  bool processNmea(uint8_t c);
  bool processUbx(uint8_t c);

  void onMessage();
  void handle();
  void handleReceive();
  void detectBaud();
  void readVersion();
  void configureBaud();
  void disableNmea();
  void enableUbx();
  void enableNav5();
  void enableSbas();
  void detectGpsL5();
  void configureRate();
  void configureGnss();
  void setBaud(int baud);
  void setState(State state, State ackState, State timeoutState);
  void setState(State state);
  void handleError();
  void handleCfgValGet() const;
  void handleNavPvt() const;
  void handleNavSat() const;
  void handleVersion() const;
  void checkSupport(const char* payload) const;

  bool isLegacyProto() const { return _model.state.gps.support.version < GPS_M10; }
  void send(const auto& msg, State nextState, State timeoutState)
  {
    _ubxMsg = Gps::UbxMessage();
    _port->write(msg.getData(), msg.getSize());
    setState(nextState, nextState, timeoutState);
  }
  void send(const auto& msg, State nextState)
  {
    send(nextState, nextState, ERROR);
  }

  Device::SerialDevice* _port;
  Gps::UbxParser _ubxParser;
  Gps::UbxMessage _ubxMsg;
  Gps::NmeaMessage _nmeaMsg;
  uint32_t _timeout;
  int _targetBaud;
  int _currentBaud;
  State _ackState;
  State _timeoutState;
  uint8_t _counter;
  uint8_t _fixHoldCounter;
};

} // namespace Espfc::Sensor
