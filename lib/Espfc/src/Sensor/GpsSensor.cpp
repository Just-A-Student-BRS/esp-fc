#include "Sensor/GpsSensor.hpp"
#include <Arduino.h>
#include <Gps.hpp>
#include <cmath>
#include <cstdlib>
#include <tuple>
#include <cstring>

namespace Espfc::Sensor {

static constexpr std::array<int, 6> BAUDS{
    9600, 115200, 230400, 57600, 38400, 19200,
};

static constexpr std::array<uint16_t, 6> NMEA_MSG_OFF{
    Gps::NMEA_MSG_GGA, Gps::NMEA_MSG_GLL, Gps::NMEA_MSG_GSA, Gps::NMEA_MSG_GSV, Gps::NMEA_MSG_RMC, Gps::NMEA_MSG_VTG,
};

static constexpr std::array<std::tuple<uint16_t, uint8_t>, 2> UBX_MSG_ON{{
    {Gps::UBX_NAV_PVT, 1u},
    {Gps::UBX_NAV_SAT, 10u},
}};

GpsSensor::GpsSensor(Model& model): _model(model), _fixHoldCounter(0) {}

int GpsSensor::begin(Device::SerialDevice* port, int baud)
{
  _port = port;
  _targetBaud = _currentBaud = 9600;
  _timer.setRate(50);

  _state = GpsSensor::RECEIVE; // Skip detection loops and stay in RECEIVE state
  setBaud(9600);

  return 1;
}

int GpsSensor::update()
{
  if (!_port) return 0;

  if (!_timer.check()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_GPS_READ);

  bool updated = false;
  uint8_t buff[32];
  size_t read = 0;
  while ((read = _port->readMany(buff, sizeof(buff))))
  {
    for (size_t i = 0; i < read; i++)
    {
      updated |= processNmea(buff[i]);            // Processes NMEA sentences and extracts coordinates, fix, alt, speed, heading
    }
  }

  // Real-time status update: light up icon if valid data or bytes flow, turn grey if disconnected
  if (updated || read > 0)
  {
    _model.state.gps.present = true;
    _state = GpsSensor::RECEIVE;
  }
  else
  {
    _model.state.gps.present = false;
  }

  return 1;
}

bool GpsSensor::processUbx(uint8_t c)
{
  _ubxParser.parse(c, _ubxMsg);
  if (!_ubxMsg.isReady()) return false;

  onMessage();

  handle();
  _ubxMsg = Gps::UbxMessage();

  return true;
}

bool GpsSensor::processNmea(uint8_t c)
{
  static char line[128];
  static uint8_t idx = 0;
  bool parsed = false;

  if (c == '$') {
    idx = 0;
    line[idx++] = c;
  } else if (c == '\r' || c == '\n') {
    if (idx > 6) {
      line[idx] = '\0';
      
      // Check for frame error text log
      static const char* txtMsg = "GNTXT,01,01,01,More than 100 frame errors";
      if (!_model.state.gps.frameError && std::strncmp(_nmeaMsg.payload, txtMsg, std::strlen(txtMsg)) == 0)
      {
        _model.state.gps.frameError = true;
        if (!_model.isModeActive(MODE_ARMED)) _model.logger.err().logln("GPS RX Frame Err");
      }

      // 1. Parse GPGGA / GNGGA (Position, Fix, Satellites, Altitude)
      if (std::strncmp(line, "$GPGGA", 6) == 0 || std::strncmp(line, "$GNGGA", 6) == 0) {
        char *p = line;
        int field = 0;
        double raw_lat = 0, raw_lon = 0, alt = 0;
        int sats = 0;
        int fix = 0;

        while ((p = std::strchr(p, ',')) != NULL) {
          p++;
          field++;
          if (field == 2) raw_lat = std::atof(p);       // Raw Latitude (DDMM.MMMM)
          else if (field == 4) raw_lon = std::atof(p);  // Raw Longitude (DDDMM.MMMM)
          else if (field == 6) fix = std::atoi(p);      // Fix quality
          else if (field == 7) sats = std::atoi(p);     // Satellite count
          else if (field == 9) alt = std::atof(p);      // Altitude MSL (meters)
        }

        if (fix > 0 && raw_lat != 0.0 && raw_lon != 0.0) {
          int deg_lat = (int)(raw_lat / 100.0);
          double min_lat = raw_lat - (deg_lat * 100.0);
          double dec_lat = deg_lat + (min_lat / 60.0);

          int deg_lon = (int)(raw_lon / 100.0);
          double min_lon = raw_lon - (deg_lon * 100.0);
          double dec_lon = deg_lon + (min_lon / 60.0);

          int32_t lat_scaled = (int32_t)(dec_lat * 10000000.0);
          int32_t lon_scaled = (int32_t)(dec_lon * 10000000.0);

          // Position Smoothing Filter (EMA): Prevent random 100m jumps
          static int32_t smoothed_lat = 0;
          static int32_t smoothed_lon = 0;

          if (smoothed_lat == 0 || smoothed_lon == 0) {
            smoothed_lat = lat_scaled;
            smoothed_lon = lon_scaled;
          } else {
            smoothed_lat = smoothed_lat + (int32_t)((float)(lat_scaled - smoothed_lat) * 0.1f);
            smoothed_lon = smoothed_lon + (int32_t)((float)(lon_scaled - smoothed_lon) * 0.1f);
          }

          _model.state.gps.location.raw.lat = smoothed_lat;
          _model.state.gps.location.raw.lon = smoothed_lon;
          _model.state.gps.numSats = sats;
          _model.state.gps.fixType = fix;
          _model.state.gps.accuracy.horizontal = (sats >= 4) ? 100 : 999;
          _model.state.gps.location.raw.height = (int32_t)(alt * 1000.0f); // Meters to millimeters

          _fixHoldCounter = 15; // Hold state stable for 15 valid cycles
          calculateHomeVector();
          parsed = true;
        }
      }
      
      // 2. Parse GPVTG / GNVTG (Ground Speed and Heading)
      else if (std::strncmp(line, "$GPVTG", 6) == 0 || std::strncmp(line, "$GNVTG", 6) == 0) {
        char *p = line;
        int field = 0;
        float course = 0;
        float speed_kmh = 0;

        while ((p = std::strchr(p, ',')) != NULL) {
          p++;
          field++;
          if (field == 1) course = std::atof(p);         // Heading in degrees
          else if (field == 7) speed_kmh = std::atof(p); // Speed in km/h
        }

        uint32_t speed_mmps = (uint32_t)((speed_kmh / 3.6f) * 1000.0f); // km/h to mm/s
        if (speed_mmps < 50) {
          speed_mmps = 0; // Deadband filter for stationary noise
        }

        _model.state.gps.velocity.raw.groundSpeed = speed_mmps;
        _model.state.gps.velocity.raw.heading = (int32_t)(course * 100000.0f); // deg * 1e5

        parsed = true;
      }
    }
    idx = 0;
  } else if (idx < sizeof(line) - 1) {
    line[idx++] = c;
  } else {
    idx = 0;
  }

  // Bind fix state directly to active coordinate validation and hold counter
  if (_fixHoldCounter > 0 && _model.state.gps.location.raw.lat != 0) {
    _fixHoldCounter--;
    _model.state.gps.fix = true;
  } else {
    _model.state.gps.fix = false;
  }

  return parsed;
}

void GpsSensor::onMessage()
{
  if (_state == GpsSensor::DETECT_BAUD)
  {
    _state = GpsSensor::GET_VERSION;
    _model.logger.info().log(F("GPS DET")).logln(_currentBaud);
  }
}

void GpsSensor::handle()
{
  switch (_state)
  {
    case GpsSensor::DETECT_BAUD:
      detectBaud();
      break;

    case GpsSensor::GET_VERSION:
      readVersion();
      break;

    case GpsSensor::CONFIGURE_BAUD:
      configureBaud();
      break;

    case GpsSensor::DISABLE_NMEA:
      disableNmea();
      break;

    case GpsSensor::ENABLE_UBX:
      enableUbx();
      break;

    case GpsSensor::ENABLE_NAV5:
      enableNav5();
      break;

    case GpsSensor::ENABLE_SBAS:
      enableSbas();
      break;

    case GpsSensor::DETECT_GPS_L5:
      detectGpsL5();
      break;

    case GpsSensor::CONFIGURE_GNSS:
      configureGnss();
      break;

    case GpsSensor::CONFIGURE_NAV_RATE:
      configureRate();
      break;

    case GpsSensor::ERROR:
      handleError();
      break;

    case GpsSensor::RECEIVE:
    case GpsSensor::WAIT:
    default:
      handleReceive();
      break;
  }
}

void GpsSensor::handleReceive()
{
  if (_state == GpsSensor::RECEIVE)
  {
    _model.state.gps.present = true;
  }

  if (_ubxMsg.isReady())
  {
    if (_ubxMsg.isAck())
    {
      _state = _ackState;
    }
    else if (_ubxMsg.isNak())
    {
      _state = _timeoutState;
      _model.logger.err().log(F("GPS NAK")).loghex(_ubxMsg.payload[0]).loghex(_ubxMsg.payload[1]).endl();
    }
    else if (_ubxMsg.isResponse(Gps::UBX_CFG_VALGET))
    {
      handleCfgValGet();
      _state = _ackState;
      _counter = 0;
    }
    else if (_ubxMsg.isResponse(Gps::UbxMonVer::ID))
    {
      handleVersion();
      _state = _ackState;
      _counter = 0;
    }
    else if (_ubxMsg.isResponse(Gps::UbxNavPvt92::ID))
    {
      handleNavPvt();
    }
    else if (_ubxMsg.isResponse(Gps::UbxNavSat::ID))
    {
      handleNavSat();
    }
  }
  else if (_state == GpsSensor::WAIT && micros() > _timeout)
  {
    _state = _timeoutState;
    _model.state.gps.present = false;
    _model.logger.err().logln(F("GPS TOUT"));
  }
}

void GpsSensor::detectBaud()
{
  if (micros() > _timeout)
  {
    if (_counter < BAUDS.size())
    {
      setBaud(BAUDS[_counter]);
      _counter++;
    }
    else
    {
      _state = GpsSensor::ERROR;
      _counter = 0;
      setBaud(_targetBaud);
    }
    _timeout = micros() + DETECT_TIMEOUT;
  }
}

void GpsSensor::readVersion()
{
  send(Gps::UbxMonVer{}, GpsSensor::CONFIGURE_BAUD);
  _timeout = micros() + 3 * TIMEOUT;
}

void GpsSensor::configureBaud()
{
  if (isLegacyProto())
  {
    send(
        Gps::UbxCfgPrt20{
            .portId = 1,
            .resered1 = 0,
            .txReady = 0,
            .mode = 0x08c0,
            .baudRate = (uint32_t)_targetBaud,
            .inProtoMask = 0x07,
            .outProtoMask = 0x07,
            .flags = 0,
            .resered2 = 0,
        },
        GpsSensor::DISABLE_NMEA, GpsSensor::DISABLE_NMEA);
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_UART1_BAUDRATE, uint32_t>(_targetBaud));
    send(req, GpsSensor::DISABLE_NMEA, GpsSensor::DISABLE_NMEA);
  }
  delay(30);
  setBaud(_targetBaud);
  delay(5);
}

void GpsSensor::disableNmea()
{
  if (isLegacyProto())
  {
    const Gps::UbxCfgMsg3 m{
        .msgId = NMEA_MSG_OFF[_counter],
        .rate = 0,
    };
    _counter++;
    if (_counter < NMEA_MSG_OFF.size())
    {
      send(m, _state);
    }
    else
    {
      _counter = 0;
      send(m, GpsSensor::ENABLE_UBX);
      _model.logger.info().logln(F("GPS NMEA OFF"));
    }
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GGA_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GLL_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GSA_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_GSV_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_RMC_UART1, bool>(0));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_NMEA_VTG_UART1, bool>(0));
    send(req, GpsSensor::ENABLE_UBX);
    _model.logger.info().logln(F("GPS NMEA* OFF"));
  }
}

void GpsSensor::enableUbx()
{
  if (isLegacyProto())
  {
    const Gps::UbxCfgMsg3 m{
        .msgId = std::get<0>(UBX_MSG_ON[_counter]),
        .rate = std::get<1>(UBX_MSG_ON[_counter]),
    };
    _counter++;
    if (_counter < UBX_MSG_ON.size())
    {
      send(m, _state);
    }
    else
    {
      send(m, GpsSensor::ENABLE_NAV5);
      _counter = 0;
      _timeout = micros() + 10 * TIMEOUT;
      _model.logger.info().logln(F("GPS UBX ON"));
    }
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_UBX_NAV_PVT_UART1, uint8_t>(1));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_MSGOUT_UBX_NAV_SAT_UART1, uint8_t>(10));
    send(req, GpsSensor::ENABLE_NAV5);
    _model.logger.info().logln(F("GPS UBX* ON"));
  }
}

void GpsSensor::enableNav5()
{
  if (isLegacyProto())
  {
    send(
        Gps::UbxCfgNav5{
            .mask = {.value = 0xffff},
            .dynModel = 8,
            .fixMode = 3,
            .fixedAlt = 0,
            .fixedAltVar = 10000,
            .minElev = 5,
            .drLimit = 0,
            .pDOP = 250,
            .tDOP = 250,
            .pAcc = 100,
            .tAcc = 300,
            .staticHoldThresh = 0,
            .dgnssTimeout = 60,
            .cnoThreshNumSVs = 0,
            .cnoThresh = 0,
            .reserved0 = {0, 0},
            .staticHoldMaxDist = 200,
            .utcStandard = 0,
            .reserved1 = {0, 0, 0, 0, 0},
        },
        GpsSensor::ENABLE_SBAS);
    _model.logger.info().logln(F("GPS NAV5"));
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_NAVSPG_DYNMODEL, uint8_t>(8));
    send(req, GpsSensor::ENABLE_SBAS);
    _model.logger.info().logln(F("GPS NAVSPG*"));
  }
}

void GpsSensor::enableSbas()
{
  if (_model.state.gps.support.sbas)
  {
    if (isLegacyProto())
    {
      send(
          Gps::UbxCfgSbas8{
              .mode = 1,
              .usage = 1,
              .maxSbas = 3,
              .scanmode2 = 0,
              .scanmode1 = 0,
          },
          GpsSensor::DETECT_GPS_L5);
      _model.logger.info().logln(F("GPS SBAS"));
    }
    else
    {
      Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
      req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});
      req.write(Gps::UbxCfgValsetItem<Gps::CFG_SBAS_PRNSCANMASK, uint64_t>(0));
      send(req, GpsSensor::DETECT_GPS_L5);
      _model.logger.info().logln(F("GPS SBAS*"));
    }
  }
  else
  {
    setState(GpsSensor::DETECT_GPS_L5);
  }
}

void GpsSensor::detectGpsL5()
{
  Gps::UbxRequest req(Gps::UBX_CFG_VALGET);
  req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});
  req.write(Gps::CFG_SIGNAL_GPS_L5);
  send(req, GpsSensor::CONFIGURE_GNSS, GpsSensor::CONFIGURE_GNSS);
}

void GpsSensor::configureRate()
{
  uint16_t mRate = 200;
  if (_currentBaud > 100000) mRate = 100;
  if (_model.state.gps.support.version == GPS_M10 && _currentBaud > 200000) mRate = 40;
  const uint16_t nRate = 1;

  if (isLegacyProto())
  {
    const Gps::UbxCfgRate6 m{
        .measRate = mRate,
        .navRate = nRate,
        .timeRef = 0,
    };
    send(m, GpsSensor::RECEIVE);
    _model.logger.info().log(F("GPS NAVRATE")).log(mRate).log(nRate).logln(1000 / mRate);
  }
  else
  {
    Gps::UbxRequest req(Gps::UBX_CFG_VALSET);
    req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_MEAS, uint16_t>(mRate));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_NAV, uint16_t>(nRate));
    req.write(Gps::UbxCfgValsetItem<Gps::CFG_RATE_TIMEREF, uint8_t>(0));
    send(req, GpsSensor::RECEIVE);
    _model.logger.info().log(F("GPS NAVRATE*")).log(mRate).log(nRate).logln(1000 / mRate);
  }
}

void GpsSensor::setBaud(int baud)
{
  if (baud != _currentBaud)
  {
    _port->updateBaudRate(baud);
    _currentBaud = baud;
    _model.logger.info().log(F("GPS BAUD")).logln(baud);
  }
}

void GpsSensor::setState(State state, State ackState, State timeoutState)
{
  setState(state);
  _ackState = ackState;
  _timeoutState = timeoutState;
}

void GpsSensor::setState(State state)
{
  _state = state;
  _timeout = micros() + TIMEOUT;
}

void GpsSensor::handleError()
{
  if (_counter == 0)
  {
    _model.logger.err().logln(F("GPS ERROR"));
    _counter++;
  }
  _model.state.gps.present = false;
}

void GpsSensor::configureGnss()
{
  const bool useDualBand = _model.config.gps.enableDualBand && _model.state.gps.support.gpsL5;
  bool enableGPS = _model.config.gps.enableGPS;
  bool enableGLO = _model.config.gps.enableGLONASS;
  bool enableGAL = _model.config.gps.enableGalileo;
  bool enableBDS = _model.config.gps.enableBeiDou;
  bool enableQZSS = _model.config.gps.enableQZSS;
  bool enableSBAS = _model.config.gps.enableSBAS;

  const auto& support = _model.state.gps.support;
  switch (_model.config.gps.gnssMode)
  {
    case 1:
      enableGPS = support.gps;
      enableGLO = enableGAL = enableBDS = enableQZSS = false;
      break;
    case 2:
      enableGPS = support.gps;
      enableGLO = support.glonass;
      enableGAL = enableBDS = enableQZSS = false;
      break;
    case 3:
      enableGPS = support.gps;
      enableGAL = support.galileo;
      enableGLO = enableBDS = enableQZSS = false;
      break;
    case 4:
      enableGPS = support.gps;
      enableBDS = support.beidou;
      enableGLO = enableGAL = enableQZSS = false;
      break;
    case 5:
      enableGPS = support.gps;
      enableGLO = support.glonass;
      enableGAL = support.galileo;
      enableBDS = support.beidou;
      enableQZSS = support.qzss;
      break;
  }

  size_t written = 0;
  if (isLegacyProto())
  {
    Gps::UbxRequest req{Gps::UBX_CFG_GNSS};
    uint8_t numBlocks = _model.state.gps.support.gps + _model.state.gps.support.sbas +
                        _model.state.gps.support.galileo + _model.state.gps.support.beidou +
                        _model.state.gps.support.qzss + _model.state.gps.support.glonass +
                        _model.state.gps.support.imes;

    written += req.write(
        Gps::UbxCfgGnssHeader{.msgVer = 0, .numTrkChHw = 32, .numTrkChUse = 0xff, .numConfigBlocks = numBlocks});
    if (_model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 0,
                                                .resTrkCh = 8,
                                                .maxTrkCh = 16,
                                                .flagsEnable = enableGPS,
                                                .sigCfgMask = (uint8_t)(useDualBand ? 0x20 : 0x01),
                                                .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.sbas)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 1, .resTrkCh = 1, .maxTrkCh = 3, .flagsEnable = enableSBAS, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.galileo)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 2, .resTrkCh = 4, .maxTrkCh = 8, .flagsEnable = enableGAL, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.beidou)
    {
      written += req.write(Gps::UbxCfgGnssBlock{.gnssId = 3,
                                                .resTrkCh = 8,
                                                .maxTrkCh = 16,
                                                .flagsEnable = enableBDS,
                                                .sigCfgMask = (uint8_t)(false ? 0x80 : 0x01),
                                                .flagsHigh = 0x01});
    }
    if (_model.state.gps.support.imes)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 4, .resTrkCh = 0, .maxTrkCh = 8, .flagsEnable = 0, .sigCfgMask = 0x01, .flagsHigh = 0x03});
    }
    if (_model.state.gps.support.qzss)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 5, .resTrkCh = 0, .maxTrkCh = 3, .flagsEnable = enableQZSS, .sigCfgMask = 0x01, .flagsHigh = 0x05});
    }
    if (_model.state.gps.support.glonass)
    {
      written += req.write(Gps::UbxCfgGnssBlock{
          .gnssId = 6, .resTrkCh = 8, .maxTrkCh = 14, .flagsEnable = enableGLO, .sigCfgMask = 0x01, .flagsHigh = 0x01});
    }
    send(req, GpsSensor::CONFIGURE_NAV_RATE);
  }
  else
  {
    Gps::UbxRequest req{Gps::UBX_CFG_VALSET};
    written += req.write(Gps::UbxCfgValsetHeader{.version = 0, .layers = 0x01});
    if (_model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GPS_ENA, bool>(enableGPS));
    }
    if (_model.state.gps.support.sbas)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_SBAS_ENA, bool>(enableSBAS));
    }
    if (_model.state.gps.support.galileo)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GAL_ENA, bool>(enableGAL));
    }
    if (_model.state.gps.support.qzss)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_QZSS_ENA, bool>(enableQZSS));
    }
    if (_model.state.gps.support.glonass)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GLO_ENA, bool>(enableGLO));
    }
    if (_model.state.gps.support.beidou)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_BDS_ENA, bool>(enableBDS));
    }
    if (useDualBand && _model.state.gps.support.gps)
    {
      written += req.write(Gps::UbxCfgValsetItem<Gps::CFG_SIGNAL_GPS_L5, bool>(useDualBand));
    }
    send(req, GpsSensor::CONFIGURE_NAV_RATE);
  }

  _model.logger.info().log(F("GPS GNSS"));
  if (isLegacyProto()) _model.logger.log(F("LEGACY"));
  if (enableGPS) _model.logger.log(F("GPS"));
  if (useDualBand) _model.logger.log(F("L1+L5"));
  if (enableGLO) _model.logger.log(F("GLO"));
  if (enableGAL) _model.logger.log(F("GAL"));
  if (enableBDS) _model.logger.log(F("BDS"));
  if (enableSBAS) _model.logger.log(F("SBAS"));
  if (enableQZSS) _model.logger.log(F("QZSS"));
  _model.logger.logln(written);
}

void GpsSensor::calculateHomeVector() const
{
  if (!_model.state.gps.isHomeValid())
  {
    _model.state.gps.distanceToHome = 0;
    _model.state.gps.directionToHome = 0;
    return;
  }

  const int32_t lat1 = _model.state.gps.location.home.lat;
  const int32_t lon1 = _model.state.gps.location.home.lon;
  const int32_t lat2 = _model.state.gps.location.raw.lat;
  const int32_t lon2 = _model.state.gps.location.raw.lon;

  const auto [distance, bearing] = Gps::calculateDistanceAndBearing(lat1, lon1, lat2, lon2);

  _model.state.gps.distanceToHome = distance;
  _model.state.gps.directionToHome = bearing;
}

void GpsSensor::handleCfgValGet() const
{
  const uint32_t key = *(reinterpret_cast<const uint32_t*>(_ubxMsg.payload) + sizeof(Gps::UbxCfgValsetHeader));
  if (key == Gps::CFG_SIGNAL_GPS_L5)
  {
    _model.state.gps.support.gpsL5 = true;
    _model.logger.info().logln(F("GPS DET L5"));
  }
}

void GpsSensor::handleNavPvt() const
{
  const auto& m = *_ubxMsg.getAs<Gps::UbxNavPvt92>();

  _model.state.gps.fix = m.fixType == 3 && m.flags.gnssFixOk;
  _model.state.gps.fixType = m.fixType;
  _model.state.gps.numSats = m.numSV;

  _model.state.gps.accuracy.pDop = m.pDOP;
  _model.state.gps.accuracy.horizontal = m.hAcc;
  _model.state.gps.accuracy.vertical = m.vAcc;
  _model.state.gps.accuracy.speed = m.sAcc;
  _model.state.gps.accuracy.heading = m.headAcc;

  _model.state.gps.location.raw.lat = m.lat;
  _model.state.gps.location.raw.lon = m.lon;
  _model.state.gps.location.raw.height = m.hSML;

  _model.state.gps.velocity.raw.groundSpeed = m.gSpeed;
  _model.state.gps.velocity.raw.heading = m.headMot;

  _model.state.gps.velocity.raw.north = m.velN;
  _model.state.gps.velocity.raw.east = m.velE;
  _model.state.gps.velocity.raw.down = m.velD;
  _model.state.gps.velocity.raw.speed3d =
      lrintf(std::hypot(static_cast<float>(_model.state.gps.velocity.raw.groundSpeed),
                        static_cast<float>(_model.state.gps.velocity.raw.down)));

  if (m.valid.validDate && m.valid.validTime)
  {
    _model.state.gps.dateTime.year = m.year;
    _model.state.gps.dateTime.month = m.month;
    _model.state.gps.dateTime.day = m.day;
    _model.state.gps.dateTime.hour = m.hour;
    _model.state.gps.dateTime.minute = m.min;
    _model.state.gps.dateTime.second = m.sec;
    int32_t msec = m.nano / 1000000;
    if (msec < 0)
    {
      msec += 1000;
    }
    _model.state.gps.dateTime.msec = msec;
  }

  uint32_t now = micros();
  _model.state.gps.interval = now - _model.state.gps.lastMsgTs;
  _model.state.gps.lastMsgTs = now;

  calculateHomeVector();
}

void GpsSensor::handleNavSat() const
{
  const auto& m = *_ubxMsg.getAs<Gps::UbxNavSat>();
  _model.state.gps.numCh = m.numSvs;
  for (uint8_t i = 0; i < SAT_MAX; i++)
  {
    if (i < m.numSvs)
    {
      _model.state.gps.svinfo[i].id = m.sats[i].svId;
      _model.state.gps.svinfo[i].gnssId = m.sats[i].gnssId;
      _model.state.gps.svinfo[i].cno = m.sats[i].cno;
      _model.state.gps.svinfo[i].quality.value = m.sats[i].flags.value;
    }
    else
    {
      _model.state.gps.svinfo[i] = GpsSatelite{};
    }
  }
}

void GpsSensor::handleVersion() const
{
  const char* payload = (const char*)_ubxMsg.payload;

  _model.logger.info().log(F("GPS VER")).logln(payload);
  _model.logger.info().log(F("GPS VER")).logln(payload + 30);

  if (std::strcmp(payload + 30, "00080000") == 0)
  {
    _model.state.gps.support.version = GPS_M8;
  }
  else if (std::strcmp(payload + 30, "00090000") == 0)
  {
    _model.state.gps.support.version = GPS_M9;
  }
  else if (std::strcmp(payload + 30, "00190000") == 0)
  {
    _model.state.gps.support.version = GPS_F9;
  }
  else if (std::strcmp(payload + 30, "000A0000") == 0)
  {
    _model.state.gps.support.version = GPS_M10;
  }

  if (_ubxMsg.length >= 70)
  {
    checkSupport(payload + 40);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 40);
  }
  if (_ubxMsg.length >= 100)
  {
    checkSupport(payload + 70);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 70);
  }
  if (_ubxMsg.length >= 130)
  {
    checkSupport(payload + 100);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 100);
  }
  if (_ubxMsg.length >= 160)
  {
    checkSupport(payload + 130);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 130);
  }
}

void GpsSensor::checkSupport(const char* payload) const
{
  if (std::strstr(payload, "GPS") != nullptr)
  {
    _model.state.gps.support.gps = true;
  }
  if (std::strstr(payload, "SBAS") != nullptr)
  {
    _model.state.gps.support.sbas = true;
  }
  if (std::strstr(payload, "GLO") != nullptr)
  {
    _model.state.gps.support.glonass = true;
  }
  if (std::strstr(payload, "GAL") != nullptr)
  {
    _model.state.gps.support.galileo = true;
  }
  if (std::strstr(payload, "BDS") != nullptr)
  {
    _model.state.gps.support.beidou = true;
  }
  if (std::strstr(payload, "QZSS") != nullptr)
  {
    _model.state.gps.support.qzss = true;
  }
  if (std::strstr(payload, "IMES") != nullptr)
  {
    _model.state.gps.support.imes = true;
  }
  const char* pv = std::strstr(payload, "PROTVER=");
  if (pv != nullptr)
  {
    _model.state.gps.support.protVerMajor = (uint8_t)std::atoi(pv + 8);
  }
}

} // namespace Espfc::Sensor
