#include "TaskKiss.h"
#include "Task.h"
#include "project_configuration.h"
#include <vector>

static const int N_DIGI = 8;

KissTask::KissTask(logging::Logger &logger, TaskQueue<std::shared_ptr<APRSMessage>> &fromKiss, TaskQueue<std::shared_ptr<APRSMessage>> &toKiss) : Task(TASK_KISS, TaskKiss), mLogger(logger), mFromKiss(fromKiss), mToKiss(toKiss) {
}

KissTask::~KissTask() {
}

bool KissTask::decode_address(uint8_t *buffer, char *address, uint8_t *cr) {
  // null out the address to start
  memset(address, 0, 10);

  // base part of the address
  char *p = address;
  for (int i = 0; i < 6; i++) {
    if (buffer[i] != 0x40)
      *p++ = buffer[i] >> 1;
  }

  // SSID also contains repeated and final bits
  *cr          = (buffer[6] & 128) ? 1 : 0;
  uint8_t ssid = (buffer[6] >> 1) & 15;
  if (ssid)
    sprintf(p, "-%d", ssid);

  // printf("decode_address():  %s\n", address);

  return ((buffer[6] & 1) == 0);
}

String KissTask::decode_kiss(std::vector<uint8_t> buffer) {
  String packet;
  size_t n = buffer.size();

  // decode the packet
  if ((buffer[0] != 0xC0) || (buffer[1] != 0) || (buffer[n - 1] != 0xC0)) {
    mLogger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, TASK_KISS, "Invalid delimiters %.02X %.02X ... %.02X", buffer[0], buffer[1], buffer[n - 1]);
    return packet;
  }

  // handle any escaped characters
  int adj = 0;
  for (int i = 2; i < n - 2; i++) {
    if ((buffer[i] == 0xDB) && (buffer[i + 1] == 0xDC)) {
      buffer[i] = 0xC0;
      memcpy(&buffer[i], &buffer[i + 2], n - i);
      ++adj;
    }
    if ((buffer[i] == 0xDB) && (buffer[i + 1] == 0xDD)) {
      buffer[i] = 0xDB;
      memcpy(&buffer[i], &buffer[i + 2], n - i);
      ++adj;
    }
  }

  // source and destination addresses
  char    source[10], destination[10];
  uint8_t source_ctl, destination_ctl;
  decode_address(&buffer[2], destination, &destination_ctl);
  uint8_t more = decode_address(&buffer[9], source, &source_ctl);

  // digipeater addresses
  size_t  i = 16;
  char    digi[N_DIGI][10];
  uint8_t digih[N_DIGI];
  for (int d = 0; d < N_DIGI; d++) {
    digi[d][0] = 0;
    digih[d]   = 0;
    if (!more)
      break;
    more = decode_address(&buffer[i], digi[d], &digih[d]);
    i += 7;
  }

  // check for UI frame
  if ((buffer[i] & 3) != 3) {
    mLogger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, TASK_KISS, "Not a UI frame (0x%.02X)", buffer[i]);
    return packet;
  }

  // check for correct PID
  if (buffer[i + 1] != 0xF0) {
    mLogger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, TASK_KISS, "Unexpected PID 0x%.02X", buffer[i + 1]);
    return packet;
  }

  // display some stuff, finally!
  packet = String(source) + ">" + destination;
  for (int d = 0; d < N_DIGI; d++) {
    if (digi[d][0] == 0)
      break;
    packet += "," + String(digi[d]) + String(digih[d] ? "*" : "");
  }
  buffer[n - 1 - adj] = 0;
  packet += ":";
  packet += String(&buffer[i + 2], n - 1 - adj - i - 2);

  mLogger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, TASK_KISS, "KISS packet:  '%s'", packet.c_str());

  return packet;
}

void KissTask::split(String in, String delims, std::vector<String> &out) {
  ssize_t off   = 0;
  int     count = 0;
  while ((off = in.indexOf(delims)) != -1) {
    out.push_back(in.substring(0, off));
    in = in.substring(off + 1);
    count++;
  }

  // loop terminates before last element has been pushed back
  out.push_back(in);
}

void KissTask::encode_callsign(String call, bool final, std::vector<uint8_t> &out) {
  // split call from SSID
  std::vector<String> tmp;
  split(call, "-", tmp);
  int ssid = 0;
  if (tmp.size() > 1)
    ssid = std::atoi(tmp[1].c_str());

  // append callsign to vector
  int i = 0;
  for (char c : tmp[0]) {
    i++;
    out.push_back(c << 1);
  }

  // pad with spaces
  for (; i < 6; i++)
    out.push_back(0x40);

  // final byte
  bool digipeated = call[call.length() - 1] == '*';
  out.push_back((digipeated << 7) | (3 << 5) | (ssid << 1) | final);
}

void KissTask::encode_kiss(String packet, std::vector<uint8_t> &data) {

  mLogger.log(logging::LoggerLevel::LOGGER_LEVEL_DEBUG, TASK_KISS, "LoRa packet:  '%s'", packet.c_str());

  // '>' delimits source and destination
  size_t isrc = packet.indexOf(">");
  String src  = packet.substring(0, isrc);

  // ':' delimits routing information and payload
  size_t ipayload = packet.indexOf(":");
  String payload  = packet.substring(ipayload + 1);

  // split destination and path into components
  String destpath = packet.substring(isrc + 1, ipayload);
  std::vector<String> components;
  split(destpath, ",", components);

  // build the encoded message
  std::vector<uint8_t> tmp;
  encode_callsign(components[0], false, tmp);
  encode_callsign(src, components.size() == 1, tmp);
  for (int i = 1; i < components.size(); i++)
    encode_callsign(components[i], (components.size() == (i + 1)), tmp);
  tmp.push_back(0x03);
  tmp.push_back(0xF0);
  tmp.insert(tmp.end(), payload.begin(), payload.end());

  // escape characters, if needed
  data.push_back(0xC0);
  data.push_back(0x00);
  for (uint8_t c : tmp) {
    if (c == 0xC0) {
      data.push_back(0xDB);
      data.push_back(0xDC);
    } else if (c == 0xDB) {
      data.push_back(0xDB);
      data.push_back(0xDD);
    } else
      data.push_back(c);
  }
  data.push_back(0xC0);
}

bool KissTask::setup(System &system) {

  // listen for clients on port 8001 if enabled
  if (system.getUserConfig()->kiss.active) {
    int port = system.getUserConfig()->kiss.port;
    mServer.begin(port);
    mLogger.log(logging::LoggerLevel::LOGGER_LEVEL_INFO, TASK_KISS, "Listening on Port %d", port);
  }

  return true;
}

bool KissTask::loop(System &system) {

  // new connection?
  if (!mClient.connected()) {
    mClient = mServer.available();
  }

  // connected?
  if (mClient.connected()) {

    // data from radio?
    if (!mToKiss.empty()) {

      // pull element from the queue
      std::shared_ptr<APRSMessage> msg = mToKiss.getElement();
      String                       tmp = msg->encode();

      // convert to a KISS-formatted byte string
      std::vector<uint8_t> data;
      encode_kiss(tmp, data);

      // send to the socket
      mClient.write(data.data(), data.size());
    }

    // data from socket?
    if (mClient.available()) {

      // read data from the client
      uint8_t              buffer[512];
      int                  n = mClient.read(buffer, 512);
      std::vector<uint8_t> data(buffer, buffer + n);

      // convert to TNC2 monitor byte string
      String msg = decode_kiss(data);

      // add to radio queue if there was data
      if (msg.length()) {
        std::shared_ptr<APRSMessage> tmp(new APRSMessage());
        tmp->decode(msg);
        mFromKiss.addElement(tmp);
      }
    }
  }

  // make sure the buffer doesn't get too long
  else if (!mToKiss.empty())
    mToKiss.getElement();

  return true;
}
