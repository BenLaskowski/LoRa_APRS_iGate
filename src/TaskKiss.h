#ifndef _TASK_KISS_H
#define _TASK_KISS_H

#include "System/TaskManager.h"
#include <APRSMessage.h>
#include <WiFiServer.h>
#include <vector>

class KissTask : public Task {

private:
  // TCP connections
  WiFiServer mServer; // socket, etc., provided by the system
  WiFiClient mClient; // active client

  // implementation of the interface
  virtual bool setup(System &system) override;
  virtual bool loop(System &system) override;

  // helper functions
  bool   decode_address(uint8_t *buffer, char *address, uint8_t *cr); // returns 1 if there are more addresses to decode
  String decode_kiss(std::vector<uint8_t> buffer);
  void   split(String in, String delims, std::vector<String> &out);
  void   encode_callsign(String call, bool final, std::vector<uint8_t> &out);
  void   encode_kiss(String packet, std::vector<uint8_t> &data);

  // references to global data
  logging::Logger                         &mLogger;
  TaskQueue<std::shared_ptr<APRSMessage>> &mFromKiss;
  TaskQueue<std::shared_ptr<APRSMessage>> &mToKiss;

public:
  explicit KissTask(logging::Logger &logger, TaskQueue<std::shared_ptr<APRSMessage>> &fromKiss, TaskQueue<std::shared_ptr<APRSMessage>> &toKiss);
  virtual ~KissTask();
};

#endif // _TASK_KISS_H
