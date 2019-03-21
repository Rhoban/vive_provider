#pragma once

#include <vive_provider/vive.pb.h>
#include <google/protobuf/text_format.h>
#include <queue>
#include <rhoban_utils/sockets/udp_broadcast.h>
#include <mutex>
#include <thread>

namespace vive_provider {

/**
 * Return the default port for communications
 */
int getDefaultPort();


/**
 * Manage messages, either to send them through UDP or to receive them.
 *
 * Can also be used to store/load messages
 */
class UDPMessageManager {
public:  
  /**
   * Negative values for port_read and port_write implies that the message
   * manager is not connected
   */
  UDPMessageManager(int port_read, int port_write);
  ~UDPMessageManager();

  /**
   * Send a message through UDP broadcaster
   */
  void sendMessage(const GlobalMsg & message);

  /**
   * Push a message to internal memory. If vive_timestamp of the msg is missing
   * or is already in existing collection, the message is ignored and an error
   * message is printed.
   */
  void pushMessage(const GlobalMsg & msg);  

  void saveMessages(const std::string & path) const;
  void loadMessages(const std::string & path);

  const std::map<uint64_t, GlobalMsg> & getMessages() const;

  /**
   * Return the last entry before time_stamp.
   * If there are no entry before time_stamp, returns an empty entry.
   */
  GlobalMsg getMessage(uint64_t time_stamp);

  /**
   * Return time_stamp of the first entry. If empty, returns 0.
   */
  uint64_t getStart() const;

private:
  int port_read;
  int port_write;

  bool continue_to_run;
  std::unique_ptr<std::thread> thread;
  std::mutex mutex;

  /**
   * Messages are order by vive_timestamp
   */
  std::map<uint64_t, GlobalMsg> messages;

  std::unique_ptr<rhoban_utils::UDPBroadcast> broadcaster;

  void run();
};

}
