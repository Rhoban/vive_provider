#include <vive_provider/udp_message_manager.h>
#include <rhoban_utils/util.h>

#include <chrono>
#include <fstream>
#include <iostream>

#define PACKET_MAX_SIZE 10000

using namespace std::chrono;

namespace vive_provider
{
int getDefaultPort()
{
  return 37020;
}

UDPMessageManager::UDPMessageManager(int port_read, int port_write) : port_read(port_read), port_write(port_write)
{
  continue_to_run = true;
  broadcaster.reset(new rhoban_utils::UDPBroadcast(port_read, port_write));
  if (port_read >= 0)
  {
    broadcaster->openRead();
    thread.reset(new std::thread([this]() { this->run(); }));
  }
  if (port_write >= 0)
  {
    broadcaster->openWrite();
  }
}

UDPMessageManager::~UDPMessageManager()
{
  continue_to_run = false;
  if (port_read >= 0)
  {
    thread->join();
  }
  if (port_write >= 0)
  {
    broadcaster->closeWrite();
  }
}

const std::map<uint64_t, GlobalMsg>& UDPMessageManager::getMessages() const
{
  return messages;
}

void UDPMessageManager::run()
{
  // Receiving informations
  char data[PACKET_MAX_SIZE];
  size_t len;
  GlobalMsg msg;
  while (continue_to_run)
  {
    len = PACKET_MAX_SIZE;
    if (!broadcaster->checkMessage((unsigned char*)data, len))
      continue;
    if (len >= PACKET_MAX_SIZE)
    {
      std::cerr << DEBUG_INFO << " packet is too long: " << len << std::endl;
      continue;
    }
    msg.Clear();
    std::string string_data(data, len);
    if (!msg.ParseFromString(string_data))
    {
      std::cerr << DEBUG_INFO << "Invalid format for a packet of size: " << len << std::endl;
      continue;
    }
    pushMessage(msg);
  }
}

void UDPMessageManager::sendMessage(const GlobalMsg& message)
{
  std::string raw_message;
  if (!message.SerializeToString(&raw_message))
  {
    std::cerr << "Invalid Message !" << std::endl;
    return;
  }
  broadcaster->broadcastMessage((const unsigned char*)raw_message.c_str(), raw_message.size());
}

void UDPMessageManager::pushMessage(const GlobalMsg& msg)
{
  if (!msg.has_vive_timestamp())
  {
    std::cerr << DEBUG_INFO << "No vive_timestamp provided" << std::endl;
    return;
  }

  uint64_t ts = msg.vive_timestamp();
  std::lock_guard<std::mutex> lock(mutex);
  if (messages.count(ts) != 0)
  {
    std::cerr << DEBUG_INFO << "Duplicated entry for ts: " << ts << std::endl;
    return;
  }
  messages[ts] = msg;
}

void UDPMessageManager::saveMessages(const std::string& path) const
{
  GlobalCollection collection;
  for (const auto& entry : messages)
  {
    *(collection.add_messages()) = entry.second;
  }
  std::ofstream out(path, std::ios::binary);
  if (!out.good())
  {
    throw std::runtime_error(DEBUG_INFO + " failed to open file '" + path + "'");
  }
  collection.SerializeToOstream(&out);
}

void UDPMessageManager::loadMessages(const std::string& path)
{
  GlobalCollection collection;
  collection.Clear();
  std::ifstream in(path);
  if (!in.good())
  {
    throw std::runtime_error(DEBUG_INFO + " failed to open file '" + path + "'");
  }
  collection.ParseFromIstream(&in);
  for (const auto& msg : collection.messages())
  {
    pushMessage(msg);
  }
}

GlobalMsg UDPMessageManager::getMessage(uint64_t time_stamp, bool system_clock) const
{
  if (system_clock)
  {
    time_stamp -= clock_offset;
  }
  // If empty or no data prior to time_stamp, return empty message
  if (messages.size() == 0 || messages.begin()->first > time_stamp)
  {
    return GlobalMsg();
  }
  auto it = messages.upper_bound(time_stamp);
  if (it == messages.end())
  {
    it--;
  }
  return it->second;
}

uint64_t UDPMessageManager::getStart(bool system_clock) const
{
  if (messages.size() == 0)
  {
    return 0;
  }
  uint64_t start = messages.begin()->first;
  if (system_clock)
  {
    start += clock_offset;
  }
  return start;
}

uint64_t UDPMessageManager::getLast(bool system_clock) const
{
  if (messages.size() == 0)
  {
    return 0;
  }
  uint64_t end = messages.end()->first;
  if (system_clock)
  {
    end += clock_offset;
  }
  return end;
}

void UDPMessageManager::autoUpdateOffset()
{
  if (messages.size() > 0)
  {
    const GlobalMsg& msg = messages.begin()->second;
    if (!msg.has_time_since_epoch())
    {
      std::cerr << "Message has no 'time_since_epoch'" << std::endl;
      return;
    }
    if (!msg.has_vive_timestamp())
    {
      std::cerr << "Message has no 'vive_timestamp'" << std::endl;
      return;
    }
    clock_offset = msg.time_since_epoch() - msg.vive_timestamp();
  }
}

void UDPMessageManager::setOffset(int64_t new_offset)
{
  clock_offset = new_offset;
}

int64_t UDPMessageManager::getOffset() const
{
  return clock_offset;
}

}  // namespace vive_provider
