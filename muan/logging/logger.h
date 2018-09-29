#ifndef MUAN_LOGGING_LOGGER_H_
#define MUAN_LOGGING_LOGGER_H_

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"
#include "muan/logging/filewriter.h"
#include "muan/logging/textlogger.h"
#include "muan/queues/message_queue.h"
#include "muan/units/units.h"
#include "muan/utils/proto_utils.h"
#include "muan/utils/threading_utils.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/optional/optional.hpp"

namespace muan {
namespace logging {

/*
 * A logging class that takes QueueReader objects and logs them to disk as CSV
 * files. It also supports logging text.
 *
 * Simple use:
 *
 * Logger logger;
 *
 * MessageQueue<protobuf_class> mq(100);
 * logger.AddQueue("some_queue", mq.MakeReader());
 *
 * auto textlog = logger.GetTextLogger("log_name");
 * textlog("Everything is on fire! Send help.");
 *
 * You only want to have one logger object running at a time. You should add
 * QueueReaders to it as needed.
 *
 * A note about realtime behavior - The Logger class is running in a thread
 * that is not expected to be realtime. Logging messages is realtime, but the
 * following operations are not realtime:
 *  - Adding a Queue to be logged
 *  - Creating a textlog
 * However, these operations should only happen in the beginning of the robot
 * code, when the subsystems are being initialized, this should not be a
 * problem.
 *
 * Right now, textlog uses std::string, which is non-realtime if it needs to be
 * expanded, so it is up to callers to ensure that the construction of the
 * logging string is realtime.
 */
class Logger {
  FRIEND_TEST(Logger, TextLogger);

 public:
  Logger();
  explicit Logger(std::unique_ptr<FileWriter>&& writer);
  virtual ~Logger() = default;

  // Adds a QueueReader<protobuf_class> to the list of queues to be logged,
  // under the name "name". The name will determine the file that it logs to,
  // as well as serving as a human-readable name in other places.
  template <class T>
  void AddQueue(const std::string& name, T* queue);

  // This is designed to be used with std::thread to run the logger. It will
  // run forever, calling the Update function.
  void operator()();

  // This starts the logger if you have previously stopped it by calling Stop().
  void Start();

  // This stops the logger from running. It can be resumed calling Start().
  void Stop();

  // Log with format strings, such as
  // LOG(level, "x=%d y=%f", x, y);
  template <typename... Ts>
  static void LogText(int level, const char* filename, int line, Ts... args);
#define LOG(level, fmt, ...) \
  muan::logging::Logger::LogText(level, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

  // Very much not realtime. Public only because a bunch of tests use it.
  void Update();

 private:
  std::unique_ptr<FileWriter> writer_;
  std::atomic<bool> running_{false};

  class GenericReader {
   public:
    virtual ~GenericReader() = default;
    virtual std::experimental::optional<std::string> GetMessageAsCSV(
        bool header) = 0;
  };

  template <class R>
  class Reader : public GenericReader {
   public:
    explicit Reader(R reader);
    std::experimental::optional<std::string> GetMessageAsCSV(
        bool header) override;

   private:
    R reader_;
  };

  struct QueueLog {
    std::unique_ptr<GenericReader> reader;
    std::string name;  // Human friendly name for this log
    std::string filename;
    bool write_header;  // Do we still need to write the header?
  };

  std::vector<std::unique_ptr<QueueLog>> queue_logs_;
  TextLogger::LogQueue::QueueReader textlog_reader_;
  static TextLogger text_logger;
};  // class Logger
}  // namespace logging
}  // namespace muan

#include "logger.hpp"

#endif  // MUAN_LOGGING_LOGGER_H_
