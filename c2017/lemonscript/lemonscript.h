#ifndef C2017_LEMONSCRIPT_LEMONSCRIPT_H_
#define C2017_LEMONSCRIPT_LEMONSCRIPT_H_

#include <atomic>
#include <string>
#include <vector>
#include "c2017/lemonscript/ls_gen.h"
#include "c2017/queue_manager/queue_manager.h"
#include "gflags/gflags.h"
#include "muan/webdash/queue_types.h"
#include "muan/webdash/webdash.pb.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/lemonscript/lemonscript/AvailableCppCommandDeclaration.h"
#include "third_party/lemonscript/lemonscript/LemonScriptCompiler.h"
#include "third_party/lemonscript/lemonscript/lemonscript.h"

namespace c2017 {
namespace lemonscript {

DECLARE_string(auto_mode);

class Lemonscript {
 public:
  Lemonscript();
  ~Lemonscript();

  std::vector<std::string> auto_list;

  void operator()();

  void Start();  // Start running lemonscript
  void Stop();   // Pause running lemonscript
  void Kill();   // Stop the thread

 private:
  void UpdateAutoRoutine();
  muan::webdash::AutoSelectionQueue::QueueReader auto_selection_reader_ =
      muan::webdash::WebDashQueueWrapper::GetInstance()
          .auto_selection_queue()
          .MakeReader();
  ::lemonscript::LemonScriptState *state_;
  ::lemonscript::LemonScriptCompiler *compiler_;
  std::vector<const ::lemonscript::AvailableCppCommandDeclaration *> decls_;
  std::atomic<bool> running_;
  std::atomic<bool> started_;
};

}  // namespace lemonscript
}  // namespace c2017

#endif  // C2017_LEMONSCRIPT_LEMONSCRIPT_H_
