#ifndef MUAN_QUEUES_MESSAGE_QUEUE_HPP_
#define MUAN_QUEUES_MESSAGE_QUEUE_HPP_

#include "message_queue.h"

namespace muan {

namespace queues {

template <typename T, uint32_t size>
MessageQueue<T, size>::MessageQueue(MessageQueue<T, size>&& move_from) noexcept
    : messages_(move_from.messages_),
      back_(move_from.back_) {}

template <typename T, uint32_t size>
void MessageQueue<T, size>::WriteMessage(const T& message) {
  aos::MutexLocker locker_{&queue_lock_};
  // Push messages into the back
  messages_[back_ % size] = message;

  back_++;
}

template <typename T, uint32_t size>
std::experimental::optional<T> MessageQueue<T, size>::NextMessage(
    uint32_t& next) const {
  aos::MutexLocker locker_{&queue_lock_};

  // Make sure the reader's index is within the bounds of still-valid messages,
  // and if it is at the end of the queue return nullopt.
  if (next >= back_) {
    next = back_;
    return std::experimental::nullopt;
  }

  if (next < front()) {
    next = front();
  }

  auto current = next++;
  return messages_[current % size];
}

template <typename T, uint32_t size>
typename MessageQueue<T, size>::QueueReader MessageQueue<T, size>::MakeReader()
    const {
  return MessageQueue<T, size>::QueueReader{*this};
}

template <typename T, uint32_t size>
uint32_t MessageQueue<T, size>::front() const {
  return front(back_);
}

template <typename T, uint32_t size>
uint32_t MessageQueue<T, size>::front(uint32_t back) const {
  return std::max<uint32_t>(back, size) - size;
}

template <typename T, uint32_t size>
MessageQueue<T, size>::QueueReader::QueueReader(
    MessageQueue<T, size>::QueueReader&& move_from) noexcept
    : queue_{move_from.queue_} {
  next_message_ = std::move(move_from.next_message_);
}

template <typename T, uint32_t size>
MessageQueue<T, size>::QueueReader::QueueReader(
    const MessageQueue<T, size>& queue)
    : queue_(queue) {
  next_message_ = queue_.front();
}

template <typename T, uint32_t size>
std::experimental::optional<T>
MessageQueue<T, size>::QueueReader::ReadMessage() {
  return queue_.NextMessage(next_message_);
}

}  // namespace queues

}  // namespace muan

#endif /* MUAN_QUEUES_MESSAGE_QUEUE_HPP_ */