#pragma once

#include <functional>
#include <memory>

#include <injection_messages/OneNumber.h>

class Incrementer {
 public:
  struct MiddlewareHandle {
    using Callback =
        std::function<bool(const injection_messages::OneNumber::Request&,
                           injection_messages::OneNumber::Response&)>;
    virtual void registerCallback(Callback cb) = 0;
    virtual ~MiddlewareHandle() = default;
  };

  Incrementer(std::unique_ptr<MiddlewareHandle> mw);

 private:
  std::unique_ptr<MiddlewareHandle> mw_;
};
