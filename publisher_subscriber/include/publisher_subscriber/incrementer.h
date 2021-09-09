#pragma once

#include <std_msgs/Int64.h>

#include <functional>
#include <memory>

class Incrementer {
 public:
  struct MiddlewareHandle {
    using Callback = std::function<void(const std_msgs::Int64::ConstPtr&)>;
    virtual void registerCallback(Callback cb) = 0;
    virtual void publish(std_msgs::Int64 msg) = 0;
    virtual ~MiddlewareHandle() = default;
  };

  Incrementer(std::unique_ptr<MiddlewareHandle> mw);

 private:
  std::unique_ptr<MiddlewareHandle> mw_;
};
