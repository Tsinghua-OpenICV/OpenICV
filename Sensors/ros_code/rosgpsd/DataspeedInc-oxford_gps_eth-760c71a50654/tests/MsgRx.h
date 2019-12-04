#ifndef _MSG_RX_H_
#define _MSG_RX_H_

#include <ros/ros.h>

template <typename MsgT>
class MsgRx {
public:
  MsgRx() : valid_(false) {};
  MsgRx(const MsgT& msg) : valid_(false) { set(msg); };
  const MsgT& get() const { return msg_; }
  void set(const MsgT& msg) { msg_ = msg; valid_ = true; }
  void clear() { valid_ = false; }
  bool valid() const { return valid_; }
private:
  MsgT msg_;
  bool valid_;
};

#endif // _MSG_RX_H_
