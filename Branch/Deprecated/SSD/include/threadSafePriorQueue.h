#ifndef THREADSAFEPRIORQUEUE_H_
#define THREADSAFEPRIORQUEUE_H_

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <opencv2/opencv.hpp>

template <typename T, typename OP>
class ThreadSafePriorQueue {
 public:
  ThreadSafePriorQueue(const int max_size) { max_size_ = max_size; }
  ThreadSafePriorQueue() {max_size_=0; }
  ThreadSafePriorQueue& operator=(const ThreadSafePriorQueue& other) = delete;
  ThreadSafePriorQueue(const ThreadSafePriorQueue& other) = delete;

  ~ThreadSafePriorQueue() { BreakAllWait(); }

  void Enqueue(const T& element) {
	  while (true && max_size_ != 0) 
	  { 
		  if (queue_.size() >= max_size_) 
		  {
			  std::this_thread::sleep_for(std::chrono::microseconds(10000));
		  } 
		  else 
		  {
		  	break;
		  }
	  }
    std::lock_guard<std::mutex> lock(mutex_);
	  queue_.emplace(element);
    cv_.notify_one();
  }

  bool Dequeue(T* element) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    *element = std::move(queue_.front());
    queue_.pop();
    return true;
  }
  
  const T& Top()
  {
	  return queue_.top();
  }

  bool WaitDequeue(T* element) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this]() { return break_all_wait_ || !queue_.empty(); });
    if (break_all_wait_) 
    {
      return false;
    }
    *element = std::move(queue_.top());
    // std::cout<<"queue size : "<<queue_.size()<<std::endl;
    queue_.pop();
    return true;
  }
 
  bool WaitDequeue2(T* element, int dstIndex) 
  { 
	  dstIndex_=dstIndex;
	  std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { 
		return break_all_wait_ || (!queue_.empty() && queue_.top().first==dstIndex_);
	  });
      if (break_all_wait_) 
	  {
    	  return false;
      }
	  *element = std::move(queue_.top());
	  queue_.pop();
	  return true;
}

  typename std::queue<T>::size_type Size() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  bool Empty() {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void BreakAllWait() {
    break_all_wait_ = true;
    cv_.notify_all();
  }

 private:
  volatile bool break_all_wait_ = false;
  std::mutex mutex_;
  std::priority_queue<T, vector<T>, OP> queue_;
  std::condition_variable cv_;
  volatile int dstIndex_;
  volatile int max_size_;
};

#endif  // THREAD_SAFE_QUEUE_H_
