#ifndef CLEAN_SLAM_SRC_OBSERVER_H
#define CLEAN_SLAM_SRC_OBSERVER_H

#include <vector>
namespace clean_slam {

template <typename T> class Observer;

template <typename T> class Observable {
public:
  void AddObserver(Observer<T> *observer) { _observers.push_back(observer); }
  void OnDelete() {
    for (auto &observer : _observers) {
      observer->OnDelete(this);
    }
  }
  size_t NumOfObservers() const { return _observers.size(); }

  ~Observable() { OnDelete(); }

private:
  std::vector<Observer<T> *> _observers;
};

template <typename Derived> class Observer {
public:
  void OnDelete(Observable<Derived> *observable) {
    static_cast<Derived *>(this)->OnDelete(observable);
  }
};

} // namespace clean_slam
#endif /* OBSERVER_H */
