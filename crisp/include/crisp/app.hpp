#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include <crisp/data.hpp>

namespace crisp {
using ctrl_t = std::function<void(
  model_t const& m, data_t const& d, Ref<VectorXr> u, Ref<VectorXr> udot)>;
using hook_t = std::function<void(model_t const& m, data_t const& d)>;

class AppManager {
protected:
  std::mutex mtx_;
  std::unique_ptr<model_t> m_;
  std::unique_ptr<data_t> d_;

public:
  AppManager() = default;
  virtual ~AppManager() = default;

  auto& mutex() const { return mtx_; }
  auto& mutex() { return mtx_; }

  auto& model() const { return *m_; }
  auto& model() { return *m_; }

  auto& data() const { return *d_; }
  auto& data() { return *d_; }

  virtual void setControl(ctrl_t&& ctrl) = 0;
  virtual void addPostStepHook(hook_t&& hook) = 0;

  void init(char const* title = "CRISP🍟", int width = 1200, int height = 900) {
    initConfig();
    initEngine();
    openWindow(title, width, height);
  }

  void shutdown() {
    closeWindow();
    stopEngine();
  }

  virtual void initConfig() = 0;
  virtual void initEngine() = 0;
  virtual void stopEngine() = 0;

  virtual void openWindow(char const* title, int width, int height) = 0;
  virtual void closeWindow() = 0;

  virtual bool isOpen() const = 0;
  virtual void render() = 0;
};

std::unique_ptr<AppManager> make_app(
  std::unique_ptr<model_t> m, bool run = true);
std::unique_ptr<AppManager> make_app(  //
  path_t const& file_name, bool run = true);
}  // namespace crisp
