#include <rclcpp/rclcpp.hpp>
#include <audioalert_msg/msg/audio_alert.hpp>
#include <audioalert_msg/msg/audio_advert.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <sndfile.hh>
#include <alsa/asoundlib.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <condition_variable>
#include <atomic>

using AudioAlert  = audioalert_msg::msg::AudioAlert;
using AudioAdvert = audioalert_msg::msg::AudioAdvert;

static constexpr snd_pcm_uframes_t CHUNK_FRAMES     = 2048;
static constexpr float            DUCK_LEVEL       = 0.1f;
static constexpr float            DUCK_RAMP_FACTOR = 0.2f;
static constexpr float            RESTORE_FACTOR   = 0.05f;

struct Buffer {
  std::vector<float> samples;
  int samplerate;
};

class FleetPlayback : public rclcpp::Node {
public:
  FleetPlayback()
  : Node("fleet_msg_playback"),
    shutdown_(false),
    alert_playing_(false),
    cancel_alert_(false),
    cancel_advert_(false)
  {
    alert_sub_  = create_subscription<AudioAlert>(
      "SpeakerAudioAlertRequest", 10,
      std::bind(&FleetPlayback::on_alert, this, std::placeholders::_1));
    advert_sub_ = create_subscription<AudioAdvert>(
      "SpeakerAdvertRequest", 10,
      std::bind(&FleetPlayback::on_advert, this, std::placeholders::_1));
    status_pub_ = create_publisher<AudioAlert>("SpeakerStatus_Playback", 10);

    auto pkg = ament_index_cpp::get_package_share_directory("audio_alert_cpp");
    if (auto env = std::getenv("AUDIO_ALERT_PATH"); env && std::filesystem::exists(env)) {
      data_dir_ = env;
    } else {
      data_dir_ = pkg + "/resources";
    }

    load_map(alert_map_,  data_dir_ + "/alert_mapping.json");
    load_map(advert_map_, data_dir_ + "/advert_mapping.json");

    alert_thread_  = std::thread(&FleetPlayback::alert_loop,  this);
    advert_thread_ = std::thread(&FleetPlayback::advert_loop, this);
  }

  ~FleetPlayback() override {
    {
      std::lock_guard<std::mutex> lk(mu_);
      shutdown_ = true;
      cv_.notify_all();
    }
    alert_thread_.join();
    advert_thread_.join();
  }

private:
  rclcpp::Subscription<AudioAlert>::SharedPtr  alert_sub_;
  rclcpp::Subscription<AudioAdvert>::SharedPtr advert_sub_;
  rclcpp::Publisher<AudioAlert>::SharedPtr     status_pub_;

  std::map<std::string,Buffer> alert_map_, advert_map_;
  std::string                  data_dir_;

  std::mutex               mu_;
  std::condition_variable  cv_;
  std::queue<std::string>  alert_q_, advert_q_;
  std::thread              alert_thread_, advert_thread_;
  bool                     shutdown_;

  std::atomic<bool> alert_playing_;
  std::atomic<bool> cancel_alert_;
  std::atomic<bool> cancel_advert_;

  // immediately preempt any in-flight alert
  void on_alert(const AudioAlert::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!alert_map_.count(msg->alert_name)) {
      RCLCPP_WARN(get_logger(), "Unknown alert '%s'", msg->alert_name.c_str());
      return;
    }
    cancel_alert_ = true;     // stop current alert
    alert_q_ = {};            // clear queue
    alert_q_.push(msg->alert_name);
    cv_.notify_all();
  }

  // immediately preempt any in-flight advert
  void on_advert(const AudioAdvert::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!advert_map_.count(msg->advert_name)) {
      RCLCPP_WARN(get_logger(), "Unknown advert '%s'", msg->advert_name.c_str());
      return;
    }
    cancel_advert_ = true;    // stop current advert
    advert_q_ = {};           // clear queue
    advert_q_.push(msg->advert_name);
    cv_.notify_all();
  }

  void load_map(std::map<std::string,Buffer>& M, const std::string& path) {
    std::ifstream in(path);
    if (!in) { RCLCPP_ERROR(get_logger(), "Cannot open %s", path.c_str()); return; }
    nlohmann::json j; in >> j;
    for (auto &it : j.items()) {
      Buffer buf;
      std::string fn = data_dir_ + "/" + it.value().value("file","");
      SndfileHandle sf{fn};
      buf.samplerate = sf.samplerate();
      std::vector<float> tmp(sf.frames()*sf.channels());
      sf.read(tmp.data(), tmp.size());
      if (sf.channels()==1) {
        buf.samples.resize(tmp.size()*2);
        for (size_t i=0;i<tmp.size();++i) {
          buf.samples[2*i]   = tmp[i];
          buf.samples[2*i+1] = tmp[i];
        }
      } else {
        buf.samples = std::move(tmp);
      }
      M[it.key()] = std::move(buf);
      RCLCPP_INFO(get_logger(), "Loaded '%s'", it.key().c_str());
    }
  }

  // Alert loop: full-volume, immediate preempt
  void alert_loop() {
    while (true) {
      std::string name;
      {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [&]{ return !alert_q_.empty() || shutdown_; });
        if (shutdown_) break;
        name = std::move(alert_q_.front());
        alert_q_.pop();
      }

      cancel_alert_  = false;
      alert_playing_ = true;

      AudioAlert st; st.alert_name = name;
      status_pub_->publish(st);

      play_buffer(alert_map_[name], /*duck=*/false, &cancel_alert_);

      alert_playing_ = false;
      AudioAlert none; none.alert_name = "none";
      status_pub_->publish(none);

      cv_.notify_all();  // wake advert thread to restore
    }
  }

  // Advert loop: duck around alerts, immediate preempt on new advert
  void advert_loop() {
    while (true) {
      std::string name;
      {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [&]{ return !advert_q_.empty() || shutdown_; });
        if (shutdown_) break;
        name = std::move(advert_q_.front());
        advert_q_.pop();
        cancel_advert_ = false;
      }
      play_buffer(advert_map_[name], /*duck=*/true, &cancel_advert_);
    }
  }

  // Core playback (samples→PCM, duck/restore ramp, cancel via pointer)
  void play_buffer(
    const Buffer &B,
    bool duck,
    std::atomic<bool>* cancel_ptr)
  {
    snd_pcm_t *pcm=nullptr;
    if (snd_pcm_open(&pcm,"default",SND_PCM_STREAM_PLAYBACK,0)<0) {
      RCLCPP_ERROR(get_logger(),"Cannot open ALSA device");
      return;
    }
    snd_pcm_set_params(pcm,
      SND_PCM_FORMAT_S16_LE,
      SND_PCM_ACCESS_RW_INTERLEAVED,
      2, B.samplerate,1,500000);

    size_t total = B.samples.size()/2;
    std::vector<int16_t> pcm_buf(B.samples.size());
    for (size_t i=0;i<B.samples.size();++i) {
      float v = std::clamp(B.samples[i],-1.0f,1.0f);
      pcm_buf[i] = int16_t(v*32767);
    }

    float gain=1.0f; size_t offset=0;
    while (offset<total) {
      if (cancel_ptr && *cancel_ptr) {
        snd_pcm_drop(pcm);
        break;
      }

      float target = (!duck||!alert_playing_) ? 1.0f : DUCK_LEVEL;
      float rate   = (target<gain ? DUCK_RAMP_FACTOR : RESTORE_FACTOR);
      gain += (target - gain)*rate;

      auto frames = std::min<size_t>(CHUNK_FRAMES, total-offset);
      std::vector<int16_t> chunk(frames*2);
      for (size_t f=0; f<frames; ++f) {
        float L = pcm_buf[(offset+f)*2+0]*gain/32767.0f;
        float R = pcm_buf[(offset+f)*2+1]*gain/32767.0f;
        chunk[2*f+0] = int16_t(std::clamp(L,-1.0f,1.0f)*32767);
        chunk[2*f+1] = int16_t(std::clamp(R,-1.0f,1.0f)*32767);
      }

      auto w = snd_pcm_writei(pcm,chunk.data(),frames);
      if (w<0) {
        w = snd_pcm_recover(pcm,w,0);
        if (w<0) break;
      }
      offset += frames;
    }

    snd_pcm_drain(pcm);
    snd_pcm_close(pcm);
  }
};

int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<FleetPlayback>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

