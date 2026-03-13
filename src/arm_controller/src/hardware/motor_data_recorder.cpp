#include "arm_controller/hardware/motor_data_recorder.hpp"

#include <chrono>
#include <iostream>

MotorDataRecorder::MotorDataRecorder(
    const std::string& output_file)
: output_file_(output_file)
{
}

MotorDataRecorder::~MotorDataRecorder()
{
    stop();
}

void MotorDataRecorder::start()
{
    file_.open(output_file_);

    file_ << "timestamp,interface";

    for(size_t i=0;i<DOF;i++)
        file_ << ",pos" << i;

    for(size_t i=0;i<DOF;i++)
        file_ << ",vel" << i;

    for(size_t i=0;i<DOF;i++)
        file_ << ",eff" << i;

    file_ << "\n";

    running_ = true;

    start_time_ = std::chrono::steady_clock::now();

    writer_thread_ =
        std::thread(&MotorDataRecorder::writer_loop,this);
}

void MotorDataRecorder::stop()
{
    running_ = false;

    if(writer_thread_.joinable())
        writer_thread_.join();

    if(file_.is_open())
        file_.close();
}

void MotorDataRecorder::register_interface(
    const std::string& interface,
    const std::vector<uint32_t>& motor_ids)
{
    std::lock_guard<std::mutex> lock(config_mutex_);

    if(interfaces_.count(interface))
        return;

    auto [it, inserted] = interfaces_.try_emplace(interface);
    if (!inserted) {
        return;
    }
    auto& buf = it->second;

    buf.motor_ids = motor_ids;

    for(size_t i=0;i<motor_ids.size();i++)
    {
        buf.motor_index[motor_ids[i]] = i;
        buf.updated[i] = false;
    }

    buf.ring.resize(buf.capacity);
}

void MotorDataRecorder::on_motor_status_update(
    const std::string& interface,
    uint32_t motor_id,
    const MotorStatus& status)
{
    std::lock_guard<std::mutex> cfg_lock(config_mutex_);
    auto it = interfaces_.find(interface);

    if(it == interfaces_.end())
        return;

    auto& buf = it->second;
    std::lock_guard<std::mutex> buf_lock(buf.mutex);

    auto idx_it = buf.motor_index.find(motor_id);

    if(idx_it == buf.motor_index.end())
        return;

    size_t idx = idx_it->second;

    buf.latest_state[idx] = status;

    buf.updated[idx] = true;

    bool all = true;

    for(size_t i=0;i<buf.motor_ids.size();i++)
    {
        if(!buf.updated[i])
        {
            all = false;
            break;
        }
    }

    if(!all)
        return;

    auto now = std::chrono::steady_clock::now();

    double ts = std::chrono::duration<double>(now - start_time_).count();

    auto& slot = buf.ring[buf.head];

    slot.timestamp = ts;

    for(size_t i=0;i<buf.motor_ids.size();i++)
    {
        slot.pos[i] = buf.latest_state[i].position;
        slot.vel[i] = buf.latest_state[i].velocity;
        slot.eff[i] = buf.latest_state[i].effort;

        buf.updated[i] = false;
    }

    buf.head = (buf.head + 1) % buf.capacity;
}

void MotorDataRecorder::writer_loop()
{
    // ✅ 缓冲写入，每 200 samples flush 一次
    std::ostringstream buffer;
    int sample_count = 0;
    const int FLUSH_THRESHOLD = 200;

    while(running_)
    {
        std::vector<std::pair<std::string, std::vector<Snapshot>>> drained_per_interface;
        {
            std::lock_guard<std::mutex> cfg_lock(config_mutex_);
            drained_per_interface.reserve(interfaces_.size());

            for(auto& kv : interfaces_)
            {
                const std::string& interface = kv.first;
                auto& buf = kv.second;

                std::vector<Snapshot> drained;
                {
                    std::lock_guard<std::mutex> buf_lock(buf.mutex);
                    while(buf.tail != buf.head)
                    {
                        drained.push_back(buf.ring[buf.tail]);
                        buf.tail = (buf.tail + 1) % buf.capacity;
                    }
                }

                if (!drained.empty())
                {
                    drained_per_interface.emplace_back(interface, std::move(drained));
                }
            }
        }

        for (auto& interface_data : drained_per_interface)
        {
            const std::string& interface = interface_data.first;
            auto& drained = interface_data.second;

            for (const auto& s : drained)
            {
                buffer << s.timestamp << "," << interface;

                for(size_t i=0;i<DOF;i++)
                    buffer << "," << s.pos[i];

                for(size_t i=0;i<DOF;i++)
                    buffer << "," << s.vel[i];

                for(size_t i=0;i<DOF;i++)
                    buffer << "," << s.eff[i];

                buffer << "\n";

                sample_count++;

                // ✅ 每 200 samples flush 一次
                if (sample_count >= FLUSH_THRESHOLD)
                {
                    file_ << buffer.str();
                    file_.flush();
                    buffer.str("");
                    buffer.clear();
                    sample_count = 0;
                }

            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }

    // ✅ 停止前 flush 剩余数据
    if (sample_count > 0)
    {
        file_ << buffer.str();
        file_.flush();
    }
}
