#include "bag_source_ui.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>

namespace
{
    std::string channel_label(const BagChannel& c)
    {
        std::string label = c.topic + "  (" + c.type;
        if (c.message_count > 0)
            label += ", " + std::to_string(c.message_count) + " msgs";
        return label + ")";
    }

    std::vector<BagChannel>::const_iterator find_channel(const std::vector<BagChannel>& channels, const std::string& topic)
    {
        return std::find_if(
            channels.begin(),
            channels.end(),
            [&topic](const BagChannel& c)
            {
                return c.topic == topic;
            });
    }

    //! Combo over the channels of one kind; writes the chosen topic.
    void topic_combo(const char* label, const std::vector<BagChannel>& channels, BagChannel::Kind kind, std::string& topic)
    {
        const auto current = find_channel(channels, topic);
        const std::string preview = current != channels.end() ? channel_label(*current) : "<choose a topic>";
        if (ImGui::BeginCombo(label, preview.c_str()))
        {
            for (const auto& c : channels)
            {
                if (c.kind != kind)
                    continue;
                const bool selected = c.topic == topic;
                if (ImGui::Selectable(channel_label(c).c_str(), selected))
                    topic = c.topic;
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    //! The topic of the only channel of this kind, or an empty string.
    std::string only_topic(const std::vector<BagChannel>& channels, BagChannel::Kind kind)
    {
        std::string topic;
        for (const auto& c : channels)
        {
            if (c.kind != kind)
                continue;
            if (!topic.empty())
                return {};
            topic = c.topic;
        }
        return topic;
    }
} // namespace

BagSourceConfigUi::BagSourceConfigUi(const std::vector<std::filesystem::path>& files)
{
    config_.files = files;
    if (files.empty())
    {
        list_error_ = "no .bag or .mcap file";
        return;
    }
    channels_ = list_bag_channels(files.front(), list_error_);
    compression_hint_ = bag_compression_hint(files);
    config_.lidar_topic = only_topic(channels_, BagChannel::Kind::Lidar);
    config_.imu_topic = only_topic(channels_, BagChannel::Kind::Imu);
}

bool BagSourceConfigUi::draw()
{
    if (config_.files.size() == 1)
        ImGui::TextUnformatted(config_.files.front().filename().string().c_str());
    else if (!config_.files.empty())
    {
        ImGui::Text("%zu files, topics from %s", config_.files.size(), config_.files.front().filename().string().c_str());
        if (ImGui::IsItemHovered())
        {
            std::string list;
            for (const auto& file : config_.files)
                list += file.string() + "\n";
            ImGui::SetTooltip("%s", list.c_str());
        }
    }
    if (!list_error_.empty())
    {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", list_error_.c_str());
        return false;
    }

    if (!compression_hint_.empty())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 40.0f);
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", compression_hint_.c_str());
        ImGui::PopTextWrapPos();
    }

    topic_combo("LiDAR topic", channels_, BagChannel::Kind::Lidar, config_.lidar_topic);
    topic_combo("IMU topic", channels_, BagChannel::Kind::Imu, config_.imu_topic);

    // the layout presets only apply to PointCloud2; a Livox CustomMsg has a fixed layout
    const auto lidar = find_channel(channels_, config_.lidar_topic);
    if (lidar != channels_.end() && lidar->type.ends_with("PointCloud2"))
    {
        const char* layouts[] = { "auto (from the message fields)", "Hesai", "Ouster" };
        int layout = static_cast<int>(config_.lidar_layout);
        if (ImGui::Combo("PointCloud2 layout", &layout, layouts, IM_ARRAYSIZE(layouts)))
            config_.lidar_layout = static_cast<BagSourceConfig::LidarLayout>(layout);
    }

    int unit = config_.imu_acc_in_g ? 0 : 1;
    ImGui::TextUnformatted("IMU acceleration unit:");
    ImGui::SameLine();
    ImGui::RadioButton("g", &unit, 0);
    ImGui::SameLine();
    ImGui::RadioButton("m/s^2", &unit, 1);
    config_.imu_acc_in_g = unit == 0;

    ImGui::InputScalarN(
        "IMU -> lidar rotation (qx qy qz qw)", ImGuiDataType_Double, config_.imu_to_lidar.data(), 4, nullptr, nullptr, "%.6f");
    const auto& q = config_.imu_to_lidar;
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    const bool unit_quaternion = std::abs(norm - 1.0) < 1e-3;
    if (!unit_quaternion)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "the rotation is not a unit quaternion (norm %.6f)", norm);

    ImGui::InputDouble("Chunk length [s]", &config_.chunk_seconds, 1.0, 5.0, "%.1f");
    if (config_.chunk_seconds < 0.0)
        config_.chunk_seconds = 0.0;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Length of one chunk of message log time; 0 reads the whole file as one chunk");

    const bool ready = !config_.lidar_topic.empty() && !config_.imu_topic.empty() && unit_quaternion;
    ImGui::BeginDisabled(!ready);
    const bool open = ImGui::Button("Open");
    ImGui::EndDisabled();
    return open && ready;
}

std::string BagSourceConfigUi::title() const
{
    return "Open ROS bag / MCAP";
}

std::shared_ptr<RecordingSource> BagSourceConfigUi::open(std::string& error) const
{
    return open_bag_source(config_, error);
}
