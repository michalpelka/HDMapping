#pragma once

#include "recording_source_factory.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

/**
 * @brief Loads the IMU of a Mandeye recording: the imu****.csv files, or, when they hold no samples (units with an
 * external IMU such as a Murata log into a subfolder instead), the csv files of the folder's MURATA_IMU.
 * @param csv_files The recording's imu****.csv files.
 * @param folder The recording folder, which holds MURATA_IMU.
 * @param imu_number The imuId to read from files that carry one.
 * @param imu_data Receives every sample, unsorted.
 * @param imuSpans Receives one span per file read.
 */
void load_mandeye_imu(
    const std::vector<std::string>& csv_files,
    const std::filesystem::path& folder,
    int imu_number,
    Imu& imu_data,
    std::vector<InputFileSpan>& imuSpans);

//! A Mandeye recording folder: lidarNNNN.laz chunks paired with imuNNNN.csv files by their last four stem characters,
//! plus the optional calibration (.mjc or calibration .json), .sn file and MURATA_IMU folder.
class LazCsvSource : public RecordingSource
{
public:
    //! Sorts input_file_names, pairs the files and loads and checks the calibration; exits the process on a wrong
    //! calibration file, as load_data always did. Returns null when there are not enough files or the folder is missing.
    static std::unique_ptr<LazCsvSource> open(std::vector<std::string>& input_file_names, bool debugMsg);

    //! "Mandeye LAZ/CSV".
    static std::string name();
    //! True when a path is a folder or a file of a Mandeye recording (.laz .las .csv .json .mjc .sn); open() ignores
    //! the other files.
    static bool accepts(const std::vector<std::filesystem::path>& paths);
    //! Opens folders with their top-level files, a single file with the files of its folder, several files as given.
    static std::shared_ptr<RecordingSource> open(const std::vector<std::filesystem::path>& paths, std::string& error);
    //! File dialog filter of the files of a Mandeye recording.
    static std::vector<std::string> fileFilter();

    std::filesystem::path workingDirectory() const override;
    size_t chunkCount() const override;
    std::string chunkName(size_t i) const override;
    std::vector<Point3Di> loadPoints(size_t i, const PointFilter& filter) const override;
    ChunkStamp stamp(size_t i) const override;
    void loadImu(Imu& imu_data, std::vector<InputFileSpan>& imuSpans, const std::function<void(double)>& progress) const override;

private:
    LazCsvSource() = default;

    std::filesystem::path working_directory_;
    std::vector<std::string> laz_files_; //!< the first chunkCount() LAZ files, each paired with csv_files_[i]
    std::vector<std::string> csv_files_;
    std::string sn_file_;
    int imu_number_to_use_ = 0;
    std::unordered_map<int, Eigen::Affine3d> calibration_;
};
