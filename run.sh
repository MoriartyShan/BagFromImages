#!/usr/bin/env bash
BIN="$1"
INPUT_PATH="$2"
OUTPUT_PATH="$3"
videos=(
#"Recfront_20100101_081006.mp4"
#"Recfront_20200608_154417.mp4"
#"Recfront_20200608_154641.mp4"
#"Recfront_20200608_154801.mp4"
#"Recfront_20200608_154913.mp4"
)
imus=(
#"logfile_2010_01_01_08_10_13.csv"
#"logfile_2020_06_08_15_44_25.csv"
#"logfile_2020_06_08_15_46_46.csv"
#"logfile_2020_06_08_15_48_05.csv"
#"logfile_2020_06_08_15_49_17.csv"
)
length=${#videos[@]}
echo "totoally has ${length} videos"
for((i=0;i<length;i++)); do
    echo "Processing ${i} ${videos[i]} ${imus[i]} bin=${BIN}"
    ${BIN} --video_topic=/camera/image_raw --imu_topic=/imu0 --data_path="${OUTPUT_PATH}" --video_path="${INPUT_PATH}/camera/${videos[i]}" --imu_path="${INPUT_PATH}/imu/${imus[i]}" --filename="${i}.bag"
done