while getopts s: flag
do
    case "${flag}" in
        s) session_name=${OPTARG};;
    esac
done

splitsize=10000000000
destination="~/war-projects/Data/raw/bagfiles"
session_name="$(date '+%Y-%m-%d-T%H-%M-%S')-$session_name"

$recording_path="$destination/$session_name"

if [ ! -d "$recording_path" ]; then
  mkdir $recording_path
fi
echo "Recording to $recording_path"

cd ~/cameraimu_ws
source install/setup.bash
ros2 run camera_imu CameraIMU 

$recording_file="$recording_path/recording"

ros2 bag record -o $recording_file --a -b $splitsize