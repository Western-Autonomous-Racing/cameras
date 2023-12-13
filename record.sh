while getopts s:i: flag
do
    case "${flag}" in
        s) session_name=${OPTARG};;
        i) imu_only=${OPTARG};;
    esac
done

# if session is empty provide default name
if [ -z "$session_name" ]
then
  session_name="recording"
fi
if [ -z "$imu_only" ]
then
  imu_only=false
else
  imu_only=true
fi

splitsize=10000000000
destination="/home/$(whoami)/war-projects/Data/raw/bagfiles"
session_name="$(date '+%Y-%m-%d-T%H-%M-%S')-$session_name"

recording_path="$destination/$session_name"

if [ ! -d "$recording_path" ]; then
  mkdir -p "$recording_path"
fi
echo "Recording to $recording_path"

cd ~/cameraimu_ws
source install/setup.bash

if [ "$imu_only" = false ] ; then
  ros2 run camera-imu CameraIMUNode &
else
  ros2 run camera-imu CameraIMUNode --ros-args -p enable_imu:=true -p enable_camera:=false &
fi

recording_file="$recording_path/recording"

ros2 topic list
ros2 bag record -o "$recording_file" -a -b $splitsize &
record_pid=$!

# Add a trap to stop recording when the script is terminated
trap 'kill $record_pid' SIGINT SIGTERM

# Wait for the script to be terminated
if [ "$imu_only" = false ] ; then
  wait $record_pid
else
  sleep 7h
  kill $record_pid
fi