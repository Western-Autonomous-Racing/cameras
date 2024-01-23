while getopts s: flag
do
    case "${flag}" in
        s) session_name=${OPTARG};;
    esac
done

# if session is empty provide default name
if [ -z "$session_name" ]
then
  session_name="recording"
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
ros2 run camera-imu CameraIMUNode &

recording_file="$recording_path/recording"

ros2 bag record -o "$recording_file" -a -b $splitsize &
record_pid=$!

# Add a trap to stop recording when the script is terminated
trap 'kill $record_pid' SIGINT SIGTERM

# Wait for the script to be terminated
wait $record_pid
