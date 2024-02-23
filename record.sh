while getopts s:t: flag
do
  case "${flag}" in
    s) session_name=${OPTARG};;
    t) topics=${OPTARG};;
  esac
done

# if session is empty provide default name
if [ -z "$session_name" ]
then
  session_name="recording"
fi

# of session is empty provide default topics
if [ -z "$topics" ]
then
  topics="/rgb_camera/color/image_raw /stereo_camera/left/image_raw /stereo_camera/right/image_raw /imu"
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
camera_imu_pid=$!
echo "CameraIMUNode PID: $camera_imu_pid"
ros2 run camera-imu imu_node.py &
imu_node_pid=$!
echo "IMU Node PID: $imu_node_pid"

recording_file="$recording_path/recording"
FS=' ' read -r -a topic_array <<< "$topics"

ros2 bag record -o "$recording_file" "${topic_array[@]}" &
bag_record_pid=$!
echo "Bag Record PID: $bag_record_pid"

# Function to stop all processes
stop_processes() {
  kill $camera_imu_pid $imu_node_pid $bag_record_pid
}

# Add a trap to stop all processes when the script is terminated
trap 'stop_processes' SIGINT SIGTERM

# Wait for the script to be terminated
wait $camera_imu_pid $imu_node_pid $bag_record_ $bag_record_pidpid

# Reset the trap to the default behavior
trap - SIGINT SIGTERM