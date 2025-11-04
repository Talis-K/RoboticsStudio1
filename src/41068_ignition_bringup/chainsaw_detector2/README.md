# chainsaw_detector

ROS 2 ament_python package that classifies audio as **chainsaw** vs **ambient** using FFT/PSD and harmonicity checks.

## Topics

Publishes:

- `/audio/classification` (`std_msgs/String`): `"chainsaw"` or `"ambient"`
- `/audio/chainsaw_confidence` (`std_msgs/Float32`): 0..1
- `/audio/dominant_frequency` (`std_msgs/Float32`): Hz
- `/audio/psd_band_power` (`std_msgs/Float32`): 0..1

Subscribes:

- `audio_topic` (default `/audio`):
  - If `audio_common_msgs/msg/AudioData` is available: expects PCM 16-bit little-endian bytes (mono or stereo).
  - Otherwise falls back to `std_msgs/msg/Float32MultiArray` of mono samples.

## Parameters
- `audio_topic` (string, default `/audio`)
- `sample_rate` (int, default `16000`)
- `frame_ms` (int, default `500`)
- `hop_ms` (int, default `250`)
- `chainsaw_low_hz` (float, default `90.0`)
- `chainsaw_high_hz` (float, default `400.0`)
- `confidence_thresh` (float, default `0.55`)
- `pcm_width_bits` (int, default `16`)
- `channels` (int, default `1`)

## Build & Run

```bash
# In your ROS 2 workspace
cd ~/ros2_ws/src
# Copy this folder here
cp -r /mnt/data/chainsaw_detector .

cd ..
colcon build --packages-select chainsaw_detector
. install/setup.bash

# Run (adjust audio topic to your Gazebo/Ignition microphone topic)
ros2 launch chainsaw_detector chainsaw_detector.launch.py audio_topic:=/microphone/audio sample_rate:=16000
```

## GUI Integration

Point your GUI subscribers to the published topics above to display class, confidence, and dominant frequency.
