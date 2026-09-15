# Automatic LAN dataset receiver

Run the receiver on whichever PC should save the data. It connects to the X5;
no destination IP is hard-coded on the robot, and the receiving PC needs neither
ROS nor a camera SDK. Only one receiver can own the queue at a time.

## Linux / macOS / Windows WSL

On Ubuntu/Debian, install the build dependencies once:

```sh
sudo apt install g++ pkg-config libcpp-httplib-dev libssl-dev nlohmann-json3-dev
bash tool/receiver/build.sh
build/receiver/episode_receive http://192.168.3.179:8000 "$HOME/datasets/luxi" --release-source
```

On macOS, install a C++17 compiler, cpp-httplib, OpenSSL and nlohmann-json with
pkg-config files, then use the same build script. This platform has not yet been
tested. On Windows, use WSL and a destination under `/mnt/c/...` or another
mounted drive; native Windows support is not provided by this POSIX receiver.
Paths containing spaces must be quoted. The actual tested PC is Ubuntu x86_64.

Stop with Ctrl-C. To change receiving PCs, stop the old receiver, then run the
same command on the new PC with that PC's destination directory. An uncleanly
stopped receiver's lease expires after 20 seconds. The dashboard shows the
current PC, directory, transfer progress and last verified episode. Merely
opening the dashboard does not select that PC as a receiver.

## Recording and transfer

1. The three camera services publish RGB JPEGs. Enable the controller program,
   complete operator-supervised arm alignment, then start an episode with A or
   the dashboard. Use the configured task label. End successfully with A or
   `成功并保存`; failure/cancellation remains separately labelled.
2. The X5 stages the bag, aligned samples and metadata in a 2 GiB tmpfs at
   `/run/luxi_dataset/episodes`. Keep 0.25 GiB free. An unavailable RAM mount
   causes the launcher to fail; it never falls back to the SD card. Swap must be
   disabled. OS/service logs can still use the SD card.
3. Once the recorder atomically commits an episode and its manifest, the PC
   downloads it into `.partial-episode-ID`. HTTP Range resumes incomplete files;
   every file's length and SHA-256 are checked. Corrupt partial files are retried
   from the beginning. All manifest-listed files are retained, including failed
   or cancelled episodes and their labels.
4. After fsync of all files/directories, the PC atomically renames the directory
   to its final episode ID. Only then does it acknowledge the exact manifest.
   With `--release-source`, the X5 removes that episode's RAM copy. The checksum
   receipt remains in RAM. Without this option the X5 retains its copy.
5. A disconnected PC leaves episodes queued in RAM. Stop recording if the buffer
   fills; recording stops automatically at the reserve threshold. Unsaved RAM
   data is lost on X5 power failure or reboot. This is post-episode transfer,
   not continuous streaming of an unfinished recording to the PC.

Automatic deletion is refused for disk-backed recording profiles. A checksum
receipt verifies transfer integrity, not the demonstration's quality or the
correctness of camera mounting. The raw schema is `luxi_arms_v1`, not an already
converted LeRobot dataset. Review `manifest.json` (`status`, `qc`, `samples`),
`samples.jsonl`, `camera_config.json`, `robot_config.json` and the ROS bag.

The LAN endpoints share the dashboard's trusted-LAN access model. A receiver
lease avoids two PCs consuming/deleting the same queue; it does not add user
login or encryption. A browser-originated request cannot claim/release a queue.
