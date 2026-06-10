# time_sync_utils

This repository contains utilities for handling and monitoring time-synchronized data in ROS 2. It consists of two packages: `topic_monitor` for validating message synchronization (publishing standard `diagnostic_msgs` plus a Trigger service), and `nmea_gpsd` for bridging GPS data.


## 1. topic_monitor

The `topic_monitor` package is a diagnostic tool used to verify that various ROS 2 topics are active and that their timestamps are synchronized within a specified threshold. It reads a list of target topics from a configuration file and **runs continuously**, maintaining live per-topic statistics (message count, approximate rate, age of the last message, and the latest header timestamp).

It reports using **standard interfaces only** (no custom messages):

* It continuously publishes a `diagnostic_msgs/DiagnosticArray` on the conventional global **`/diagnostics`** topic — one `DiagnosticStatus` per monitored topic.
* It exposes a `std_srvs/Trigger` service for an on-demand pass/fail check.

### Nodes

* **`topic_monitor_node`**: The primary C++ node that subscribes to topics and performs timing analysis. It is meant to be launched persistently (it is included in `sensor_bringup/bluerov_launch.py`).

### Diagnostics (`/diagnostics`)

Each monitored topic becomes one `DiagnosticStatus` named `timesync: <topic>`, with:

* **`level`**: `OK` (0), `WARN` (1), `ERROR` (2), or `STALE` (3). `WARN`/`ERROR` come from the timestamp offset crossing the warn/error thresholds; `STALE` means no message (ever, or within `stale_timeout_seconds`); `ERROR` is also used when a header timestamp cannot be extracted.
* **`message`**: human-readable explanation of the level.
* **`hardware_id`**: the ROS message type.
* **`values`**: `message count`, `rate (Hz)`, `age (s)`, `has timestamp`, `time diff (s)`, `is reference`.

The reference topic is the first entry in the topics file that is currently receiving fresh, stamped messages; all other timestamps are compared against it.

#### Viewing the status lights

A `diagnostic_aggregator` is launched alongside the monitor (see `config/diagnostic_aggregator.yaml`) which republishes a grouped tree on **`/diagnostics_agg`**. Open the standard GUI:

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor   # tree view of /diagnostics_agg (status lights)
# or, for the raw un-aggregated feed:
ros2 run rqt_runtime_monitor rqt_runtime_monitor   # reads /diagnostics directly
```

### Service (`check_time_sync`)

* **`check_time_sync`** (`std_srvs/srv/Trigger`): Empty request. `success` is true only if every topic is `OK`; `message` contains a one-line summary followed by a per-topic breakdown. The service also publishes a fresh `/diagnostics` snapshot when called.

```bash
ros2 service call /bluerov2/check_time_sync std_srvs/srv/Trigger "{}"
```

You can also call it from the built-in `rqt_service_caller` GUI plugin.

### Parameters

* **`topics_file`** (string, default: `config/topics.yaml`): The path to the YAML file listing topics to monitor.
* **`relative_path`** (bool, default: `true`): If true, the node looks for the `topics_file` relative to the package's share directory.
* **`sync_threshold_warn_seconds`** (double, default: `0.1`): The time difference (in seconds) between a topic's timestamp and the reference topic that triggers a warning.
* **`sync_threshold_error_seconds`** (double, default: `1.0`): The time difference that triggers an error.
* **`stale_timeout_seconds`** (double, default: `2.0`): If no new message has arrived within this many wall-clock seconds, the topic is reported as `STALE`.
* **`publish_period_seconds`** (double, default: `1.0`): How often the `DiagnosticArray` is published on `/diagnostics`.
* **`diagnostic_name_prefix`** (string, default: `timesync`): Prefix on each `DiagnosticStatus.name`, used by the aggregator's `GenericAnalyzer` to group the statuses.

### QoS Configuration

Each topic in the `topics.yaml` file can optionally specify custom QoS (Quality of Service) settings. If not specified, default values are used.

**Available QoS Options:**

* **`history_depth`** (integer, default: `10`): The depth of the message history queue.
* **`reliability`** (string, default: `"reliable"`): The reliability policy. Options:
  * `"reliable"`: Ensures message delivery (may retry)
  * `"best_effort"`: Does not guarantee message delivery
* **`durability`** (string, default: `"volatile"`): The durability policy. Options:
  * `"volatile"`: Only subscribers that are alive at publication time receive messages
  * `"transient_local"`: Late-joining subscribers may receive previously published messages

**Example:**

```yaml
topics:
  - name: /example_topic
    type: sensor_msgs/msg/NavSatFix
    qos:
      history_depth: 5
      reliability: best_effort
      durability: transient_local
```

---

## 2. nmea_gpsd

The `nmea_gpsd` package is a ROS 2 utility designed to bridge ROS-based NMEA data to the `gpsd` service. It provides nodes to convert standard ROS messages (like `NavSatFix`) into NMEA sentences and forward them to `gpsd` via various protocols.

**Note:** The serial and TCP implementations in this package are currently considered partially developed and may require further refinement for specific production environments.

### Nodes

#### `navsat_gpsd_serial`

This node converts ROS `sensor_msgs/msg/NavSatFix` and `geometry_msgs/msg/TwistWithCovarianceStamped` messages into standard NMEA `$GPGGA` and `$GPRMC` sentences. These sentences are published to a ROS topic and optionally written to a serial port.

* **Subscribed Topics:**
* `gnss_1/llh_position` (`sensor_msgs/msg/NavSatFix`): Source of position and fix status.
* `gnss_1/velocity` (`geometry_msgs/msg/TwistWithCovarianceStamped`): Source of speed and course.


* **Published Topics:**
* `nmea_constructed` (`nmea_msgs/msg/Sentence`): The generated NMEA sentences.


* **Parameters:**
* `serial_port` (string, default: `/dev/tnt1`): The virtual or physical serial port to write NMEA sentences to.
* `baud_rate` (int, default: `9600`): Baud rate for the serial port.
* `nmea_talker_id` (string, default: `GP`): The talker ID prefix (e.g., `GP` for GPS, `GN` for GLONASS).
* `default_num_satellites` (int, default: `4`): Default satellite count used in GGA sentences.
* `default_hdop` (double, default: `1.5`): Default Horizontal Dilution of Precision.
* `default_geoid_separation` (double, default: `0.0`): Default geoidal separation in meters.



#### `nmea_gpsd_socket`

This node acts as a TCP server that listens for a connection from `gpsd`. Once connected, it forwards NMEA sentences from a ROS topic directly to the TCP client.

* **Subscribed Topics:**
* Defined by `nmea_topic` parameter (default: `/nmea`) (`nmea_msgs/msg/Sentence`).


* **Parameters:**
* `gpsd_host` (string, default: `0.0.0.0`): The address the TCP server binds to.
* `gpsd_port` (int, default: `3001`): The port `gpsd` will connect to.
* `nmea_topic` (string, default: `/nmea`): The ROS topic containing NMEA sentences to forward.



#### `nmea_gpsd_udp`

This node forwards NMEA data to `gpsd` via UDP. It can also synthesize `$GPRMC` sentences by combining raw NMEA GGA data with SBG-specific UTC time messages.

* **Subscribed Topics:**
* `nmea_topic` (default: `/nmea`): ROS topic for incoming NMEA sentences.
* `utc_topic` (default: `/sbg/utc_time`): ROS topic for `sbg_driver/msg/SbgUtcTime` used to generate RMC sentences.


* **Parameters:**
* `gpsd_host` (string, default: `127.0.0.1`): The destination IP where `gpsd` is listening for UDP.
* `gpsd_port` (int, default: `3001`): The destination UDP port.






## License

Both packages are licensed under the Apache License 2.0.