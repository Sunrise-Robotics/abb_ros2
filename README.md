> [!NOTE]
> Sunrise robotics has forked the original package and made changes to support their needs.
> This mainly revolves around adding support for 1) Omnicore controllers and 2) starting the controller
> with initial joint values.
>
> These changes are manifested in the change of the dependecies (abb_egm_rws_managers) and the way
> the initial joint values are retrieved from the controller in the hardware interface. A standalone
> function was added to retrieve the joint values from RWS but without the use of the RWS managers.
> This is a temporary solution and we will try to help with merging the changes upstream.

This is a meta-package containing everything to run an ABB robot or simulation with ROS 2.

- `abb_bringup`: Launch files and ros2_control config files that are generic to many types of ABB robots.
- `abb_hardware_interface`: A ros2_control hardware interface using abb_libegm.
- `abb_rws_client`: A package containg nodes for RWS only communication.
- `robot_specific_config`: Packages containing robot description and config files that are unique to each type of ABB robot.
- `abb_resources`: A small package containing ABB-related xacro resources.
- `docs`: More detailed documentation.
- `robot_studio_resources`: Code and a pack-and-go solution to begin using RobotStudio easily.
- `abb_ros2`: A meta-package that exists to reserve the repo name in rosdistro



There are three ways to use this package:

- With an actual, physical ABB robot

- With ROS 2 simulating the robot controllers

- With an ABB RobotStudio simulation. Requires 2 PC's (one Windows, one Linux)

Detailed setup instructions can be found [here](docs/README.md).

## Limitations:

The IRB1200-5-0.9 is the only robot that has robot description and config files as of March 2022. Pull requests to add additional robot types are welcome.

## Contributing

### pre-commit Formatting Checks

This package has a pre-commit check that runs in CI. You can use this locally and set it up to run automatically before you commit something. To install, use pip:

    pip3 install pre-commit

To run over all the files in the repo manually:

    pre-commit run -a

To run pre-commit automatically before committing in a local repo, install git hooks:

    pre-commit install


# Tracing

If you're wondering what tracing is and how to use this information, you most probably don't need tracing at this point.
Despite this passive agressive tone (which is introduced to add a little bit of humor), we provide some information to get you started with tracing.

## Tracepoints and source files

Sunrise added the tracepoints within the `read` and `write` functions of the hardware interface.
You can see this by inspecting the `read` and `write` functions of the [`abb_hardware_interface.cpp`](abb_hardware_interface/src/abb_hardware_interface.cpp) file.

Additionally, we added the source and header files for these tracepoints in the `src` and `include` directories, respectively.
These files were generated with the `lttng-gen-tp` tool using the definition in `hardware_interface.tp`.
After creation, they were slightly modified to adjust the paths of the include.

## Building with tracing enabled

Tracing is turned off by default. To turn it on, define the `ENABLE_TRACING` variable at build time with
`--cmake-args -DENABLE_TRACING=OFF`.
Example with colcon:
```bash
$ colcon build --packages-select abb_hardware_interface --cmake-args -DENABLE_TRACING=OFF
```

## Tracing with ros2_tracing

Once built, you should start a tracing session with `ros2_tracing`:
```bash
ros2 trace --session-name abb-hardware-interface -u hardware_interface:* --list
```
The arguments are as follows:
- `--session-name` is the name of the session. This is then reflected in the directory where the trace data is stored (e.g. `~/.ros/tracing/abb-hardware-interface`).
- `-u hardware_interface:*` will trace all events from the `hardware_interface` namespace.
- `--list` will list all the events that can be traced.

After that, run the ROS 2 control nodes as usual.

## Inspecting the trace data

To inspect the data of the tracing session, we recommend using `babeltrace2`.
If created the trace data according to the example above, you can inspect it with:
```bash
babeltrace2 ~/.ros/tracing/abb-hardware-interface | less
```

This will open a pager with the trace data. It will look something like this:

```
[14:51:33.967204443] (+0.000080284) sunrise-rtc hardware_interface:write_start: { cpu_id = 0 }, { vpid = 1519705, procname = "ros2_control_no", vtid = 1519779 }, { if_name = "ABBMultiInterfaceHardwareSmall" }
[14:51:33.967218440] (+0.000013997) sunrise-rtc hardware_interface:write_end: { cpu_id = 0 }, { vpid = 1519705, procname = "ros2_control_no", vtid = 1519779 }, { if_name = "ABBMultiInterfaceHardwareSmall" }
[14:51:33.967219471] (+0.000001031) sunrise-rtc hardware_interface:write_start: { cpu_id = 0 }, { vpid = 1519705, procname = "ros2_control_no", vtid = 1519779 }, { if_name = "ABBMultiInterfaceHardwareBig" }
[14:51:33.967221196] (+0.000001725) sunrise-rtc hardware_interface:write_end: { cpu_id = 0 }, { vpid = 1519705, procname = "ros2_control_no", vtid = 1519779 }, { if_name = "ABBMultiInterfaceHardwareBig" }
[14:51:33.971103015] (+0.003881819) sunrise-rtc hardware_interface:read_start: { cpu_id = 0 }, { vpid = 1519705, procname = "ros2_control_no", vtid = 1519779 }, { if_name = "ABBMultiInterfaceHardwareSmall" }
[14:51:33.971147404] (+0.000044389) sunrise-rtc hardware_interface:read_end: { cpu_id = 0 }, { vpid = 1519705, procname = "ros2_control_no", vtid = 1519779 }, { if_name = "ABBMultiInterfaceHardwareSmall" }
[14:51:33.971148861] (+0.000001457) sunrise-rtc hardware_interface:read_start: { cpu_id = 0 }, { vpid = 1519705, procname = "ros2_control_no", vtid = 1519779 }, { if_name = "ABBMultiInterfaceHardwareBig" }
[14:51:33.971154327] (+0.000005466) sunrise-rtc hardware_interface:read_end: { cpu_id = 0 }, { vpid = 1519705, procname = "ros2_control_no", vtid = 1519779 }, { if_name = "ABBMultiInterfaceHardwareBig" }
```

In this example we have two controllers; `ABBMultiInterfaceHardwareSmall` and `ABBMultiInterfaceHardwareBig`.
The events `write_start`, `write_end`, `read_start`, and `read_end` events are logged for both controllers.

# Little bit of help with setting up tracing dependencies

First and foremost, I hope you will not need to do this.

## Tracing with ros2_tracing

The [How to use ros2_tracing to trace and analyze an application](https://docs.ros.org/en/humble/Tutorials/Advanced/ROS2-Tracing-Trace-and-Analyze.html) guide is a good starting point to get tracing going. We are using Humble, which means you will need to build ROS 2 from source. This has also some other implications.
For example ...

### Building FastDDS so it's up to date

The FastDDS package that is available on upstream is not up to date with the version that allows you to  use tracing.
It boils down to some development libraries that are needed for LTTng. Please follow the
[instructions](https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#fastddsgen-sl)
in the FastDDS documentation.

### Building LTTng modules

This part is required only if you are on a real-time computer.
That is, a Linux with the `PREEMPT_RT` patch applied to the kernel.
Follow the [instructions](https://github.com/ros2/lttng_modules) available in the repo.