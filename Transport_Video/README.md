# video_io

ROS2 package to play and save videos

- [Installation instructions](#installation_instructions)

<a name=installation_instructions></a>

# Installation instructions

Video codec dependencies

    # ubuntu-restricted extras required to play H264 codecs?
    sudo apt-get install ubuntu-restricted-extras

    # Codec to play `.avi` with the default movie player.
    sudo apt-get install libdvdnav4  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly libdvd-pkg

# Example

To run the basic example, type:

    ros2 launch video_io example.launch.py

This will prompt you to pick an index for one of the examples. The examples provided are:

1. Basic playing/saving of video
2. Saving a (temporally) downsampled video, recording only ever nth frame
3. Play a video at lower resolution (downsampled pixels) and record the video
4. Play a video, and save the video only after receiving a command. This command contains a burst duration variable.

You can experiment with different parameters in the ./config/example.yaml file. Each example is contained in it's own namespace, as signified with it's YAML indent.

# License and reuse

Currently this repository is only accessible for maimon lab members. We plan to release this repository in the future under the LGPLv3 license.
