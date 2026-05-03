## Reminders

 - PX4-Autopilot should be built spearetly from the whole proyect with:
        "cd ~/ws/sim/PX4-Autopilot"
        "make px4_sitl gz_x500"
        if theres a problem while building: cd ~/ws/sim/PX4-Autopilot
                                             rm -rf build

- to start DOS bridge agent:
    "MicroXRCEAgent udp4 -p 8888"