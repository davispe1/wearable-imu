# test-rig / control/

Host-side rig controller. Commands the rig through scripted motions and
simultaneously logs ground-truth joint angles for comparison against
the IMU pipeline output.

TODO:
    - Implement rig serial/USB interface.
    - Implement motion script player.
    - Implement ground-truth logger (timestamped angle CSV).
    - Implement comparison tool (rig angles vs IMU estimated angles → error plot).
