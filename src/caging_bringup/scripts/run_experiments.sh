#!/bin/bash
# Run the experiments of the paper headless, one after the other (Steps 10 and 11).
#
#   cd ~/caging_multirobot_ros2 && source install/setup.bash
#   bash src/caging_bringup/scripts/run_experiments.sh                 # box, 2 experiments x 2 references
#   OBJECTS="box hemisphere" bash src/caging_bringup/scripts/run_experiments.sh
#
# Step 11, repeated runs with a noisy virtual Vicon (each repetition uses its own seed):
#   REPEATS=5 NOISE_XY=0.002 NOISE_YAW=0.005 RESULTS=results/repeat \
#       bash src/caging_bringup/scripts/run_experiments.sh
#
# Each run: task.launch.py auto_start:=true without GUIs, wait for /task_state DONE or ABORTED,
# stop everything. The CSV files go to $RESULTS (default results/); then the plots and the table:
#   ros2 run caging_control plot_results results/*.csv --table
# At the end the script prints how many runs finished (DONE) = object kept until the end of the lap.
OBJECTS=${OBJECTS:-box}
ORIENTATIONS=${ORIENTATIONS:-"static tangent"}
REFERENCES=${REFERENCES:-"waypoints smooth"}
REPEATS=${REPEATS:-1}          # runs per configuration (Step 11)
NOISE_XY=${NOISE_XY:-0.0}      # Vicon noise, std of x and y [m]   (0 = perfect poses)
NOISE_YAW=${NOISE_YAW:-0.0}    # Vicon noise, std of theta [rad]
RESULTS=${RESULTS:-results}    # folder for the CSV files and the logs
WORLD=$(ros2 pkg prefix caging_gazebo)/share/caging_gazebo/worlds/empty_world.sdf
mkdir -p $RESULTS/logs
TOTAL=0
DONE=0

for OBJ in $OBJECTS; do for O in $ORIENTATIONS; do for R in $REFERENCES; do
for SEED in $(seq $REPEATS); do
  NAME=${OBJ}_${O}_${R}_run${SEED}
  echo "=== $NAME   (Vicon noise: $NOISE_XY m, $NOISE_YAW rad, seed $SEED)"
  # own ROS domain and Gazebo partition: never mix with another simulation
  export ROS_DOMAIN_ID=$((90 + RANDOM % 9)) GZ_PARTITION=experiments_$$_$NAME
  setsid ros2 launch caging_bringup task.launch.py world:="-s $WORLD" rviz:=false auto_start:=true \
      object:=$OBJ orientation:=$O reference:=$R results_dir:=$RESULTS \
      vicon_noise_xy:=$NOISE_XY vicon_noise_yaw:=$NOISE_YAW seed:=$SEED \
      > $RESULTS/logs/$NAME.log 2>&1 &
  GROUP=$!
  STATE=""
  for i in $(seq 240); do              # at most 4 minutes per run
    STATE=$(grep -o "TASK STATE: [A-Z]*" $RESULTS/logs/$NAME.log | tail -1 | cut -d' ' -f3)
    [ "$STATE" = "DONE" ] || [ "$STATE" = "ABORTED" ] && break
    sleep 1
  done
  echo "    final state: ${STATE:-none}"
  grep -h "CSV written" $RESULTS/logs/$NAME.log | sed 's/.*CSV written: /    /'
  TOTAL=$((TOTAL + 1))
  [ "$STATE" = "DONE" ] && DONE=$((DONE + 1))
  kill -- -$GROUP 2>/dev/null; sleep 5; kill -9 -- -$GROUP 2>/dev/null
done; done; done; done

echo "=== summary: $DONE of $TOTAL runs finished the lap (DONE); the others were aborted or timed out"
