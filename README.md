
This fork is just mainly to integrate ORB-Slam3 for use with duckietown sim right now for the 
object detection excersise. Hopefully on real bot as well.

need an update to new code of ORB-Slam3 and to compile on jetson for full duckie bot integration.

to clone use:

git clone --recurse-submodules https://github.com/arcanon/mooc-exercises.git mooc-test

to compile,

  cd object-detection
  use 'dts exercises build -d' and build mooc-exercises/object-detection/solution/ORB_SLAM3
  use 'dts exercises build' to build the exercise
  you can run in sime with 'dts exercises test --sim --scenarios assets/test_scenarios'
  note you need enough objects in sim to make sure there are enough features for detection in the FOV

Old readme.
This is an exercises repository to support the Duckietown MOOC.
