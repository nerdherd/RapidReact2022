# Refactoring Steps

Finished steps are strikethroughed

1. Have one Constants file
    - Hatchbot example uses a Constants class with static final classes nested inside (in the same file)
    - Should be easy to do
    - Dylan didn't actually say to do this but yeah

1. ~~Hungarian notation~~
    - Python script is done but will apply last just in case

1. Remove commented out bloat code

1. Organize subsystems
    - Combine everybot subsystems with actual subsystems

1. Change static setup methods to be constructors

1. Migrate from timing based to command based 
    - Timing methods in everybot should be moved to OI using JoystickButton.whenpressed

1. Change references to static methods to call from the class rather than the instance
    - Or rather, change all methods to be called from the instance
