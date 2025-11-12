# Run Full Clean and Rebuild
    
<i>Do this if you made a change in robot_library but it isn't showing up in your robot project</i>

From the robot project root, run:
    
    ./gradlew clean build --refresh-dependencies


Then from VS Code, do:
- Ctrl+Shift+P → Java: Clean Java Language Server Workspace
- Then → Java: Reload Project
    This clears stale IntelliSense and build path data.
