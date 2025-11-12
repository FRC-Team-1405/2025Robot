# How to install robot_library

Follow these steps to install and set up `robot_library` as part of your composite Gradle project.

1. **Clone the `robot_library` Repository**

2. **Folder Structure**
Ensure the folder structure looks similar to this:  

    VS Code/  
    ├── 2025Robot/  
    └── robot_library/

3. **Reload Projects in VS Code**
- Open VS Code.
- Press Ctrl+Shift+P and select Java: Reload Projects.

4. **Build and Run the Robot Code**
- The robot code should now build and run successfully, including the robot_library dependencies.


# Run Full Clean and Rebuild
    
<i>Do this if you made a change in robot_library but it isn't showing up in your robot project</i>

From the robot project root, run:
    
    ./gradlew clean build --refresh-dependencies


Then from VS Code, do:
- Then → Java: Reload Project
    This clears stale IntelliSense and build path data.
