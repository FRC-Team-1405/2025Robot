# TODO
- The eight steps (https://docs.yagsl.com/configuring-yagsl/the-eight-steps) says we should be using driveFieldOrientedDirectAngle as our default command while following the eight steps. I have been using the yagsl-example repo's default command which is driveFieldOrientedAnglularVelocity. i'm not sure if following the wiki and changing this will fix it but worth a shot.

# Controls
- A Button resets gyro


# How to configure
- Module Offsets:
   - Phoenix Tuner > Encoder > Self Test > Refresh (Click)
   - Copy the 'Absolute Position' rotations and multiply by 360deg
   - <b>Please match the encoder to the ID not the name in the .json file!!!</b> We have swapped the modules for purposes of having the correct rotation and front/back of the robot. this means that although the json file is named frontleft.json the IDs contained inside are actually the IDs of the backright module. See wiki for more info: https://docs.yagsl.com/configuring-yagsl/when-to-invert
- Module Location:
   - because we needed to swap modules as described above, the locations do not match the IDs! so be sure the locations match the file name of the json file

# Yet Another Generic Swerve Library (YAGSL) Example project

YAGSL is intended to be an easy implementation of a generic swerve drive that should work for most
square swerve drives. The project is documented
on [here](https://github.com/BroncBotz3481/YAGSL/wiki). The JSON documentation can also be
found [here](docs/START.md)

This example is intended to be a starting place on how to use YAGSL. By no means is this intended to
be the base of your robot project. YAGSL provides an easy way to generate a SwerveDrive which can be
used in both TimedRobot and Command-Based Robot templates.


# Overview

### Installation

Vendor URL:

```
https://broncbotz3481.github.io/YAGSL-Lib/yagsl/yagsl.json
```

[Javadocs here](https://broncbotz3481.github.io/YAGSL/)  
[Library here](https://github.com/BroncBotz3481/YAGSL/)  
[Code here](https://github.com/BroncBotz3481/YAGSL/tree/main/swervelib)  
[WIKI](https://github.com/BroncBotz3481/YAGSL/wiki)  
[Config Generation](https://broncbotz3481.github.io/YAGSL-Example/)

# Create an issue if there is any errors you find!

We will be actively montoring this and fix any issues when we can!

## Development

* Development happens here on `YAGSL-Example`. `YAGSL` and `YAGSL-Lib` are updated on a nightly
  basis.

# Support our developers!
<a href='https://ko-fi.com/yagsl' target='_blank'><img height='35' style='border:0px;height:46px;' src='https://az743702.vo.msecnd.net/cdn/kofi3.png?v=0' border='0' alt='Buy Me a Robot at ko-fi.com'></a>

### TL;DR Generate and download your configuration [here](https://broncbotz3481.github.io/YAGSL-Example/) and unzip it so that it follows structure below:

```text
deploy
└── swerve
    ├── controllerproperties.json
    ├── modules
    │   ├── backleft.json
    │   ├── backright.json
    │   ├── frontleft.json
    │   ├── frontright.json
    │   ├── physicalproperties.json
    │   └── pidfproperties.json
    └── swervedrive.json
```

### Then create your SwerveDrive object like this.

```java
import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.util.Units;


SwerveDrive swerveDrive=new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Units.feetToMeters(14.5));
```

# Migrating Old Configuration Files

1. Delete `wheelDiamter`, `gearRatio`, `encoderPulsePerRotation` from `physicalproperties.json`
2. Add `optimalVoltage` to `physicalproperties.json`
3. Delete `maxSpeed` and `optimalVoltage` from `swervedrive.json`
4. **IF** a swerve module doesn't have the same drive motor or steering motor as the rest of the
   swerve drive you **MUST** specify a `conversionFactor` for BOTH the drive and steering motor in
   the modules configuration JSON file. IF one of the motors is the same as the rest of the swerve
   drive and you want to use that `conversionFactor`, set the `conversionFactor` in the module JSON
   configuration to 0.
5. You MUST specify the maximum speed when creating a `SwerveDrive`
   through `new SwerveParser(directory).createSwerveDrive(maximumSpeed);`
6. IF you do not want to set `conversionFactor` in `swervedrive.json`. You can pass it into the
   constructor as a parameter like this

```java
double DriveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(WHEEL_DIAMETER), GEAR_RATIO, ENCODER_RESOLUTION);
double SteeringConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(GEAR_RATIO, ENCODER_RESOLUTION);
SwerveDrive swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, SteeringConversionFactor, DriveConversionFactor);
```

### Falcon Support would not have been possible without support from Team 1466 Webb Robotics!

# Configuration Tips

### My Robot Spins around uncontrollably during autonomous or when attempting to set the heading!

* Invert the gyro scope.
* Invert the drive motors for every module. (If front and back become reversed when turning)

### Angle motors are erratic.

* Invert the angle motor.

### My robot is heavy.

* Implement momentum velocity limitations in SwerveMath.

### Ensure the IMU is centered on the robot

# Maintainers
- @thenetworkgrinch
- @Technologyman00 

# Special Thanks to Team 7900! Trial N' Terror
Without the debugging and aid of Team 7900 the project could never be as stable or active as it is. 
