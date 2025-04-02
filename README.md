# SwerveBot Legacy 1
This code was the first test code that we ran. For some strange reason, the Studica dependency doesn't get recognized by VSCode intellisense and its debugger tools, although it builds and deploys to the robot just fine. This code is incredibly incorrect (uses DIO encoders instead of SPARK MAX encoders, uses a generic analog gyro instead of the navX2, PID values not tuned, no elevator control, etc.), resulting in a noncontrollable robot.

This code was modified from the WPILib SwerveBot example project.