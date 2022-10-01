package org.firstinspires.ftc.teamcode.Utility.Odometry.DeadWheels;

import org.firstinspires.ftc.teamcode.Utility.Mecanum.MecanumNavigation.*;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

public interface Localizer {

    void update(RobotHardware robotHardware);

    Navigation2D getCurrentPosition();

    void setCurrentPosition(Navigation2D currentPosition);

    Frame2D getRobotFrame();

}


