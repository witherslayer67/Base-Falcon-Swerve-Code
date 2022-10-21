// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Explaining names:
    //F = Front
    //R = Rear
    //L = Left
    //R = Right
    //D = Drive
    //S = Steer
    //Ex. fldm = Front Left Drive Motor

    //Drive Motor(s) ports
    public static int fldmPort = 0;
    public static int flsmPort = 1;

    public static int frdmPort = 2;
    public static int frsmPort = 3;

    public static int rrdmPort = 4;
    public static int rrsmPort = 5;

    public static int rldmPort = 6;
    public static int rlsmPort = 7;
    //Drive Motor(s) ports ^

    //Explaining Names
    //F = Front
    //R = Rear
    //L = Left
    //R = Right
    //E = Encoder
    //Example. flePort = Front Left Encoder Port
    //Encoder Ports
    public static int flePort = 0;
    public static int frePort = 1;
    public static int rrePort = 2;
    public static int rlePort = 3;
    //Encoder Ports ^

    //Explaining Names
    //F = Front
    //R = Rear
    //L = Left
    //R = Right
    //E = Encoder
    //O = Offset
    //Example. fleo = Front Left Encoder Offset
    //Encoder Offsets
    public static double fleo = Math.toRadians(-39.814453);
    public static double freo = Math.toRadians(-239.589844);
    public static double rreo = Math.toRadians(-249.697266);
    public static double rleo = Math.toRadians(-198.457031);
    //Encoder Offsets ^

    public static double wheelBase = 14.0; //i think that is total # of falcons

    public static double maxVoltage = 12.0;


    public static XboxController driveController = new XboxController(0);

    public static int translateYAxis = 0;
    public static int translateXAxis = 1;
    public static double deadzone = 0.05;
    public static double maxVelocity = (6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

    public static double maxAngularVelocity = maxVelocity /
            Math.hypot(wheelBase / 2.0, wheelBase / 2.0);

    public static double falconRPMToUPS = 2048.0 / 600.0;
    public static double translationRateLimit = 2.5;
    public static double rotationRateLimit = 2.5;
    public static int talonCount = 14;


}
