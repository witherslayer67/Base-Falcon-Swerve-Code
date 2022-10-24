package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import static java.lang.Math.abs;
import static java.lang.Math.pow;


public class DriveManuallyCommand extends CommandBase {
        private final DriveSubsystem drive;
        private final Timer locateTimer;
        private SlewRateLimiter translationXLimiter;
        private SlewRateLimiter translationYLimiter;
        private SlewRateLimiter rotationLimiter;
        private double translationXPercent;
        private double translationYPercent;
        private double rotationPercent;
        public DriveManuallyCommand(DriveSubsystem drive){
            this.drive = drive;
            addRequirements(drive);

            locateTimer = new Timer();
            locateTimer.start();

            translationXLimiter = new SlewRateLimiter(Constants.translationRateLimit);
            translationYLimiter = new SlewRateLimiter(Constants.translationRateLimit);
            rotationLimiter = new SlewRateLimiter(Constants.rotationRateLimit);

        }
        @Override
        public void execute() {
            translationXPercent = -Constants.driveController.getRawAxis(1);
            translationYPercent = -Constants.driveController.getRawAxis(0);
            rotationPercent = -Constants.driveController.getRawAxis(4);

            if (abs(translationXPercent) < Constants.deadzone){
                translationXPercent = 0.0;
            }

            if (abs(translationYPercent) < Constants.deadzone){
                translationYPercent = 0.0;
            }

            if (abs(rotationPercent) < Constants.deadzone){
                rotationPercent = 0.0;
            }

            translationXPercent *= 1.00;
            translationYPercent *= 1.00;
            rotationPercent *= 1.00;
            /*
            translationXPercent *= abs(pow(translationXPercent, 2))
            translationYPercent *= abs(pow(translationYPercent, 2));
            rotationPercent *= abs(pow(rotationPercent, 2); //bruh
             */

            translationXPercent = translationXLimiter.calculate(translationXPercent);
            translationYPercent = translationYLimiter.calculate(translationYPercent);
            rotationPercent = rotationLimiter.calculate(rotationPercent);
            /*
            translationXPercent = translationXLimiter.calculate(translationXPercent * abs(pow(translationXPercent, 1.96)));
            translationYPercent = translationYLimiter.calculate(translationYPercent * abs(pow(translationYPercent, 1.96)));
            rotationPercent = rotationLimiter.calculate(rotationPercent * abs(pow(rotationPercent, 1.96)));
             */
            if (abs(translationXPercent) < Constants.deadzone){
                translationXPercent = 0.0;
            }

            if (abs(translationYPercent) < Constants.deadzone){
                translationYPercent = 0.0;
            }

            if (abs(rotationPercent) < Constants.deadzone){
                rotationPercent = 0.0;
            }

            drive.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            translationXPercent * abs(pow(translationXPercent, 1.96)) * Constants.maxVelocity,
                            translationYPercent * abs(pow(translationYPercent, 1.96)) * Constants.maxVelocity,
                            rotationPercent * abs(pow(rotationPercent, 1.96)) * Constants.maxAngularVelocity,
                            (drive.isRobotOriented() ? Rotation2d.fromDegrees(0.0) : drive.getGyroscopeRotation())
                    )
            );
        }
        @Override
        public void end(boolean interrupted) {
            // Stop the drive
            drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        }
}

