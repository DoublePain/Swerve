package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      //  configureButtonBindings();
    }}
/*/
    public class ZeroHeadingCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;

    public ZeroHeadingCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem); // Ensure that no other command is using SwerveSubsystem
    }

    @Override
    public void initialize() {
        swerveSubsystem.zeroHeading();
    }

    @Override
    public boolean isFinished() {
        return true; // Command finishes immediately after zeroing the heading
    }
}

    private void configureButtonBindings() {
      new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
    }} */