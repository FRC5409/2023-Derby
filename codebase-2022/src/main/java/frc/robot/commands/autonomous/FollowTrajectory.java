package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


public class FollowTrajectory extends CommandBase {

    private final DriveTrain m_drivetrain;

    private final DifferentialDriveVoltageConstraint autoVoltageConstraint;
    private final TrajectoryConfig config;

    private final Trajectory trajectory;

    private final RamseteCommand ramseteCommand;

    public FollowTrajectory(DriveTrain drivetrain) {

        m_drivetrain = drivetrain;

        // Create a voltage constraint to ensure we don't accelerate too fast
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.kDriveTrain.kCharacterization.ksVolts,
                                        Constants.kDriveTrain.kCharacterization.kvVoltSecondsPerMeter,
                                        Constants.kDriveTrain.kCharacterization.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveTrain.kCharacterization.kDriveKinematics,
            Constants.kAuto.kMaxVoltage
        );

        config = new TrajectoryConfig(Constants.kDriveTrain.kCharacterization.kMaxSpeedMetersPerSecond,
                                                        Constants.kDriveTrain.kCharacterization.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveTrain.kCharacterization.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

        
        trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );



        // Create Ramsete Command
        m_drivetrain.resetOdometry(trajectory.getInitialPose());

        ramseteCommand = new RamseteCommand(
            trajectory,
            m_drivetrain::getPose2d,
            new RamseteController(Constants.kDriveTrain.kCharacterization.kRamseteB,
                                    Constants.kDriveTrain.kCharacterization.kRamseteZeta),

            new SimpleMotorFeedforward(Constants.kDriveTrain.kCharacterization.ksVolts,
                                        Constants.kDriveTrain.kCharacterization.kvVoltSecondsPerMeter,
                                        Constants.kDriveTrain.kCharacterization.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveTrain.kCharacterization.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(Constants.kDriveTrain.kCharacterization.kPDriveVel, 0, 0),
            new PIDController(Constants.kDriveTrain.kCharacterization.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVoltages,
            m_drivetrain
        );

        addRequirements(drivetrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        return false;
    }

}