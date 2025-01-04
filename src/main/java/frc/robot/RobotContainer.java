/*
 * Hey future T3 :) Before I leave I just wanted to thank 
 * this team for giving me the experiences it did.
 * I had no idea I was going to fall in love with making
 * things out of my own hands and watching them come to life.
 * But robotics showed me that, and I will forever be grateful
 * for it. I hope that as you guys code, build, and design
 * for this season you enjoy it as much as I always did. 
 * I know it's cheesy but please don't forget to stop 
 * and smell the roses. Remember why you are doing what you 
 * are doing. Even though things suck when stuff doesn't go
 * as planned, it's a part of the learning process and it 
 * makes you a smarter person as you adapt to it. I know we
 * can't always win and that's okay. You're learning. And that's
 * what matters. I don't think I can even imagine a high school 
 * experience without T3 in it. I devoted 4 years of my life to 
 * this crap and I have absolutely no regrets. As someone who has 
 * been programming for the team for the past few years I know
 * that it can be alot sometimes, but don't worry. Just do your
 * best with the knowledge you have. Also, I wouldn't have done
 * anything other than be the programmer for this team and I
 * loved that I did it. Thank you to all of the people that made
 * my robotics experiences so much better by just being in it.
 * Special shoutouts to: my favorite co-captains Anshu and Ryan,
 * Carrie my favorite coding partner, Anika the little sister
 * I've always wanted, and last but most certainly not least, 
 * Mr. Ware and Mr.Garren for enabling me to have such an
 * amazing educational experience with robotics. 
 * 
 * Thank you for everything,
 * Akshita Santra 
 */

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * This is the RobotContainer class. It holds the configuration for the robot,
 * including control bindings and the necessary subsystems.
 * The main purpose is to set up commands for robot functions like driving, 
 * swerve controls, and autonomous behaviors.
 */
public class RobotContainer {
    
    // Maximum speed and angular rate values
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // Desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Controllers */
    private final CommandXboxController joystick = new CommandXboxController(0); // Joystick controller

    /* Subsystems */
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // Drivetrain subsystem

    /* Drive Requests for controlling swerve drive */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband for precision control
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for swerve

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // Swerve brake request
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); // Point wheels at a specific direction

    /* Telemetry Logger */
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /**
     * Configure the button bindings for the robot controls.
     * This method sets up the control bindings for driving and other robot operations.
     */
    private void configureBindings() {
        
        // Set default command for the drivetrain to handle field-centric driving
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Button A: Activate swerve drive brake when pressed
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Button B: Point wheels towards the joystick direction when pressed
        joystick.b().whileTrue(drivetrain.applyRequest(() -> 
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Left bumper: Reset field-centric heading when pressed
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        // If in simulation, set initial field-relative pose
        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        // Register telemetry for logging
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Constructor for RobotContainer. Configures all button bindings and sets up commands.
     */
    public RobotContainer() {
        configureBindings();
    }

    /**
     * Get the autonomous command to be executed at the start of the match.
     * Currently, no autonomous command is configured.
     * 
     * @return The autonomous command.
     */
    public Command getAutonomousCommand() {
        // Placeholder for autonomous command configuration
        return Commands.print("No autonomous command configured");
    }
}
