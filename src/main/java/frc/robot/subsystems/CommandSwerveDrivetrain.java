package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstantsCompBot;
import frc.robot.generated.TunerConstantsCompBot.TunerSwerveDrivetrain;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.shooter.Turret;
import frc.robot.utilities.simulation.MapleSimSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    // Values for the simulation
    private static final double kSimLoopPeriod = 0.002; // 2 ms
    private static final double kSimOdometryFrequencyHz = 0; // use default value
    private Notifier m_simNotifier = null;
    private static final Mass ROBOT_MASS = Pounds.of(145.0);   // mass including bumpers and battery
    private static final Distance ROBOT_WIDTH = Inches.of(27.0 + 3 * 3.0);    // bumper length/width
    
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // // values from 2024 competition. Maybe should be tuned
    // private static final PIDConstants PATH_PLANNER_TRANSLATION_PID = new PIDConstants(5, 0, 0);
    // private static final PIDConstants PATH_PLANNER_ANGLE_PID       = new PIDConstants(5, 0, 0);

    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final AprilTagVision m_aprilTagVision;

    private MapleSimSwerveDrivetrain m_mapleSimSwerveDrivetrain = null;

    private FollowPath.Builder m_blinePathBuilder;

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */

    public CommandSwerveDrivetrain(
        AprilTagVision aprilTagVision,
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        // The default Phoenix odometry rate is more aggressive than we need for desktop sim,
        // which can cause "stale" status signal warnings from the module Talons.
        super(drivetrainConstants, Utils.isSimulation() ? kSimOdometryFrequencyHz : 0.0, 
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));

        // setupPathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_aprilTagVision = aprilTagVision;

        // if (RobotBase.isReal() && Constants.OPTIMIZE_CAN) {
        //     optimizeCAN();
        // }
    }

    @SuppressWarnings("unused")
    private void optimizeCAN() {
        // According to CTRE Support, the variables needed for odometry have
        // already been set with the appropriate update frequency,
        // so only need to do the optimizeBus call

        // Optimize all the motors and CANcoders
        for (var module : this.getModules()) {
            module.getDriveMotor().optimizeBusUtilization();
            module.getSteerMotor().optimizeBusUtilization();
            module.getEncoder().optimizeBusUtilization();
        }

        // Also, do the Pigeon
        getPigeon2().optimizeBusUtilization();
    }
    
    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            RoboRioSim.setVInVoltage(12.0);
            if (m_aprilTagVision != null) {
                m_aprilTagVision.updateSimulation(this);
            }
        }
        
        if (m_aprilTagVision != null) {
            m_aprilTagVision.addVisionMeasurements(this);
        }

        // This is here because it needs the odometry Pose. Leave it for now.
        Pose2d pose = getPose();
        Translation2d target = Shoot.shotAutoTarget(pose);
        Translation2d turretToTarget = Turret.getTranslationToGoal(pose, target);
        SmartDashboard.putNumber("turret/distToShotTarget", Units.metersToInches(turretToTarget.getNorm()));

        SmartDashboard.putNumber("drivetrain/pidgeonVelocityZWorld", getPigeon2().getAngularVelocityZWorld().getValueAsDouble());

        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    // WARNING: simulation only works (sort of) with a Falcon500 as the steer motor
    @SuppressWarnings("unchecked")
    private void startSimThread() {
        m_mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            ROBOT_MASS,   // mass including bumpers and battery
            ROBOT_WIDTH,  // bumper length - robot is square
            ROBOT_WIDTH,  // bumper width
            DCMotor.getKrakenX60(1), // drive motor type
            DCMotor.getFalcon500(1), // steer motor type
            1.2, // wheel COF
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstantsCompBot.FrontLeft,
            TunerConstantsCompBot.FrontRight,
            TunerConstantsCompBot.BackLeft,
            TunerConstantsCompBot.BackRight);
            
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(m_mapleSimSwerveDrivetrain::update);
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Set the robot pose on the field - generally used to set start of Auto
     * @param pose  Pose of the robot, relative to Blue (0,0)
     */
    public void setPose(Pose2d pose) {
        if (m_mapleSimSwerveDrivetrain != null) {
            m_mapleSimSwerveDrivetrain.m_mapleSimDrive.setSimulationWorldPose(pose);
            // this is kind of needed but screws up start of Auto
            Timer.delay(0.05); // wait for simulation to update              
        }
        this.resetPose(pose);
    }
    
    /**
     * Fetch the current robot pose on the field
     * @return Pose2d of the robot
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    public ChassisSpeeds getRobotCentricSpeeds() {
        return getState().Speeds;
    }

    public ChassisSpeeds getFieldCentricSpeeds() {
        SwerveDriveState state = getState();
        return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is
     *         empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // public void setupPathPlanner() {
    //     try {
    //         // Load the RobotConfig from the settings file created by GUI. 
    //         // You should probably store this in your Constants file
    //         RobotConfig config = RobotConfig.fromGUISettings();

    //         // TODO: fix code to allow FF
    //         // final boolean enableFeedforward = true;
    //         // Configure AutoBuilder last
    //         AutoBuilder.configure(
    //                 // Robot pose supplier
    //                 this::getPose,   
    //                 // null - no need to set pose at very start of Auto
    //                 null,
    //                 // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //                 () -> this.getState().Speeds,
    //                 // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
    //                 // optionally outputs individual module feedforwards
    //                 (speedsRobotRelative, moduleFeedForwards) -> {
    //                     // Consumer of ChassisSpeeds to drive the robot
    //                     this.setControl(autoRequest.withSpeeds(speedsRobotRelative));
    //                 },
    //                 // PPHolonomicController is the built in path following controller for holonomic
    //                 // drive trains
    //                 new PPHolonomicDriveController(
    //                         PATH_PLANNER_TRANSLATION_PID,
    //                         PATH_PLANNER_ANGLE_PID),
    //                 // The robot configuration
    //                 config,
    //                 // whether to flip directions for Red
    //                 () -> FieldConstants.isRedAlliance(),
    //                 // Reference to this subsystem to set requirements
    //                 this
    //         );

    //     } catch (Exception e) {
    //         // Handle exception as needed
    //         e.printStackTrace();
    //     }

    //     // Preload PathPlanner Path finding
    //     // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        
    //     CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    //     CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    // }

    /**
     * Get the path follower with events.
     *
     * @param path PathPlanner path.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command followPath(PathPlannerPath path) {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        // return AutoBuilder.followPath(path);
        return Commands.waitTime(Seconds.of(1.0));  // dummy for now
    }

    // public Command pathFindToPose(Pose2d targetPose, PathConstraints constraints) {
        
    //     return AutoBuilder.pathfindToPose(targetPose, constraints);
    // }

    // public static PathPlannerPath loadPath(String pathName) {
    //     try {
    //         PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    //         return path;
    //     } catch (Exception e) {
    //         DriverStation.reportError(String.format("Unable to load PP path %s", pathName), true);
    //     }
    //     return null;
    // }

    public void setupBline() {
        m_blinePathBuilder = new FollowPath.Builder(
                this,
                this::getPose,
                () -> this.getState().Speeds,
                speedsRobotRelative -> {
                    this.setControl(autoRequest.withSpeeds(speedsRobotRelative));
                },
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(0.2, 0.0, 0.0))
                .withDefaultShouldFlip()
                .withTRatioBasedTranslationHandoffs(true);
    }

    // Follow a BLine Path
    public Command followPath(Path path) {
        Command cmd = m_blinePathBuilder.build(path);

        // // Builder options persist. Clear this before building any later path command.
        // m_blinePathBuilder.withPoseReset(ignored -> {});
        return cmd;
    }
}
