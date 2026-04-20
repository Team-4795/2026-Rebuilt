package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Utility class used to build auto routines. does everything followpath builder does AND MORE!!!
 */
public class BLineAutoChooser extends FollowPath.Builder {
  private List<Path> options = new ArrayList<>();
  private SendableChooser<Command> chooser = new SendableChooser<>();
  private Drive drive;

  /**
   * Create and populate a sendable chooser with all PathPlannerAutos in the project
   *
   * @param defaultAutoName The name of the auto that should be the default option. If this is an
   *     empty string, or if an auto with the given name does not exist, the default option will be
   *     Commands.none()
   * @return SendableChooser populated with all autos
   */
  public SendableChooser<Command> buildAutoChooser() {
    return buildAutoChooserWithOptionsModifier((stream) -> stream);
  }

  /**
   * holy shit i cant believe that worked my brains too small for ts - cathy
   *
   * <p>please do not be mean to it; it will crumble to dust under any pressure
   *
   * <p>Builder class for constructing {@link FollowPath} commands with a fluent API.
   *
   * <p>The Builder allows you to configure a path follower once with all the robot-specific
   * parameters, then build multiple commands for different paths. This avoids repeating the same
   * configuration for each path.
   *
   * <p><b>Important:</b> This builder is mutable and stateful. Optional settings configured through
   * {@code with...} methods persist for all subsequent {@link #build(Path)} calls until you change
   * them again. For example, once {@link #withPoseReset(Consumer)} is set, later built commands
   * will continue resetting pose unless you override it (for example, with a no-op consumer).
   *
   * <h2>Required Parameters</h2>
   *
   * <p>The constructor requires:
   *
   * <ul>
   *   <li>Drive subsystem - For command requirements
   *   <li>Pose supplier - Returns current robot pose
   *   <li>Robot-relative speeds supplier - Returns current chassis speeds
   *   <li>Robot-relative speeds consumer - Accepts commanded chassis speeds
   *   <li>Three PID controllers for translation, rotation, and cross-track correction
   * </ul>
   *
   * <h2>Optional Configuration</h2>
   *
   * <ul>
   *   <li>{@link #withShouldFlip(Supplier)} - Custom alliance flip logic
   *   <li>{@link #withDefaultShouldFlip()} - Use DriverStation alliance for flipping
   *   <li>{@link #withShouldMirror(Supplier)} - Custom vertical mirror logic
   *   <li>{@link #withPoseReset(Consumer)} - Reset odometry to path start pose
   * </ul>
   *
   * <h2>Example</h2>
   *
   * <pre>{@code
   * FollowPath.Builder builder = new FollowPath.Builder(
   *     driveSubsystem,
   *     this::getPose,
   *     this::getSpeeds,
   *     this::drive,
   *     translationPID,
   *     rotationPID,
   *     crossTrackPID
   * ).withDefaultShouldFlip();
   *
   * Command cmd = builder.build(myPath);
   * }</pre>
   */
  public BLineAutoChooser(
      Subsystem driveSubsystem,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
      Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
      PIDController translationController,
      PIDController rotationController,
      PIDController crossTrackController) {
    super(
        driveSubsystem,
        poseSupplier,
        robotRelativeSpeedsSupplier,
        robotRelativeSpeedsConsumer,
        translationController,
        rotationController,
        crossTrackController);
    this.drive = (Drive) driveSubsystem;
  }

  /**
   * Create and populate a sendable chooser with all PathPlannerAutos in the project. The default
   * option will be Commands.none()
   *
   * @param optionsModifier A lambda function that can be used to modify the options before they go
   *     into the AutoChooser
   * @return SendableChooser populated with all autos
   */
  // public static SendableChooser<Command> buildAutoChooserWithOptionsModifier(
  //     Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
  //   return buildAutoChooserWithOptionsModifier("", optionsModifier);
  // }

  /**
   * Create and populate a sendable chooser with BLine autos
   *
   * @param defaultAutoName The name of the auto that should be the default option. If this is an
   *     empty string, or if an auto with the given name does not exist, the default option will be
   *     Commands.none()
   * @param optionsModifier A lambda function that can be used to modify the options before they go
   *     into the AutoChooser
   * @return SendableChooser populated with all autos
   */
  public SendableChooser<Command> buildAutoChooserWithOptionsModifier(Function<Stream<Path>, Stream<Path>> optionsModifier) {

    List<String> autoNames = getAllAutoNames();

    for (String autoName : autoNames) {
      Path auto = new Path(autoName);

        options.add(auto);
        chooser.addOption(autoName, this.build(auto));
    }
    chooser.setDefaultOption("None", Commands.none());
    chooser.addOption("None", Commands.none());

    optionsModifier.apply(options.stream());
    // .forEach(auto -> chooser.addOption(auto.toString(), this.build(auto)));

    return chooser;
  }

  /**
   * creates and registers a path composed of multiple other paths :)
   *
   * @param name The name of the auto
   * @param paths The name of the paths to string together
   */
  public void createPathSequence(String name, String... pathNames) {
    Path[] paths = new Path[pathNames.length];
    for (int i = 0; i < pathNames.length; i++) {
      paths[i] = new Path(pathNames[i]);
    }
    createPathSequence(name, paths);
  }

  /**
   * creates and registers a path composed of multiple other paths :)
   *
   * @param name The name of the auto
   * @param paths The paths to string together or some
   */
  public void createPathSequence(String name, Path... paths) {
    Command[] commands = new Command[paths.length];
    withPoseReset(drive::setPose);
    commands[0] = build(paths[0]);
    withPoseReset(pose -> {});
    for (int i = 1; i < paths.length; i++) {
      commands[i] = build(paths[i]);
    }
    Command auto = new SequentialCommandGroup(commands);
    chooser.addOption(name, auto);
  }
  /**
   * i think this might be the only way to run wait commands in bline
   *
   * @param name name of auto
   * @param commands uhhh commands to put together, does paths very jankily
   */
  public void createCommandPathSequence(String name, Command... commands) {
    Command auto = new SequentialCommandGroup(commands);
    chooser.addOption(name, auto);
  }

  /**
   * Get a list of all auto names in the project
   *
   * @return List of all auto names
   */
  public static List<String> getAllAutoNames() {
    File[] autoFiles = new File(Filesystem.getDeployDirectory(), "autos/paths").listFiles();

    if (autoFiles == null) {
      return new ArrayList<>();
    }

    return Stream.of(autoFiles)
        .filter(file -> !file.isDirectory())
        .map(File::getName)
        .filter(name -> name.endsWith(".json"))
        .map(name -> name.substring(0, name.lastIndexOf(".")))
        .collect(Collectors.toList());
  }

  public FollowPath buildWithPoseReset(Path path, Consumer<Pose2d> poseResetConsumer) {
    this.withPoseReset(poseResetConsumer);
    return this.build(path);
  }

  // create your autos here
  public void createAutos() {
    // rembrandts autos
    createPathSequence(
        "Rembrandts Back Bump", "Rembrandts P1", "Rembrandts P2 Bump", "Rembrandts P3");
    createPathSequence(
        "Rembrandts Back Trench", "Rembrandts P1", "Rembrandts P2 Trench", "Rembrandts P3");

    // testin wait command
    createCommandPathSequence(
        "testing",
        buildWithPoseReset(new Path("Rembrandts P1"), drive::setPose),
        Commands.waitSeconds(5),
        buildWithPoseReset(new Path("Rembrandts P2 Bump"), pose -> {}));
  }
}
