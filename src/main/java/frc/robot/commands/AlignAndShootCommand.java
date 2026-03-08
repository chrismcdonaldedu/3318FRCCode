// ============================================================================
// FILE: src/main/java/frc/robot/commands/AlignAndShootCommand.java
//
// PURPOSE: Rotates the robot to aim at a vision target, then fires.
//
// SEQUENCE:
//   1. ALIGN    — Start shooter wheels AND rotate toward vision target simultaneously
//   2. CLEAR    — Brief feeder reverse to prevent double-feeding (with active alignment)
//   3. FEED     — Push game piece through feeder + hopper + intake roller (committed, no abort)
//   4. DONE     — Command finishes, everything stops
//
// VISION SOURCE: Reads VisionResult from the background RioVisionThread via
//   an AtomicReference.  No PhotonVision dependency.
// ============================================================================
package frc.robot.commands;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.HubActivityTracker;
import frc.robot.subsystems.*;
import frc.robot.vision.VisionResult;

public class AlignAndShootCommand extends Command {

    // --------------------------------------------------------------------------
    // Static telemetry snapshot
    // --------------------------------------------------------------------------
    private static volatile String telemetryState = "IDLE";
    private static volatile boolean telemetryCommandActive = false;
    private static volatile boolean telemetryHasTarget = false;
    private static volatile boolean telemetryGeometryFeasible = false;
    private static volatile boolean telemetryHasShootableTarget = false;
    private static volatile double telemetryYawDeg = Double.NaN;
    private static volatile double telemetryPitchDeg = Double.NaN;
    private static volatile double telemetryTargetRps = Double.NaN;
    private static volatile String telemetryLastAbortReason = "";

    // All the subsystems this command needs to control
    private final SwerveSubsystem  swerve;
    private final ShooterSubsystem shooter;
    private final FeederSubsystem  feeder;
    private final HopperSubsystem  hopper;
    private final IntakeSubsystem  intake;

    // Vision data from the background RioVisionThread
    private final AtomicReference<VisionResult> visionRef;

    // PD controller for rotating toward the target.
    private final PIDController turnPID = new PIDController(
            Constants.Vision.TURN_kP,
            0,
            Constants.Vision.TURN_kD);

    // ---- State machine ----
    private enum State { ALIGN, CLEAR, FEED, DONE }
    private State state;

    private final Timer stateTimer = new Timer();
    private double lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;
    private double calculatedRPS = Constants.Shooter.TARGET_RPS;

    private static final double ALIGN_TIMEOUT_SEC = 3.0;

    // Height of the HUB tags above ground (for pitch-based distance estimation).
    // This is the tag's vertical POSITION on the field, NOT its physical size.
    // Uses Constants.Shooter.HUB_SCORING_HEIGHT_M to avoid duplicate constants.

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public AlignAndShootCommand(SwerveSubsystem swerve,
                                 ShooterSubsystem shooter,
                                 FeederSubsystem feeder,
                                 HopperSubsystem hopper,
                                 IntakeSubsystem intake,
                                 AtomicReference<VisionResult> visionRef) {
        this.swerve  = swerve;
        this.shooter = shooter;
        this.feeder  = feeder;
        this.hopper  = hopper;
        this.intake  = intake;
        this.visionRef = visionRef;

        addRequirements(swerve, shooter, feeder, hopper, intake);
        turnPID.setTolerance(Constants.Vision.YAW_TOLERANCE_DEG);
    }

    // --------------------------------------------------------------------------
    // initialize()
    // --------------------------------------------------------------------------
    @Override
    public void initialize() {
        state = State.ALIGN;
        stateTimer.reset();
        stateTimer.start();
        lastValidTargetSeenSec = Double.NEGATIVE_INFINITY;
        telemetryState = State.ALIGN.name();
        telemetryCommandActive = true;
        telemetryHasTarget = false;
        telemetryGeometryFeasible = false;
        telemetryHasShootableTarget = false;
        telemetryYawDeg = Double.NaN;
        telemetryPitchDeg = Double.NaN;
        telemetryTargetRps = Double.NaN;
        telemetryLastAbortReason = "";

        if (!HubActivityTracker.isOurHubActive()) {
            double secsToShift = HubActivityTracker.secondsUntilNextShiftChange();
            System.out.println("[AlignAndShoot] WARNING: Alliance HUB is INACTIVE! "
                    + String.format("%.1fs", secsToShift) + " until next shift change.");
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", true);
        } else {
            SmartDashboard.putBoolean("AlignShoot/HubInactiveWarning", false);
        }

        calculatedRPS = Constants.Shooter.TARGET_RPS;
        telemetryTargetRps = calculatedRPS;
        shooter.setShooterVelocity(calculatedRPS);
        SmartDashboard.putString("AlignShoot/State", "ALIGN");
    }

    // --------------------------------------------------------------------------
    // execute()
    // --------------------------------------------------------------------------
    @Override
    public void execute() {
        switch (state) {

            case ALIGN: {
                VisionResult result = visionRef.get();
                boolean hasResult = isResultFresh(result);
                telemetryHasTarget = hasResult;

                if (!hasResult) {
                    swerve.drive(0, 0, 0, false);
                    telemetryGeometryFeasible = false;
                    telemetryHasShootableTarget = false;
                    telemetryYawDeg = Double.NaN;
                    telemetryPitchDeg = Double.NaN;

                    if (stateTimer.hasElapsed(ALIGN_TIMEOUT_SEC)) {
                        System.out.println("[AlignAndShoot] No alliance HUB tag found, aborting.");
                        telemetryLastAbortReason = "No alliance HUB tag found";
                        transitionTo(State.DONE);
                    }
                    break;
                }

                if (!isShotGeometryFeasible(result)) {
                    System.out.println("[AlignAndShoot] Shot geometry not feasible, aborting.");
                    telemetryHasShootableTarget = false;
                    telemetryLastAbortReason = "Shot geometry not feasible";
                    transitionTo(State.DONE);
                    break;
                }
                lastValidTargetSeenSec = Timer.getFPGATimestamp();
                telemetryHasShootableTarget = true;
                telemetryGeometryFeasible = true;

                updateShooterFromVision(result);

                double rotCmd = calculateAlignmentRotation(result);

                if (turnPID.atSetpoint() && shooter.isAtSpeed(calculatedRPS)) {
                    swerve.drive(0, 0, 0, false);
                    transitionTo(State.CLEAR);
                } else if (turnPID.atSetpoint()) {
                    swerve.drive(0, 0, 0, false);
                } else {
                    swerve.drive(0, 0, rotCmd, false);
                }
                break;
            }

            case CLEAR: {
                if (!hasShootableTarget()) {
                    System.out.println("[AlignAndShoot] Vision lost or geometry invalid before feed, aborting.");
                    telemetryLastAbortReason = "Vision lost before feed";
                    transitionTo(State.DONE);
                    break;
                }
                // Maintain alignment while clearing
                alignIfVisionAvailable();
                feeder.setPower(Constants.Shooter.CLEAR_POWER);
                if (stateTimer.hasElapsed(Constants.Shooter.CLEAR_TIME_SEC)) {
                    feeder.stop();
                    transitionTo(State.FEED);
                }
                break;
            }

            case FEED: {
                // Once committed to feeding, finish the feed to avoid jamming.
                // A game piece partially in the shooter must be pushed through.
                feeder.setPower(Constants.Shooter.FEED_POWER);
                hopper.setPower(Constants.Shooter.FEED_POWER);
                intake.setRollerPower(Constants.Shooter.FEED_POWER);
                if (stateTimer.hasElapsed(Constants.Shooter.FEED_TIME_SEC)) {
                    transitionTo(State.DONE);
                }
                break;
            }

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
        shooter.stop();
        feeder.stop();
        hopper.stop();
        intake.setRollerPower(0);
        telemetryState = "IDLE";
        telemetryCommandActive = false;
        telemetryHasShootableTarget = false;
        telemetryTargetRps = Double.NaN;
        SmartDashboard.putString("AlignShoot/State", "IDLE");
        if (interrupted) {
            System.out.println("[AlignAndShoot] Command was interrupted.");
            telemetryLastAbortReason = "Interrupted";
        }
    }

    // --------------------------------------------------------------------------
    // Helpers
    // --------------------------------------------------------------------------

    private void transitionTo(State newState) {
        state = newState;
        stateTimer.reset();
        stateTimer.start();
        telemetryState = newState.toString();
        SmartDashboard.putString("AlignShoot/State", newState.toString());
    }

    /** Update shooter velocity from the current vision result. */
    private void updateShooterFromVision(VisionResult result) {
        double yawDeg = result.yawDeg();
        SmartDashboard.putNumber("AlignShoot/YawError", yawDeg);
        SmartDashboard.putNumber("AlignShoot/TargetTagId", result.tagId());
        telemetryYawDeg = yawDeg;

        double distanceM = result.estimateDistanceM(
                Constants.Vision.TAG_HEIGHT_M,
                Constants.Vision.FOCAL_LENGTH_PIXELS);
        if (!Double.isFinite(distanceM) || distanceM <= 0) {
            distanceM = estimateDistanceFromPitch(result.pitchDeg());
        }
        calculatedRPS = ShooterSubsystem.calculateTargetRPS(distanceM);
        telemetryTargetRps = calculatedRPS;
        shooter.setShooterVelocity(calculatedRPS);
        SmartDashboard.putNumber("AlignShoot/CalculatedRPS", calculatedRPS);
        SmartDashboard.putNumber("AlignShoot/EstDistanceM", distanceM);
    }

    /** Calculate the rotation command to align toward the vision target. */
    private double calculateAlignmentRotation(VisionResult result) {
        double yawDeg = result.yawDeg();
        double rotCmd = turnPID.calculate(yawDeg, 0);
        return MathUtil.clamp(rotCmd,
                -Constants.Vision.MAX_ROT_CMD, Constants.Vision.MAX_ROT_CMD);
    }

    /** Actively align toward the target if vision data is available. */
    private void alignIfVisionAvailable() {
        VisionResult result = visionRef.get();
        if (isResultFresh(result)) {
            double rotCmd = calculateAlignmentRotation(result);
            if (!turnPID.atSetpoint()) {
                swerve.drive(0, 0, rotCmd, false);
                return;
            }
        }
        swerve.drive(0, 0, 0, false);
    }

    /** Check if a VisionResult is recent enough to use (not stale). */
    private boolean isResultFresh(VisionResult result) {
        if (result == null || result.tagId() < 0) return false;
        double age = Timer.getFPGATimestamp() - result.timestampSec();
        return age < Constants.Vision.TARGET_LOSS_TOLERANCE_SEC;
    }

    private boolean hasShootableTarget() {
        VisionResult result = visionRef.get();
        boolean hasResult = isResultFresh(result);
        telemetryHasTarget = hasResult;

        if (hasResult) {
            boolean geometryFeasible = isShotGeometryFeasible(result);
            telemetryGeometryFeasible = geometryFeasible;
            if (geometryFeasible) {
                lastValidTargetSeenSec = Timer.getFPGATimestamp();
                telemetryHasShootableTarget = true;
                return true;
            }
            telemetryHasShootableTarget = false;
            return false;
        }
        telemetryGeometryFeasible = false;
        boolean recent = hasRecentValidTarget();
        telemetryHasShootableTarget = recent;
        return recent;
    }

    private boolean hasRecentValidTarget() {
        return Timer.getFPGATimestamp() - lastValidTargetSeenSec
                <= Constants.Vision.TARGET_LOSS_TOLERANCE_SEC;
    }

    private boolean isShotGeometryFeasible(VisionResult result) {
        double pitchDeg = result.pitchDeg();
        double yawDeg = result.yawDeg();
        SmartDashboard.putNumber("AlignShoot/TargetPitchDeg", pitchDeg);
        SmartDashboard.putNumber("AlignShoot/YawGeometryCheck", yawDeg);
        telemetryPitchDeg = pitchDeg;

        // Reject shots where the robot is facing far off-target.
        // The PID will align us, but if yaw is wildly wrong the target
        // may be a misdetection or the wrong tag entirely.
        if (Math.abs(yawDeg) > Constants.Vision.YAW_TOLERANCE_DEG * 10) {
            return false;
        }

        return isShotPitchFeasible(pitchDeg);
    }

    /**
     * Fallback distance estimation using pitch angle + camera mount geometry.
     * distance = (tagHeight - cameraHeight) / tan(cameraPitch + targetPitch)
     */
    private double estimateDistanceFromPitch(double targetPitchDeg) {
        double totalPitchRad = Constants.Vision.CAMERA_PITCH_RAD
                + Math.toRadians(targetPitchDeg);
        double heightDiff = Constants.Shooter.HUB_SCORING_HEIGHT_M - Constants.Vision.CAMERA_UP_M;
        double tanPitch = Math.tan(totalPitchRad);
        if (Math.abs(tanPitch) < 1e-6) return Double.NaN;
        return heightDiff / tanPitch;
    }

    // --------------------------------------------------------------------------
    // Static telemetry accessors (for dashboard)
    // --------------------------------------------------------------------------
    public static String getTelemetryState() { return telemetryState; }
    public static boolean isTelemetryCommandActive() { return telemetryCommandActive; }
    public static boolean telemetryHasTarget() { return telemetryHasTarget; }
    public static boolean telemetryGeometryFeasible() { return telemetryGeometryFeasible; }
    public static boolean telemetryHasShootableTarget() { return telemetryHasShootableTarget; }
    public static double getTelemetryYawDeg() { return telemetryYawDeg; }
    public static double getTelemetryPitchDeg() { return telemetryPitchDeg; }
    public static double getTelemetryTargetRps() { return telemetryTargetRps; }
    public static String getTelemetryLastAbortReason() { return telemetryLastAbortReason; }

    static boolean isShotPitchFeasible(double pitchDeg) {
        return Double.isFinite(pitchDeg)
                && pitchDeg >= Constants.Vision.MIN_SHOT_PITCH_DEG
                && pitchDeg <= Constants.Vision.MAX_SHOT_PITCH_DEG;
    }
}
