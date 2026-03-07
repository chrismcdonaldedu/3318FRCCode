// ============================================================================
// FILE: src/main/java/frc/robot/subsystems/ClimberSubsystem.java
//
// PURPOSE: Controls the climber winch that lifts the robot at the end of a match.
//
// STATUS: DISABLED — no climber hardware is currently installed.
//         All motor code is commented out; methods are no-op stubs.
//         Uncomment when climber hardware is added back.
//
// Hardware: Two TalonFX (Kraken X60) with one leader + one follower.
//   The follower copies the leader's output automatically.
//
// KEY FIXES FROM v1:
//   1. StrictFollower is stored and reapplied in stop(). This prevents the
//      follower from getting "stuck" in a neutral state if the leader changes
//      control modes.
//   2. stop() now explicitly puts BOTH motors back into follower mode or stops
//      the leader (follower stops automatically when leader does).
//   3. Added manual power scaling so the operator can climb carefully.
//   4. Added telemetry so the team can monitor winch position during endgame.
// ============================================================================
package frc.robot.subsystems;

// --- CLIMBER DISABLED: hardware imports commented out ---
// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.StrictFollower;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    // --- CLIMBER DISABLED: no hardware on robot ---
    // private final TalonFX leaderWinch =
    //         new TalonFX(Constants.CAN.CLIMBER_LEADER, new CANBus(Constants.CAN.CTRE_CAN_BUS));
    // private final TalonFX followerWinch =
    //         new TalonFX(Constants.CAN.CLIMBER_FOLLOWER, new CANBus(Constants.CAN.CTRE_CAN_BUS));
    //
    // private final PositionVoltage positionRequest = new PositionVoltage(0);
    //
    // private final StrictFollower followRequest =
    //         new StrictFollower(Constants.CAN.CLIMBER_LEADER);

    // --------------------------------------------------------------------------
    // Constructor
    // --------------------------------------------------------------------------
    public ClimberSubsystem() {
        // --- CLIMBER DISABLED: motor configuration commented out ---
        // TalonFXConfiguration cfg = new TalonFXConfiguration();
        // cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        // cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Climber.FWD_SOFT_LIMIT;
        // cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        // cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Climber.REV_SOFT_LIMIT;
        // cfg.CurrentLimits.StatorCurrentLimit       = Constants.Climber.STATOR_LIMIT_A;
        // cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        // cfg.Slot0.kP = Constants.Climber.CLIMBER_kP;
        // cfg.Slot0.kD = Constants.Climber.CLIMBER_kD;
        // leaderWinch.getConfigurator().apply(cfg);
        // followerWinch.getConfigurator().apply(cfg);
        // followerWinch.setControl(followRequest);
        // leaderWinch.getPosition().setUpdateFrequency(10);
        // leaderWinch.getVelocity().setUpdateFrequency(4);
        // leaderWinch.getStatorCurrent().setUpdateFrequency(4);
        // leaderWinch.getDeviceTemp().setUpdateFrequency(1);
        // followerWinch.getPosition().setUpdateFrequency(4);
        // followerWinch.getVelocity().setUpdateFrequency(4);
        // followerWinch.getDeviceTemp().setUpdateFrequency(1);
    }

    @Override
    public void periodic() {
        // --- CLIMBER DISABLED ---
        // SmartDashboard.putNumber("Climber/WinchPositionRot", getWinchPositionRot());
        // SmartDashboard.putNumber("Climber/CurrentAmps", getCurrentAmps());
    }

    public double getWinchPositionRot() {
        // --- CLIMBER DISABLED ---
        // return leaderWinch.getPosition().getValueAsDouble();
        return 0.0;
    }

    public double getCurrentAmps() {
        // --- CLIMBER DISABLED ---
        // return leaderWinch.getStatorCurrent().getValueAsDouble();
        return 0.0;
    }

    public boolean isAtLevel1Target() {
        // --- CLIMBER DISABLED ---
        // return Math.abs(getWinchPositionRot() - Constants.Climber.LEVEL1_TARGET_ROT)
        //         <= Constants.Climber.LEVEL1_TOLERANCE_ROT;
        return false;
    }

    public void setWinchPower(double power) {
        // --- CLIMBER DISABLED ---
        // leaderWinch.set(power * Constants.Climber.MANUAL_POWER_SCALE);
    }

    public void autoClimbLevel1() {
        // --- CLIMBER DISABLED ---
        // leaderWinch.setControl(
        //         positionRequest.withPosition(Constants.Climber.LEVEL1_TARGET_ROT));
    }

    public void stop() {
        // --- CLIMBER DISABLED ---
        // leaderWinch.stopMotor();
        // followerWinch.setControl(followRequest);
    }
}
