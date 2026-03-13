package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class AlignAndShootCommandTest {

    @Test
    void realisticHubShotPitchIsConsideredFeasible() {
        double nominalPitchDeg =
                (Constants.Vision.MIN_SHOT_PITCH_DEG + Constants.Vision.MAX_SHOT_PITCH_DEG) * 0.5;
        assertTrue(AlignAndShootCommand.isShotPitchFeasible(nominalPitchDeg));
    }

    @Test
    void extremePitchStillRejected() {
        assertFalse(AlignAndShootCommand.isShotPitchFeasible(Constants.Vision.MAX_SHOT_PITCH_DEG + 1.0));
    }

    @Test
    void alignShootMatchesAlignOnlyYawTolerance() {
        assertEquals(Constants.Vision.YAW_TOLERANCE_DEG, Constants.AlignShoot.YAW_TOLERANCE_DEG, 1e-9);
    }

    @Test
    void alignShootMatchesAlignOnlyRotationCap() {
        assertEquals(Constants.Vision.MAX_ROT_CMD, Constants.AlignShoot.MAX_AUTO_AIM_OMEGA_RADPS, 1e-9);
    }
}
