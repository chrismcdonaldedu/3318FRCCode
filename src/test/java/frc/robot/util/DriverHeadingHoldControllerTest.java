package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants;

class DriverHeadingHoldControllerTest {

    @Test
    void staysPassiveWhileDriverCommandsTurn() {
        DriverHeadingHoldController controller = new DriverHeadingHoldController();

        DriverHeadingHoldController.Output output = controller.update(20.0, 1.0, 0.4);

        assertFalse(output.active());
        assertEquals(0.4, output.commandedOmegaRadPerSec(), 1e-9);
        assertEquals(0.0, output.correctionOmegaRadPerSec(), 1e-9);
    }

    @Test
    void capturesHeadingAndHoldsWhenTranslationStarts() {
        DriverHeadingHoldController controller = new DriverHeadingHoldController();

        DriverHeadingHoldController.Output initial = controller.update(15.0, 1.0, 0.0);
        DriverHeadingHoldController.Output correcting = controller.update(12.0, 1.0, 0.0);

        assertTrue(initial.active());
        assertEquals(15.0, initial.targetHeadingDeg(), 1e-9);
        assertTrue(correcting.active());
        assertEquals(15.0, correcting.targetHeadingDeg(), 1e-9);
        assertEquals(3.0, correcting.headingErrorDeg(), 1e-9);
        assertTrue(correcting.correctionOmegaRadPerSec() > 0.0);
        assertEquals(
                correcting.correctionOmegaRadPerSec(),
                correcting.commandedOmegaRadPerSec(),
                1e-9);
    }

    @Test
    void staysPassiveBelowMinimumTranslationSpeed() {
        DriverHeadingHoldController controller = new DriverHeadingHoldController();

        DriverHeadingHoldController.Output output = controller.update(
                15.0,
                Constants.Swerve.HEADING_HOLD_MIN_TRANSLATION_MPS - 0.01,
                0.0);

        assertFalse(output.active());
        assertEquals(0.0, output.commandedOmegaRadPerSec(), 1e-9);
        assertEquals(0.0, output.correctionOmegaRadPerSec(), 1e-9);
    }

    @Test
    void resetsWhenTranslationStops() {
        DriverHeadingHoldController controller = new DriverHeadingHoldController();

        controller.update(10.0, 1.0, 0.0);
        DriverHeadingHoldController.Output stopped = controller.update(9.0, 0.0, 0.0);
        DriverHeadingHoldController.Output resumed = controller.update(7.0, 1.0, 0.0);

        assertFalse(stopped.active());
        assertEquals(0.0, stopped.commandedOmegaRadPerSec(), 1e-9);
        assertTrue(resumed.active());
        assertEquals(7.0, resumed.targetHeadingDeg(), 1e-9);
        assertEquals(0.0, resumed.commandedOmegaRadPerSec(), 1e-9);
    }

    @Test
    void wrapsHeadingErrorAcrossMinus180To180() {
        DriverHeadingHoldController controller = new DriverHeadingHoldController();

        controller.update(179.0, 1.0, 0.0);
        DriverHeadingHoldController.Output output = controller.update(-179.0, 1.0, 0.0);

        assertTrue(output.active());
        assertEquals(-2.0, output.headingErrorDeg(), 1e-9);
        assertTrue(output.correctionOmegaRadPerSec() < 0.0);
    }
}
