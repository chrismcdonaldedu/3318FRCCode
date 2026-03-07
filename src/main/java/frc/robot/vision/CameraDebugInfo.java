package frc.robot.vision;

import frc.robot.Constants;

public record CameraDebugInfo(
        String status,
        int configuredDeviceId,
        int activeDeviceId,
        String activeCameraName,
        String activeCameraPath,
        String enumeratedCameras,
        String lastError,
        long frameCount,
        double lastFrameTimestampSec) {

    public static CameraDebugInfo defaultState() {
        return new CameraDebugInfo(
                "STARTING",
                Constants.Vision.CAMERA_DEVICE_ID,
                -1,
                "",
                "",
                "",
                "",
                0,
                Double.NaN);
    }

    public CameraDebugInfo withStatus(String nextStatus) {
        return new CameraDebugInfo(
                nextStatus,
                configuredDeviceId,
                activeDeviceId,
                activeCameraName,
                activeCameraPath,
                enumeratedCameras,
                lastError,
                frameCount,
                lastFrameTimestampSec);
    }

    public CameraDebugInfo withEnumeration(String summary) {
        return new CameraDebugInfo(
                status,
                configuredDeviceId,
                activeDeviceId,
                activeCameraName,
                activeCameraPath,
                summary,
                lastError,
                frameCount,
                lastFrameTimestampSec);
    }

    public CameraDebugInfo withActiveCamera(int nextActiveDeviceId, String name, String path) {
        return new CameraDebugInfo(
                status,
                configuredDeviceId,
                nextActiveDeviceId,
                name == null ? "" : name,
                path == null ? "" : path,
                enumeratedCameras,
                lastError,
                frameCount,
                lastFrameTimestampSec);
    }

    public CameraDebugInfo withError(String nextStatus, String error) {
        return new CameraDebugInfo(
                nextStatus,
                configuredDeviceId,
                activeDeviceId,
                activeCameraName,
                activeCameraPath,
                enumeratedCameras,
                error == null ? "" : error,
                frameCount,
                lastFrameTimestampSec);
    }

    public CameraDebugInfo withFrame(long nextFrameCount, double timestampSec) {
        return new CameraDebugInfo(
                "STREAMING",
                configuredDeviceId,
                activeDeviceId,
                activeCameraName,
                activeCameraPath,
                enumeratedCameras,
                lastError,
                nextFrameCount,
                timestampSec);
    }
}
