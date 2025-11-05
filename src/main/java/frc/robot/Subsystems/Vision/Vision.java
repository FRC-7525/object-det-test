package frc.robot.Subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import org.photonvision.PhotonCamera;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    
    private PhotonCamera camera;
    private static Vision instance;

    private Angle targetYaw;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision(FRONT_LEFT_CAM_NAME);
        }

        return instance;
    }

    private Vision(String name) {
        camera = new PhotonCamera(name);
    }

    public Angle getYawToTarget() {
        return targetYaw;
    }

    @Override
    public void periodic() {
        var results = camera.getAllUnreadResults();

        for (var result : results) {
            var target = result.getBestTarget();

            if (target != null) targetYaw = Degrees.of(target.getYaw());
        }
    }
}
