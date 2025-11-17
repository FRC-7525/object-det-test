package frc.robot.Subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import org.photonvision.PhotonCamera;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.GamePieceFinder.GamePieceFinder;

public class Vision extends SubsystemBase {
    
    private PhotonCamera camera;
    private static Vision instance;

    private Angle targetYaw; 
    
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision(FRONT_RIGHT_CAM_NAME);
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
            GamePieceFinder.getInstance().addVisionSample(target);
            if (target != null) {
                targetYaw = Degrees.of(target.getYaw());
                System.out.println(targetYaw.in(Degrees));
        }
    }
    }
}
