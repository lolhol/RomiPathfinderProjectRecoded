package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.extern.carto.GoogleCartographer;
import frc.robot.render.JFrameRenderer;
import frc.robot.subsystems.SubsystemDrivetrain;
import frc.robot.subsystems.SubsystemLidar;
import frc.robot.subsystems.SubsystemPathExec;

public class RobotContainer {
    /**
     * not initialized
     */
    JFrameRenderer renderer = null;
    GoogleCartographer cartographer = null;
    SubsystemLidar subsystemLidar = null;
    SubsystemPathExec subsystemPathExec = null;

    /**
     * initialized
     */
    SubsystemDrivetrain drivetrain = new SubsystemDrivetrain();

    /**
     * misc
     */
    private int renderTickCount = 25;

    public void autoInit() {
        subsystemPathExec = new SubsystemPathExec(drivetrain, aByte -> {
            double value = (aByte & 0xff) / 100.;

            int color;
            if (value == 2.55) {
                color = 127;
            } else {
                if (value < 0.5) {
                    color = (aByte & 0xff) + 127;
                } else {
                    color = (aByte & 0xff) - 127;
                }
            }

            return color < 0;
        }, timeTookMS -> System.out.println("Finished Path! Took " + timeTookMS + "ms!"), renderer);

        new Thread(() -> {
            cartographer = new GoogleCartographer(out -> {
                if (renderTickCount >= 25) {
                    if (renderer == null) {
                        renderer = new JFrameRenderer(out.mapSizeX, out.mapSizeY, out.map,
                                (pos, resolution, originX, originY) -> {
                                    subsystemPathExec.setEndGoal(pos, this.renderer, resolution, originX, originY);
                                    subsystemPathExec.setState(true);
                                });
                        renderer.reDraw();
                    } else {
                        renderer.updateResolution(out.resolution);
                        renderer.putData(out.map, out.mapSizeX, out.mapSizeY);
                    }

                    renderTickCount = 0;
                } else {
                    renderTickCount++;
                }

                if (renderer != null) {
                    renderer.updateResolution(out.resolution);
                    renderer.updateGlobalOrigin(new double[]{out.originX, out.originY});
                    renderer.updatePosition(out.functions.GetGlobalData());
                }
            });

            cartographer.initiate("src/main/java/frc/robot/extern/config", "cartographer_config_main.lua", false,
                    false, 10);
        }).start();

        subsystemPathExec.setDefaultCommand(
                new RunCommand(() -> subsystemPathExec.tick(cartographer.getCartographerMapData()), subsystemPathExec));

        subsystemLidar = new SubsystemLidar(
                scanCartesianData -> cartographer.updateLidarData(System.currentTimeMillis(), scanCartesianData[0],
                        scanCartesianData[1],
                        scanCartesianData[2]), false, "/dev/ttyUSB0");
        subsystemLidar.setScanning(true);
        subsystemLidar.startScanning();
    }
}
