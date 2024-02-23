package frc.robot.extern.lidar;

import ev3dev.sensors.slamtec.RPLidarA1;
import ev3dev.sensors.slamtec.RPLidarA1ServiceException;
import ev3dev.sensors.slamtec.model.Scan;

import java.util.HashMap;

public class RPLidar {
    final RPLidarA1 lidar;
    final HashMap<Long, Scan> args = new HashMap<>();

    public RPLidar(String connectionPort) throws RPLidarA1ServiceException {
        this.lidar = new RPLidarA1(connectionPort);
        lidar.init();

        lidar.addListener(
                arg0 -> args.put(System.currentTimeMillis(), arg0)
        );
    }

    /**
     * @throws RPLidarA1ServiceException
     */
    public void startLidar() throws RPLidarA1ServiceException {
        lidar.scan();
    }

    /**
     * @throws RPLidarA1ServiceException
     */
    public void stopLidar() throws RPLidarA1ServiceException {
        lidar.close();
    }

    /**
     * @return if no scans have income since the last scan get, will return null. Otherwise it will return a map of MS and scans since last req of scan get.
     */
    public HashMap<Long, Scan> getLastScans() {
        HashMap<Long, Scan> tmp = (HashMap<Long, Scan>) args.clone();
        args.clear();
        return tmp;
    }
}
