package frc.robot.extern.carto;

import brigero.cartographer4java.Cartographer4Java;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class GoogleCartographer {
    private boolean isInitiated = false;
    private final Cartographer4Java cartographerPort;
    private CartographerOut oldOutput = null;
    private int scanCount = 0;
    private final OnCartoReqEvent event;

    public GoogleCartographer() {
        this.cartographerPort = new Cartographer4Java();
        this.event = null;
    }

    public GoogleCartographer(OnCartoReqEvent event) {
        this.cartographerPort = new Cartographer4Java();
        this.event = event;
    }

    public void initiate(String pathToFolder, String mainFile, boolean useImu, boolean useOdom, double lidarHZ) {
        isInitiated = true;
        cartographerPort.init(pathToFolder, mainFile, useImu, useOdom, lidarHZ);
    }

    public CartographerOut getCartographerMapData() {
        if (!isInitiated) {
            return null;
        }

        // Needed bc the map render is slow **I THINK**
        if (scanCount >= 2 || oldOutput == null) {
            byte[] rawMap = cartographerPort.paintMap();
            scanCount = 0;

            ByteBuffer byteBuffer = ByteBuffer.wrap(rawMap).order(ByteOrder.nativeOrder());
            final long mapSizeX = byteBuffer.getLong();
            final long mapSizeY = byteBuffer.getLong();
            final double originX = byteBuffer.getDouble();
            final double originY = byteBuffer.getDouble();
            final double resolution = byteBuffer.getDouble();
            final byte[] mapBytes = new byte[byteBuffer.remaining()];
            byteBuffer.get(mapBytes);

            if (mapBytes.length == 0) {
                return null;
            }

            oldOutput = new CartographerOut(mapSizeX, mapSizeY, originX, originY, resolution, mapBytes,
                    new CartographerOut.CartoFunctions() {
                        @Override
                        public float[] GetGlobalData() {
                            return GetRobotPositionData();
                        }

                        @Override
                        public float getGlobalX() {
                            return cartographerPort.posX();
                        }

                        @Override
                        public float getGlobalY() {
                            return cartographerPort.posY();
                        }

                        @Override
                        public float getTheta() {
                            return cartographerPort.angle();
                        }
                    });
        }

        if (event != null) {
            event.onCartoDataReq(oldOutput);
        }

        return oldOutput;
    }

    public float getGlobalX() {
        return cartographerPort.posX();
    }

    public float getGlobalY() {
        return cartographerPort.posY();
    }

    public void deleteCartoAndOptimize() {
        cartographerPort.stopAndOptimize();

    }

    public float getCartoAngleRadians() {
        return cartographerPort.angle();
    }

    /**
     * @param timeStampMS should be in System.currentTimeMillis()
     * @param cartesianX  float[] of the lidar data IN CARTESIAN
     * @param cartesianY  float[] of the lidar data IN CARTESIAN
     * @param intensities float[] of the how reflective a surface is. If lidar does not return this param, just make
     *                    a new array filled with **255**
     */
    public void updateLidarData(long timeStampMS, float[] cartesianX,
                                float[] cartesianY,
                                float[] intensities) {
        if (!isInitiated) {
            return;
        }

        scanCount++;
        cartographerPort.updateLidarData(timeStampMS, cartesianX, cartesianY, intensities);
    }

    public float[] GetGlobalPosXY() {
        return new float[]{getGlobalX(), getGlobalY()};
    }

    public float[] GetRobotPositionData() {
        return new float[]{this.getGlobalX(), this.getGlobalY(), this.getCartoAngleRadians()};
    }

    public DataOutputFinish getCallback() {
        return scanCartesianData -> updateLidarData(System.currentTimeMillis(), scanCartesianData[0],
                scanCartesianData[1],
                scanCartesianData[2]);
    }

    public interface OnCartoReqEvent {
        void onCartoDataReq(CartographerOut out);
    }
}
