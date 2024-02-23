public class SampleMap {
    private static final int WIDTH = 10;
    private static final int HEIGHT = 10;

    // 0 represents an open cell, 1 represents an obstacle
    public static byte[] map = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 1, 0, 1, 0, 1, 0, 0, 0,
            0, 0, 1, 0, 1, 0, 1, 0, 0, 0,
            0, 0, 1, 0, 1, 0, 1, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    public static int getWidth() {
        return WIDTH;
    }

    public static int getHeight() {
        return HEIGHT;
    }
}
