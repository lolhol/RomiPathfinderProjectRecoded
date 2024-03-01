package frc.robot.render;

import frc.robot.util.MathUtil;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;

public class JFrameRenderer extends JPanel {
    private byte[] data;

    private double[] curPosGlobal = null;
    private final List<double[]> additionalInfo = new ArrayList<>();

    private final JFrame frame;

    private int widthPX, heightPX;

    private double originX, originY, resolution;

    private final OnMouseClickBoard onClick;

    public JFrameRenderer(int width, int height, byte[] initDat, OnMouseClickBoard onClickBoard) {
        this.widthPX = width;
        this.heightPX = height;
        this.data = initDat;
        this.onClick = onClickBoard;

        frame = new JFrame("Grid Map");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.getContentPane().add(this);
        frame.pack();
        frame.setSize(width, height);
        frame.setVisible(true);

        frame.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                onClick.onMouseClickPosition(new int[]{e.getX(), e.getY() - 35}, resolution, originX, originY);
            }
        });

        this.reDraw();
    }

    public void setSize(int width, int height) {
        this.widthPX = width;
        this.heightPX = height;
        frame.setSize(width, height);
    }

    public void setCurPosition(double[] pos) {
        curPosGlobal = pos;
    }

    public void setCurPosition(float[] pos) {
        curPosGlobal = new double[]{pos[0], pos[1], pos[2]};
    }

    public void clearAdditionalData() {
        additionalInfo.clear();
    }

    public void putData(byte[] newData, int newW, int newH) {
        this.data = newData.clone();
        this.setSize(newW, newH);
        this.reDraw();
    }

    public void addAdditionalInfo(int[] positionMap) {
        additionalInfo.add(MathUtil.fromMapToGlobal(positionMap, resolution, originX, originY));
    }

    public void addAdditionalInfo(double[] positionMap) {
        additionalInfo.add(positionMap);
    }

    public void addAdditionalInfo(float[] positionMap) {
        if (positionMap.length <= 2) {
            additionalInfo.add(new double[]{positionMap[0], positionMap[1]});
        } else {
            additionalInfo.add(new double[]{positionMap[0], positionMap[1], positionMap[2]});
        }
    }

    public void updateResolution(double newRes) {
        resolution = newRes;
    }

    public void updateGlobalOrigin(double[] newOrigin) {
        originX = newOrigin[0];
        originY = newOrigin[1];
    }

    public void updatePosition(float[] newPosition) {
        if (curPosGlobal != null) {
            Graphics g = this.getGraphics();

            int[] curPosMap = MathUtil.fromGlobalToMap(curPosGlobal, resolution, originX, originY);
            int[] point2 = getPointInDirection(curPosMap, curPosGlobal[2], 20);
            g.setColor(Color.WHITE);
            g.fillRect(curPosMap[0], curPosMap[1], 2, 2);
            g.drawLine(curPosMap[0], curPosMap[1], point2[0], point2[1]);

            this.setCurPosition(newPosition);
            curPosMap = MathUtil.fromGlobalToMap(curPosGlobal, resolution, originX, originY);
            point2 = getPointInDirection(curPosMap, curPosGlobal[2], 20);

            g.setColor(Color.BLUE);
            g.fillRect(curPosMap[0], curPosMap[1], 2, 2);
            g.setColor(Color.CYAN);
            g.drawLine(curPosMap[0], curPosMap[1], point2[0], point2[1]);
        }
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        if (data == null || data.length == 0)
            return;

        for (int y = 0; y < heightPX; y++) {
            for (int x = 0; x < widthPX; x++) {
                int color = getColor(y, x, data, widthPX);

                g.setColor(new Color(color, color, color));
                g.fillRect(x, y, 1, 1);
            }
        }

        int[] prevPos = null;
        for (double[] cur : additionalInfo) {
            int[] i = MathUtil.fromGlobalToMap(cur, resolution, originX, originY);
            g.setColor(Color.RED);
            g.fillRect(i[0], i[1], 2, 2);

            if (cur.length > 2) {
                g.setColor(Color.BLUE);
                int[] point2 = getPointInDirection(i, cur[2], 20);
                g.drawLine(i[0], i[1], point2[0], point2[1]);
            }

            if (prevPos != null) {
                g.setColor(Color.ORANGE);
                g.drawLine(prevPos[0], prevPos[1], i[0], i[1]);
            }
            
            prevPos = i;
        }

        if (curPosGlobal != null) {
            int[] curPosMap = MathUtil.fromGlobalToMap(curPosGlobal, resolution, originX, originY);
            g.setColor(Color.BLUE);
            g.fillRect(curPosMap[0], curPosMap[1], 2, 2);

            g.setColor(Color.CYAN);
            int[] point2 = getPointInDirection(curPosMap, curPosGlobal[2], 20);
            g.drawLine(curPosMap[0], curPosMap[1], point2[0], point2[1]);
        }
    }

    public static int getColor(int y, int x, byte[] data, int widthPX) {
        double value = (data[y * widthPX + x] & 0xff) / 100.;

        int color;
        if (value == 2.55) {
            color = 127;
        } else {
            if (value < 0.5) {
                color = (data[y * widthPX + x] & 0xff) + 127;
            } else {
                color = (data[y * widthPX + x] & 0xff) - 127;
            }
        }

        /*if (color > 255) {
            color = 255;
        } else if (color < 0) {
            color = 0;
        }*/
        return color < 0 ? 0 : 255;
    }

    private int[] getPointInDirection(int[] coordinates, double angle, double dist) {
        int x = coordinates[0];
        int y = coordinates[1];
        int newX = x + (int) (dist * Math.cos(angle));
        int newY = y + (int) (dist * Math.sin(angle));
        return new int[]{newX, newY};
    }

    public void reDraw() {
        this.repaint();
    }
}
