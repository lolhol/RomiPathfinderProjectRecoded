import frc.robot.pathing.calculator.AStarCalculator;
import frc.robot.pathing.calculator.Node;
import org.junit.jupiter.api.Test;

public class AStarPathfinderTest {
    @Test
    public void test1() {
        AStarCalculator calculator = new AStarCalculator();
        Node start = new Node(0, 0, null);
        Node end = new Node(3, 3, null);
        Node nodes = calculator.run(SampleMap.map, SampleMap.getWidth(), start, end,
                100, 2, aByte -> aByte == 1);

        System.out.println(start.convertTo1D(SampleMap.getWidth()));

        if (nodes != null) {
            for (Node i : nodes.reverse()) {
                SampleMap.map[i.convertTo1D(SampleMap.getWidth())] = 4;
            }
        }

        SampleMap.map[start.convertTo1D(SampleMap.getWidth())] = 5;
        SampleMap.map[end.convertTo1D(SampleMap.getWidth())] = 5;

        int c = 0;
        for (int i = 0; i < SampleMap.getWidth(); i++) {
            StringBuilder tmpStr = new StringBuilder();
            tmpStr.append("[");
            for (int j = 0; j < SampleMap.getWidth(); j++) {
                tmpStr.append(SampleMap.map[i * SampleMap.getWidth() + j] == 1 ? "#" :
                        SampleMap.map[i * SampleMap.getWidth() + j] == 5 ? "&" :
                                SampleMap.map[i * SampleMap.getWidth() + j] == 4 ? c++ : " ");
            }
            tmpStr.append("]");

            System.out.println(tmpStr);
        }
    }

    @Test
    public void testIsObstructedBetweenPoints() {
        byte[] grid = {
                0, 0, 0, 0, 0,
                0, 1, 1, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 1, 1, 0,
                0, 0, 0, 0, 0
        };

        int width = 5;

        Node point1 = new Node(0, 2, null);
        Node point2 = new Node(3, 3, null);

        boolean isObstructed = point1.isObstructedBetweenPoints(point2, grid, width, value -> value == 1);
        System.out.println(isObstructed);
    }
}
