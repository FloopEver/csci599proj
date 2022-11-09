import java.util.Arrays;

public class Convert {
    public static double[] helper(double[] start, double[] input) {
        double[] res = new double[3];

        res[0] = (input[0] - start[0])/100.0;
        res[1] = (input[1] - start[1])/100.0;
        res[2] = -1 * (input[2] - start[2])/100.0;

        return res;

    }

    public static void main(String[] args) {
        double[] start = {-2429.386719, -13270.625977, 150};
        double[] input = {12417, 12305, 532.75};
        System.out.println(Arrays.toString(helper(start, input)));
    }
}
