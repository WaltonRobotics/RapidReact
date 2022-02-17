package frc.robot.util.averages;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.math.RoundingMode;

public class CumulativeAverage implements MovingAverage {

    private BigDecimal sum = BigDecimal.ZERO;
    private BigInteger count = BigInteger.ZERO;

    @Override
    public void clear() {
        sum = BigDecimal.ZERO;
        count = BigInteger.ZERO;
    }

    @Override
    public void addData(double num) {
        sum = sum.add(new BigDecimal(num));
        count = count.add(BigInteger.ONE);
    }

    @Override
    public int getNumValues() {
        return count.intValue();
    }

    @Override
    public double getMean() {
        // Safeguard against divide-by-zero
        if (count.equals(BigInteger.ZERO)) {
            return 0.0;
        }

        return sum.divide(new BigDecimal(count), RoundingMode.HALF_UP).doubleValue();
    }

}
