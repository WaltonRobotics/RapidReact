package com.team254.lib.geometry;

import com.team254.lib.util.CSVWritable;
import frc.robot.util.interpolation.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
