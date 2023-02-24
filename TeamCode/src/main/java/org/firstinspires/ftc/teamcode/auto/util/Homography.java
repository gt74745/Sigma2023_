package org.firstinspires.ftc.teamcode.auto.util;

import Jama.Matrix;
import org.opencv.core.Point;

public class Homography {
    // Rotation matrix times [0,0,66]^T
//    private static final Matrix TRANSLATION_VECTOR = new Matrix( new double[][] {
//            {15.629244},
//            {36.820196},
//            {0}
//    } );

    private static final Matrix ROTATION_MATRIX = new Matrix( new double[][]{
//            {0.0000000, 0.9205049, 0.3907311},
//            {-0.0000000, -0.3907311, 0.9205049},
//            {1.0000000, -0.0000000, 0.0000000}
            {0.0000000, -0.9205049, 0.3907311},
            {-0.0000000, -0.3907311, 0.9205049},
            {1.0000000, -0.0000000, 0.0000000}
    });

    //Value from camera calibration
    private static final Matrix CAMERA_MATRIX = new Matrix( new double[][] {
            {1.45508789e3, 0, 6.98702405e2},
            {0, 1.44057202e3, 3.38917510e2},
            {0, 0, 1}
    } );

    private static final Matrix TRANSLATION_VECTOR = ROTATION_MATRIX.times(new Matrix(new double[][] {
            {0},
            {0},
            {40}
    }));

    //Z axis is always 1 since all the objects are on the ground
    final static double Z_CONST = 1;


    /**
     * Calculates the position of the corresponding point in 3D space from a camera point
     * @param point camera point at which to find the corresponding 3D point
     * @return point's position relative to the camera
     */
    public static Point positionFromPoint( Point point ) {

        //Change point into a matrix
        Matrix pointMatrix = new Matrix( new double[][] {
                {point.x},
                {point.y},
                {1}
        } );

        Matrix left = ROTATION_MATRIX.inverse().times(CAMERA_MATRIX.inverse()).times(pointMatrix);
        Matrix right = ROTATION_MATRIX.inverse().times(TRANSLATION_VECTOR);
        double scalar = (Z_CONST + right.get(2, 0)) / left.get(2, 0);

        Matrix pos = ROTATION_MATRIX.inverse().times(CAMERA_MATRIX.inverse().times(pointMatrix).times(scalar).minus(TRANSLATION_VECTOR));
//        pos = pos.times(1/21.4639466);

        return new Point(pos.get(1, 0), pos.get(0, 0));
    }
}
