package org.firstinspires.ftc.teamcode.auto.util;

import Jama.Matrix;
import org.opencv.core.Point;

public class Homography {
    private static final Matrix ROTATION_MATRIX = new Matrix( new double[][]{
            {  0.9659258, 0.2588190,  0.0000000},
            {-0.0225576, -0.0841860, -0.9961947},
            {0.2578342,  0.9622502, 0.0871557 } // (95,0,15) good
//            {  0.9563047, 0.2923717,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            {0.2923717,  0.9563047,  0.0000000} (90,0,17)
//            {  0.9612617, 0.2756374,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            {0.2756374,  0.9612617,  0.0000000 } // (90,0,16)
//            {  0.9781476, 0.2079117,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            { 0.2079117,  0.9781476,  0.0000000 } // (90,0,12) too little angle
//            {  0.9659258, 0.2588190,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            {0.2588190,  0.9659258,  0.0000000 } // (90,0,15) pretty good
//            {  0.9304176, 0.3665012,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            {0.3665012,  0.9304176,  0.0000000 } // (90,0,21.5) // decent
//            {  0.9510565, 0.3090170,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            {0.3090170,  0.9510565,  0.0000000 } // (90,0,18)
//            {  0.9396926, 0.3420202,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            {0.3420202,  0.9396926,  0.0000000 } // better! (90,0,20)
//            {  0.9063078, 0.4226183,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            {0.4226183,  0.9063078,  0.0000000 }// (90,0,25)
//            {  0.9205049, 0.3907311,  0.0000000},
//            {0.0000000,  0.0000000, -1.0000000},
//            {0.3907311,  0.9205049,  0.0000000 } // good! (90,0,23)
    });

    //Value from camera calibration
    private static final Matrix CAMERA_MATRIX = new Matrix( new double[][] {
            {1.45508789e3, 0, 6.98702405e2},
            {0, 1.44057202e3, 3.38917510e2},
            {0, 0, 1}
    } );


    //    private static final Matrix TRANSLATION_VECTOR = new Matrix(new double[][] {
//            {0},
//            {0},
//            {40}
//    });
    private static final Matrix TRANSLATION_VECTOR = ROTATION_MATRIX.times(new Matrix(new double[][] {
            {0},
            {0},
            {-40}
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

        double scale = 1;
        return new Point(scale * (pos.get(0, 0) + 70), scale * (pos.get(1, 0) - 100));
    }

}

