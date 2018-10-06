//
// Created by aadc on 10.09.18.
//

#include "PixelMetricTransformer.h"

PixelMetricTransformer::PixelMetricTransformer(double t_matrix[2][3]) {
    inverted = Matx23d(t_matrix[0][0], t_matrix[0][1], t_matrix[0][2], t_matrix[1][0], t_matrix[1][1], t_matrix[1][2]);
    invertAffineTransform(inverted, matrix);
}

double defaultInit[2][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};

PixelMetricTransformer::PixelMetricTransformer():PixelMetricTransformer(defaultInit) { }

Point2d PixelMetricTransformer::toMeter(double x, double y) {
    vector<Point2d> vec;
    vec.push_back(Point2d(x,y));
    transform(vec, vec, matrix);
    return Point2d(vec.front().x, vec.front().y );
}

Point2d PixelMetricTransformer::toPixel(double x, double y) {
    vector<Point2d> vec;
    vec.push_back(Point2d(x,y));
    transform(vec, vec, inverted);
    return Point2d(vec.front().x, vec.front().y );
}
